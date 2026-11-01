#!/usr/bin/env python3
"""export_scales.py — 从 y3t_params.json 导出逐通道 requant 尺度镜像

背景 (2026-10-19)
    引擎的 requant 需要三样东西, 以前固件只给了"退化值"(multM=1/shift=0):
      · multM[group][ot*16+ch]  —— 每个输入通道组的逐输出通道乘子 (Q32)
      · bias_q13[ot*16+ch]      —— 逐输出通道偏置 (Q13)
      · shM0/shM1/shY/multY     —— 组公共移位 + 输出乘子/移位 (寄存器)
    这些数在 y3t_params.json 的 groups[*].multM / bias_q13 / shiftY / multY 里
    全都有, 只是从来没人导成 DDR 镜像 —— 所以板子只能跑 identity, 数值不是网络输出。

    ★ 本脚本同时修正了一个此前没人发现的坑:
      `y3t_exec_rom_identity()` 原来从 ROM 地址 0 连写 1024 个 1 再写 512 个 0,
      但引擎 (engine_y3t.v:49) 规定 multM 在 ROM 0..2047、bias 在 **2048..3071**。
      那 512 个 0 实际落在 multm_rom[1024..1535], 正好是 **group 1 的 multM** ——
      把多组层 (ic>16, ng2) 第二组输入的贡献整体乘成了 0。
      本脚本产出的是"正确布局"的镜像, 固件侧 identity 也已按 2048 修正。

产物
    sw/engine_scales.bin      DDR 镜像 (块对齐, 8 字 = 1 个 256b 块)
    sw/y3t_scale_table.h      逐层字偏移 + 移位/乘子, 固件 #include 即用
    sw/engine_scales.json     元信息, 供对拍/排查

blob 内每层的排布 (字地址, 32-bit 字)
    [w_off_g0 .. +cot*16)            group0 multM   -> 引擎 ROM 偏移 0
    [w_off_g1 .. +cot*16)            group1 multM   -> 引擎 ROM 偏移 1024  (仅多组层)
    [w_off_bias .. +cot*16)          bias_q13       -> 引擎 ROM 偏移 2048

    每段长度都是 16 的倍数, 所以起始字偏移天然是 8 的倍数, rom_stream 的
    `ddr_blk + (off>>3)` 才能精确落在块边界上。

用法
    /e/anconda/python.exe tools/export_scales.py
    /e/anconda/python.exe tools/export_scales.py --dump L0 L17   # 额外自检
"""
import argparse
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

DEF_PARAMS = r'E:\ziguang_xj1\omini\tools\y3t\demo_hw\y3t_params.json'
DEF_TABLE = os.path.join(ROOT, 'sw', 'y3t_net_table.h')


def cpad16(c):
    return ((c + 15) // 16) * 16


def parse_layer_order(path):
    """从 y3t_net_table.h 抽出层名顺序 (必须与 g_net 一致)。
    注意表里的名字是定宽字符串, 带尾随空格 ("L0          ")。"""
    txt = open(path, encoding='utf-8').read()
    names = [t.strip() for t in re.findall(r'"([^"]*)"', txt)]
    names = [n for n in names if re.fullmatch(r'(L\d+|H_cv\d+_k\d+)', n)]
    return names


def parse_layers(path):
    """从层表抽 {name: (cit, cot, mode)}。

    表字段序 (tools/gen_net_table.py 生成):
      0 w_src_blk 1 w_words 2 in0 3 in1 4 out 5 split 6 k 7 s 8 pad
      9 cit 10 cot 11 H 12 W 13 Ho 14 Wo 15 mode 16 act 17 bits16 18 ich 19 och

    ★ 不能对整行 `re.findall(r'\\d+')`: 名字里含数字 (H_cv20_k0 的 20/0),
      会把字段序号整体错位。先把引号串抠掉再取数。"""
    txt = open(path, encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'"([^"]*)"\s*\}', txt):
        nm = m.group(1).strip()
        head = txt.rfind('{', 0, m.start())
        seg = txt[head:m.start()]
        nums = re.findall(r'(\d+)\s*u', seg)      # 表里每个数字都带 u 后缀
        if len(nums) < 16:
            raise SystemExit('解析失败 %s: %r' % (nm, seg[:90]))
        out[nm] = (int(nums[9]), int(nums[10]), int(nums[15]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', default=DEF_PARAMS)
    ap.add_argument('--table', default=DEF_TABLE)
    ap.add_argument('--outdir', default=os.path.join(ROOT, 'sw'))
    ap.add_argument('--dump', nargs='*', default=[],
                    help='额外把这些层的前若干字打印出来自检')
    args = ap.parse_args()

    d = json.load(open(args.params, encoding='utf-8'))
    byname = {j['name']: j for j in d['jobs']}
    order = parse_layer_order(args.table)
    geom = parse_layers(args.table)

    conv_in_params = [j['name'] for j in d['jobs'] if j['type'] == 'conv']
    table_convs = [n for n in order if geom[n][2] == 0]      # mode==0 即 conv
    if table_convs != conv_in_params:
        print('[FAIL] 层表里的 conv 顺序与 params 不一致, 先对齐再用')
        print('  table :', table_convs)
        print('  params:', conv_in_params)
        return 1

    blob = []            # 逐字 (uint32) 累积, 只收 conv
    recs = []
    # ★ g_scale 的索引必须与 g_net 一一对应 (y3t_exec.c 用 g_scale[layer]),
    #   所以 pool/up 也要占一个位置, 内容填 0 —— 那些层根本不读尺度。
    for nm in order:
        cit, cot, mode = geom[nm]
        n = cot * 16
        if mode != 0:
            recs.append(dict(name=nm, n=0, w_g0=0, w_g1=0, w_bias=0,
                             shM0=0, shM1=0, shY=0, multY=1,
                             oc=0, cot=cot, cit=cit, ngroups=0, mode=mode))
            print(f'  {nm:12s} (mode={mode} 非 conv, 无尺度)')
            continue

        j = byname[nm]
        oc = j['oc']
        groups = j['groups']
        assert len(groups) in (1, 2), '%s groups=%d' % (nm, len(groups))

        # ---- group0 multM ----
        w_g0 = len(blob)
        for ocg in range(n):
            blob.append(int(groups[0]['multM'][ocg]) if ocg < oc else 0)

        # ---- group1 multM (仅多组层) ----
        w_g1 = 0
        if len(groups) == 2:
            w_g1 = len(blob)
            for ocg in range(n):
                blob.append(int(groups[1]['multM'][ocg]) if ocg < oc else 0)

        # ---- bias_q13 ----
        w_b = len(blob)
        for ocg in range(n):
            blob.append(int(j['bias_q13'][ocg]) & 0xFFFFFFFF if ocg < oc else 0)

        shM0 = int(groups[0]['shiftM'])
        shM1 = int(groups[1]['shiftM']) if len(groups) == 2 else shM0
        shY = int(j['shiftY'][0])
        multY = int(j['multY'][0]) & 0xFFFFFFFF

        if len(set(j['multY'])) != 1 or len(set(j['shiftY'])) != 1:
            print('[FAIL] %s 的 multY/shiftY 层内不均匀, 引擎单寄存器装不下' % nm)
            return 1

        recs.append(dict(name=nm, n=n, w_g0=w_g0, w_g1=w_g1, w_bias=w_b,
                         shM0=shM0, shM1=shM1, shY=shY, multY=multY,
                         oc=oc, cot=cot, cit=cit, ngroups=len(groups), mode=mode))
        print(f'  {nm:12s} oc={oc:4d} cot={cot:3d} 段长={n:4d}  '
              f'g0@{w_g0:6d} g1@{w_g1:6d} bias@{w_b:6d}  '
              f'shM={shM0}/{shM1} shY={shY} multY={multY}')

    total_words = len(blob)
    if total_words % 8:
        pad = 8 - total_words % 8
        blob.extend([0] * pad)
        total_words += pad
        print('[info] 末尾补 %d 字对齐到 256b 块' % pad)

    # ---- 写 bin ----
    arr = np.array(blob, dtype=np.uint32)
    binp = os.path.join(args.outdir, 'engine_scales.bin')
    arr.tofile(binp)

    # ---- 写 header ----
    hdrp = os.path.join(args.outdir, 'y3t_scale_table.h')
    with open(hdrp, 'w', encoding='utf-8', newline='\n') as f:
        f.write('/* ==================================================================\n')
        f.write(' * y3t_scale_table.h —— 逐通道 requant 尺度表 (由 tools/export_scales.py 生成, 勿手改)\n')
        f.write(' *   源: y3t_params.json 的 groups[*].multM / bias_q13 / shiftY / multY\n')
        f.write(' *   blob: engine_scales.bin, 上传到 DDR 后把块号传给 rom_blk\n')
        f.write(' *   引擎 ROM 布局: multM group0 -> 0, group1 -> 1024, bias -> 2048\n')
        f.write(' *   注意 identity 若从地址 0 连写 bias 会踩掉 group1 的 multM (见脚本头注释)\n')
        f.write(' * ================================================================== */\n')
        f.write('#ifndef Y3T_SCALE_TABLE_H\n#define Y3T_SCALE_TABLE_H\n\n')
        f.write('#include <stdint.h>\n\n')
        f.write('#define Y3T_SCALE_LAYERS   %du\n' % len(recs))
        f.write('#define Y3T_SCALE_WORDS    %du   /* bin 总字数 (x4 = 字节) */\n' % total_words)
        f.write('#define Y3T_SCALE_BLK_DEF  580000u /* 建议的 DDR 落点 (权重区之后, 空闲) */\n\n')
        f.write('typedef struct {\n')
        f.write('\tuint16_t n;        /* 每段字数 (= cot*16) */\n')
        f.write('\tuint16_t shM0, shM1, shY;\n')
        f.write('\tuint32_t w_g0;     /* group0 multM 在 blob 内的字偏移 */\n')
        f.write('\tuint32_t w_g1;     /* group1 multM 字偏移, 0 = 本层只有 1 组 */\n')
        f.write('\tuint32_t w_bias;   /* bias_q13 字偏移 */\n')
        f.write('\tuint32_t multY;    /* 输出乘子 (S4 级) */\n')
        f.write('} y3t_scale_t;\n\n')
        f.write('static const y3t_scale_t g_scale[Y3T_SCALE_LAYERS] = {\n')
        for r in recs:
            f.write('\t{ %4du, %2du, %2du, %2du, %6du, %6du, %6du, %10du }, /* %s */\n'
                    % (r['n'], r['shM0'], r['shM1'], r['shY'],
                       r['w_g0'], r['w_g1'], r['w_bias'], r['multY'], r['name']))
        f.write('};\n\n#endif /* Y3T_SCALE_TABLE_H */\n')

    # ---- 写 json ----
    jp = os.path.join(args.outdir, 'engine_scales.json')
    with open(jp, 'w', encoding='utf-8', newline='\n') as f:
        json.dump({'total_words': total_words,
                   'total_bytes': total_words * 4,
                   'src_params': args.params,
                   'layers': recs}, f, indent=1)

    print(f'\n{binp}: {total_words} words = {total_words*4} B '
          f'({total_words*4/1024:.1f} KB)')
    print(f'{hdrp}')
    print(f'{jp}')

    # ---- 自检打印 ----
    for nm in args.dump:
        r = [x for x in recs if x['name'] == nm]
        if not r:
            print('[dump] 没有这层: %s' % nm)
            continue
        r = r[0]
        print(f'[dump] {nm}: g0[0:4]={arr[r["w_g0"]:r["w_g0"]+4].tolist()}  '
              f'bias[0:4]={[int(x) for x in arr[r["w_b"]:r["w_b"]+4].astype(np.int32)]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
