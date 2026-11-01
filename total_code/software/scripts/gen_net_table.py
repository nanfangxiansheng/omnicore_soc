#!/usr/bin/env python3
"""gen_net_table.py — 从 y3t_params.json 生成**完整 29 job** 的整网执行表

为什么要重生成 (2026-10-19)
    原来的 `y3t_net_table.h` 只有 23 个 conv, pool/up/concat 全部缺失。
    结果是 L2 直接拿 L0 的 96×96 输出当 48×48 输入读 —— 那是**裁剪不是池化**,
    整网数值必然错。要真正端到端, 必须把非卷积层补齐。

为什么必须重算缓冲区 (不能只"插几行")
    原表的 conv→conv 是**相邻邻接**分配的 (L2.in_base == L0.out_base)，
    插进 pool 之后邻接关系断裂, 所有基址都得重排。

布局 (y3tram 32768 块 × 32B = 1MB)
    [0 .. act_hi)        激活区 —— 带生命周期复用的分配结果
    [wb_lo .. 32768)     权重区 —— 每层用 wload_dma 现搬, wb = 32768 - w_words
                         (单层最大 L11 = 36,864 > 14,336, 由固件拆子 job)

    为什么权重必须在**上方**: 峰值是开头 IN(9216) + L0(9216) = 18,432 块,
    激活区至少要 18,432; 而 y3tram 只有 32,768 ⇒ 只能给权重剩 14,336。
    旧布局 (权重在下、激活在 18432 以上) 只给激活 14,336 —— 装不下 IN+L0。

    concat (L16) 不单独执行: 它被折叠成 L17 的 in0/in1 + split
    (engine_y3t.v 的双输入区零拷贝), 所以可执行 job = 30 - 1 = 29。

用法
    /e/anconda/python.exe tools/gen_net_table.py
    /e/anconda/python.exe tools/gen_net_table.py --report   # 打印分配明细
"""
import argparse
import json
import re
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

DEF_PARAMS = r'E:\ziguang_xj1\omini\tools\y3t\demo_hw\y3t_params.json'
DEF_LAYOUT = os.path.join(ROOT, 'sw', 'engine_layout.json')
DEF_OUT = os.path.join(ROOT, 'sw', 'y3t_net_table.h')

Y3T_BLOCKS = 32768          # y3tram 容量 (256b 块), 见 y3t_mem_ahb #(.WORDS(32768))
TAPS_1X1 = 1
TAPS_3X3 = 9


def cpad16(c):
    return ((c + 15) // 16) * 16


def cot_of(c):
    return cpad16(c) // 16


def alloc_activations(jobs, tmeta, persist=frozenset()):
    """带生命周期复用的首次适配分配。返回 {name: base} 与峰值。

    生命周期: 一个张量从"它被某 job 产出"到"最后一个消费它的 job"。
    concat 不执行 —— 它的两个来源的生命周期要延长到 concat 的消费者 (L17)。

    persist: 这些张量**永不复用**。检测头的末级输出就是 —— 它们没有消费者
    (要等推理结束后由 CPU 从 y3tram 读回来解码), 按普通生命周期会被判成
    "立即死亡"从而被别人覆盖。实测 4 个头的 out 全被分到 0 块, 跑完整网
    只剩最后一个头的数据。"""
    # 每个 job 的消费集合。concat 不执行 —— 消费 concat 的 job 实际读的是
    # concat 的两个来源, 所以这里要**替换**成 srcs, 而不是把 concat 名记进来。
    concat_of = {j['name']: j for j in jobs if j['type'] == 'concat'}
    consume = {}
    for j in jobs:
        n = j['name']
        if j['type'] == 'concat':
            continue
        s = j.get('src')
        if s in concat_of:
            consume[n] = list(concat_of[s]['srcs'])
        elif s:
            consume[n] = [s]
        else:
            consume[n] = []

    exec_jobs = [j for j in jobs if j['type'] != 'concat']
    order = {j['name']: i for i, j in enumerate(exec_jobs)}

    # 每个张量的 (def_index, last_use_index)
    last_use = {j['name']: -1 for j in exec_jobs}
    last_use['IN'] = -1
    for jn, srcs in consume.items():
        i = order[jn]
        for s in srcs:
            if s not in last_use:
                raise SystemExit('未知张量 %s (被 %s 消费)' % (s, jn))
            last_use[s] = max(last_use[s], i)

    live = {}             # name -> (base, size)
    # 初始堆 = 整个 y3tram。真正的"激活区上界"由高水位决定, 这里先不设限,
    # 分配完再用 max(base+size) 反推 —— 否则会陷入"先要知道 act_hi 才能分配"的死循环。
    free = [(0, Y3T_BLOCKS)]   # [(base, size)]
    out = {}
    peak = 0

    def size_of(nm):
        H, W, C = tmeta[nm]
        return cot_of(C) * H * W

    def take(nbytes):
        """首次适配。返回 base, 失败返回 None。"""
        for k, (b, s) in enumerate(free):
            if s >= nbytes:
                free.pop(k)
                if s > nbytes:
                    free.append((b + nbytes, s - nbytes))
                    free.sort()
                return b
        return None

    def give(base, nbytes):
        free.append((base, nbytes))
        # 合并相邻
        free.sort()
        merged = []
        for b, s in free:
            if merged and merged[-1][0] + merged[-1][1] == b:
                merged[-1] = (merged[-1][0], merged[-1][1] + s)
            else:
                merged.append((b, s))
        free[:] = merged

    # IN 先生效
    base = take(size_of('IN'))
    if base is None:
        raise SystemExit('激活区不足: IN 装不下')
    live['IN'] = (base, size_of('IN'))
    out['IN'] = base

    for i, j in enumerate(exec_jobs):
        # 释放"最后一个使用者就是上一个 job 之前"的张量
        for nm in list(live.keys()):
            if nm in persist:
                continue          # 检测头输出: 留着给 CPU 读
            if last_use.get(nm, -1) < i:
                b, s = live.pop(nm)
                give(b, s)
        sz = size_of(j['name'])
        b = take(sz)
        if b is None:
            raise SystemExit('激活区不足: job %s 需要 %d 块, 空闲=%d'
                             % (j['name'], sz, sum(s for _, s in free)))
        live[j['name']] = (b, sz)
        out[j['name']] = b
        # 同时存活的张量必须互不相交 —— 静态不重叠是**错**的判据:
        # 生命周期复用本来就允许两个张量用同一段地址(只要不同时活)。
        sp = sorted((bb, nn, ss) for nn, (bb, ss) in live.items())
        for (b1, n1, s1), (b2, n2, s2) in zip(sp, sp[1:]):
            if b1 + s1 > b2:
                raise SystemExit('[FAIL] 第 %d 步 %s: %s[%d..%d) 与 %s[%d..%d) 同时存活且重叠'
                                 % (i, j['name'], n1, b1, b1 + s1, n2, b2, b2 + s2))
        peak = max(peak, sum(s for _, s in live.values()))

    # 峰值按"同时最大占用"算更准: 用高水位而不是累计
    hi = max((b + s) for b, s in [(out[n], size_of(n)) for n in out])
    return out, hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', default=DEF_PARAMS)
    ap.add_argument('--layout', default=DEF_LAYOUT)
    ap.add_argument('--out', default=DEF_OUT)
    ap.add_argument('--report', action='store_true')
    args = ap.parse_args()

    d = json.load(open(args.params, encoding='utf-8'))
    jobs, tmeta = d['jobs'], d['tmeta']
    byname = {j['name']: j for j in jobs}
    concat_of = {j['name']: j for j in jobs if j['type'] == 'concat'}

    # 权重在 engine_weights.bin 里的块偏移
    lay = json.load(open(args.layout, encoding='utf-8'))['layers']

    exec_jobs = [j for j in jobs if j['type'] != 'concat']

    # 检测头末级: H_cv2{j}_k{n} 是 4ch ltrb 框头, H_cv3{j}_k{n} 是 nc 分数头。
    # 它们的输出要留在 y3tram 里等 CPU 解码, 所以标记为 persistent。
    lastk = {}
    for j in jobs:
        m = re.match(r'H_(cv[23])(\d+)_k(\d+)$', j['name'])
        if m:
            key = (m.group(1), int(m.group(2)))
            lastk[key] = max(lastk.get(key, -1), int(m.group(3)))
    head_names = []
    for jj in sorted({k[1] for k in lastk}):
        head_names.append('H_cv2%d_k%d' % (jj, lastk[('cv2', jj)]))
        head_names.append('H_cv3%d_k%d' % (jj, lastk[('cv3', jj)]))
    persist = frozenset(head_names)
    print('检测头输出 (不复用):', ', '.join(head_names))

    regions, act_hi = alloc_activations(jobs, tmeta, persist)
    wb_top = Y3T_BLOCKS
    wb_space = wb_top - act_hi

    print('激活区高水位 = %d 块 (%.1f KB)' % (act_hi, act_hi * 32 / 1024))
    print('权重区 [%d, %d) = %d 块 (%.1f KB)'
          % (act_hi, wb_top, wb_space, wb_space * 32 / 1024))

    # 重叠校验已在分配过程中逐步完成 (上面 live 的两两比较), 这里只汇总。
    print('[ OK ] 同时存活的张量互不重叠 (逐 job 校验)')

    if args.report:
        print('\n%-12s %-6s %6s %8s %8s %8s %s' %
              ('job', 'type', 'blocks', 'in0', 'in1', 'out', 'split'))
        for j in jobs:
            n = j['name']
            if j['type'] == 'concat':
                print('%-12s %-6s %6s %8s %8s %8s %s' %
                      (n, 'concat', '', '(%s,%s)' % tuple(j['srcs']), '', '', ''))
                continue
            H, W, C = tmeta[n]
            blk = cot_of(C) * H * W
            sp = ''
            if j.get('src') in concat_of:
                cj = concat_of[j['src']]
                i0, i1 = regions[cj['srcs'][0]], regions[cj['srcs'][1]]
                sp = cot_of(tmeta[cj['srcs'][0]][2])
            else:
                i0 = i1 = regions[j['src']] if j.get('src') else regions['IN']
                sp = cot_of(tmeta[j['src']][2]) if j.get('src') else ''
            print('%-12s %-6s %6d %8s %8s %8d %s' %
                  (n, j['type'], blk, i0, i1, regions[n], sp))

    # ---- 写表 ----
    lines = []
    A = lines.append
    A('/* ==================================================================')
    A(' * y3t_net_table.h —— 整网执行表 (由 tools/gen_net_table.py 生成, 勿手改)')
    A(' *   源: y3t_params.json 的 jobs/tmeta + engine_layout.json 的权重块偏移')
    A(' *   含 23 conv + 5 pool + 1 up = 29 个可执行 job;')
    A(' *   concat(L16) 不单独执行, 折叠为 L17 的 in1 + split (双输入区零拷贝)。')
    A(' *   激活区复用: 按生命周期首次适配, 高水位 %d 块。' % act_hi)
    A(' *   权重区 [%d, %d): 每层现搬 wb = %d - w_words。' % (act_hi, wb_top, wb_top))
    A(' * ================================================================== */')
    A('#ifndef Y3T_NET_TABLE_H')
    A('#define Y3T_NET_TABLE_H')
    A('')
    A('#include <stdint.h>')
    A('')
    A('#define NET_WB_TOP   %du' % wb_top)
    A('#define NET_WB_LO    %du' % act_hi)
    A('#define NET_LAYERS   %du' % len(exec_jobs))
    A('')
    A('/* types */')
    A('#define NT_CONV  0u')
    A('#define NT_POOL  3u   /* 引擎 cfg_mode=3: 2x2 maxpool s2 */')
    A('#define NT_UPS   2u   /* 引擎 cfg_mode=2: 2x2 最近邻 */')
    A('')
    A('typedef struct {')
    A('\tuint32_t w_src_blk;  /* DDR 内权重块偏移 (engine_weights.bin 口径), 非 conv 为 0 */')
    A('\tuint32_t w_words;    /* 本层权重 256b 字数 */')
    A('\tuint32_t in0;        /* 输入激活块 (concat 的第一个来源) */')
    A('\tuint32_t in1;        /* 第二个输入激活块; 仅 concat 消费者用, 其余 = in0 */')
    A('\tuint32_t out;        /* 输出激活块 */')
    A('\tuint32_t split;      /* 输入通道分组: g1 起点 (cit 个数); 无 concat 时 = cit */')
    A('\tuint32_t k, s, pad;')
    A('\tuint32_t cit, cot;   /* 通道 tile 数 (1 tile = 16 ch) */')
    A('\tuint32_t H, W, Ho, Wo;')
    A('\tuint32_t mode;      /* 引擎 cfg_mode: 0 conv / 2 upsample / 3 pool2 */')
    A('\tuint32_t act, bits16;')
    A('\tuint32_t ich, och;   /* 真实通道数 (仅打印/校验) */')
    A('\tconst char *name;')
    A('} net_layer_t;')
    A('')
    A('static const net_layer_t g_net[NET_LAYERS] = {')
    for j in exec_jobs:
        n = j['name']
        H, W, C = tmeta[n]
        t = j['type']
        wsb = lay.get(n, {}).get('blk_offset', 0)
        if t == 'conv':
            oc, ic, kh = j['oc'], j['ic'], j['kh']
            cot, cit = cot_of(oc), cot_of(ic)
            taps = TAPS_1X1 if kh == 1 else TAPS_3X3
            ww = cot * cit * taps * 8
            mode = 0
            k = 1 if kh == 1 else 2          # 表口径: 1->1x1, 2->3x3 (engine_y3t.v:131)
            s = j['stride']
            pad = j['pad']
            act = 1 if j['relu'] else 0
            bits16 = 1
        elif t == 'pool':
            oc = ic = C
            cot = cit = cot_of(C)
            ww = 0
            mode = 3
            k = 2
            s = 2
            pad = 0
            act = 0
            bits16 = 1
            kh = 2
        else:  # up
            oc = ic = C
            cot = cit = cot_of(C)
            ww = 0
            mode = 2
            k = 1
            s = 1
            pad = 0
            act = 0
            bits16 = 1
            kh = 1

        srcs = j.get('srcs')
        if j.get('src') in [x['name'] for x in jobs if x['type'] == 'concat']:
            cj = byname[j['src']]
            i0, i1 = regions[cj['srcs'][0]], regions[cj['srcs'][1]]
            sp = cot_of(tmeta[cj['srcs'][0]][2])
        elif j.get('src'):
            i0 = i1 = regions[j['src']]
            sp = cit
        else:
            i0 = i1 = regions['IN']
            sp = cit

        # 【BUG-FIX 2026-10-23】原来 H,W 取的是 tmeta[n] —— 那是**本层输出**尺寸，
        #   却又当输入尺寸去推 Ho,Wo（pool 写 H//2、up 写 H*2）。conv 层因
        #   s=1/pad=1 输入==输出所以一直没错，pool/upsample 输入≠输出就整层错。
        #   板上实测：L1(2x2 maxpool 96->48) 用表内 H=48 只有 41.8% 一致；
        #   把 HW 传成输入尺寸 96x96 后 → 100%。引擎的 HW 必须=输入尺寸。
        #   正确做法：Hi,Wi = 源层输出尺寸；Ho,Wo = 本层输出尺寸(tmeta[n])。
        if j.get('src') in concat_of:
            src_nm = j['src']
        elif j.get('src'):
            src_nm = j['src']
        else:
            src_nm = 'IN'
        Hi, Wi = tmeta[src_nm][0], tmeta[src_nm][1]
        Ho, Wo = H, W
        H, W = Hi, Wi

        A('\t{ %6du, %6du, %6du, %6du, %6du, %4du, %2du, %2du, %2du, %4du, %4du, '
          '%3du, %3du, %3du, %3du, %2du, %2du, %2du, %4du, %4du, "%s" },'
          % (wsb, ww, i0, i1, regions[n], sp, k, s, pad, cit, cot,
             H, W, Ho, Wo, mode, act, bits16, ic, oc, n))
    A('};')
    A('')
    A('#endif /* Y3T_NET_TABLE_H */')

    with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n%s: %d job' % (args.out, len(exec_jobs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
