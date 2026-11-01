#!/usr/bin/env python3
"""export_engine_weights.py — G6 INT8 权重 → 引擎交织 256b 字 DDR 镜像

按 engine_gen.py 的 weights_words() 交织规则, 把 y3t_int8.bin 的每层权重
转换成引擎可直接消费的 256b 字序列, 输出:
  engine_weights.bin  — DDR 权重镜像 (上传到板上 DDR @480000 块起)
  engine_layout.json  — 每层在镜像中的块偏移 (供 layer_table 更新为引擎口径)

交织规则 (engine_gen.py 契约):
  每 (ot, it, tap) 占 8 个连续 256b 字; 第 k 字 = [och 2k, 2k+1][ich 0..15][tap]
  ot/cit/taps 遍历, 权重 pad 到 cot×16 / cit×16 通道。
"""
import json
import numpy as np
import sys
import os

WORD = 32


def cpad(c):
    return ((c + 15) // 16) * 16


def weights_words(wq, oc, ic, kh, kw, cot, cit, taps):
    """int8 [oc,ic,kh*kw] → 引擎交织 [cot*cit*taps*8, 32] uint8 (与 engine_gen 相同)。"""
    wp = np.zeros((cot * 16, cit * 16, taps), np.uint8)
    wp[:oc, :ic, :] = (wq.reshape(oc, ic, taps).astype(np.int16) & 0xFF) \
        .astype(np.uint8)
    out = np.zeros((cot * cit * taps * 8, 32), np.uint8)
    i = 0
    for ot in range(cot):
        for it in range(cit):
            for tap in range(taps):
                for k in range(8):
                    for b in range(32):
                        out[i, b] = wp[ot * 16 + 2 * k + b // 16,
                                       it * 16 + b % 16, tap]
                    i += 1
    return out


def main():
    params = sys.argv[1] if len(sys.argv) > 1 else \
        r'E:\ziguang_xj1\omini\tools\y3t\demo_hw\y3t_params.json'
    binp = sys.argv[2] if len(sys.argv) > 2 else \
        r'E:\ziguang_xj1\omini\tools\y3t\demo_hw\y3t_int8.bin'
    outdir = sys.argv[3] if len(sys.argv) > 3 else \
        r'E:\ziguang_xj1\prj_fusion\sw'

    d = json.load(open(params))
    w_off = d['w_off']
    wbin = np.fromfile(binp, dtype=np.uint8)

    layout = {}
    all_words = []
    cursor = 0
    conv_names = [j['name'] for j in d['jobs'] if j['type'] == 'conv']

    print(f"converting {len(conv_names)} layers...")
    for n in conv_names:
        j = [x for x in d['jobs'] if x['name'] == n][0]
        oc, ic, kh, kw = j['oc'], j['ic'], j['kh'], j['kw']
        cot, cit = (oc + 15) // 16, (ic + 15) // 16
        taps = 1 if kh == 1 else 9
        wq = wbin[w_off[n]:w_off[n] + oc * ic * kh * kw].reshape(oc, ic, kh * kw)
        ww = weights_words(wq, oc, ic, kh, kw, cot, cit, taps)
        layout[n] = {
            'blk_offset': cursor,           # 256b 块偏移 (相对镜像基址)
            'words': len(ww),
            'byte_offset_int8': int(w_off[n]),
        }
        all_words.append(ww)
        cursor += len(ww)
        print(f"  {n:12s} {len(ww):6d} words ({len(ww)*32:8d} B)  blk@{cursor - len(ww)}")

    blob = np.concatenate(all_words).astype(np.uint8)
    outbin = os.path.join(outdir, 'engine_weights.bin')
    blob.tofile(outbin)
    with open(os.path.join(outdir, 'engine_layout.json'), 'w') as f:
        json.dump({'total_words': int(cursor),
                   'total_bytes': int(cursor * WORD),
                   'layers': layout}, f, indent=1)

    print(f"\n{outbin}: {cursor} words = {cursor*WORD/1024/1024:.2f} MB")
    print(f"upload: python tools/upload_weights.py --port COM5 "
          f"--bin {outbin} --base-blk 480000")
    print("NOTE: layer_table.h 的 w_src_blk 需改用 engine_layout.json 的 blk_offset")


if __name__ == '__main__':
    main()
