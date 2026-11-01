#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
merge_lanes.py — 把 4 个 8 位 lane hex 合并成 1 个 32 位 hex。

背景
    ominicore_bram_slave.v 原来用 4 个 8 位阵列 (lane0..lane3) 拼成 32 位字,
    那是为了逐字节写使能。但 PDS/ADS 对【8 位宽】的行为级 RAM 一律推 LUTRAM
    (DRM36K 最小配置是 4096x9, 宽度不足 9 位要浪费), 导致 TCM 吃掉全片 30% 的 LUT,
    并把 P&R 从 200 s 拖到 2340 s。

    v16 起改为单个 32 位阵列 + 4 个字节使能 —— 宽度够, 工具才愿意推 DRM。
    镜像格式也随之从 4 个 lane 文件改成 1 个 32 位字文件。

字节序
    lane0 = bit[7:0] (最低字节), lane3 = bit[31:24] ——
    与原 RTL 的 {lane3, lane2, lane1, lane0} 拼接顺序一致。

用法
    python tools/merge_lanes.py [ipcore_dir] [out_file] [words]
    python tools/merge_lanes.py
    -> <工程根>/ipcore/tcmram_l0/omnicore_fw32.hex

    2026-10-18: 默认路径改为【按脚本自身位置】定位工程根, 不再依赖 cwd。
    原因: 从 sw/ 里 make hex 调它时, 相对路径 "ipcore" 会解析成 sw/ipcore
    (不存在), 报 "[ERR] missing ipcore/.../omnicore_fw_l0.hex"。
"""
from __future__ import print_function
import io
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJ = os.path.dirname(_HERE)          # tools/ 的上一级 = 工程根


def read_hex(path):
    with io.open(path, "r") as f:
        return [int(x, 16) for x in f.read().split()]


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "ipcore"
    if not os.path.isdir(root):
        # cwd 下没有, 就退回工程根下的 ipcore
        alt = os.path.join(_PROJ, root)
        if os.path.isdir(alt):
            root = alt
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        root, "tcmram_l0", "omnicore_fw32.hex")
    words = int(sys.argv[3]) if len(sys.argv) > 3 else 24576

    lanes = []
    for i in range(4):
        p = os.path.join(root, "tcmram_l%d" % i, "omnicore_fw_l%d.hex" % i)
        if not os.path.isfile(p):
            print("[ERR] missing %s" % p)
            return 1
        lanes.append(read_hex(p))

    longest = max(len(x) for x in lanes)
    if longest < words:
        print("[WARN] lane 只有 %d 行, 少于 words=%d, 其余按 0 补齐"
              % (longest, words))

    nz = 0
    with io.open(out, "w", newline="\r\n") as f:
        for w in range(words):
            v = 0
            for k in range(4):
                if w < len(lanes[k]):
                    v |= (lanes[k][w] & 0xFF) << (8 * k)
            if v:
                nz += 1
            f.write(u"%08x\n" % v)

    print("wrote %s  (%d words, %d non-zero = %.1f%%)"
          % (out, words, nz, 100.0 * nz / max(words, 1)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
