# macs_audit.py — Y3T-F 逐层审计 + 200K 器件资源对账
# 读 export_y3t/y3t_params.json（export_y3t.py 产物），输出：
#   - 逐层 MACs / 权重字节 / 输出尺寸表（markdown）
#   - DSP/tick 预算（MAC 阵列 lane 数参数化）
#   - BRAM 权重/激活策略结论
# 用法: macs_audit.py [--lanes 64] [--mhz 50]

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).parent

# 器件口径（用户确认）：紫光 200K 级
QUOTA = {"lut_k": 130, "dsp": 800, "bram": "TBD-DEV-01"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lanes", type=int, default=64, help="MAC 阵列乘法器路数")
    ap.add_argument("--mhz", type=float, default=50.0)
    ap.add_argument("--params", default=str(HERE / "export_y3t" / "y3t_params.json"))
    args = ap.parse_args()

    d = json.load(open(args.params))
    jobs, tmeta, woff = d["jobs"], d["tmeta"], d["w_off"]
    rows, tot_macs, tot_w = [], 0, 0
    for j in jobs:
        n = j["name"]
        if j["type"] != "conv":
            h, w, c = tmeta[n]
            src = j.get("src") or (j.get("srcs") or [None])[0]
            si = tmeta[src] if src else (0, 0, 0)
            rows.append((n, j["type"], si, (h, w, c), 0, 0))
            continue
        h, w, _ = tmeta[j["src"]]
        ho = (h + 2 * j["pad"] - j["kh"]) // j["stride"] + 1
        wo = (w + 2 * j["pad"] - j["kw"]) // j["stride"] + 1
        macs = j["oc"] * ho * wo * j["ic"] * j["kh"] * j["kw"]
        wb = j["oc"] * j["ic"] * j["kh"] * j["kw"]
        tot_macs += macs
        tot_w += wb
        rows.append((n, f"conv{k_str(j)}", (h, w, j["ic"]), (ho, wo, j["oc"]), macs, wb))

    print(f"| 层 | 算子 | 输入 HxWxC | 输出 HxWxC | MACs | 权重 B |")
    print(f"| --- | --- | --- | --- | ---: | ---: |")
    for n, t, si, so, m, wb in rows:
        sis = f"{si[0]}x{si[1]}x{si[2]}" if isinstance(si, tuple) else "-"
        sos = f"{so[0]}x{so[1]}x{so[2]}"
        print(f"| {n} | {t} | {sis} | {sos} | {m:,} | {wb:,} |")
    print(f"| **合计** | | | | **{tot_macs:,}** | **{tot_w:,}** |")

    lanes, mhz = args.lanes, args.mhz
    ticks = tot_macs / lanes
    fps = mhz * 1e6 / ticks
    print(f"""
## 预算推算（{lanes} lane MAC 阵列 @ {mhz} MHz）

- 纯计算 tick：{ticks:,.0f} cyc/帧 → 峰值 {fps:.1f} fps（不含权重搬运/读写，M2 tick 实测回填）
- DSP：{lanes} / {QUOTA['dsp']}（{(100*lanes/QUOTA['dsp']):.1f}%）——引擎乘法用；剩余可扩 lane
- LUT：引擎（REARCH 裁剪版）+ boardsoc（~15K，TBD 实测）vs {QUOTA['lut_k']}K
- 权重 int8 总量 {tot_w/1e6:.3f}MB → **超出片上 BRAM 预算概率高（TBD-DEV-01）**，
  走 ROM-DMA 分层直灌（engine_rom_dma 同构）或外挂 SRAM；
  备选：S4 旋钮再降（宽度 ×0.25 / 单尺度头）
- 激活峰值（int16）：L0 {tmeta['L0'][0]}x{tmeta['L0'][1]}x{tmeta['L0'][2]} = "
      f"{tmeta['L0'][0]*tmeta['L0'][1]*tmeta['L0'][2]*2/1024:.0f}KB，双缓冲 "
      f"{tmeta['L0'][0]*tmeta['L0'][1]*tmeta['L0'][2]*4/1024:.0f}KB → 可行
""")


def k_str(j):
    return f"{j['kh']}x{j['kw']}s{j['stride']}"


if __name__ == "__main__":
    main()
