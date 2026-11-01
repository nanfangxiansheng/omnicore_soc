# fw_gen.py — M3 固件工件生成器（CPU 预处理 → Y3T 引擎 → CPU 后处理）
#
# 输入: demo_hw/y3t_params.json + y3t_int8.bin（demo_hw_sim 产出的导出层表/权重）
#       + 一张源图（固件内置的"相机帧"，CPU 预处理输入）
# 输出:
#   sim/board/y3t_mem.hex   引擎 RAM 预载（仅权重区, @addr 稀疏; IN 帧区留零——
#                           帧由固件预处理后经 y3t_mem_ahb 写入）
#   sw/board/y3t_net.h      cfg 写流（非 ROM 对 + ROM 段表/值表, ROM_DATA 自增压缩）
#                           + job 边界 + 头输出解码元数据 + sigmoid 定点表
#   sw/board/y3t_img.h      源图 RGB565（SW_SRC_W×SW_SRC_H）
#
# 用法:
#   /home/hcy/anaconda3/envs/gputorch/bin/python fw_gen.py \
#       --params demo_hw/y3t_params.json --bin demo_hw/y3t_int8.bin \
#       --image /home/hcy/ziguang/ziguang/images/IMG_20260925_143535.jpg

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def cpad(c):
    return ((c + 15) // 16) * 16


def tile_words(h, w, c):
    return cpad(c) // 16 * h * w


def weights_words(wq, oc, ic, kh, kw, cot, cit, taps):
    """与 engine_gen.weights_words 完全一致（引擎交织）。"""
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=str(HERE / "demo_hw" / "y3t_params.json"))
    ap.add_argument("--bin", default=str(HERE / "demo_hw" / "y3t_int8.bin"))
    ap.add_argument("--regions", default=str(HERE.parent.parent / "sim" / "y3t" / "regions.json"))
    ap.add_argument("--image", required=True)
    ap.add_argument("--board-hex", default=str(HERE.parent.parent / "sim" / "board" / "y3t_mem.hex"))
    ap.add_argument("--sw-dir", default=str(HERE.parent.parent / "sw" / "board"))
    args = ap.parse_args()

    d = json.load(open(args.params))
    jobs, tmeta = d["jobs"], d["tmeta"]
    regions = json.load(open(args.regions))["regions"]
    wb_base = json.load(open(args.regions))["wb_base"]
    wbin = np.fromfile(args.bin, dtype=np.uint8)

    byname = {j["name"]: j for j in jobs}
    exec_jobs = [j for j in jobs if j["type"] != "concat"]

    # ---------------- 权重区布局（与 engine_gen 逐位一致的重算） ----------------
    wb_of = {}
    wwords_total = 0
    for j in exec_jobs:
        if j["type"] != "conv":
            continue
        oc, ic, kh = j["oc"], j["ic"], j["kh"]
        cot, cit = (oc + 15) // 16, (ic + 15) // 16
        taps = 1 if kh == 1 else 9
        wb_of[j["name"]] = wwords_total
        wwords_total += cot * cit * taps * 8

    # ---------------- ① y3t_mem.hex（@addr 稀疏, 仅权重区） ----------------
    rows = []
    for j in exec_jobs:
        if j["type"] != "conv":
            continue
        n = j["name"]
        oc, ic, kh, kw = j["oc"], j["ic"], j["kh"], j["kw"]
        cot, cit = (oc + 15) // 16, (ic + 15) // 16
        taps = 1 if kh == 1 else 9
        wq = wbin[d["w_off"][n]:d["w_off"][n] + oc * ic * kh * kw] \
            .reshape(oc, ic, kh * kw)
        ww = weights_words(wq, oc, ic, kh, kw, cot, cit, taps)
        rows.append((wb_base + wb_of[n], ww))
    out = Path(args.board_hex)
    with open(out, "w") as f:
        f.write(f"@{wb_base:08x}\n")
        for base, ww in rows:
            for r in ww:
                f.write(f"{int.from_bytes(r.tobytes(), 'little'):064x}\n")
    n_words = sum(len(ww) for _, ww in rows)
    print(f"[fw_gen] {out}  @0x{wb_base:x} 起 {n_words} 字（权重区）")

    # ---------------- ② cfg 写流 ----------------
    # 每 job: 非 ROM 对(pre) → ROM 段(每段 0F 起始 + 若干 10 自增) → START
    pre_regs, pre_vals, seg_vals = [], [], []
    job_meta = []            # (n_pre 或 pre_off.., segs[(start,cnt)])
    for j in exec_jobs:
        n = j["name"]
        pre = []
        if j["type"] == "conv":
            oc, ic, kh, kw = j["oc"], j["ic"], j["kh"], j["kw"]
            cot, cit = (oc + 15) // 16, (ic + 15) // 16
            src_job = byname.get(j["src"])
            if src_job is None:
                in0 = in1 = regions["IN"][0]
                split = cit
            elif src_job["type"] == "concat":
                s0, s1 = src_job["srcs"]
                in0, in1 = regions[s0][0], regions[s1][0]
                split = cpad(tmeta[s0][2]) // 16
            else:
                in0 = in1 = regions[j["src"]][0]
                split = cit
            g0, g1 = j["groups"][0], \
                j["groups"][1] if len(j["groups"]) > 1 else j["groups"][0]
            sh, sw_ = tmeta[j["src"]][0], tmeta[j["src"]][1]
            pre = [
                (0x01, (j["pad"] << 24) | (j["stride"] << 20) |
                       ((1 if kh == 1 else 0) << 16)),
                (0x02, (sw_ << 16) | sh),
                (0x03, (tmeta[n][1] << 16) | tmeta[n][0]),
                (0x04, (split << 16) | (cot << 8) | cit),
                (0x05, (j["shiftY"][0] << 16) | (g1["shiftM"] << 8) | g0["shiftM"]),
                (0x06, j["multY"][0]),
                (0x07, (cot << 8) | (1 << 1) | (1 if j["relu"] else 0)),
                (0x0B, in0), (0x0C, in1),
                (0x0D, wb_base + wb_of[n]), (0x0E, regions[n][0]),
            ]
            segs = []
            for g, grp in enumerate(j["groups"]):
                segs.append((g * 1024, cot * 16,
                             [grp["multM"][ocg] if ocg < oc else 0
                              for ocg in range(cot * 16)]))
            segs.append((2048, cot * 16,
                         [(j["bias_q13"][ocg] if ocg < oc else 0) & 0xFFFFFFFF
                          for ocg in range(cot * 16)]))
        elif j["type"] == "up":
            sh, sw_, sc = tmeta[j["src"]]
            pre = [(0x01, (1 << 20) | 2), (0x02, (sw_ << 16) | sh),
                   (0x07, (cpad(sc) // 16) << 8),
                   (0x0B, regions[j["src"]][0]), (0x0E, regions[n][0])]
            segs = []
        else:  # pool
            sh, sw_, sc = tmeta[j["src"]]
            pre = [(0x01, 3), (0x02, (sw_ << 16) | sh),
                   (0x03, (tmeta[n][1] << 16) | tmeta[n][0]),
                   (0x07, (cpad(sc) // 16) << 8),
                   (0x0B, regions[j["src"]][0]), (0x0E, regions[n][0])]
            segs = []
        pre_off = len(pre_regs)
        for r, v in pre:
            pre_regs.append(r)
            pre_vals.append(v & 0xFFFFFFFF)
        seg_off = len(seg_vals)
        for start, cnt, vals in segs:
            seg_vals.extend(v & 0xFFFFFFFF for v in vals)
        job_meta.append({
            "name": n, "type": j["type"],
            "pre_off": pre_off, "n_pre": len(pre),
            "seg_off": seg_off,
            "segs": [(s, c) for s, c, _ in segs],
        })

    n_pre = len(pre_regs)
    n_segvals = len(seg_vals)
    print(f"[fw_gen] cfg: 非ROM对 {n_pre}, ROM值 {n_segvals}"
          f" (表 ~{(n_pre*5 + n_segvals*4)/1024:.0f} KB)")

    # ---------------- 头输出解码元数据 ----------------
    def head_job(branch, j):
        mt = [x for x in jobs
              if x["name"].startswith(f"H_{branch}{j}_k") and x["type"] == "conv"]
        return max(mt, key=lambda x: int(x["name"].rsplit("_k", 1)[1]))

    heads = []
    for j in (0, 1):
        for branch in ("cv2", "cv3"):
            jb = head_job(branch, j)
            h, w, _ = tmeta[jb["src"]]
            s_q31 = int(round(jb["s_out"] * (1 << 31)))
            heads.append({"name": jb["name"], "addr": regions[jb["name"]][0],
                          "h": h, "w": w, "oc": jb["oc"], "s_q31": s_q31})

    # sigmoid 定点表: 索引 = Q5.3 logit (Q31 乘积 >>28), 覆盖 ±16 (512 项)
    # → conf 千分数; 阶梯 1/32, 陡段 conf 误差 <8‰
    sig = [int(round(1000.0 / (1.0 + math.exp(-i / 8.0))))
           for i in range(-256, 256)]

    # ---------------- 写 y3t_net.h ----------------
    swd = Path(args.sw_dir)
    lines = []
    lines.append("/* y3t_net.h — 自动生成（tools/y3t/fw_gen.py），勿手改。")
    lines.append(" * cfg 写流 / ROM 段 / 头解码元数据。大表放 .bootrodata（Boot BRAM）。*/")
    lines.append("#ifndef Y3T_NET_H")
    lines.append("#define Y3T_NET_H")
    lines.append("#include <stdint.h>")
    lines.append(f"#define Y3T_N_JOBS {len(job_meta)}")
    lines.append(f"#define Y3T_N_PRE {n_pre}")
    lines.append(f"#define Y3T_N_SEGVAL {n_segvals}")
    lines.append("")
    for hd in heads:
        nm = hd["name"].upper()
        lines.append(f"#define Y3T_H_{nm}_ADDR 0x{hd['addr']:x}u")
        lines.append(f"#define Y3T_H_{nm}_H {hd['h']}")
        lines.append(f"#define Y3T_H_{nm}_W {hd['w']}")
        lines.append(f"#define Y3T_H_{nm}_OC {hd['oc']}")
        lines.append(f"#define Y3T_H_{nm}_SQ31 0x{hd['s_q31'] & 0xFFFFFFFF:08x}u"
                     f"  /* s_out={hd['s_q31'] * 2.0**-31:.6g} */")
    lines.append("")
    lines.append('static const uint8_t y3t_pre_regs[Y3T_N_PRE] __attribute__((section(".bootrodata"))) = {')
    lines.append(",".join(str(r) for r in pre_regs) + "};")
    lines.append('static const uint32_t y3t_pre_vals[Y3T_N_PRE] __attribute__((section(".bootrodata"))) = {')
    for i in range(0, n_pre, 8):
        lines.append(",".join(f"0x{v:08x}u" for v in pre_vals[i:i + 8]) + ",")
    lines.append("};")
    lines.append('static const uint32_t y3t_seg_val[Y3T_N_SEGVAL] __attribute__((section(".bootrodata"))) = {')
    for i in range(0, n_segvals, 8):
        lines.append(",".join(f"0x{v:08x}u" for v in seg_vals[i:i + 8]) + ",")
    lines.append("};")
    # job 表: {pre_off, n_pre, seg_off, n_seg, segs 指针段}
    seg_dir = []       # (start, cnt) 平铺
    for jm in job_meta:
        jm["segdir_off"] = len(seg_dir)
        for s, c in jm["segs"]:
            seg_dir.append((s, c))
        jm["n_seg"] = len(jm["segs"])
    lines.append('static const uint16_t y3t_seg_dir[] __attribute__((section(".bootrodata"))) = {')
    lines.append(",".join(f"0x{s:04x}u,0x{c:04x}u" for s, c in seg_dir) + "};")
    lines.append('typedef struct { uint16_t pre_off, n_pre, seg_off, n_seg, segdir_off; } y3t_job_t;')
    lines.append('static const y3t_job_t y3t_job[Y3T_N_JOBS] __attribute__((section(".bootrodata"))) = {')
    for jm in job_meta:
        lines.append(f"{{{jm['pre_off']},{jm['n_pre']},{jm['seg_off']},"
                     f"{jm['n_seg']},{jm['segdir_off']}}},")
    lines.append("};")
    # sigmoid 表
    lines.append('#define Y3T_SIG_MID 256')
    lines.append('#define Y3T_SIG_LO  -256')
    lines.append('#define Y3T_SIG_HI  255')
    lines.append('static const uint16_t y3t_sig[512] __attribute__((section(".bootrodata"))) = {')
    for i in range(0, 512, 16):
        lines.append(",".join(str(v) for v in sig[i:i + 16]) + ",")
    lines.append("};")
    lines.append("#endif /* Y3T_NET_H */")
    (swd / "y3t_net.h").write_text("\n".join(lines) + "\n")
    print(f"[fw_gen] {swd/'y3t_net.h'}")

    # ---------------- ③ y3t_img.h（源图 RGB565, CPU 预处理输入） ----------------
    import cv2
    SW_W, SW_H = 128, 96
    im = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    assert im is not None, f"读图失败 {args.image}"
    im = cv2.resize(im, (SW_W, SW_H), interpolation=cv2.INTER_AREA)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    px = ((im[:, :, 0].astype(np.uint16) & 0xF8) << 8) | \
         ((im[:, :, 1].astype(np.uint16) & 0xFC) << 3) | \
         (im[:, :, 2].astype(np.uint16) >> 3)
    lines = ["/* y3t_img.h — 自动生成（tools/y3t/fw_gen.py），勿手改。",
             " * 源图 RGB565（模拟相机帧）。CPU 预处理: 最近邻缩放 + letterbox",
             " * pad 114 + 量化, 经 y3t_mem_ahb 写入引擎 IN 区。*/",
             "#ifndef Y3T_IMG_H", "#define Y3T_IMG_H",
             "#include <stdint.h>",
             f"#define SW_SRC_W {SW_W}", f"#define SW_SRC_H {SW_H}",
             'static const uint16_t y3t_src[SW_SRC_W * SW_SRC_H] '
             '__attribute__((section(".bootrodata"))) = {']
    flat = px.reshape(-1)
    for i in range(0, flat.size, 12):
        lines.append(",".join(f"0x{v:04x}u" for v in flat[i:i + 12]) + ",")
    lines.append("};")
    lines.append("#endif /* Y3T_IMG_H */")
    (swd / "y3t_img.h").write_text("\n".join(lines) + "\n")
    print(f"[fw_gen] {swd/'y3t_img.h'}  ({SW_W}x{SW_H} RGB565 = {flat.size*2/1024:.0f} KB)")


if __name__ == "__main__":
    main()
