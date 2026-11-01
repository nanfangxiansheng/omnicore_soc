# demo_hw_sim.py — 输入一张图片 → Y3T 引擎 RTL 仿真 → 打印硬件输出数值 + 解码检测框
#
# 串起 GUIDE-01 §四/§五 全链，一条命令跑完：
#   ① 图片 → letterbox 96²(RGB) → int16 量化输入 input.i16
#   ② best.pt → 校准 → BN 折叠 → per-oc int8 权重 + 层表 y3t_params.json（复用 export_y3t.py）
#   ③ 同一张图过 numpy 整数黄金 → golden/*.i16
#   ④ engine_gen.py → sim/y3t/{mem.hex, stream.hex, expected/, regions.json}
#   ⑤ iverilog 编译 RTL + vvp 跑引擎全部 job → memdump.hex
#   ⑥ engine_gen.py --check → 逐层 PASS/FAIL（G6：RTL vs 黄金 bit-exact）
#   ⑦ 从 memdump.hex 读硬件头输出（one2one 末级 conv）→ 反量化/sigmoid/dist2bbox
#      → 打印检测框与原始 int16 数值 + 画框图 + dets.json
#
# 用法（python≥3.8 + torch/ultralytics 环境，本机为 conda gputorch）：
#   /home/hcy/anaconda3/envs/gputorch/bin/python demo_hw_sim.py \
#       --image /home/hcy/ziguang/ziguang/images/IMG_20260925_143535.jpg
# 可选：--ckpt --nc --conf --data(类别名) --out --with-float --skip-sim

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SIM = HERE.parent.parent / "sim"
SIMY = SIM / "y3t"
PY = sys.executable


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- ① 预处理
def letterbox96(im_bgr, new=96, color=114):
    """ultralytics LetterBox(center=True) 同款：等比缩放 + 居中 pad 114。
    返回 (rgb_u8[96,96,3], r, dw, dh)（r/dw/dh 用于坐标映射回原图）。"""
    h, w = im_bgr.shape[:2]
    r = min(new / h, new / w)
    nw, nh = round(w * r), round(h * r)
    im = cv2.resize(im_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (new - nw) / 2, (new - nh) / 2
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    top, bot = int(round(dh - 0.1)), int(round(dh + 0.1))
    out = np.full((new, new, 3), color, np.uint8)
    out[top:top + nh, left:left + nw] = im
    return cv2.cvtColor(out, cv2.COLOR_BGR2RGB), r, dw, dh


# ---------------------------------------------------------------- ② 导出链
def build_int8(ckpt, nc, outd):
    import export_y3t as ex
    model = ex.build_model(str(ckpt), nc=nc)
    amax = ex.calibrate(model)
    jobs, weights, tmeta, s_in = ex.build_jobs(model, amax)
    woff, blob, cur = {}, bytearray(), 0
    for name, wq in weights:
        b = wq.astype(np.int8).tobytes()
        woff[name] = cur
        blob += b
        cur += len(b)
    outd.mkdir(parents=True, exist_ok=True)
    (outd / "y3t_int8.bin").write_bytes(bytes(blob))
    (outd / "y3t_params.json").write_text(json.dumps(
        {"s_in": s_in, "jobs": jobs,
         "tmeta": {k: list(v) for k, v in tmeta.items()},
         "w_off": woff}, indent=1))
    return jobs, weights, tmeta, s_in


# ---------------------------------------------------------------- ③ 黄金
def run_golden(jobs, weights, tmeta, s_in, frame_u8, outd):
    from export_y3t import golden_run
    outs = golden_run(jobs, weights, tmeta, s_in, frame_u8)
    (outd / "golden").mkdir(exist_ok=True)
    for n, a in outs.items():
        a.astype(np.int16).tofile(outd / "golden" / f"{n}.i16")
    outs["IN"].tofile(outd / "input.i16")
    return outs


# ---------------------------------------------------------------- ④⑤⑥ 引擎
def engine_gen(check=None):
    cmd = [PY, str(HERE / "engine_gen.py"),
           "--params", str(OUTD / "y3t_params.json"),
           "--bin", str(OUTD / "y3t_int8.bin"),
           "--input", str(OUTD / "input.i16"),
           "--golden", str(OUTD / "golden"),
           "--out", str(SIMY)]
    if check:
        cmd += ["--check", check]
    r = subprocess.run(cmd, cwd=str(SIM), capture_output=True, text=True)
    tail = (r.stdout + r.stderr).strip()
    log(tail[-3000:] if check else tail.splitlines()[-1])
    if r.returncode != 0:
        sys.exit("engine_gen 失败" + ("（G6 对拍未全 PASS）" if check else ""))


def compile_sim():
    SIM.mkdir(parents=True, exist_ok=True)
    r = subprocess.run("iverilog -g2005 -o tb_y3t.vvp tb_y3t.v ../rtl/y3t/*.v",
                       shell=True, cwd=str(SIM), capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"iverilog 编译失败:\n{r.stderr[-2000:]}")


def run_sim():
    r = subprocess.run(["../tb_y3t.vvp"], cwd=str(SIMY),
                       capture_output=True, text=True)
    log((r.stdout + r.stderr).strip()[-1500:])
    if r.returncode != 0 or "[FAIL]" in r.stdout or "[FAIL]" in r.stderr:
        sys.exit("RTL 仿真失败（协议错误/超时）")
    if not (SIMY / "memdump.hex").exists():
        sys.exit("RTL 仿真未产出 memdump.hex")


# ---------------------------------------------------------------- ⑦ 解码
def load_memdump(path, upto):
    """memdump.hex 前 upto 个 256b 字 → [upto, 32] uint8（内存字节序）。
    注意：$writememh 每行是 256 位向量值十六进制（MSB 在前），与内存字节序
    相反，须逐字反转；每 16 字还会插一行 `// 0x…` 地址注释，须跳过。"""
    words = np.zeros((upto, 32), np.uint8)
    n = 0
    for line in Path(path).read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("//"):
            continue
        if n >= upto:
            break
        words[n] = np.frombuffer(
            bytes.fromhex(s.lower().replace("x", "0").zfill(64)[-64:]),
            np.uint8)[::-1]
        n += 1
    assert n == upto, f"memdump 行数不足: {n} < {upto}"
    return words


def region_hwc(words, addr, h, w, c):
    """tile-major pad16 区域 → int16 [H,W,C]（HWC）。"""
    tw = ((c + 15) // 16) * h * w
    i16 = np.ascontiguousarray(words[addr:addr + tw]).view("<i2").reshape(tw, 16)
    out = np.zeros((h, w, c), np.int16)
    t, rem = divmod(c, 16)
    for tt in range(t):
        out[:, :, tt * 16:(tt + 1) * 16] = \
            i16[tt * h * w:(tt + 1) * h * w].reshape(h, w, 16)
    if rem:
        out[:, :, t * 16:] = i16[t * h * w:(t + 1) * h * w, :rem].reshape(h, w, rem)
    return out


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60.0, 60.0)))


def decode_hw_out(params, regions, words, nc, conf_thr):
    """从硬件 memdump 解码 one2one 头输出：每个尺度 H_cv2{j}_kK(4ch ltrb) +
    H_cv3{j}_kK(nc 分数) → 反量化 → sigmoid → dist2bbox(xywh, 网络输入 px)。
    返回 (dets, head_names, layer_stats)。"""
    tmeta = {k: tuple(v) for k, v in params["tmeta"].items()}
    byname = {j["name"]: j for j in params["jobs"]}
    lastk = {}
    for j in params["jobs"]:
        mt = re.fullmatch(r"H_(cv[23])(\d+)_k(\d+)", j["name"])
        if mt:
            key = (mt.group(1), int(mt.group(2)))
            lastk[key] = max(lastk.get(key, -1), int(mt.group(3)))

    dets, head_names = [], []
    for j in sorted({jj for (_, jj) in lastk}):
        bjob = byname[f"H_cv2{j}_k{lastk[('cv2', j)]}"]
        sjob = byname[f"H_cv3{j}_k{lastk[('cv3', j)]}"]
        head_names += [bjob["name"], sjob["name"]]
        h, w, _ = tmeta[bjob["src"]]
        stride = 96 // h
        ra, rh, rw, rc = regions[bjob["name"]]
        qb16 = region_hwc(words, ra, rh, rw, rc)[..., :bjob["oc"]]  # 去 pad 通道
        qb = qb16.astype(np.float64) * bjob["s_out"]
        ra, rh, rw, rc = regions[sjob["name"]]
        qs16 = region_hwc(words, ra, rh, rw, rc)[..., :sjob["oc"]]
        qs = qs16.astype(np.float64) * sjob["s_out"]
        conf = sigmoid(qs)
        cls = conf.argmax(axis=2)
        for yy in range(rh):
            for xx in range(rw):
                if conf[yy, xx, cls[yy, xx]] < conf_thr:
                    continue
                lt, rb = qb[yy, xx, :2], qb[yy, xx, 2:]
                ax, ay = xx + 0.5, yy + 0.5
                x1, y1, x2, y2 = ax - lt[0], ay - lt[1], ax + rb[0], ay + rb[1]
                dets.append({
                    "job_scale": j, "stride": stride,
                    "cls": int(cls[yy, xx]),
                    "conf": float(conf[yy, xx, cls[yy, xx]]),
                    "xywh96": [round(float(v), 2) for v in
                               ((x1 + x2) / 2 * stride, (y1 + y2) / 2 * stride,
                                (x2 - x1) * stride, (y2 - y1) * stride)],
                    "raw_score_i16": int(qs16[yy, xx, cls[yy, xx]]),
                    "raw_score_deq": round(float(qs[yy, xx, cls[yy, xx]]), 4),
                    "raw_box_i16": [int(v) for v in qb16[yy, xx]],
                })
    dets.sort(key=lambda d: -d["conf"])

    # 全部已执行 job 的硬件输出统计（整数域 int16 数值本身）
    layer_stats = []
    for name, typ in [(j["name"], j["type"]) for j in
                      json.loads((SIMY / "regions.json").read_text())["jobs"]]:
        ra, rh, rw, rc = regions[name]
        q = region_hwc(words, ra, rh, rw, rc)
        layer_stats.append({"name": name, "type": typ, "shape": [rh, rw, rc],
                            "min": int(q.min()), "max": int(q.max()),
                            "absmax": int(np.abs(q).max())})
    return dets, head_names, layer_stats


def iou_xyxy(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def draw(fname, dets, names, outp, max_w=960):
    im = cv2.imread(str(fname))
    H, W = im.shape[:2]
    th = max(1, W // 480)
    for i, d in enumerate(dets):
        cx, cy, bw, bh = d["xywh_orig"]
        p0 = (int(max(cx - bw / 2, 0)), int(max(cy - bh / 2, 0)))
        p1 = (int(min(cx + bw / 2, W - 1)), int(min(cy + bh / 2, H - 1)))
        cv2.rectangle(im, p0, p1, (0, (200 - 40 * i) % 256, 255), th)
        cv2.putText(im, f"{names[d['cls']]} {d['conf']:.2f}", (p0[0], max(p0[1] - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4 * th, (0, 0, 255), th)
    if W > max_w:
        im = cv2.resize(im, (max_w, round(H * max_w / W)))
    cv2.imwrite(str(outp), im)


def main():
    global OUTD
    ap = argparse.ArgumentParser(
        description="输入图片 → Y3T 引擎 RTL 仿真 → 硬件输出数值/检测框 demo")
    ap.add_argument("--image", required=True)
    ap.add_argument("--ckpt", default="runs/detect/runs/y3t_custom/weights/best.pt")
    ap.add_argument("--nc", type=int, default=5)
    ap.add_argument("--data", default="/home/hcy/ziguang/ziguang/data.yaml",
                    help="data.yaml（取类别名）")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--max-det", type=int, default=100)
    ap.add_argument("--out", default=str(HERE / "demo_hw"))
    ap.add_argument("--with-float", action="store_true", help="附带 float 模型 predict 对照")
    ap.add_argument("--skip-sim", action="store_true",
                    help="复用已有导出工件与 memdump.hex，只做对拍+解码")
    args = ap.parse_args()
    OUTD = Path(args.out) if Path(args.out).is_absolute() else HERE / args.out
    ckpt = Path(args.ckpt)
    ckpt = ckpt if ckpt.is_absolute() else HERE / ckpt
    img_path = Path(args.image)
    assert img_path.exists(), f"找不到图片 {img_path}"
    assert ckpt.exists(), f"找不到 ckpt {ckpt}"

    names = [f"class{i}" for i in range(args.nc)]
    try:
        import yaml
        dd = yaml.safe_load(Path(args.data).read_text())
        nm = dd.get("names")
        if isinstance(nm, dict):
            nm = [nm[i] for i in sorted(nm)]
        if nm and len(nm) == args.nc:
            names = list(nm)
    except Exception as e:
        log(f"[warn] 类别名读取失败({e})，用占位名")

    OUTD.mkdir(parents=True, exist_ok=True)
    (OUTD / "hw_out").mkdir(exist_ok=True)

    # ① 预处理
    im_bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
    frame, r, dw, dh = letterbox96(im_bgr)
    log(f"[1] 输入 {img_path.name} {im_bgr.shape[1]}x{im_bgr.shape[0]}"
        f" → letterbox 96²(r={r:.4f}, pad=114)")

    if not args.skip_sim:
        # ② 导出链
        log(f"[2] 导出链: {ckpt.name} nc={args.nc} → 校准/BN 折叠/int8 …")
        jobs, weights, tmeta, s_in = build_int8(ckpt, args.nc, OUTD)
        n_conv = sum(1 for j in jobs if j["type"] == "conv")
        log(f"    层表 {len(jobs)} jobs（conv {n_conv}），s_in={s_in:.6g}")

        # ③ 黄金（同一张图）
        log("[3] numpy 整数黄金逐层（同一张真实图）…")
        run_golden(jobs, weights, tmeta, s_in, frame, OUTD)

        # ④⑤ RTL 仿真
        log(f"[4] engine_gen → {SIMY}（mem.hex/stream.hex/expected）")
        SIMY.mkdir(parents=True, exist_ok=True)
        engine_gen()
        log("[5] iverilog 编译 RTL + vvp 跑引擎全网络 …")
        compile_sim()
        run_sim()

    # ⑥ G6 对拍
    log("[6] G6 对拍（引擎 memdump vs numpy 黄金，逐层 bit-exact）:")
    engine_gen(check="y3t/memdump.hex")

    # ⑦ 硬件输出解码
    log("[7] 解码硬件输出（memdump.hex 中的 one2one 头末级 conv）:")
    params = json.loads((OUTD / "y3t_params.json").read_text())
    regions = json.loads((SIMY / "regions.json").read_text())["regions"]
    upto = max(v[0] + ((v[3] + 15) // 16) * v[1] * v[2] for v in regions.values())
    words = load_memdump(SIMY / "memdump.hex", upto)
    dets, head_names, layer_stats = decode_hw_out(params, regions, words,
                                                  args.nc, args.conf)
    dets = dets[:args.max_det]
    for d in dets:
        cx, cy, bw, bh = d["xywh96"]
        d["xywh_orig"] = [round((cx - dw) / r, 1), round((cy - dh) / r, 1),
                          round(bw / r, 1), round(bh / r, 1)]

    log(f"\n    已执行 job 硬件输出 int16 统计（min/max/|max|）:")
    for s in layer_stats:
        log(f"    {s['name']:14s} {s['type']:6s} "
            f"{s['shape'][0]}x{s['shape'][1]}x{s['shape'][2]:<3d}"
            f"  min={s['min']:7d}  max={s['max']:7d}  |max|={s['absmax']}")

    log(f"\n    头末级硬件输出（原始 int16）→ 解码检测（conf≥{args.conf}，one2one 免 NMS）:")
    if dets:
        log(f"    {'类别':10s} {'conf':>6s} {'score_i16':>9s} {'score_deq':>9s}"
            f"    box_i16(4)        xywh@96²            xywh@原图")
        for d in dets:
            log(f"    {names[d['cls']]:10s} {d['conf']:6.3f} {d['raw_score_i16']:9d}"
                f" {d['raw_score_deq']:9.3f}  {str(d['raw_box_i16']):17s}"
                f" {str(d['xywh96']):19s} {str(d['xywh_orig'])}")
    else:
        log("    （无 ≥ 阈值的检测）")

    # 产物
    for n in head_names:
        ra, rh, rw, rc = regions[n]
        region_hwc(words, ra, rh, rw, rc).tofile(OUTD / "hw_out" / f"{n}.i16")
    sjobs = [n for n in head_names if n.startswith("H_cv3")]
    for n in sjobs:
        ra, rh, rw, rc = regions[n]
        q = region_hwc(words, ra, rh, rw, rc)
        with open(OUTD / "hw_out" / f"{n}.txt", "w") as f:
            for yy in range(rh):
                f.write(" ".join(",".join(str(int(v)) for v in q[yy, xx])
                                 for xx in range(rw)) + "\n")
    (OUTD / "dets.json").write_text(json.dumps(
        {"image": str(img_path), "ckpt": str(ckpt), "nc": args.nc,
         "conf_thr": args.conf, "names": names, "dets": dets,
         "layer_stats": layer_stats}, indent=1, ensure_ascii=False))
    draw(img_path, dets, names, OUTD / "vis.jpg")
    log(f"\n    产物: {OUTD}/{{dets.json, vis.jpg, hw_out/, golden/, input.i16}}")
    log("[G6] 引擎(RTL)输出与黄金 bit-exact —— 上表数值即硬件输出数值")

    # 附：float 模型对照（同一原图，ultralytics 自带预处理/解码）
    if args.with_float:
        from ultralytics import YOLO
        log("\n[对照] float PyTorch predict（conf≥0.01）:")
        rf = YOLO(str(ckpt)).predict(source=str(img_path), imgsz=96, conf=0.01,
                                     iou=1.0, device="cpu", verbose=False)[0]
        fd = []
        if rf.boxes is not None:
            for xyxy, c, cf in zip(rf.boxes.xyxy.tolist(), rf.boxes.cls.tolist(),
                                   rf.boxes.conf.tolist()):
                fd.append({"cls": int(c), "conf": float(cf), "xyxy": xyxy})
        fd.sort(key=lambda d: -d["conf"])
        for d in fd[:10]:
            best, bi = 0.0, -1
            for i, m in enumerate(dets):
                v = iou_xyxy(d["xyxy"], [m["xywh_orig"][0] - m["xywh_orig"][2] / 2,
                                         m["xywh_orig"][1] - m["xywh_orig"][3] / 2,
                                         m["xywh_orig"][0] + m["xywh_orig"][2] / 2,
                                         m["xywh_orig"][1] + m["xywh_orig"][3] / 2])
                if v > best:
                    best, bi = v, i
            tag = f"↔ sim {names[dets[bi]['cls']]} {dets[bi]['conf']:.3f} IoU={best:.2f}" \
                if bi >= 0 and best > 0.3 else "（sim 无匹配框）"
            log(f"    {names[d['cls']]:10s} {d['conf']:6.3f}  xyxy={[round(v,1) for v in d['xyxy']]}  {tag}")


if __name__ == "__main__":
    main()
