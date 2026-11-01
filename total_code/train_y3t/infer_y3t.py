# infer_cpu_y3t.py — 用 best.pt 在 CPU 上推理 YOLO 数据集并可视化
#
# 用法：
#   python infer_cpu_y3t.py --weights runs/detect/runs/y3t_custom/weights/best.pt \
#                           --data mydata/data.yaml \
#                           --imgsz 96 \
#                           --conf 0.25 \
#                           --out runs/infer_cpu
#
# 功能：
#   1. 读取 data.yaml，定位 val 图像目录；
#   2. 对每张图做推理，保存带框的可视化结果；
#   3. 输出每张图的检测数量统计和总耗时/FPS。

import argparse
import time
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO


def load_data_yaml(path: Path):
    d = yaml.safe_load(path.read_text())
    root = Path(d.get("path", path.parent))
    val_rel = d.get("val", "images/val")
    val_dir = (root / val_rel).resolve()
    names = d.get("names", [])
    if isinstance(names, dict):
        names = [names[i] for i in sorted(names)]
    return val_dir, names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True, help="data.yaml 路径")
    ap.add_argument("--imgsz", type=int, default=96)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=1.0)#默认使用了one 2 one，这里关闭IOU 过滤
    ap.add_argument("--out", default="runs/infer_cpu")
    ap.add_argument("--max-images", type=int, default=0, help="0=全部")
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    weights = Path(args.weights)
    data_yaml = Path(args.data)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    val_dir, names = load_data_yaml(data_yaml)
    print(f"[info] val 目录: {val_dir}")
    print(f"[info] 类别: {names}")

    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = sorted([p for p in val_dir.rglob("*") if p.suffix.lower() in exts])
    if args.max_images > 0:
        images = images[: args.max_images]
    print(f"[info] 待推理图像: {len(images)} 张")

    model = YOLO(str(weights))
    model.to(args.device)

    total_det = 0
    per_class = {i: 0 for i in range(len(names))}
    t0 = time.time()

    for idx, img_path in enumerate(images, 1):
        results = model.predict(
            source=str(img_path),
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            device=args.device,
            verbose=False,
        )
        r = results[0]

        # 统计
        if r.boxes is not None and len(r.boxes) > 0:
            cls = r.boxes.cls.cpu().numpy().astype(int)
            total_det += len(cls)
            for c in cls:
                per_class[c] = per_class.get(c, 0) + 1

        # 可视化：用 ultralytics 自带 plot，再转 BGR 保存
        plotted = r.plot()  # RGB ndarray
        out_path = out_dir / f"{img_path.stem}_pred.jpg"
        cv2.imwrite(str(out_path), cv2.cvtColor(plotted, cv2.COLOR_RGB2BGR))

        if idx % 20 == 0 or idx == len(images):
            elapsed = time.time() - t0
            print(f"[{idx}/{len(images)}] 已处理，平均 {elapsed / idx * 1000:.1f} ms/张")

    dt = time.time() - t0
    print("\n===== 推理完成 =====")
    print(f"图像总数   : {len(images)}")
    print(f"总检测框数 : {total_det}")
    print(f"总耗时     : {dt:.2f} s")
    print(f"平均延迟   : {dt / max(len(images), 1) * 1000:.2f} ms/张")
    print(f"吞吐       : {len(images) / max(dt, 1e-9):.2f} FPS")
    print("\n各类别检测数量:")
    for i, n in enumerate(names):
        print(f"  {n:12s}: {per_class.get(i, 0)}")
    print(f"\n可视化结果已保存到: {out_dir.resolve()}")


if __name__ == "__main__":
    main()