# train_custom_y3t.py — 自采数据集训练 Y3T-F（YOLOv3-tiny FPGA 版）入口
#
# 用法（详见 doc/npu develop/GUIDE-01-自采数据训练导出指南.md）：
#   python train_custom_y3t.py --data mydata/data.yaml --nc 3 --epochs 200
#
# 前提：data.yaml 为 ultralytics 标准格式，标注为 YOLO txt（class cx cy w h 归一化）。
# 本脚本自动：
#   1. 打 patches_y3t（ReLU + DetectFPGA——引擎适配前提，绝不能省）；
#   2. 以 y3t_fpga.yaml 为模板生成 y3t_custom.yaml（nc 换成你的类别数）；
#   3. 构建 ultralytics 训练任务并训练，产物 best.pt 直接可被 export_y3t.py --nc 导出。

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))


def make_custom_yaml(nc: int, path: Path):
    tpl = (HERE / "y3t_fpga.yaml").read_text()
    s = re.sub(r"^nc: \d+", f"nc: {nc}", tpl, flags=re.M)
    s = s.replace("y3t_fpga.yaml", "y3t_custom.yaml")
    path.write_text(s)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="ultralytics data.yaml 路径")
    ap.add_argument("--nc", type=int, required=True, help="类别数（= data.yaml names 数）")
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--imgsz", type=int, default=96)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--project", default=str(HERE / "runs"))
    ap.add_argument("--name", default="y3t_custom")
    args = ap.parse_args()

    import patches_y3t
    patches_y3t.apply_all(verbose=True)          # ① ReLU + DetectFPGA（必须最先打）

    yaml_path = make_custom_yaml(args.nc, HERE / "y3t_custom.yaml")
    print(f"[train] 模型定义 → {yaml_path} (nc={args.nc})")

    from ultralytics import YOLO
    model = YOLO(str(yaml_path))                 # ② 从结构定义构建（随机初始化）
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        exist_ok=True,
        plots=False,
    )
    best = Path(args.project) / args.name / "weights" / "best.pt"
    print(f"[train] 完成。best 权重：{best}")
    print(f"[train] 下一步导出（对齐 G3/G6 仿真链）：")
    print(f"  python export_y3t.py --ckpt {best} --nc {args.nc}")


if __name__ == "__main__":
    main()
