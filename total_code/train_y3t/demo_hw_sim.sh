#!/usr/bin/env bash
# demo_hw_sim.sh — 一条命令: 输入图片 → int8 导出 → Y3T 引擎 RTL 仿真 → 硬件输出数值/检测框
# 用法: ./demo_hw_sim.sh [额外参数，如 --with-float / --conf 0.2 / --image 另一张图]
cd "$(dirname "$0")"
exec /home/hcy/anaconda3/envs/gputorch/bin/python demo_hw_sim.py \
    --image /home/hcy/ziguang/ziguang/images/IMG_20260925_143535.jpg \
    --ckpt runs/detect/runs/y3t_custom/weights/best.pt \
    --nc 5 \
    --data /home/hcy/ziguang/ziguang/data.yaml \
    "$@"
