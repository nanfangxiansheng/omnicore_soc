# qat_y3t.py — Y3T-F QAT 冒烟（CPU，合成数据）
#
# 目标（M1-5，验证训练路径，非正式训练）：
#   1. 权重 per-oc 对称 int8 fake-quant + STE 注入后，forward/backward 正常；
#   2. 合成回归任务上 loss 明确下降（可训练性证据）；
#   3. 训练后 state_dict 直接喂 export_y3t.py --ckpt，G3 bit-exact 门仍 PASS
#      （证明「训练 → 导出 → 黄金对拍」全链与训练态权重兼容）。
#
# 说明：激活 int16（32767 级）量化误差 ~1e-5 相对量级，本冒烟只注入权重 fake-quant；
# 正式训练（colorblocks2 全集，ultralytics trainer + 检测 loss）独立排期。

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))



class QuantSTE(torch.autograd.Function):
    """per-out-channel 对称 int8，直通估计器。"""

    @staticmethod
    def forward(ctx, w):
        s = w.detach().abs().amax(dim=(1, 2, 3), keepdim=True).clamp(min=1e-8) / 127.0
        return (w / s).round().clamp(-128, 127) * s

    @staticmethod
    def backward(ctx, g):
        return g


def inject_fakequant(model):
    """对全部 Conv2d 注册前向钩子，交换权重为 fake-quant 视图。"""
    n = 0

    def hook(mod, inp):
        w = mod.weight
        mod.weight = torch.nn.Parameter(QuantSTE.apply(w), requires_grad=True)

    for m in model.modules():
        if isinstance(m, torch.nn.Conv2d) and m.weight is not None and m.out_channels > 1:
            m.register_forward_pre_hook(hook)
            n += 1
    return n


def synthetic_batch(bs=8, n=3, size=96, seed=0):
    g = torch.Generator().manual_seed(seed)
    x = torch.rand(bs, n, size, size, generator=g)
    gt_boxes = torch.tensor([[0.25, 0.25, 0.5, 0.5],
                             [0.6, 0.4, 0.3, 0.3],
                             [0.4, 0.7, 0.25, 0.2]])          # (N,4) xywh 相对
    gt_cls = torch.tensor([0, 3, 6])
    return x, gt_boxes, gt_cls


def proxy_loss(out, gt_boxes, gt_cls, device):
    """代理监督：one2many boxes 对固定 GT 框做 MSE，scores 对类别做 CE。
    只证明可训练性，非检测损失。"""
    boxes = out["one2many"]["boxes"]          # (bs,4,N) raw
    scores = out["one2many"]["scores"]        # (bs,7,N)
    tgtb = boxes.new_zeros(boxes.shape)
    for k in range(gt_boxes.shape[0]):
        tgtb[:, :, k] = gt_boxes[k].to(device)     # GT 框指派到固定位置列
    l_box = ((boxes - tgtb) ** 2).mean()
    # 每个 GT 框指派给其在 N 维上最响应的位置：这里用最近位置列近似
    tgt = scores.new_zeros(scores.shape)
    for k, c in enumerate(gt_cls):
        tgt[:, c, k] = 1.0
    l_cls = torch.nn.functional.binary_cross_entropy_with_logits(scores, tgt)
    return l_box + l_cls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--save", default=str(HERE / "qat_smoke_ckpt.pt"))
    args = ap.parse_args()

    torch.manual_seed(0)
    from export_y3t import build_model
    model = build_model()
    model.train()
    n_q = inject_fakequant(model)
    print(f"[QAT] fake-quant 注入 {n_q} 个 Conv2d")

    x, gt_boxes, gt_cls = synthetic_batch()
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    first = last = None
    for step in range(args.steps):
        opt.zero_grad()
        out = model(x)
        loss = proxy_loss(out, gt_boxes, gt_cls, x.device)
        loss.backward()
        opt.step()
        v = loss.item()
        if step == 0:
            first = v
        last = v
        if step % 10 == 0 or step == args.steps - 1:
            print(f"[QAT] step {step:3d}  loss {v:.5f}")
    drop = 1 - last / first
    print(f"[QAT] loss {first:.5f} → {last:.5f}（{drop*100:.1f}%↓）")
    torch.save(model.state_dict(), args.save)
    print(f"[QAT] ckpt 已存 {args.save}")
    ok = drop > 0.30
    print(f"[QAT] {'PASS' if ok else 'FAIL'}（要求 ≥30% 下降）")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
