# smoke_y3t.py — Y3T-F 结构冒烟
#
# S0 补丁应用   S1 模型实例化(y3t_fpga.yaml)   S2 96² 逐层 (res,ch) 核对
# S3 参数量/MACs 统计 → report json               S4 训练态输出(one2many/one2one)核对
# 退出码 0 = 全过。用法: python smoke_y3t.py [--device cpu]

import argparse
import json
import sys
from pathlib import Path

import torch
import torch.nn as nn

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

import patches_y3t  # noqa: E402

EXPECTED_SHAPES = [  # (层索引, 名, H, W, C) —— SPEC-Y3T-01 S3/S4 手算口径
    (0, "conv0", 96, 96, 8), (1, "pool", 48, 48, 8),
    (2, "conv1", 48, 48, 16), (3, "pool", 24, 24, 16),
    (4, "conv2", 24, 24, 32), (5, "pool", 12, 12, 32),
    (6, "conv3", 12, 12, 64), (7, "pool", 6, 6, 64),
    (8, "conv4", 6, 6, 128), (9, "pool", 3, 3, 128),
    (10, "conv5", 3, 3, 256), (11, "conv6", 3, 3, 512),
    (12, "conv7_1x1", 3, 3, 128), (13, "convP5", 3, 3, 256),
    (14, "conv8_1x1", 3, 3, 64), (15, "upsample", 6, 6, 64),
    (16, "concat", 6, 6, 192), (17, "convP4", 6, 6, 128),
]


def count_macs(model, x):
    macs = {"total": 0}
    rows = []

    def hook(m, inp, out):
        if isinstance(m, nn.Conv2d):
            oh, ow = out.shape[-2], out.shape[-1]
            k = m.kernel_size[0] * m.kernel_size[1]
            n = out.shape[1] * oh * ow * (m.in_channels // m.groups) * k
        elif isinstance(m, nn.Linear):
            n = out.shape[1] * m.in_features
        else:
            return
        macs["total"] += n
        rows.append((type(m).__name__, tuple(out.shape), n))

    hs = [m.register_forward_hook(hook) for m in model.modules()]
    model.eval()
    with torch.no_grad():
        model(x)
    for h in hs:
        h.remove()
    return macs["total"], rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()
    fails = []

    # ---- S0 补丁 ----
    msgs = patches_y3t.apply_all()
    assert patches_y3t.applied() == ["relu", "detect_fpga"], patches_y3t.applied()
    print(f"[S0] patches OK: {msgs}")

    # ---- S1 实例化 ----
    from ultralytics.nn.tasks import DetectionModel

    model = DetectionModel(str(HERE / "y3t_fpga.yaml"), ch=3, nc=7, verbose=False)
    n_params = sum(p.numel() for p in model.parameters())
    det = model.model[-1]
    assert type(det).__name__ == "DetectFPGA", type(det).__name__
    assert det.reg_max == 1, det.reg_max
    assert getattr(det, "one2one_cv2", None) is not None, "one2one 分支未构建 (yaml end2end 键未生效)"
    model.end2end = True  # 推理切换 one2one (8.4.x: end2end 为 property/双头训练后切换)
    assert det.end2end, "end2end 切换失败"
    print(f"[S1] instantiate OK: {n_params/1e6:.3f}M params, head=DetectFPGA"
          f" reg_max={det.reg_max} one2one=built e2e_switch=OK nl={det.nl}")

    # 激活核对（S5: ReLU）
    from ultralytics.nn.modules.conv import Conv
    assert isinstance(Conv.default_act, nn.ReLU), type(Conv.default_act)
    for m in model.modules():
        if isinstance(m, Conv):
            assert isinstance(m.act, nn.ReLU), f"非 ReLU 激活: {m.act}"

    # ---- S2 96² 逐层 shape ----
    x = torch.zeros(1, 3, 96, 96)
    shapes = []

    def hook(m, inp, out):
        if isinstance(out, torch.Tensor) and out.dim() == 4:
            shapes.append(tuple(out.shape[1:]))
        elif isinstance(out, (list, tuple)):
            for o in out:
                if isinstance(o, torch.Tensor) and o.dim() == 4:
                    shapes.append(tuple(o.shape[1:]))

    hs = [m.register_forward_hook(hook) for m in model.model[:18]]
    with torch.no_grad():
        model.eval()(x)
    for h in hs:
        h.remove()
    for i, name, eh, ew, ec in EXPECTED_SHAPES:
        got = shapes[i] if i < len(shapes) else None
        ok = got == (ec, eh, ew)
        if not ok:
            fails.append(f"S2 L{i} {name}: want ({ec},{eh},{ew}) got {got}")
    print(f"[S2] layer shapes: {len(shapes)} captured, "
          f"{'OK' if not any(f.startswith('S2') for f in fails) else 'MISMATCH'}")

    # 检测头输出（训练态结构 = 架构契约；eval 解码结构属 M2/M3 后处理范畴）
    model.train()
    out = model(x)
    keys = set(out.keys()) if isinstance(out, dict) else set()
    need = {"one2many", "one2one"}
    if not need.issubset(keys):
        fails.append(f"S2 训练态输出缺 {need - keys}: {keys if isinstance(out, dict) else type(out)}")
    else:
        om = out["one2many"]
        # boxes (bs,4·reg_max=4,N) scores (bs,nc=7,N) feats=头输入特征图
        nb, ns = om["boxes"].shape, om["scores"].shape
        if nb != (1, 4, 45) or ns != (1, 7, 45):   # 45 = 6²+3² 位置总数
            fails.append(f"S2 head one2many: want boxes (1,4,45)/scores (1,7,45) got {nb}/{ns}")
        oo = out["one2one"]
        if oo["boxes"].shape != nb or oo["scores"].shape != ns:
            fails.append(f"S2 head one2one 与 one2many 不同构: {oo['boxes'].shape}/{oo['scores'].shape}")
        print(f"[S2] head one2many boxes{nb} scores{ns}, one2one 同构 (45=6²+3²)")

    # ---- S3 参数/MACs ----
    macs, _ = count_macs(model, x)
    print(f"[S3] {n_params/1e6:.3f}M params, {macs/1e6:.2f}M MACs @96²")
    rep = {"params": n_params, "macs": macs,
           "head": {"type": "DetectFPGA", "reg_max": det.reg_max,
                    "nl": det.nl},
           "layer_shapes": [list(s) for s in shapes]}
    (HERE / "smoke_report.json").write_text(json.dumps(rep, indent=1))

    # ---- 汇总 ----
    if fails:
        print("\n[SMOKE][FAIL]")
        for f in fails:
            print("  -", f)
        sys.exit(1)
    print("\n[SMOKE] S0-S4 全 PASS")


if __name__ == "__main__":
    main()
