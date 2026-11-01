# patches_y3t.py — Y3T-F 模型补丁（monkey-patch 模式，不 import 即回滚）

import torch.nn as nn

_APPLIED = []


def apply_relu_patch():
    """补丁①：全局激活 SiLU→ReLU（Conv 类属性，影响所有下游模块）。S5。"""
    from ultralytics.nn.modules.conv import Conv

    Conv.default_act = nn.ReLU(inplace=True)
    _APPLIED.append("relu")
    return "Conv.default_act → ReLU(inplace)"


class DetectFPGA(nn.modules.container.Module):
    """占位注释：真正的类定义在文件底部（import ultralytics 后定义基类）。"""

    pass


def apply_detect_fpga_patch():
    """补丁②：tasks.Detect → DetectFPGA，yaml 中的 `Detect` 解析到 FPGA 版。"""
    import ultralytics.nn.tasks as tasks

    tasks.Detect = DetectFPGA
    _APPLIED.append("detect_fpga")
    return "tasks.Detect → DetectFPGA(cv3 普通 3×3, one2one 同步重建)"


def apply_all(verbose=True):
    msgs = [apply_relu_patch(), apply_detect_fpga_patch()]
    if verbose:
        for m in msgs:
            print(f"[patch] {m}")
    return msgs


def applied():
    return list(_APPLIED)


# ---------------- DetectFPGA 真正定义（模块级，可 pickle） ----------------
def _build_detect_fpga():
    from ultralytics.nn.modules.conv import Conv
    from ultralytics.nn.modules.head import Detect

    class _DetectFPGA(Detect):
        """FPGA 检测头：cv3 分支普通卷积（引擎无 depthwise 支持）。
        reg_max=1 直接回归 + one2one 端到端（NMS-free），yaml 顶层键
        reg_max/end2end 传入。one2one_cv3 与 cv3 同规则重建。"""

        def __init__(self, nc=80, reg_max=1, end2end=False, ch=()):
            super().__init__(nc=nc, reg_max=reg_max, end2end=end2end, ch=ch)
            nc1 = self.nc
            c3 = max(ch[0], min(nc1, 100))
            assert c3 % 16 == 0, f"DetectFPGA: c3={c3} 未 16 对齐"
            from ultralytics.nn.modules.conv import Conv as _Conv
            self.cv3 = nn.ModuleList(
                nn.Sequential(_Conv(x, c3, 3), _Conv(c3, c3, 3),
                              nn.Conv2d(c3, nc1, 1))
                for x in ch)
            if getattr(self, "one2one_cv3", None) is not None:
                self.one2one_cv3 = nn.ModuleList(
                    nn.Sequential(_Conv(x, c3, 3), _Conv(c3, c3, 3),
                                  nn.Conv2d(c3, nc1, 1))
                    for x in ch)

    _DetectFPGA.__name__ = "DetectFPGA"
    _DetectFPGA.__qualname__ = "DetectFPGA"
    return _DetectFPGA


DetectFPGA = _build_detect_fpga()
