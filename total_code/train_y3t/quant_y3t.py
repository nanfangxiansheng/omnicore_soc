# quant_y3t.py — Y3T-F 量化库
#
# 固定点契约（= 引擎 requant_pipe.v 数据通路，M2 移植时的逐位对齐基准）：
#   组累加：y_g   = RHU(acc_g × multM_g, shiftM_g)        （每输入通道组一次）
#   末组：  y13  = Σ_g y_g + bias_q13                      （Q6.13 域，int32 钳位）
#   激活：  v13  = act ? ReLU(y13) : y13
#   输出：  out  = sat_bits( RHU(v13 × multY, shiftY) )    （int16 ±32767，检测头前 int16）
#

from __future__ import annotations

import json
import numpy as np

Q13 = 13
I16_MAX = 32767
I16_MIN = -32767   # 引擎 requant_pipe S4 饱和为 ±32767（非对称），黄金/量化同轨
I8_MAX = 127
I8_MIN = -128
MULT_MAX = (1 << 31) - 1


def rhu(a: int, sh: int) -> int:
    """引擎 RHU：右移前加 1<<(sh-1)；sh=0 直通。算术移位。"""
    if sh == 0:
        return a
    return (a + (1 << (sh - 1))) >> sh   # Python >> 对负数即算术右移


def sat(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else (hi if v > hi else v)


def choose_mult(ratio: float) -> tuple[int, int]:
    """给定目标 ratio = mult/2^shift，取 shift 使 mult 为不超过 2^31-1 的最大整数。"""
    if ratio <= 0:
        return 0, 0
    # mult = ratio * 2^shift <= MULT_MAX → shift_max = floor(log2(MULT_MAX/ratio))
    shift = int(np.floor(np.log2(MULT_MAX / ratio)))
    shift = max(0, min(shift, 63))
    mult = int(round(ratio * (1 << shift)))
    while mult > MULT_MAX and shift > 0:      # 四舍五入进位兜底
        shift -= 1
        mult = int(round(ratio * (1 << shift)))
    return mult, shift


# ---------------------------------------------------------------- BN 折叠
def bn_fold_full(w, g, b_bn, mean, var, eps=1e-5):
    """BN 折叠: s = γ/√(var+eps) 按输出通道; W' = W·s; b' = β − mean·s。
    参数: w(O,I,kh,kw), g/b_bn/mean/var 均按输出通道。返回 (W', b')。"""
    s = g / np.sqrt(var + eps)
    w2 = w * s[:, None, None, None]
    b2 = b_bn - mean * s
    return w2, b2


# ---------------------------------------------------------------- 权重量化
def quant_w_per_oc(w_fold: np.ndarray):
    """per-out-channel 对称 int8。返回 (w_int8, scale[oc])。"""
    oc = w_fold.shape[0]
    amax = np.abs(w_fold).reshape(oc, -1).max(axis=1)
    scale = np.maximum(amax / I8_MAX, 1e-12)
    q = np.clip(np.round(w_fold / scale[:, None, None, None]), I8_MIN, I8_MAX).astype(np.int8)
    return q, scale


def quant_act_scale(amax: float) -> float:
    """激活 int16 对称固定尺度：scale = amax / 32767。"""
    return max(amax, 1e-12) / I16_MAX


def requant_params(s_a: float, s_w: float, s_out: float):
    """整层单组的 (multM, shiftM, multY, shiftY)。
    multM/2^shiftM = s_a·s_w·2^13   （acc_q → Q6.13 实数域，右移主导）
    multY  /2^shiftY = 1/(2^13·s_out) （Q6.13 → 输出量化域）"""
    multM, shiftM = choose_mult((s_a * s_w) * (1 << Q13))
    multY, shiftY = choose_mult(1.0 / ((1 << Q13) * s_out))
    return multM, shiftM, multY, shiftY


def quant_bias_q13(b_fold: float) -> int:
    return int(round(b_fold * (1 << Q13)))


def quant_w_value(w_fold: float, s_w: float) -> int:
    return sat(int(round(w_fold / s_w)), I8_MIN, I8_MAX)


def quant_a_value(a_real: float, s_a: float) -> int:
    return sat(int(round(a_real / s_a)), I16_MIN, I16_MAX)


def save_params(d: dict, path: str):
    with open(path, "w") as f:
        json.dump(d, f, indent=1)
