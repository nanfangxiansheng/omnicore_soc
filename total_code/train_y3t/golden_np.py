# golden_np.py — Y3T-F numpy 整数黄金模型（引擎逐算子语义）
#
# 与 quant_y3t.py 的固定点契约一致（= requant_pipe.v）：
#   conv: acc = Σ int16_act × int8_w (int64，引擎 int48 累加器)
#         y13 = sat32( Σ_g RHU(acc_g×multM_g, shiftM_g) + bias_q13 )
#         v13 = ReLU(y13)（cfg_act=1 时）
#         out = sat16( RHU(v13×multY, shiftY) )
#   RHU：右移前加 1<<(shift-1) 算术右移；shift=0 直通（numpy >> 即算术移位）
#   pool: int16 域 2×2 s2 max；upsample: 2× 最近邻；concat: 通道拼接
#
# 层表由 export_y3t.py 从 ultralytics 模型导出，本文件只做纯计算。

import numpy as np
from quant_y3t import rhu, I16_MIN, I16_MAX

SAT32_LO, SAT32_HI = -(1 << 31), (1 << 31) - 1


def _rhu_vec(a: np.ndarray, sh: int) -> np.ndarray:
    if sh == 0:
        return a
    return (a + (1 << (sh - 1))) >> sh


def conv2d_int16(act, w_q, bias_q13, multM, shiftM, multY, shiftY,
                 stride, pad, act_relu=True):
    """act: int16 [H,W,IC]；w_q: int8 [OC,IC,KH,KW]；mult*/shift*: [OC]。
    返回 int16 [Ho,Wo,OC]。位精确复刻引擎 MAC 阵列 + requant_pipe 数据通路。"""
    H, W, IC = act.shape
    OC, IC2, KH, KW = w_q.shape
    assert IC2 == IC
    if pad:
        a = np.zeros((H + 2 * pad, W + 2 * pad, IC), np.int16)
        a[pad:pad + H, pad:pad + W, :] = act
    else:
        a = act
    Ho = (a.shape[0] - KH) // stride + 1
    Wo = (a.shape[1] - KW) // stride + 1

    cols = np.empty((Ho * Wo, KH * KW * IC), np.int64)
    i = 0
    for y in range(Ho):
        for x in range(Wo):
            win = a[y * stride:y * stride + KH, x * stride:x * stride + KW, :]
            cols[i] = win.reshape(-1).astype(np.int64)
            i += 1
    wm = w_q.reshape(OC, -1).astype(np.int64)
    acc = cols @ wm.T                                        # [n_pos, OC]

    mM = np.atleast_1d(multM).reshape(OC)
    sM = np.atleast_1d(shiftM).reshape(OC)
    mY = np.atleast_1d(multY).reshape(OC)
    sY = np.atleast_1d(shiftY).reshape(OC)

    prod = acc * mM[None, :]
    y13 = np.empty_like(prod)
    for o in range(OC):
        y13[:, o] = _rhu_vec(prod[:, o], int(sM[o]))
    y13 = y13 + np.asarray(bias_q13).reshape(1, OC).astype(np.int64)
    y13 = np.clip(y13, SAT32_LO, SAT32_HI)
    if act_relu:
        y13 = np.maximum(y13, 0)

    out = np.empty_like(y13)
    for o in range(OC):
        out[:, o] = _rhu_vec(y13[:, o] * int(mY[o]), int(sY[o]))
    out = np.clip(out, I16_MIN, I16_MAX).astype(np.int16)
    return out.reshape(Ho, Wo, OC)


def maxpool2x2_s2(act):
    """int16 域 2×2 s2 max（偶尺寸；奇尺寸丢末行/列——M2 引擎 POOL2 落地时核对）。"""
    H, W, C = act.shape
    H2, W2 = H // 2, W // 2
    a = act[:H2 * 2, :W2 * 2, :].astype(np.int32)
    out = np.maximum(np.maximum(a[0::2, 0::2], a[1::2, 0::2]),
                     np.maximum(a[0::2, 1::2], a[1::2, 1::2]))
    return out.astype(np.int16)


def upsample2x(act):
    """int16 域 2× 最近邻（引擎 UPSAMPLE 写侧复制语义）。"""
    return np.repeat(np.repeat(act, 2, axis=0), 2, axis=1)


def _im2col(a_chw, KH, KW, stride, Ho, Wo):
    """a_chw: int64 [IC, Hp, Wp] → cols [Ho*Wo, IC*KH*KW]（通道最外，= 权重 OCHW 展开序）。"""
    cols = np.empty((Ho * Wo, a_chw.shape[0] * KH * KW), np.int64)
    i = 0
    for y in range(Ho):
        for x in range(Wo):
            win = a_chw[:, y * stride:y * stride + KH, x * stride:x * stride + KW]
            cols[i] = win.reshape(-1)
            i += 1
    return cols


def conv_job_np(act, w_q, bias_q13, groups, multY, shiftY, stride, pad, relu):
    """完整 conv job（分组感知，= requant_pipe.v 全通路）。
    act: int16 [H,W,IC] HWC；w_q: int8 [OC,IC,KH,KW]（OCHW）；
    groups: [{ch0,n,multM[OC],shiftM[OC]}]（输入通道组，逐组尺度）。
    y13 = sat32( Σ_g RHU(acc_g·multM_g, shiftM_g) + bias_q13 ) → ReLU →
    out = sat16( RHU(y13·multY, shiftY) )。"""
    H, W, IC = act.shape
    OC, IC2, KH, KW = w_q.shape
    assert IC2 == IC
    if pad:
        a = np.zeros((H + 2 * pad, W + 2 * pad, IC), np.int16)
        a[pad:pad + H, pad:pad + W, :] = act
    else:
        a = act
    Ho = (a.shape[0] - KH) // stride + 1
    Wo = (a.shape[1] - KW) // stride + 1
    a_chw = a.transpose(2, 0, 1).astype(np.int64)      # [IC, Hp, Wp]
    cols = _im2col(a_chw, KH, KW, stride, Ho, Wo)
    my = np.atleast_1d(multY).reshape(OC)
    sy = np.atleast_1d(shiftY).reshape(OC)

    y13 = np.zeros((Ho * Wo, OC), np.int64)
    for g in groups:
        c0, n = g["ch0"], g["n"]
        if n <= 0:
            continue
        sub = cols[:, c0 * KH * KW:(c0 + n) * KH * KW]
        wm = w_q[:, c0:c0 + n, :, :].reshape(OC, -1).astype(np.int64)
        acc = sub @ wm.T                                    # [Npos, OC]
        mM = np.atleast_1d(g["multM"]).reshape(OC)
        shM = g["shiftM"]                                   # 组公共标量（引擎口径）
        for o in range(OC):
            y13[:, o] += _rhu_vec(acc[:, o] * int(mM[o]), int(shM))
    y13 += np.asarray(bias_q13, np.int64).reshape(1, OC)
    y13 = np.clip(y13, SAT32_LO, SAT32_HI)
    if relu:
        y13 = np.maximum(y13, 0)
    out = np.empty_like(y13)
    for o in range(OC):
        out[:, o] = _rhu_vec(y13[:, o] * int(my[o]), int(sy[o]))
    out = np.clip(out, I16_MIN, I16_MAX).astype(np.int16)
    return out.reshape(Ho, Wo, OC)
