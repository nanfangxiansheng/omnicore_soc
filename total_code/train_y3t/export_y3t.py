# export_y3t.py — Y3T-F 导出链
#
# 链路：patches → 建模型（可选 ckpt）→ 随机帧校准定标 → BN 折叠 → int8/int16 量化
#       → 引擎层表（concat 分组尺度）→ numpy 黄金逐层 → 生成 C 参考网络 → 编译运行
#       → C vs numpy 逐层 bit-exact 门（G3）
#
# 产物（export_y3t/）：
#   y3t_int8.bin      所有 conv 权重 int8 顺序拼接（偏移见层表）
#   y3t_params.json   层表（M2 引擎 job 表与 M3 固件的唯一口径）
#   y3t_net_gen.c     生成的 C 参考网络（读 bin + input.i16，dump 每层 raw）
#   golden/*.i16      numpy 黄金逐层 int16 raw（HWC, LE）
#   input.i16         量化输入帧
#   report.json       门禁结果 + 汇总

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

import patches_y3t  # noqa: E402
from golden_np import conv_job_np, maxpool2x2_s2, upsample2x  # noqa: E402
from quant_y3t import (Q13, bn_fold_full, choose_mult, quant_act_scale,  # noqa: E402
                       quant_w_per_oc, I16_MAX, I16_MIN)
MULT_MAX_ = (1 << 31) - 1

IN_SIZE = 96
CALIB_N = 8
CALIB_SEED = 7
FRAME_SEED = 2026


# ---------------------------------------------------------------- 模型构建
def build_model(ckpt=None, nc=7):
    patches_y3t.apply_all(verbose=False)
    from ultralytics.nn.tasks import DetectionModel

    model = DetectionModel(str(HERE / "y3t_fpga.yaml"), ch=3, nc=nc, verbose=False)
    if ckpt:
        obj = torch.load(ckpt, map_location="cpu", weights_only=False)
        sd = obj.get("model", obj)
        sd = sd.float().state_dict() if hasattr(sd, "state_dict") else sd
        model.load_state_dict(sd, strict=False)
        print(f"[ckpt] 已加载 {ckpt}")
    model.eval()
    return model


# ---------------------------------------------------------------- 校准
@torch.no_grad()
def calibrate(model):
    """随机帧校准：backbone 18 层 + one2one 头各 conv（后激活）输出的 amax。"""
    amax = {}

    def mk(key):
        def hook(m, inp, out):
            t = out[0] if isinstance(out, (list, tuple)) else out
            if isinstance(t, torch.Tensor) and t.is_floating_point():
                amax[key] = max(amax.get(key, 0.0), t.abs().max().item())
        return hook

    from ultralytics.nn.modules.conv import Conv
    hs = []
    for i, layer in enumerate(model.model[:18]):
        hs.append(layer.register_forward_hook(mk(f"L{i}")))
    pat = re.compile(r"model\.18\.one2one_(cv[23])\.(\d+)\.(\d+)$")
    for name, m in model.named_modules():
        mt = pat.fullmatch(name)
        if mt is None:
            continue
        key = f"H_{mt.group(1)}{mt.group(2)}_k{mt.group(3)}"   # = build_jobs 层名
        if isinstance(m, Conv):
            hs.append(m.register_forward_hook(mk(key)))           # conv+bn+act 后
        elif isinstance(m, torch.nn.Conv2d) and not name.endswith(".conv"):
            hs.append(m.register_forward_hook(mk(key)))           # 裸 1×1 输出
    g = torch.Generator().manual_seed(CALIB_SEED)
    model.eval()           # eval 态校准：BN running stats 不被污染，双头 hook 仍全触发
    for _ in range(CALIB_N):
        model(torch.rand(1, 3, IN_SIZE, IN_SIZE, generator=g))
    for h in hs:
        h.remove()
    return amax


# ---------------------------------------------------------------- 层表构建
def conv_core(m):
    """ultralytics Conv → (W', b', has_act)；裸 Conv2d → (W, b, False)。"""
    if hasattr(m, "bn"):
        bn = m.bn
        w2, b2 = bn_fold_full(
            m.conv.weight.detach().numpy().astype(np.float64),
            bn.weight.detach().numpy().astype(np.float64),
            bn.bias.detach().numpy().astype(np.float64),
            bn.running_mean.numpy().astype(np.float64),
            bn.running_var.numpy().astype(np.float64), float(bn.eps))
        return w2, b2, True
    w = m.weight.detach().numpy().astype(np.float64)
    b = m.bias.detach().numpy().astype(np.float64) if m.bias is not None \
        else np.zeros(w.shape[0])
    return w, b, False


def make_conv_job(name, mod, src, in_groups, s_out, relu):
    """in_groups: [(n_ch, s_a)]（concat 双源 → 2 组）。"""
    w2, b2, has_act = conv_core(mod)
    relu = bool(relu and has_act)
    oc, ic, kh, kw = w2.shape
    wq, sw = quant_w_per_oc(w2)
    assert sum(n for n, _ in in_groups) == ic, (name, in_groups, ic)
    cv2d = mod.conv if hasattr(mod, "conv") else mod   # ultralytics Conv 包装 → 内部 Conv2d
    stride = int(cv2d.stride[0])
    pad = int(cv2d.padding[0])                          # autopad: k//2 (s1 conv 为 same)
    groups, ch0 = [], 0
    for n, s_a in in_groups:
        # 引擎 shiftM 为「每 job 每组一个」→ 组内取公共 shift，multM 逐通道：
        #   shM_g = floor(log2((2^31-1)/(s_a·s_w_max·2^13)))，multM[o]=round(s_a·s_w[o]·2^13·2^shM)
        sw_max = float(sw.max())
        base = (s_a * sw_max) * (1 << Q13)
        shM = 0
        if base > 0:
            import math
            shM = int(math.floor(math.log2(MULT_MAX_ / base)))
            shM = max(0, min(shM, 63))
        mm = [int(round((s_a * float(sw[o])) * (1 << Q13) * (1 << shM)))
              for o in range(oc)]
        assert all(0 <= m <= MULT_MAX_ for m in mm), (name, max(mm))
        groups.append({"ch0": ch0, "n": n, "s_a": float(s_a),
                       "shiftM": shM, "multM": mm})
        ch0 += n
    my, sy = [], []
    for o in range(oc):
        a, b_ = choose_mult(1.0 / ((1 << Q13) * s_out))
        my.append(a)
        sy.append(b_)
    return {
        "name": name, "type": "conv", "src": src, "oc": int(oc), "ic": int(ic),
        "kh": int(kh), "kw": int(kw), "stride": stride, "pad": pad,
        "relu": relu, "groups": groups, "s_out": float(s_out),
        "multY": my, "shiftY": sy,
        "bias_q13": [int(round(float(b) * (1 << Q13))) for b in b2],
    }, wq


def build_jobs(model, amax):
    """遍历 model.model[0..17]（执行序）+ one2one 头 → jobs/weights/tmeta/dom。"""
    jobs, weights = [], []
    dom = {}                     # 输出尺度域：scalar 或 [(n, scale)]（concat）
    tmeta = {"IN": (IN_SIZE, IN_SIZE, 3)}
    outname = {-1: "IN"}         # 层 idx → 输出 job 名（-1 = 网络输入）

    def s_of(key):
        a = amax.get(key)
        return quant_act_scale(a if a is not None else 1.0)

    s_in = 1.0 / I16_MAX
    dom["IN"] = s_in
    prev_out = "IN"
    prev_meta = tmeta["IN"]
    n_conv = 0
    for i, layer in enumerate(model.model[:18]):
        ly = type(layer).__name__
        name = f"L{i}"
        f = getattr(layer, "f", -1)
        if ly == "Conv":
            # ultralytics from 语义: 负数 = 相对 (i+x), 非负 = 绝对
            src = outname[i + int(f)] if isinstance(f, int) and f < 0 else \
                (outname[f] if isinstance(f, int) else prev_out)
            s_src = dom[src]
            n_src = tmeta[src][2]
            groups = s_src if isinstance(s_src, list) else [(n_src, s_src)]
            s_out = s_of(name)
            job, wq = make_conv_job(name, layer, src, groups, s_out, relu=True)
            jobs.append(job)
            weights.append((name, wq))
            n_conv += 1
            dom[name] = s_out
            tmeta[name] = (prev_meta[0] // job["stride"],
                           prev_meta[1] // job["stride"], job["oc"])
        elif ly == "MaxPool2d":
            jobs.append({"name": name, "type": "pool", "src": prev_out,
                         "c": prev_meta[2]})
            dom[name], tmeta[name] = dom[prev_out], \
                (prev_meta[0] // 2, prev_meta[1] // 2, prev_meta[2])
        elif ly == "Upsample":
            jobs.append({"name": name, "type": "up", "src": prev_out,
                         "c": prev_meta[2]})
            dom[name], tmeta[name] = dom[prev_out], \
                (prev_meta[0] * 2, prev_meta[1] * 2, prev_meta[2])
        elif ly == "Concat":
            srcs = [outname[(i + int(x)) if int(x) < 0 else int(x)] for x in f]
            dom[name] = [(tmeta[s][2], dom[s]) for s in srcs]
            tmeta[name] = (tmeta[srcs[0]][0], tmeta[srcs[0]][1],
                           sum(tmeta[s][2] for s in srcs))
            jobs.append({"name": name, "type": "concat", "srcs": srcs})
        else:
            raise RuntimeError(f"未处理的层类型 {ly}@{i}")
        outname[i] = name
        prev_out = name
        prev_meta = tmeta[name]

    # one2one 头（部署口径）
    head = model.model[18]
    fidx = list(head.f)
    amap = {}
    for j in range(head.nl):
        src_feat = outname[fidx[j]]
        s_f = dom[src_feat]
        n_f = tmeta[src_feat][2]
        for bname, branch in (("cv2", head.one2one_cv2), ("cv3", head.one2one_cv3)):
            s_cur, n_cur = s_f, n_f
            for k, step in enumerate(branch[j]):
                name = f"H_{bname}{j}_k{k}"
                last = (k == len(branch[j]) - 1)
                s_out = s_of(name)
                job, wq = make_conv_job(name, step, src_feat if k == 0 else
                                        f"H_{bname}{j}_k{k-1}",
                                        [(n_cur, s_cur)], s_out, relu=not last)
                jobs.append(job)
                weights.append((name, wq))
                n_conv += 1
                s_cur, n_cur = s_out, job["oc"]
                tmeta[name] = (tmeta[job["src"]][0], tmeta[job["src"]][1], job["oc"])
    return jobs, weights, tmeta, s_in


# ---------------------------------------------------------------- numpy 黄金
def golden_run(jobs, weights, tmeta, s_in, frame_u8):
    act = {}
    wmap = dict(weights)
    inp = np.clip(np.round((frame_u8.astype(np.float64) / 255.0) / s_in),
                  I16_MIN, I16_MAX).astype(np.int16)
    act["IN"] = inp
    outs = {"IN": inp}
    for job in jobs:
        n = job["name"]
        if job["type"] == "conv":
            a = act[job["src"]].astype(np.int16)
            out = conv_job_np(a, wmap[n], job["bias_q13"], job["groups"],
                              job["multY"], job["shiftY"],
                              job["stride"], job["pad"], job["relu"])
        elif job["type"] == "pool":
            out = maxpool2x2_s2(act[job["src"]])
        elif job["type"] == "up":
            out = upsample2x(act[job["src"]])
        else:
            a, b = act[job["srcs"][0]], act[job["srcs"][1]]
            out = np.concatenate([a, b], axis=2)
        act[n] = out
        outs[n] = out
    return outs


# ---------------------------------------------------------------- C 生成
C_HEAD = r"""/* y3t_net_gen.c — 自动生成（export_y3t.py），勿手改。
 * 黄金参考网络：argv[1]=y3t_int8.bin argv[2]=input.i16 argv[3]=outdir。
 * 逐层 dump int16 raw（HWC, LE）。语义 = quant_y3t.py 契约（= requant_pipe.v）。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static int64_t rhu(int64_t a, int sh) {
    return sh == 0 ? a : (a + ((int64_t)1 << (sh - 1))) >> sh;
}
static int64_t sat64(int64_t v, int64_t lo, int64_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint8_t *g_wbin;

static void dump(const char *dir, const char *name, const int16_t *buf, long n) {
    char p[512];
    snprintf(p, sizeof p, "%s/%s.i16", dir, name);
    FILE *fo = fopen(p, "wb");
    if (!fo || fwrite(buf, 2, n, fo) != (size_t)n) { perror("dump"); exit(2); }
    fclose(fo);
}

static void conv_job(const int16_t *act, int H, int W, int ic,
                     const int8_t *wq, const int32_t *bias_q13,
                     int oc, int kh, int kw, int stride, int pad, int relu,
                     int ngrp, const int *g_ch0, const int *g_n,
                     const int32_t *g_multM, const int32_t *g_shiftM,
                     const int32_t *multY, const int32_t *shiftY,
                     int16_t *out) {
    int Ho = (H + 2 * pad - kh) / stride + 1;
    int Wo = (W + 2 * pad - kw) / stride + 1;
    for (int oy = 0; oy < Ho; oy++)
        for (int ox = 0; ox < Wo; ox++)
            for (int o = 0; o < oc; o++) {
                int64_t y13 = 0;
                for (int g = 0; g < ngrp; g++) {
                    int64_t acc = 0;
                    for (int ky = 0; ky < kh; ky++)
                        for (int kx = 0; kx < kw; kx++) {
                            int iy = oy * stride + ky - pad, ix = ox * stride + kx - pad;
                            if (iy < 0 || iy >= H || ix < 0 || ix >= W) continue;
                            const int16_t *arow = act + ((size_t)iy * W + ix) * ic;
                            const int8_t *wrow = wq +
                                (((size_t)o * ic + g_ch0[g]) * kh + ky) * kw + kx;
                            for (int c = 0; c < g_n[g]; c++)
                                acc += (int64_t)arow[g_ch0[g] + c] *
                                       wrow[(size_t)c * kh * kw];
                        }
                    y13 += rhu(acc * (int64_t)g_multM[(size_t)g * oc + o],
                               g_shiftM[g]);
                }
                y13 = sat64(y13 + bias_q13[o], -(1LL << 31), (1LL << 31) - 1);
                if (relu && y13 < 0) y13 = 0;
                int64_t v = rhu(y13 * (int64_t)multY[o], shiftY[o]);
                out[((size_t)oy * Wo + ox) * oc + o] =
                    (int16_t)sat64(v, -32767, 32767);
            }
}

static void pool_job(const int16_t *act, int H, int W, int c, int16_t *out) {
    int H2 = H / 2, W2 = W / 2;
    for (int y = 0; y < H2; y++)
        for (int x = 0; x < W2; x++)
            for (int k = 0; k < c; k++) {
                int16_t v0 = act[(((size_t)(2 * y)) * W + 2 * x) * c + k];
                int16_t v1 = act[(((size_t)(2 * y + 1)) * W + 2 * x) * c + k];
                int16_t v2 = act[(((size_t)(2 * y)) * W + 2 * x + 1) * c + k];
                int16_t v3 = act[(((size_t)(2 * y + 1)) * W + 2 * x + 1) * c + k];
                int16_t m = v0 > v1 ? v0 : v1;
                m = m > v2 ? m : v2;
                m = m > v3 ? m : v3;
                out[((size_t)y * W2 + x) * c + k] = m;
            }
}

static void up_job(const int16_t *act, int H, int W, int c, int16_t *out) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            for (int k = 0; k < c; k++) {
                int16_t v = act[((size_t)y * W + x) * c + k];
                for (int dy = 0; dy < 2; dy++)
                    for (int dx = 0; dx < 2; dx++)
                        out[((size_t)((2 * y + dy) * (2 * W) + 2 * x + dx)) * c + k] = v;
            }
}

/* 逐位置通道拼接：out[i*(ca+cb) : +ca] = a[i*ca : +ca]，其余 = b。 */
static void concat_job(const int16_t *a, int ca, const int16_t *b, int cb,
                       long npos, int16_t *out) {
    for (long i = 0; i < npos; i++) {
        memcpy(out + i * (ca + cb), a + i * ca, (size_t)ca * 2);
        memcpy(out + i * (ca + cb) + ca, b + i * cb, (size_t)cb * 2);
    }
}

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s wbin input.i16 outdir\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror("wbin"); return 2; }
    fseek(f, 0, SEEK_END); long wlen = ftell(f); fseek(f, 0, SEEK_SET);
    g_wbin = malloc(wlen);
    if (!g_wbin || fread(g_wbin, 1, wlen, f) != (size_t)wlen) { perror("read"); return 2; }
    fclose(f);
    long n_in = /*NIN*/;
    int16_t *A_IN = malloc((size_t)n_in * 2);
    FILE *fi = fopen(argv[2], "rb");
    if (!fi || fread(A_IN, 2, n_in, fi) != (size_t)n_in) { perror("input"); return 2; }
    fclose(fi);
/*BODY*/
    return 0;
}
"""


def gen_c(jobs, weights, woff, tmeta, n_in):
    """生成 C 网络体：常量表 + 缓冲 + 逐层调用 + dump。"""
    lines = []
    for job in jobs:
        n = job["name"]
        shp = tmeta[n]
        cnt = int(np.prod(shp))
        lines.append(f'static int16_t A_{n}[{cnt}];')
    lines.append("")
    for job in jobs:
        if job["type"] != "conv":
            continue
        n = job["name"]
        off = woff[n]
        total = job["oc"] * job["ic"] * job["kh"] * job["kw"]
        lines.append(f'#define WOFF_{n} {off}  /* {total}B */')
        lines.append(f'static const int32_t B_{n}[{job["oc"]}] = {{ '
                     + ",".join(str(b) for b in job["bias_q13"]) + ' };')
        ng = len(job["groups"])
        lines.append(f'static const int G0_{n}[{ng}] = {{ '
                     + ",".join(str(g["ch0"]) for g in job["groups"]) + ' };')
        lines.append(f'static const int GN_{n}[{ng}] = {{ '
                     + ",".join(str(g["n"]) for g in job["groups"]) + ' };')
        mm = [v for g in job["groups"] for v in g["multM"]]
        sm = [g["shiftM"] for g in job["groups"]]
        lines.append(f'static const int32_t MM_{n}[{len(mm)}] = {{ '
                     + ",".join(map(str, mm)) + ' };')
        lines.append(f'static const int32_t SM_{n}[{len(sm)}] = {{ '
                     + ",".join(map(str, sm)) + ' };')
        lines.append(f'static const int32_t MY_{n}[{job["oc"]}] = {{ '
                     + ",".join(map(str, job["multY"])) + ' };')
        lines.append(f'static const int32_t SY_{n}[{job["oc"]}] = {{ '
                     + ",".join(map(str, job["shiftY"])) + ' };')
    lines.append("")
    for job in jobs:
        n = job["name"]
        shp = tmeta[n]
        cnt = int(np.prod(shp))
        if job["type"] == "conv":
            s = job["src"]
            ss = tmeta[s]
            lines.append(
                f'conv_job(A_{s}, {ss[0]}, {ss[1]}, {ss[2]}, '
                f'(const int8_t*)(g_wbin + WOFF_{n}), B_{n}, {job["oc"]}, '
                f'{job["kh"]}, {job["kw"]}, {job["stride"]}, {job["pad"]}, '
                f'{int(job["relu"])}, {len(job["groups"])}, G0_{n}, GN_{n}, '
                f'MM_{n}, SM_{n}, MY_{n}, SY_{n}, A_{n});')
        elif job["type"] == "pool":
            s, ss = job["src"], tmeta[job["src"]]
            lines.append(f'pool_job(A_{s}, {ss[0]}, {ss[1]}, {ss[2]}, A_{n});')
        elif job["type"] == "up":
            s, ss = job["src"], tmeta[job["src"]]
            lines.append(f'up_job(A_{s}, {ss[0]}, {ss[1]}, {ss[2]}, A_{n});')
        else:
            a, b = job["srcs"]
            npos = tmeta[a][0] * tmeta[a][1]
            lines.append(f'concat_job(A_{a}, {tmeta[a][2]}, A_{b}, {tmeta[b][2]}, '
                         f'{npos}, A_{n});')
        lines.append(f'dump(argv[3], "{n}", A_{n}, {cnt});')
    c = C_HEAD.replace("/*NIN*/", str(n_in))
    c = c.replace("/*BODY*/", "\n".join(lines))
    return c


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=None)
    ap.add_argument("--nc", type=int, default=7, help="类别数（须与训练一致）")
    ap.add_argument("--out", default=str(HERE / "export_y3t"))
    args = ap.parse_args()
    out = Path(args.out)
    gold = out / "golden"
    gold.mkdir(parents=True, exist_ok=True)

    model = build_model(args.ckpt, nc=args.nc)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"[S1] 模型就绪: {n_params/1e6:.3f}M params")

    amax = calibrate(model)
    jobs, weights, tmeta, s_in = build_jobs(model, amax)
    n_macs = 0
    for j in jobs:
        if j["type"] == "conv":
            h, w, _ = tmeta[j["src"]]
            ho = (h + 2 * j["pad"] - j["kh"]) // j["stride"] + 1
            wo = (w + 2 * j["pad"] - j["kw"]) // j["stride"] + 1
            n_macs += j["oc"] * ho * wo * j["ic"] * j["kh"] * j["kw"]
    wbytes = sum(w.size for _, w in weights)
    print(f"[S2] 层表: {len(jobs)} jobs ({n_conv_n(jobs)} conv), "
          f"{n_macs/1e6:.2f}M MACs, int8 权重 {wbytes/1e6:.3f}MB")

    # 权重 bin
    woff, blob, cur = {}, bytearray(), 0
    for name, wq in weights:
        b = wq.astype(np.int8).tobytes()
        woff[name] = cur
        blob += b
        cur += len(b)
    (out / "y3t_int8.bin").write_bytes(bytes(blob))

    # 输入帧（随机；正式部署换真实帧重跑同一脚本）
    rng = np.random.default_rng(FRAME_SEED)
    frame = rng.integers(0, 256, (IN_SIZE, IN_SIZE, 3), dtype=np.uint8)
    inp_q = np.clip(np.round((frame.astype(np.float64) / 255.0) / s_in),
                    I16_MIN, I16_MAX).astype(np.int16)
    inp_q.tofile(out / "input.i16")

    # numpy 黄金
    outs = golden_run(jobs, weights, tmeta, s_in, frame)
    for name, arr in outs.items():
        arr.astype(np.int16).tofile(gold / f"{name}.i16")

    # C 生成 + 编译 + 运行
    csrc = gen_c(jobs, weights, woff, tmeta, inp_q.size)
    (out / "y3t_net_gen.c").write_text(csrc)
    cc = shutil.which("gcc") or shutil.which("cc") or shutil.which("g++")
    exe = out / "y3t_net"
    r = subprocess.run([cc, "-O2", "-o", str(exe), str(out / "y3t_net_gen.c")],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-2000:])
        sys.exit("C 编译失败")
    cdir = out / "c_out"
    cdir.mkdir(exist_ok=True)
    r = subprocess.run([str(exe), str(out / "y3t_int8.bin"),
                        str(out / "input.i16"), str(cdir)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-2000:])
        sys.exit("C 运行失败")

    # 门禁 G3: C vs numpy 逐层 bit-exact（IN 为输入本身，天然一致，跳过）
    fails = []
    for name, arr in outs.items():
        if name == "IN":
            continue
        a = arr.astype(np.int16).ravel()
        b = np.fromfile(cdir / f"{name}.i16", dtype=np.int16)
        if a.size != b.size or not np.array_equal(a, b):
            nbad = int((a != b).sum()) if a.size == b.size else -1
            fails.append((name, nbad))
    gate_g3 = not fails
    print(f"[G3] C vs numpy bit-exact: {'PASS' if gate_g3 else 'FAIL'}"
          f" ({len(outs)} 层)")
    for name, nbad in fails:
        print(f"    - {name}: {nbad} 处不一致")

    report = {
        "params": n_params, "macs": n_macs, "wbytes": wbytes,
        "jobs": len(jobs), "conv_jobs": n_conv_n(jobs),
        "s_in": s_in, "gate_G3_bitexact": gate_g3, "mismatches": fails,
    }
    (out / "report.json").write_text(json.dumps(report, indent=1))
    # y3t_params.json = 完整层表（M2/M3 唯一口径）
    (out / "y3t_params.json").write_text(json.dumps(
        {"s_in": s_in, "jobs": jobs, "tmeta": {k: list(v) for k, v in tmeta.items()},
         "w_off": woff}, indent=1))
    sys.exit(0 if gate_g3 else 1)


def n_conv_n(jobs):
    return sum(1 for j in jobs if j["type"] == "conv")


if __name__ == "__main__":
    main()
