# engine_gen.py — y3t_params.json → Y3T 引擎仿真工件
#
# 产物（sim/y3t/）：
#   mem.hex            mem_model 全镜像（256b 字）：输入帧(tile-major pad16) + 权重(引擎交织)
#   stream.hex         job 指令流：[op a b]（op1=写 cfg 寄存器 idx=a 值=b；op2=等 done；op0=结束）
#   regions.json       张量区域 {name: [word_addr, H, W, Cpad]} + wb_base + job 列表
#   expected/<job>.hex 黄金输出（tile-major pad16, 256b 字）——对拍基准
#   --check memdump.hex：引擎 dump 对拍
#
# 布局契约（DESIGN-M2 §三）：
#   激活/输出 tile-major：word = base + tile*H*W + y*W + x，每字 16ch int16，通道 pad 到 16
#   权重：per (ot, it, tap) 8 个连续 256b 字；第 k 字 = [och 2k,2k+1][ich 0..15][tap]
#   ROM：multm_rom[g*1024+ot*16+ch]、bias_rom[ot*16+ch]（pad 通道写 0 → pad 输出恒 0）

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
WORD = 32


def cpad(c):
    return ((c + 15) // 16) * 16


def tile_words(h, w, c):
    return cpad(c) // 16 * h * w


def hwc_to_tiles(arr, h, w, c):
    """int16 [H,W,C] → [words, 32B]（tile-major pad16，256b 字）。"""
    tiles = cpad(c) // 16
    out = np.zeros((tiles * h * w, 16), np.int16)
    t, rem = divmod(c, 16)
    for tt in range(t):
        out[tt * h * w:(tt + 1) * h * w] = \
            arr[:, :, tt * 16:(tt + 1) * 16].reshape(h * w, 16)
    if rem:
        out[t * h * w:(t + 1) * h * w, :rem] = arr[:, :, t * 16:].reshape(h * w, rem)
    return out.astype(np.int16)


def words_to_hexbytes(words_i16):
    """[words,16] int16 → [words,32] uint8（小端 256b 字）。"""
    b = words_i16.astype("<i2").tobytes()
    n = len(b) // WORD
    return np.frombuffer(b, np.uint8).reshape(n, WORD)


def weights_words(wq, oc, ic, kh, kw, cot, cit, taps):
    """int8 [oc,ic,kh*kw] → 引擎交织 [cot*cit*taps*8, 32] uint8。"""
    wp = np.zeros((cot * 16, cit * 16, taps), np.uint8)
    wp[:oc, :ic, :] = (wq.reshape(oc, ic, taps).astype(np.int16) & 0xFF) \
        .astype(np.uint8)
    out = np.zeros((cot * cit * taps * 8, 32), np.uint8)
    i = 0
    for ot in range(cot):
        for it in range(cit):
            for tap in range(taps):
                for k in range(8):
                    for b in range(32):
                        out[i, b] = wp[ot * 16 + 2 * k + b // 16,
                                       it * 16 + b % 16, tap]
                    i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=str(HERE / "export_y3t" / "y3t_params.json"))
    ap.add_argument("--bin", default=str(HERE / "export_y3t" / "y3t_int8.bin"))
    ap.add_argument("--input", default=str(HERE / "export_y3t" / "input.i16"))
    ap.add_argument("--golden", default=str(HERE / "export_y3t" / "golden"))
    ap.add_argument("--out", default=str(HERE.parent.parent / "sim" / "y3t"))
    ap.add_argument("--check", default=None)
    ap.add_argument("--limit", type=int, default=0, help="只放前 N 个执行 job（0=全部）")
    args = ap.parse_args()

    d = json.load(open(args.params))
    jobs, tmeta = d["jobs"], {k: tuple(v) for k, v in d["tmeta"].items()}
    wbin = np.fromfile(args.bin, dtype=np.uint8)
    frame = np.fromfile(args.input, dtype=np.int16).reshape(96, 96, 3)
    outd = Path(args.out)
    (outd / "expected").mkdir(parents=True, exist_ok=True)

    # ---------------- 区域分配 ----------------
    regions = {}
    cursor = 0

    def alloc(name, h, w, c):
        nonlocal cursor
        regions[name] = (cursor, h, w, cpad(c))
        cursor += tile_words(h, w, c)

    alloc("IN", 96, 96, 3)
    for j in jobs:
        if j["type"] == "concat":
            continue                       # 引擎不执行（两输入区零拷贝）
        alloc(j["name"], *tmeta[j["name"]])

    # ---------------- 权重区 ----------------
    wb_base = (cursor + 7) & ~7
    wa = wb_base
    wb_of = {}
    mem_rows = max(tile_words(96, 96, 3), 1)
    # 先算总字数再建镜像
    wwords_total = 0
    for j in jobs:
        if j["type"] != "conv":
            continue
        oc, ic, kh = j["oc"], j["ic"], j["kh"]
        cot, cit = (oc + 15) // 16, (ic + 15) // 16
        taps = 1 if kh == 1 else 9
        wb_of[j["name"]] = wwords_total
        wwords_total += cot * cit * taps * 8
    total = wb_base + wwords_total
    mem = np.zeros((total, WORD), np.uint8)

    # 输入帧
    ia = regions["IN"][0]
    mem[ia:ia + tile_words(96, 96, 3)] = words_to_hexbytes(
        hwc_to_tiles(frame, 96, 96, 3))

    for j in jobs:
        if j["type"] != "conv":
            continue
        n = j["name"]
        oc, ic, kh, kw = j["oc"], j["ic"], j["kh"], j["kw"]
        cot, cit = (oc + 15) // 16, (ic + 15) // 16
        taps = 1 if kh == 1 else 9
        wq = wbin[d["w_off"][n]:d["w_off"][n] + oc * ic * kh * kw] \
            .reshape(oc, ic, kh * kw)
        ww = weights_words(wq, oc, ic, kh, kw, cot, cit, taps)
        mem[wb_base + wb_of[n]:wb_base + wb_of[n] + len(ww)] = ww

    # ---------------- job 指令流 + 期望输出 ----------------
    def cfg_w(idx, val):
        return (1, idx, int(val) & 0xFFFFFFFF)

    stream = []
    expected = {}
    byname = {j["name"]: j for j in jobs}
    n_exec = 0
    for j in jobs:
        n = j["name"]
        if j["type"] == "concat":
            continue
        if args.limit and n_exec >= args.limit:
            break
        n_exec += 1
        if j["type"] == "conv":
            oc, ic, kh = j["oc"], j["ic"], j["kh"]
            cot, cit = (oc + 15) // 16, (ic + 15) // 16
            src_job = byname.get(j["src"])
            if src_job is None:
                in0 = in1 = regions["IN"][0]
                split = cit
                s0 = s1 = None
            elif src_job["type"] == "concat":
                s0, s1 = src_job["srcs"]
                in0 = regions[s0][0]
                in1 = regions[s1][0]
                split = cpad(tmeta[s0][2]) // 16
            else:
                in0 = regions[j["src"]][0]
                in1 = in0
                split = cit
            if s0 is None:
                pass
            g0 = j["groups"][0]
            g1 = j["groups"][1] if len(j["groups"]) > 1 else g0
            assert len(set(j["multY"])) == 1 and len(set(j["shiftY"])) == 1
            sh, sw_ = tmeta[j["src"]][0], tmeta[j["src"]][1]
            stream += [
                cfg_w(0x01, (j["pad"] << 24) | (j["stride"] << 20) |
                      ((1 if kh == 1 else 0) << 16)),
                cfg_w(0x02, (sw_ << 16) | sh),
                cfg_w(0x03, (tmeta[n][1] << 16) | tmeta[n][0]),
                cfg_w(0x04, (split << 16) | (cot << 8) | cit),
                cfg_w(0x05, (j["shiftY"][0] << 16) | (g1["shiftM"] << 8) | g0["shiftM"]),
                cfg_w(0x06, j["multY"][0]),
                cfg_w(0x07, (cot << 8) | (1 << 1) | (1 if j["relu"] else 0)),
                cfg_w(0x0B, in0), cfg_w(0x0C, in1),
                cfg_w(0x0D, wb_base + wb_of[n]), cfg_w(0x0E, regions[n][0]),
            ]
            for g, grp in enumerate(j["groups"]):
                for ot in range(cot):
                    for ch in range(16):
                        ocg = ot * 16 + ch
                        stream.append(cfg_w(0x0F, g * 1024 + ot * 16 + ch))
                        stream.append(cfg_w(0x10, grp["multM"][ocg] if ocg < oc else 0))
            for ot in range(cot):
                for ch in range(16):
                    ocg = ot * 16 + ch
                    v = j["bias_q13"][ocg] if ocg < oc else 0
                    stream.append(cfg_w(0x0F, 2048 + ot * 16 + ch))
                    stream.append(cfg_w(0x10, v & 0xFFFFFFFF))
        elif j["type"] == "up":
            sh, sw_, sc = tmeta[j["src"]]
            stream += [
                cfg_w(0x01, (1 << 20) | 2),                        # mode=2 UPSAMPLE, s=1
                cfg_w(0x02, (sw_ << 16) | sh),
                cfg_w(0x07, (cpad(sc) // 16) << 8),
                cfg_w(0x0B, regions[j["src"]][0]),
                cfg_w(0x0E, regions[n][0]),
            ]
        elif j["type"] == "pool":
            sh, sw_, sc = tmeta[j["src"]]
            stream += [
                cfg_w(0x01, 3),                                    # mode=3 POOL2
                cfg_w(0x02, (sw_ << 16) | sh),
                cfg_w(0x03, (tmeta[n][1] << 16) | tmeta[n][0]),
                cfg_w(0x07, (cpad(sc) // 16) << 8),
                cfg_w(0x0B, regions[j["src"]][0]),
                cfg_w(0x0E, regions[n][0]),
            ]
        stream.append(cfg_w(0x00, 1))                  # START
        stream.append((2, 0, 0))                       # 等 done
        # 期望输出：黄金 dump（HWC）→ tile-major pad16
        g = np.fromfile(Path(args.golden) / f"{n}.i16", dtype=np.int16) \
            .reshape(tmeta[n])
        ew = words_to_hexbytes(hwc_to_tiles(g, *tmeta[n]))
        (outd / "expected" / f"{n}.hex").write_text(
            "\n".join(f"{int.from_bytes(r.tobytes(), 'little'):064x}" for r in ew) + "\n")
        expected[n] = regions[n]

    stream.append((0, 0, 0))

    # ---------------- 写文件 ----------------
    (outd / "mem.hex").write_text(
        "\n".join(f"{int.from_bytes(r.tobytes(), 'little'):064x}" for r in mem) + "\n")
    (outd / "stream.hex").write_text(
        "\n".join(f"{op:08x} {a:08x} {b:08x}" for op, a, b in stream) + "\n")
    json.dump({"regions": {k: list(v) for k, v in regions.items()},
               "wb_base": wb_base,
               "jobs": [{"name": j["name"], "type": j["type"]} for j in jobs
                        if j["type"] != "concat"]},
              open(outd / "regions.json", "w"), indent=1)
    print(f"[gen] mem {len(mem)} words, stream {len(stream)} ops, "
          f"jobs {len(expected)}, wb_base {wb_base:#x}")

    # ---------------- 对拍 ----------------
    if args.check:
        lines = [l for l in Path(args.check).read_text().splitlines()
                 if l.strip() and not l.strip().startswith("//")]
        dump = np.stack([np.frombuffer(
            bytes.fromhex(l.strip().lower().replace('x', '0').zfill(64)[-64:]),
            np.uint8) for l in lines])
        ok = True
        for name, (a, h, w, c) in expected.items():
            tw = tile_words(h, w, c)
            got = dump[a:a + tw]
            exp = np.stack([np.frombuffer(bytes.fromhex(l.zfill(64)), np.uint8)
                            for l in (outd / "expected" / f"{name}.hex")
                            .read_text().split()])
            if got.shape != exp.shape:
                print(f"  {name:12s} FAIL (shape {got.shape} vs {exp.shape})")
                ok = False
                continue
            same = np.array_equal(got, exp)
            if not same:
                bad = np.argwhere((got != exp).any(axis=1))
                print(f"  {name:12s} FAIL ({len(bad)}/{tw} words, first {bad[0]})")
            else:
                print(f"  {name:12s} PASS ({tw} words)")
            ok &= same
        print(f"[check] {'ALL PASS' if ok else 'FAIL'}")
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
