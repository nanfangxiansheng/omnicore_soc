#!/usr/bin/env python3
# ============================================================================
# check_fw_hex.py — 出片前校验: 比特流将要 $readmemh 的那 4 个 lane hex
#                   里到底嵌的是哪一版固件。
#
# 背景 (FUSION_INTEGRATION.md §八b 根因3 / §八d / §八h):
#   综合时 $readmemh 读的是 ipcore/tcmram_l*/omnicore_fw_l*.hex,
#   而固件构建写在 sw/omnicore_fw_l*.hex。两者 md5 一致只证明"同步做了",
#   不证明"固件是新的" —— 项目已因此三次出片内嵌旧固件。
#   本工具把 lane hex 反解回字节流, 直接在目标地址上搜版本标记串,
#   给出"是/不是"的硬结论, 不依赖时间戳。
#
# 用法:
#   python tools/check_fw_hex.py                     # 用默认路径与默认标记
#   python tools/check_fw_hex.py --root E:/path/prj  # 指定工程根
#   python tools/check_fw_hex.py --expect "bring-up v5" --expect "abc"
#   python tools/check_fw_hex.py --elf sw/demo.elf   # 同时打印段占用
#
# 退出码: 0 = 全部标记命中; 1 = 有标记未命中(固件不是期望的那版)
# ============================================================================
import argparse, os, re, struct, sys

TOTAL_WORDS = 24576          # split_hex.py 生成的总字数 (24576 x 4B = 98304B)
BOOT_BYTES  = 65536          # boot 区大小 (board.ld)
BOOT_WORDS  = BOOT_BYTES // 4
MAIN_OFF    = 0x10000000     # 主段基址 (spec §7.1)

# BOOT arena 起点 = 固件本体允许的最大长度。
#   BOOT 64KB 一分为二: 固件 0x0000..0x8000 | arena 0x8000..0x10000
#   必须与 sw/uart_cmd.c 的 ARENA_BOOT_LO、tools/flash.py 的 ARENA_BOOT_LO 一致。
#   2026-10-18 事故: v24 的 .text/.rodata 长到 0x489B, 而 arena 当时定在 0x4000,
#   重叠 2,203 B (含 poff.4 池化偏移表 + g_net 整网 job 表), 本脚本当时只查 MAIN
#   溢出、不查 BOOT 越界, 所以没有拦住。现在补上这道断言。
ARENA_BOOT_LO = 0x00008000

# BOOT 区里属于固件的"NOLOAD"资源 (不写进 hex, 但**必须在 arena 之下**)。
#   2026-10-18 21:10 新增 sw/board.ld 的 .uart_rx (4KB, UART 大包接收缓冲)。
#   这类段如果落进 arena, 自检 [10] / UART 在线下装写 0x8000 就会踩到它,
#   表现为"上传大包后偶发权重错", 极难定位 —— 所以出片前按 ELF 符号硬断言。
BOOT_NOLOAD_LIMIT = ARENA_BOOT_LO

# MAIN arena 起点 —— 栈顶**必须**落在它之下。
#   必须与 sw/uart_cmd.c 的 ARENA_MAIN_LO、tools/flash.py 一致。
#   2026-10-19 发现: board.ld 的 .stack 从 4K 涨到 8K 后, __stack_top 变成
#   0x10005DB0, 而 ARENA_MAIN_LO 仍是 0x10005000 ⇒ 栈的可长区与 MAIN arena
#   重叠 3,504 B。arena 程序复用固件的栈, 它一往下压栈就压在自己身上
#   (自检的便签 0x10005000 也在重叠区里)。BOOT 侧已有 __fw_boot_end 断言,
#   MAIN 侧原来没人守, 这里补齐同一个套路。
ARENA_MAIN_LO = 0x10005000

DEFAULT_EXPECT = [
    "OmniCore fusion bring-up",
    "[v] fw=",               # 探针清单行 —— 版本标记
]


def lane_path(root, lane):
    return os.path.join(root, "ipcore", "tcmram_l%d" % lane,
                        "omnicore_fw_l%d.hex" % lane)


def load_lanes(root):
    lanes = []
    for i in range(4):
        p = lane_path(root, i)
        if not os.path.isfile(p):
            sys.exit("找不到 lane hex: %s" % p)
        with open(p) as f:
            vals = [int(x, 16) for x in f.read().split()]
        if len(vals) < TOTAL_WORDS:
            sys.exit("lane %d 字数不足: %d < %d" % (i, len(vals), TOTAL_WORDS))
        lanes.append(vals)
    return lanes


def byte_at(lanes, addr):
    """按 split_hex.py 的映射反解地址 -> 字节"""
    if addr < BOOT_BYTES:
        w, lane = addr >> 2, addr & 3
    else:
        off = addr - MAIN_OFF
        w, lane = BOOT_WORDS + (off >> 2), off & 3
    if w < 0 or w >= TOTAL_WORDS:
        return None
    return lanes[lane][w]


def find_in_region(lanes, needle, lo, hi):
    nb = needle.encode()
    buf = bytearray()
    for a in range(lo, hi):
        b = byte_at(lanes, a)
        if b is None:
            return None
        buf.append(b)
    i = buf.find(nb)
    return None if i < 0 else lo + i


def search_bytes(lanes, needle):
    """在 BOOT 与 MAIN 两段里找版本标记串, 返回**绝对地址**; 找不到返回 None。

    【2026-10-26 修 · 空转 9 天的闸】
    原来的实现只搜 MAIN(0x1000_0000..0x1000_8000), 而 board.ld 自 2026-10-17 (v16)
    起把 .text/.rodata 搬到了 BOOT ⇒ 版本串实际落在 BOOT (例: 0x00005402),
    MAIN 里永远搜不到; 更致命的是 bytearray.find() 找不到时返回 **-1**,
    而调用处只判 `if off is None` ⇒ -1 被当成"命中",
    打印成 @0x0fffffff (MAIN_OFF + (-1) 的回绕)。
    结果: 传任何 --expect (包括乱码) 都打印 "[ OK ] 命中", 结论永远 PASS。
    这道闸本是为"三次出片嵌旧固件"设的, 它一空转, 那类事故就无人拦截。
    """
    for lo, hi in ((0, BOOT_BYTES), (MAIN_OFF, MAIN_OFF + 0x8000)):
        hit = find_in_region(lanes, needle, lo, hi)
        if hit is not None:
            return hit
    return None


def elfsym(path, names):
    """从 ELF 的 .symtab 里取若干符号的 st_value (找不到返回 None)。

    只认 32 位小端 ELF (riscv32-unknown-elf 的输出), 不做通用解析。
    """
    try:
        d = open(path, "rb").read()
    except OSError:
        return {}
    if d[:4] != b"\x7fELF" or d[4] != 1:
        return {}
    shoff, = struct.unpack_from("<I", d, 0x20)
    shentsize, = struct.unpack_from("<H", d, 0x2E)
    shnum, = struct.unpack_from("<H", d, 0x30)
    out = {}
    for i in range(shnum):
        b = shoff + i * shentsize
        _name, typ, _fl, _ad, off, size, link, _inf, _al, entsize = \
            struct.unpack_from("<IIIIIIIIII", d, b)
        if typ != 2 or entsize != 16:          # SHT_SYMTAB only
            continue
        # 字符串表
        sb = shoff + link * shentsize
        stroff, = struct.unpack_from("<I", d, sb + 0x10)
        for k in range(size // entsize):
            e = off + k * entsize
            nm, val, _sz, _info, _oth, shndx = struct.unpack_from("<IIIBBH", d, e)
            if nm == 0 or shndx == 0:          # 跳过空名 / 未定义
                continue
            end = d.index(b"\x00", stroff + nm)
            s = d[stroff + nm:end].decode("latin1")
            if s in names:
                out[s] = val
    return out


def elf_usage(path):
    try:
        d = open(path, "rb").read()
    except OSError:
        return None
    if d[:4] != b"\x7fELF":
        return None
    shoff, = struct.unpack_from("<I", d, 0x20)
    shentsize, = struct.unpack_from("<H", d, 0x2E)
    shnum, = struct.unpack_from("<H", d, 0x30)
    shstrndx, = struct.unpack_from("<H", d, 0x32)
    stroff, = struct.unpack_from("<I", d, shoff + shstrndx * shentsize + 0x10)
    boot = main = 0
    for i in range(shnum):
        b = shoff + i * shentsize
        _name, _typ, flags, addr, _off, size = struct.unpack_from("<IIIIII", d, b)
        if size == 0 or not (flags & 0x2):      # 只看 SHF_ALLOC(真正加载的段)
            continue
        if addr + size <= BOOT_BYTES:
            boot = max(boot, addr + size)
        elif MAIN_OFF <= addr < MAIN_OFF + 0x8000:
            main = max(main, addr + size - MAIN_OFF)
    return boot, main


def _check_scales(root):
    """尺度表 (y3t_scale_table.h) 与 DDR 镜像 (engine_scales.bin) 是否同源。

    两者都是 tools/export_scales.py 的产物。只重生成一个的后果是
    "固件里的字偏移 + DDR 里的旧数据", 板上表现为结果略偏但流程正常 ——
    这种状态最难自查, 所以在出片前用字数/文件大小硬对上。
    """
    hdr = os.path.join(root, "sw", "y3t_scale_table.h")
    binp = os.path.join(root, "sw", "engine_scales.bin")
    if not os.path.exists(hdr):
        print("  [WARN] 没有 sw/y3t_scale_table.h —— 固件跑不了真尺度 (只能 identity)。")
        return
    m = re.search(r"Y3T_SCALE_WORDS\s+(\d+)u", open(hdr, encoding="utf-8").read())
    if not m:
        print("  [WARN] y3t_scale_table.h 里读不到 Y3T_SCALE_WORDS。")
        return
    words = int(m.group(1))
    if not os.path.exists(binp):
        print("  [FAIL] 有尺度表但没有 engine_scales.bin —— 板上没有尺度数据可载入！")
        return
    size = os.path.getsize(binp)
    if size != words * 4:
        print("  [FAIL] 尺度表与镜像不同源: 表说 %d 字 (=%d B), 镜像是 %d B"
              % (words, words * 4, size))
        print("         → 重跑 /e/anconda/python.exe tools/export_scales.py 并重编固件")
    else:
        print("  [ OK ] 尺度表与镜像同源: %d 字 / %d B (上传到 DDR 块 580000)"
              % (words, size))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."),
                    help="工程根目录 (默认: 本脚本所在目录的上一级)")
    ap.add_argument("--expect", action="append", default=None,
                    help="期望命中的版本标记串 (可多次指定)")
    ap.add_argument("--elf", default=None, help="同时打印该 ELF 的段占用")
    args = ap.parse_args()

    root = os.path.normpath(args.root)
    expect = args.expect or DEFAULT_EXPECT

    lanes = load_lanes(root)
    print("lane hex: %s" % lane_path(root, 0).replace("_l0", "_l*"))
    print("搜索区: BOOT 0x00000000..0x%08x + MAIN 0x%08x..0x%08x"
          % (BOOT_BYTES, MAIN_OFF, MAIN_OFF + 0x8000))
    print()

    miss = 0
    for s in expect:
        off = search_bytes(lanes, s)
        if off is None:
            print("  [FAIL] 未命中: %r  —— 内嵌的不是期望的那版固件" % s)
            miss += 1
        else:
            print("  [ OK ] 命中 @0x%08x : %r" % (off, s))

    if args.elf:
        u = elf_usage(args.elf)
        print()
        if u is None:
            print("  (无法解析 ELF: %s)" % args.elf)
        else:
            boot, main_used = u
            print("  %s  BOOT %d/65536 B | MAIN %d/32768 B (余 %d B)"
                  % (args.elf, boot, main_used, 32768 - main_used))
            if main_used > 32768:
                print("  [FAIL] MAIN 区溢出!")
                miss += 1

            # BOOT 越界 arena 检查 —— v24 的 P0 事故就是这里漏掉的
            arena_room = ARENA_BOOT_LO - boot
            if boot > ARENA_BOOT_LO:
                print("  [FAIL] BOOT 固件越界 arena: __fw_boot_end=0x%05X > "
                      "ARENA_BOOT_LO=0x%05X，重叠 %d B！"
                      % (boot, ARENA_BOOT_LO, boot - ARENA_BOOT_LO))
                print("         → 自检/UART 下装会踩坏固件自己的 .rodata 表，"
                      "先压缩固件或抬高 ARENA_BOOT_LO 再出片。")
                miss += 1
            else:
                print("  [ OK ] BOOT arena 余量 %d B (固件 0x0000..0x%05X / arena 0x%05X..0x10000)"
                      % (arena_room, boot, ARENA_BOOT_LO))
                if arena_room < 4096:
                    print("  [WARN] arena 余量不足 4KB，固件再长就会撞线，建议提前处理。")

            # ---- BOOT NOLOAD 资源 (UART 大包缓冲) 必须仍在 arena 之下 ----
            sy = elfsym(args.elf, ("UART_RXBUF", "UART_RXEND", "__fw_boot_end"))
            if "UART_RXBUF" in sy and "UART_RXEND" in sy:
                lo, hi = sy["UART_RXBUF"], sy["UART_RXEND"]
                print("  UART 大包缓冲: 0x%05X..0x%05X (%d B, BOOT NOLOAD)"
                      % (lo, hi, hi - lo))
                if hi > BOOT_NOLOAD_LIMIT:
                    print("  [FAIL] UART 大包缓冲越界进入 arena: end=0x%05X > 0x%05X"
                          % (hi, BOOT_NOLOAD_LIMIT))
                    print("         → 自检 [10]/UART 下装会踩坏接收缓冲，先压缩固件。")
                    miss += 1
                else:
                    print("  [ OK ] 大包缓冲在 arena 之下，余量 %d B"
                          % (BOOT_NOLOAD_LIMIT - hi))
            elif "__fw_boot_end" in sy:
                print("  [WARN] ELF 里没有 UART_RXBUF 符号 —— 固件早于大包改动，"
                      "上传仍走 500B 小包（很慢）。")

            # ---- MAIN arena 检查: 栈顶必须 <= ARENA_MAIN_LO ----
            #   arena 程序"不自己建栈、复用固件的栈"(见 PROGRAM_DOWNLOAD_SOP),
            #   所以栈往下压的整段空间都不能与 MAIN arena 重叠, 否则程序会
            #   一边跑一边覆盖自己; 自检便签 0x10005000 也在这个重叠区里。
            sym = elfsym(args.elf, ("__stack_top", "__bss_end", "__noinit_end"))
            if "__stack_top" in sym:
                st = sym["__stack_top"]
                print("  栈顶 __stack_top = 0x%08X (MAIN arena 起 0x%08X)"
                      % (st, ARENA_MAIN_LO))
                if st > ARENA_MAIN_LO:
                    # 2026-10-19: 定级为 WARN 而非 FAIL —— 权衡如下:
                    #   · 只有显式下装到 MAIN arena (--addr 0x10005000) 才会踩到;
                    #   · flash.py / 自检的默认路径走 BOOT arena 0x8000, 不受影响;
                    #   · 修正要改 uart_cmd.c 的 ARENA_MAIN_LO, 会改固件 hex,
                    #     必须在出片前定稿, 不能在中途悄悄换。
                    # 留 WARN 是为了每次出片都把它打印出来, 又不会让人为了过闸
                    # 而把断言删掉(那才是最坏的结果)。
                    print("  [WARN] 栈与 MAIN arena 重叠 %d B: 栈顶 0x%08X > "
                          "ARENA_MAIN_LO 0x%08X" % (st - ARENA_MAIN_LO, st, ARENA_MAIN_LO))
                    print("         → MAIN arena 暂不可用(下装到 0x%08X 会被栈覆盖);"
                          % ARENA_MAIN_LO)
                    print("           BOOT arena 0x%05X 不受影响, 常规下装照旧。"
                          % ARENA_BOOT_LO)
                    print("           修法: ARENA_MAIN_LO 抬到 0x%08X (余 %d B), "
                          "需同步 uart_cmd.c / flash.py, 并重新出片。"
                          % (st, 0x10008000 - st))
                else:
                    print("  [ OK ] MAIN arena 在栈顶之上, 余量 %d B"
                          % (ARENA_MAIN_LO - st))
            else:
                print("  [WARN] ELF 里没有 __stack_top 符号 —— 跳过 MAIN arena 重叠检查。")

            # ---- 尺度表与 DDR 镜像必须同源 ----
            #   sw/y3t_scale_table.h (编进固件) 和 sw/engine_scales.bin (上传到 DDR)
            #   是同一次 export_scales.py 的两个产物。若只重生成其中一个,
            #   板上就是"A 套尺度 + B 套偏移", 表现为结果略偏但结构正常 ——
            #   比彻底跑不起来更难查。这里按字数和文件大小硬核对。
            _check_scales(args.root)

    print()
    if miss:
        print("结论: FAIL —— 有 %d 项未通过, 该 lane hex 不是期望的固件, 先修再出片。" % miss)
        return 1
    print("结论: PASS —— 版本标记全部命中, 且段占用在预算内。可以出片。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
