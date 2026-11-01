#!/bin/bash
# ============================================================================
# switch_fw.sh — 切换 TCM 内嵌固件 (demo ⇄ coremark) 并校验
#
# 为什么需要它: TCM 只能放一份固件, 切换必须"重建 + 同步 4 个 lane hex + 重新出片"。
#   历史上两次"比特流嵌旧固件"都是漏了同步这步 (见 BOARD_DEBUG §八b/§八d)。
#   本脚本把"重建→同步→校验"绑定在一起, 并能在切换后用【字符串搜索】确认
#   当前 ipcore 里到底是哪份固件 (不信时间戳、不信 md5)。
#
# 用法:
#   bash tools/switch_fw.sh status      # 看 ipcore 当前嵌的是哪个固件
#   bash tools/switch_fw.sh demo        # 重建并同步 demo bring-up 固件
#   bash tools/switch_fw.sh coremark    # 换入 CoreMark 固件镜像
#
# 切完必须重新跑 PDS: Compile → Place&Route → Bitstream → 烧录
# ============================================================================
set -e
cd "$(dirname "$0")/.."
ROOT=$(pwd)

# ZIGUANG (2026-10-18): cc1.exe / as.exe 依赖 ucrt64/bin 下的 DLL。
#   若 PATH 里没有 /e/msys64/ucrt64/bin, gcc 驱动能找到, 但 fork 出的 cc1 会
#   静默 exit 127 (无任何报错), 表现为"gcc 退出码 1、没有任何输出"。
#   在本次故障排查中踩过, 这里显式补上。
export PATH="/e/msys64/ucrt64/bin:$PATH"

GCC=/e/msys64/ucrt64/bin/riscv32-unknown-elf-gcc
OBJDUMP=/e/msys64/ucrt64/bin/riscv32-unknown-elf-objdump
OBJCOPY=/e/msys64/ucrt64/bin/riscv32-unknown-elf-objcopy
PY="C:/Users/86187/.workbuddy/binaries/python/versions/3.13.12/python.exe"

lanes_ok() {  # 4 个 lane: sw 与 ipcore 是否一致
    local bad=0 i
    for i in 0 1 2 3; do
        a=$(md5sum sw/omnicore_fw_l$i.hex 2>/dev/null | cut -d' ' -f1)
        b=$(md5sum ipcore/tcmram_l$i/omnicore_fw_l$i.hex 2>/dev/null | cut -d' ' -f1)
        [ "$a" = "$b" ] || bad=$((bad+1))
    done
    echo $bad
}

sync_sw_to_ipcore() {
    local i
    for i in 0 1 2 3; do
        cp sw/omnicore_fw_l$i.hex ipcore/tcmram_l$i/omnicore_fw_l$i.hex
    done
    echo "  已同步 sw/omnicore_fw_l*.hex -> ipcore/tcmram_l*/"
    # ZIGUANG (2026-10-17 v16): TCM 已从"4 个 8 位 lane 阵列"改为"1 个 32 位阵列"
    #   (8 位宽的 RAM 会被 ADS 推成 LUTRAM, 32 位才会推 DRM)。
    #   因此综合/仿真读的是 ipcore/tcmram_l0/omnicore_fw32.hex, 必须由 4 个 lane 合成,
    #   否则 bram_slave 的 $readmemh 找不到文件 -> TCM 全 0 -> 板子起不来。
    "$PY" tools/merge_lanes.py ipcore || {
        echo "  [ERR] merge_lanes.py 失败 —— 综合会因缺 omnicore_fw32.hex 而出错"
        exit 1
    }
}

identify() {   # 在 ipcore 的 lane hex 里搜固件标识串
    "$PY" - <<'PY'
import io, os, re
BOOT_BYTES, MAIN_OFF, TOTAL_WORDS = 65536, 0x10000000, 24576
BOOT_WORDS = BOOT_BYTES // 4
mem = {}
for lane in range(4):
    p = "ipcore/tcmram_l%d/omnicore_fw_l%d.hex" % (lane, lane)
    if not os.path.isfile(p):
        print("  [缺] %s" % p); raise SystemExit
    vals = [int(x, 16) for x in io.open(p).read().split()]
    for w in range(TOTAL_WORDS):
        a = (w*4 + lane) if w < BOOT_WORDS else (MAIN_OFF + (w-BOOT_WORDS)*4 + lane)
        mem[a] = vals[w]
buf = bytearray()
# ZIGUANG (2026-10-17 v16): 搜索范围要覆盖 BOOT + MAIN。
#   原来只搜 MAIN(0x1000_0000), 但 board.ld 已把 .text/.rodata 移到 BOOT,
#   版本串 "fw=v14" 现在落在 BOOT 里 —— 只搜 MAIN 会误报"无法识别"。
for a in range(0, BOOT_BYTES):
    buf.append(mem.get(a, 0))
for a in range(MAIN_OFF, MAIN_OFF + 0x8000):
    buf.append(mem.get(a, 0))
def has(s): return s.encode() in buf
marks = {
    "demo bring-up":  has("bring-up v") or has("[v] fw="),
    "CoreMark":       has("CoreMark on OmniCore") or has("for coremark."),
}
hit = [k for k, v in marks.items() if v]
print("  ipcore 当前内嵌: %s" % (" + ".join(hit) if hit else "**无法识别**"))
if len(hit) == 2:
    print("  [警告] 同时命中 demo 与 CoreMark 标记 —— 请人工确认")
elif hit == ["CoreMark"]:
    print("  提示: 跑完 CoreMark 要切回 demo 请执行: bash tools/switch_fw.sh demo")
PY
}

case "${1:-status}" in
status)
    echo "== 固件状态 =="
    echo "  sw/ 与 ipcore/ 不一致的 lane 数: $(lanes_ok)  (0 = 一致)"
    identify
    echo
    echo "  sw/demo.elf    : $(ls -la --time-style=long-iso sw/demo.elf 2>/dev/null | awk '{print $6, $7}')"
    echo "  coremark 镜像   : $(ls sw/coremark_port/coremark_fw_l0.hex 2>/dev/null || echo 无)"
    ;;

demo)
    echo "== 重建并同步 demo 固件 =="
    cd sw
    mkdir -p .tmpbuild
    TMPDIR="$ROOT/sw/.tmpbuild" TMP="$ROOT/sw/.tmpbuild" TEMP="$ROOT/sw/.tmpbuild" \
      "$GCC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -Os -g -ffreestanding -fno-builtin \
      -nostdlib -Wl,--no-warn-rwx-segments -T board.ld boot.S demo.c uart_cmd.c -o demo.elf -lgcc
    "$OBJDUMP" -d demo.elf > demo.dis
    "$OBJCOPY" -O binary --gap-fill 0 demo.elf demo.bin
    python split_hex.py demo.elf omnicore_fw 24576 65536
    rm -rf .tmpbuild
    cd "$ROOT"
    sync_sw_to_ipcore
    # 版本号从 demo.c 的 [v] 行解析, 避免"只校验前缀 [v] fw=" 放过旧固件
    FWVER=$(sed -n 's/.*\[v\] fw=\(v[0-9][0-9]*\).*/\1/p' sw/demo.c | head -1)
    echo
    "$PY" tools/check_fw_hex.py --root . --elf sw/demo.elf \
        --expect "OmniCore fusion bring-up ${FWVER}" --expect "[v] fw=${FWVER}" || true
    echo
    echo ">>> 现在去 PDS: Compile → Place&Route → Bitstream → 烧录"
    ;;

coremark)
    echo "== 换入 CoreMark 固件 =="
    for i in 0 1 2 3; do
        src="sw/coremark_port/coremark_fw_l$i.hex"
        [ -f "$src" ] || { echo "  [缺] $src"; exit 1; }
        cp "$src" "sw/omnicore_fw_l$i.hex"
        cp "$src" "ipcore/tcmram_l$i/omnicore_fw_l$i.hex"
    done
    echo "  已换入 + 同步 (4 lane)"
    # ★ v16 起 TCM 是单个 32 位阵列, 综合/仿真读 ipcore/tcmram_l0/omnicore_fw32.hex。
    #   漏了这步 => 比特流里嵌的还是上一份固件 (会静默跑成 demo), 必须补。
    "$PY" tools/merge_lanes.py ipcore || {
        echo "  [ERR] merge_lanes.py 失败 —— 综合会因 fw32 未更新而嵌入旧固件"
        exit 1
    }
    echo
    identify
    echo
    echo "  建议: 先按 Makefile.coremark 的 ITERATIONS 跑一次; 赛题要求【运行时间 >=10 秒】才算正式成绩"
    echo ">>> 现在去 PDS: Compile → Place&Route → Bitstream → 烧录"
    ;;

*)
    echo "用法: bash tools/switch_fw.sh [status|demo|coremark]"; exit 2
    ;;
esac
