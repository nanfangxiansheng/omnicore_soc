# OmniCore 融合工程 bring-up 固件
# 产物: demo.elf (boot 段 @0x0, 主段 @0x1000_0000)
# 依赖: riscv32-unknown-elf-gcc (xpack 或手装工具链, PATH 里要有)
#       demo.c 里的寄存器口径见 ../FUSION_INTEGRATION.md §三
# 生成比特流内嵌固件: 用 hex2idf 或 PDS BRAM 初始化流程把 4 个 lane hex
#       写进 ipcore 里的 boot BRAM 初始化 (S6 阶段接线)

CROSS  ?= riscv32-unknown-elf-
MARCH  := -march=rv32imac_zicsr_zifencei -mabi=ilp32

# ---- 构建环境护栏 (ZIGUANG 2026-10-18) -----------------------------------
# 坑1: 工具链在 /e/msys64/ucrt64/bin 下, 若不在 PATH, gcc 会"静默失败" ——
#      driver 返回 1、stderr 全空, 极难查。这里统一前置。
MSYS_BIN := /e/msys64/ucrt64/bin:/e/msys64/usr/bin
ifneq (,$(wildcard /e/msys64/ucrt64/bin))
  export PATH := $(MSYS_BIN):$(PATH)
endif
# 坑2: make 会把 TMP/TEMP 清空, gcc 落的临时文件变成 "C:\Windows\" 下的
#      "Cannot create temporary file ... Permission denied"。改指工程内目录。
TOPDIR := $(abspath $(CURDIR)/..)
export TMP    := $(TOPDIR)/.tmp
export TEMP   := $(TOPDIR)/.tmp
export TMPDIR := $(TOPDIR)/.tmp
$(shell mkdir -p $(TOPDIR)/.tmp $(TOPDIR)/.tmp/gcc)

# 坑3: split_hex.py / check_fw_hex.py 是 python 脚本, 干净 PATH 下可能找不到
#      python。优先用托管版解释器, 找不到再退回 PATH 里的 python。
PY ?= $(firstword $(wildcard C:/Users/86187/.workbuddy/binaries/python/versions/*/python.exe) python)

# 坑4 (2026-10-19): 尺度导出要 numpy, 而托管 python 里没有 ⇒ 单独一个变量。
#   优先 anaconda; 没有就退回 PY (可能失败, 那时手动指定 PY_NP=...) 。
PY_NP ?= $(firstword $(wildcard /e/anconda/python.exe) $(PY))

# 2026-10-16: 加 -fno-tree-loop-distribute-patterns —— 防 GCC 把清零/拷贝循环
#   改写成 memset/memcpy 调用 (nostdlib 下没有这些符号)。
CFLAGS := $(MARCH) -Os -g -ffreestanding -fno-builtin -nostdlib \
          -fno-tree-loop-distribute-patterns \
          -Wl,--no-warn-rwx-segments

all: demo.elf

HDRS := y3t.h y3t_exec.h y3t_net_table.h y3t_scale_table.h y3t_scale_table_blob.h y3t_selftest.h \
        y3t_pipeline.h perf_probe.h trap_emu.h cmd_proto.h layer_table.h

# 逐通道尺度表 + DDR 镜像。固件 (g_scale) 与 engine_scales.bin 必须**同源**,
#   否则板上会用 A 套尺度配 B 套权重 —— 那种错很难看出来, 只会觉得"结果有点偏"。
#   所以 hex 目标强制先跑这一步。
# 整网执行表 (29 job, 含 pool/up; concat 折进 L17 的 in1+split)
#   依赖 y3t_params.json 的 jobs/tmeta 与 sw/engine_layout.json 的权重块偏移。
PARAMS ?= $(TOPDIR)/../omini/tools/y3t/demo_hw/y3t_params.json
LAYOUT ?= $(CURDIR)/engine_layout.json

net_table:
	$(PY_NP) ../tools/gen_net_table.py --params $(PARAMS) --layout $(LAYOUT) --out $(CURDIR)/y3t_net_table.h

# 逐通道尺度表 + DDR 镜像。必须先有 net_table —— 它按层表顺序与 cot 排版。
scales: net_table
	$(PY_NP) ../tools/export_scales.py --params $(PARAMS) --table $(CURDIR)/y3t_net_table.h --outdir $(CURDIR)
	$(PY_NP) ../tools/pack_scales_aligned.py

# 离线验证尺度镜像: 从 blob 反读, 用 golden 逐层对拍 (需要 numpy + omini 的 golden)
verify_scales:
	$(PY_NP) ../tools/verify_scales.py

# 离线验证整网表: 用表里的几何参数重跑每层, 与 golden 逐字节比 (含 concat 顺序)
verify_net:
	$(PY_NP) ../tools/verify_net_table.py

# 出片前把所有离线验证跑一遍
verify_all: verify_scales verify_net

demo.elf: scales boot.S demo.c uart_cmd.c y3t_exec.c y3t_detect.c y3t_auto.c $(HDRS) board.ld
	$(CROSS)gcc $(CFLAGS) -T board.ld boot.S demo.c uart_cmd.c y3t_exec.c y3t_detect.c y3t_auto.c -o $@ -lgcc
	$(CROSS)objdump -d $@ > demo.dis
	$(CROSS)objcopy -O binary demo.elf demo.bin

# 分 lane hex (boot BRAM 4 lane 初始化用, 与 omnicore_fw_l*.hex 同格式)
# 注意: 综合时 $readmemh 读的是 ../ipcore/tcmram_l*/omnicore_fw_l*.hex (路径相对
#       工程根), 因此本 target 末尾把 4 个 lane hex 同步过去 —— 历史上两次
#       "比特流内嵌旧固件" (FUSION_INTEGRATION.md §八b 根因3 / §八d) 都是漏了这步。
hex: demo.elf
	$(CROSS)objcopy -O binary --gap-fill 0 demo.elf demo.bin
	$(PY) split_hex.py demo.elf omnicore_fw 24576 65536
	$(PY) -c "import shutil;[shutil.copy('omnicore_fw_l%d.hex'%i,'../ipcore/tcmram_l%d/omnicore_fw_l%d.hex'%(i,i)) for i in range(4)];print('firmware synced -> ipcore/tcmram_l*/omnicore_fw_l*.hex')"
	$(PY) ../tools/merge_lanes.py
	@V=$$(sed -n 's/.*#define FW_VERSION "\(v[0-9][0-9]*\).*/\1/p' demo.c | head -1); \
	if [ -z "$$V" ]; then echo "  [ERR] demo.c 里找不到 #define FW_VERSION"; exit 1; fi; \
	echo "  demo.c FW_VERSION = $$V"; \
	$(PY) ../tools/check_fw_hex.py --root .. --elf demo.elf \
		--expect "bring-up $$V" --expect "[v] fw=$$V"

# 单独运行固件嵌版自检 (反解 lane hex, 直查版本标记串, 不信时间戳)
verify:
	$(PY) ../tools/check_fw_hex.py --root .. --elf demo.elf --expect "[v] fw="

clean:
	rm -f demo.elf demo.dis demo.bin

.PHONY: all clean hex verify scales verify_scales net_table verify_net verify_all

# ---- 快速启动诊断版 (ZIGUANG 2026-10-16) --------------------------------
# 用途: 判断板上重启是【时间型】还是【负载型】。
#   make -C sw hex_fast   -> demo_fast.elf + 同步 4 lane hex
# 判读: 若仍在上电后约 2-3 秒重启 => 时间型(电源/热/配置电路), 转硬件侧;
#       若能撑到十几秒以上        => 负载型(与启动探针/DDR 压力相关)。
hex_fast: demo_fast.elf
	$(CROSS)objcopy -O binary --gap-fill 0 demo_fast.elf demo_fast.bin
	$(PY) split_hex.py demo_fast.elf omnicore_fw 24576 65536
	$(PY) -c "import shutil;[shutil.copy('omnicore_fw_l%d.hex'%i,'../ipcore/tcmram_l%d/omnicore_fw_l%d.hex'%(i,i)) for i in range(4)];print('FASTBOOT firmware synced')"
	$(PY) ../tools/merge_lanes.py

demo_fast.elf: boot.S demo.c uart_cmd.c y3t_exec.c $(HDRS) board.ld
	$(CROSS)gcc $(CFLAGS) -DFASTBOOT=1 -T board.ld boot.S demo.c uart_cmd.c y3t_exec.c -o $@ -lgcc
	$(CROSS)size $@

.PHONY: hex_fast

# ---- arena 整网执行器 blob（UART 下装，绕开出片迭代）----------------------
# 用途: 把带修复的执行器逻辑装进 BOOT arena(0x8000), 0x0D EXEC 运行,
#       效果等同 NET_RUN 但不需要重新出片。地址符号从 demo.elf 的 nm 提取。
blob_syms.h: demo.elf
	$(CROSS)nm demo.elf | awk '{ if ($$2=="T") printf "#define %s_ADDR 0x%s\n", toupper($$3), $$1 }' | grep -E "DDRB_READ|UART_PUTC|UART_PUTS|UART_HEX|UART_DEC" > $@
	@cat $@

netrun_blob.bin: netrun_blob.c blob_syms.h y3t_net_table.h y3t_scale_table_blob.h netrun_ld.ld
	@- $(PY) ../tools/pack_scales_aligned.py
	$(CROSS)gcc $(MARCH) -Os -g -ffreestanding -fno-builtin -nostdlib \
	  -fno-tree-loop-distribute-patterns -msmall-data-limit=0 -mno-relax \
	  -T netrun_ld.ld netrun_blob.c -o netrun_blob.elf -lgcc
	$(CROSS)objcopy -O binary netrun_blob.elf $@
	@$(CROSS)size netrun_blob.elf

.PHONY: blob
blob: netrun_blob.bin

# ---- arena rom_stream 补丁模块（串口打补丁用，固定 0x9000）----
rom_patch.bin: rom_patch.c blob_syms.h rom_patch_ld.ld
	$(CROSS)gcc $(MARCH) -Os -ffreestanding -fno-builtin -nostdlib \
	  -fno-tree-loop-distribute-patterns -msmall-data-limit=0 -mno-relax \
	  -T rom_patch_ld.ld rom_patch.c -o rom_patch.elf -lgcc
	$(CROSS)objcopy -O binary rom_patch.elf $@
	@$(CROSS)size rom_patch.elf

.PHONY: patch
patch: rom_patch.bin

# ---- arena 自治实时识别循环（相机→PP→整网→解码→画框，无限循环）----
blob_auto.bin: blob_auto.c blob_syms.h y3t_net_table.h y3t_scale_table_blob.h y3t_detect_tbl.h netrun_ld.ld
	$(CROSS)gcc $(MARCH) -Os -ffreestanding -fno-builtin -nostdlib \
	  -fno-tree-loop-distribute-patterns -msmall-data-limit=0 -mno-relax \
	  -T netrun_ld.ld blob_auto.c -o blob_auto.elf -lgcc
	$(CROSS)objcopy -O binary blob_auto.elf $@
	@$(CROSS)size blob_auto.elf

.PHONY: autob
autob: blob_auto.bin

# ---- PP 常开快版 (2026-10-25 实测 21fps; 输入完整性无保证, 演示/测速用) ----
blob_fast.bin: blob_auto_fast.c blob_syms.h y3t_net_table.h y3t_scale_table_blob.h y3t_detect_tbl.h netrun_ld.ld
	$(CROSS)gcc $(MARCH) -Os -ffreestanding -fno-builtin -nostdlib \
	  -fno-tree-loop-distribute-patterns -msmall-data-limit=0 -mno-relax \
	  -T netrun_ld.ld blob_auto_fast.c -o blob_fast.elf -lgcc
	$(CROSS)objcopy -O binary blob_fast.elf $@
	@$(CROSS)size blob_fast.elf

.PHONY: blobfast
blobfast: blob_fast.bin

# ---- meiea 使能 + GPIO 中断窗口验证 (2026-10-27) ----
meiea_test.bin: meiea_test.c netrun_ld.ld
	$(CROSS)gcc $(MARCH) -Os -ffreestanding -fno-builtin -nostdlib \
	  -fno-tree-loop-distribute-patterns -msmall-data-limit=0 -mno-relax \
	  -T netrun_ld.ld meiea_test.c -o meiea_test.elf -lgcc
	$(CROSS)objcopy -O binary meiea_test.elf $@
	@$(CROSS)size meiea_test.elf

.PHONY: meiea
meiea: meiea_test.bin
