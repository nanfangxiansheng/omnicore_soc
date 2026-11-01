/* ============================================================================
 * demo.c — OmniCore 融合工程 bring-up + 帧差检测固件 v5
 *
 * 数据通路: 相机帧(DDR) ─CPU经桥读ROI─► 灰度/帧差 ─► 检测框 ─► overlay叠画
 * Y3T 推理在环: 硬件/驱动就绪 (y3t.h), 权重加载方案定稿后替换 motion 段即可。
 *
 * 阶段:
 *   0. CPU 本地内存自检 (不碰 DDR, 用于区分 CPU 存 vs DDR 通路)   [v3 新增]
 *   1. UART 横幅 (54.000MHz, 115200) + overlay 四角静态框
 *   2. DDR 桥 256B 读写自检 (块基址 600000, 已在帧区之外)
 *   2b. 块级错位探针 (每块填块号标签, 直接量出位移几块)            [v3 新增]
 *   2c. 邻域探测 pre[-8]/post[+8]/mid[+4] (判断自检区是否被外部写入)
 *   3. 相机帧序轮询 (窗口 15s —— 实测相机 ~5~8s 才出帧, 3s 窗口会假报 STUCK)
 *   3b. 帧序速率 (1s 内 cam0/cam1 帧序号变化次数)
 *   4. 相机帧数据抽查 (含 initword 计数: 0x55555555 = 从未写入)
 *   4b. 相机写入活性探针 (同址隔 200ms 再读, 变化即相机在写)        [v4 新增]
 *   2d. 读确定性探针 (同址连读 8 次, 判"桥读抖动"vs"真实地址错位")  [v4 新增]
 *   5. Y3T 引擎在位检查 (ID="Y3T1")
 *   6. 帧内容缩略图 (32x16 ASCII 亮度图 + white/black/rowcorr 统计)  [v5 新增]
 *      —— 桥修好后第一次能"看见"帧缓冲: 判 花屏 出在【显示侧】还是【相机数据侧】
 *   7. 主循环: 帧同步 → ROI 灰度帧差 → 运动检测框 (green) + 心跳
 * ========================================================================== */
void uart_cmd_poll(void);
#include <stdint.h>
#include "trap_emu.h"
#include "y3t_selftest.h"
#include "perf_probe.h"      /* v14: 性能计数器读取/打印 (从机 @0x4000_7000) */
#include "gpio_ctl.h"        /* v65: GPIO 外设使用场景 + 中断计数 (赛题外设集成) */

/* ---- ZIGUANG (2026-10-16): 快速启动开关 (诊断重启是时间型还是负载型) ---------
 * FASTBOOT=1 时: 打印完 [0]/[1]/[2] 就立刻进主循环, 跳过 [6]/[6b]/[6c]/[2b]/[2c]/
 *   [3b]/[4b]/[2d]/[5]/[7] 等重探针 (它们本身耗时 1~2 秒并压 DDR)。
 * 判读: 若仍在上电后约 2-3 秒重启 => 时间型 (电源/热/配置电路), 转硬件侧;
 *       若能撑到十几秒 => 负载型 (与探针/DDR 压力相关)。 */
#ifndef FASTBOOT
#define FASTBOOT 0
#endif

/* ---- ZIGUANG (2026-10-26): 版本号单一来源 -----------------------------------
 * 事故: banner 与探针是两条各自手改的版本串。
 *   v55/v56/v57/v58 每次只改 "[v] fw=", banner 一直停在 "bring-up v54"
 *   ⇒ 上板肉眼看到的版本号是 v54, 与真实固件(v58)差 4 个版本;
 *   而 tools/switch_fw.sh 会用 [v] 行的号去断言 banner, 于是它必判 FAIL。
 * 现在唯一来源 = FW_VERSION, banner / 探针都从它派生。
 * 改版本号只改这一行, 顺手 grep 确认没有别处写死。 */
#define FW_VERSION "v72+blobfeat"  /* v67基线+Q8/NMS/置信度/FPS(同 blob_auto) */

/* ---- ZIGUANG (2026-10-16): 未对齐访问自检用的强制指令助手 ----------------
 * 编译器对已知错位的指针会自己拆成字节访问, 测不到硬件/模拟器行为,
 * 所以这里用 inline asm 强出一条真正的 lw/lh/... 指令。 */
/* ---- ZIGUANG (2026-10-16): -nostdlib 下的最小 memset/memcpy -----------------
 * GCC 在 -O2 会把"清零循环"识别成 memset 调用 (loop-distribute-patterns),
 * 即便带了 -fno-builtin 也会 —— 缺符号就链接失败。这里补最小实现。 */
void *memset(void *d, int c, unsigned long n) {
	unsigned char *p = (unsigned char *)d;
	while (n--) *p++ = (unsigned char)c;
	return d;
}
void *memcpy(void *d, const void *s, unsigned long n) {
	unsigned char *p = (unsigned char *)d;
	const unsigned char *q = (const unsigned char *)s;
	while (n--) *p++ = *q++;
	return d;
}

static uint32_t asm_lw (const void *p){ uint32_t v; __asm__ volatile("lw  %0, 0(%1)":"=r"(v):"r"(p)); return v; }
static uint32_t asm_lh (const void *p){ uint32_t v; __asm__ volatile("lh  %0, 0(%1)":"=r"(v):"r"(p)); return v; }
static uint32_t asm_lhu(const void *p){ uint32_t v; __asm__ volatile("lhu %0, 0(%1)":"=r"(v):"r"(p)); return v; }
static uint32_t asm_lb (const void *p){ uint32_t v; __asm__ volatile("lb  %0, 0(%1)":"=r"(v):"r"(p)); return v; }
static uint32_t asm_lbu(const void *p){ uint32_t v; __asm__ volatile("lbu %0, 0(%1)":"=r"(v):"r"(p)); return v; }
static void asm_sw(void *p, uint32_t v){ __asm__ volatile("sw %1, 0(%0)"::"r"(p),"r"(v):"memory"); }
static void asm_sh(void *p, uint32_t v){ __asm__ volatile("sh %1, 0(%0)"::"r"(p),"r"(v):"memory"); }
static void asm_sb(void *p, uint32_t v){ __asm__ volatile("sb %1, 0(%0)"::"r"(p),"r"(v):"memory"); }

                      // <stdint.h> 冲突 (riscv newlib int32_t=long int)

#define REG32(a) (*(volatile uint32_t *)(a))

/* ---- UART ---- */
#define UART_BASE    0x40000000u
#define UART_STATUS  REG32(UART_BASE + 0x04u)
#define UART_CTRL    REG32(UART_BASE + 0x08u)
#define UART_BAUDDIV REG32(UART_BASE + 0x0Cu)
#define UART_DATA    REG32(UART_BASE + 0x00u)
#define UART_TX_READY 0x2u

/* ---- GPIO ---- (v65: 寄存器宏统一在 gpio_ctl.h, 这里不再重复定义) */

/* ---- Timer ---- (v65: 宏统一在 gpio_ctl.h) ---- */

/* ---- overlay ---- */
/* overlay 开关掩码: bit0 四角自检 / bit1 运动框 (0x14 CMD_OVL_MODE 可改) */
/* v64: 默认带 bit2(检测框)。原来是 0x03(四角自检+运动框)，导致 0x15 自动循环里
 *   y3t_auto.c 的 `g_ovl_mask & 0x04` 为假 ⇒ 框使能永不点亮，
 *   而文字层不走这个判断 ⇒ 屏幕出现"有类别文字但没有框"。
 *   运行时也可用 0x14 命令临时改（tools/auto_run.py --ovl）。 */
volatile uint32_t g_ovl_mask = 0x07u;

/* v65: GPIO 中断统计 (gpio_ctl.h 里 extern, ISR 与 0x18 命令共用) */
volatile uint32_t g_irq_count = 0u;
volatile uint32_t g_irq_last_cause = 0u;
volatile uint32_t g_irq_last_pend = 0u;
volatile uint32_t g_gpio_evt = 0u;
volatile uint32_t g_timer_evt = 0u;

#define OVL_BASE     0x40006000u
#define OVL_MIN(n)   REG32(OVL_BASE + (n)*12u)
#define OVL_MAX(n)   REG32(OVL_BASE + (n)*12u + 4u)
#define OVL_COLOR(n) REG32(OVL_BASE + (n)*12u + 8u)
#define OVL_EN       REG32(OVL_BASE + 0x60u)
#define OVL_FRAME    REG32(OVL_BASE + 0x78u)
#define OVL_VER      REG32(OVL_BASE + 0x7Cu)

/* ---- DDR 桥 ---- */
#define DDRB_CTRL   REG32(0x80000000u)
#define DDRB_ADDR   REG32(0x80000004u)
#define DDRB_STATUS REG32(0x80000008u)
#define DDRB_DATA   REG32(0x8000000Cu)
#define DDRB_DONE   0x1u

#define CAM0_FRAME0_BLK 0u
#define SELFTEST_BLK    600000u  /* 移出帧区: 帧区占 0..432000 块 */

void uart_putc(char c) {   /* 非 static: uart_cmd.c 外部引用 */
	while (!(UART_STATUS & UART_TX_READY))
		;
	UART_DATA = (uint8_t)c;
}
void uart_puts(const char *s) { while (*s) uart_putc(*s++); }   /* v37: 非 static, y3t_auto.c 也用 */
void uart_hex(uint32_t v) {
	for (int i = 28; i >= 0; i -= 4) {
		uint32_t d = (v >> i) & 0xF;
		uart_putc((char)(d < 10 ? '0' + d : 'a' + d - 10));
	}
}
void uart_dec(uint32_t v) {
	char buf[11]; int i = 10;
	buf[10] = 0;
	do { buf[--i] = (char)('0' + v % 10); v /= 10; } while (v);
	uart_puts(&buf[i]);
}
static uint64_t mtime(void) {
	uint32_t hi, lo;
	do {
		hi = TIMER_MTIME_HI;
		lo = TIMER_MTIME_LO;
	} while (hi != TIMER_MTIME_HI);
	return ((uint64_t)hi << 32) | lo;
}

/* ---- DDR 桥突发读/写 ----
 * ZIGUANG 修复 (v7): DDR IP 的 AXI 用户口 ARLEN/AWLEN 只有 4 位
 *   (fusion_top.v: axi_awlen / axi_arlen 均为 [3:0]), 主控输出 8 位在连线处被截断
 *   => 单次 AXI 突发【最多 16 拍 (512B)】。超过 16 拍时主控只发 16 拍,
 *   桥的 FIFO 拿不满数据 => CPU 弹 DATA 时 hreadyout 恒 0 => 死等 (这正是
 *   [6] 缩略图卡死的原因: 它是全工程唯一 beats>16 的调用; 视频通路
 *   BURST_SIZE=16 恰好卡在上限所以一直正常)。
 *   修法: 固件侧把传输分块到 <=16 拍 (不改 RTL, 风险最小)。
 */
/* 【BUG-FIX 2026-10-22 10:40】16 → 8。
 *   板上实测(同一份数据只改帧大小):
 *     每帧 8 块(256B) 上传 128 字 -> 坏 0 个
 *     每帧 127 块(4064B)     -> 坏 196/512 (38%), 且坏在"每 16 块批的最后 4 块"
 *   原因: AXI AWLEN 仅 4 位 => 单突发 <=16 拍(512B), 而 590000*32B=0x1201F00
 *   起的 512B 突发会跨过 0x1202000 这个 4KB 边界, 控制器按 AXI 规则截断,
 *   每批丢最后约 4 拍(128B)。8 拍/批(256B) 基本不会跨 4KB, 实测全对。
 *   代价: DDR 事务数翻倍, 3MB 权重约 700ms/条, 可接受。 */
#define DDRB_MAX_BEATS 8u
/* 【BUG-FIX 2026-10-22 14:45】批间插入固定延时。
 *   根因: ddr_bridge 的 start_tgl / done_tgl_core 是**跨域翻转同步**,
 *   两次翻转之间必须留够传播时间(2~3 拍 core + 2~3 拍 ahb)。
 *   原来批间是"清 done 就立刻 start", 间隔只有 1~2 拍 ⇒ 翻转被吞 ⇒
 *   批次状态错乱。v29 试过"等 done 落 0", 结果间隔更短、坏字率从 38% 升到 63%,
 *   正好反证了这个机制。
 *   这里改成纯 nop 延时(不碰总线, 不会被优化掉), 约 DDRB_BATCH_GAP 次空转。 */
#define DDRB_BATCH_GAP 400u
static void ddrb_gap(void)
{
	volatile uint32_t d;
	for (d = 0; d < DDRB_BATCH_GAP; d++) { }
}

int ddrb_read(uint32_t blk, uint32_t *dst, uint32_t beats) {   /* 非 static: uart_cmd.c 外部引用 */
	uint32_t done = 0;
	while (beats) {
		uint32_t n = beats > DDRB_MAX_BEATS ? DDRB_MAX_BEATS : beats;
		DDRB_ADDR = blk + done;
		DDRB_CTRL = (n << 8) | 0u;
		DDRB_CTRL = (n << 8) | 1u;
		uint32_t guard = 0;
		while (!(DDRB_STATUS & DDRB_DONE))
			if (++guard > 2000000u) return -1;
		for (uint32_t i = 0; i < n * 8u; i++)
			dst[done * 8u + i] = DDRB_DATA;
		DDRB_STATUS = DDRB_DONE;
		ddrb_gap();
		done += n; beats -= n;
	}
	return 0;
}
int ddrb_write(uint32_t blk, const uint32_t *src, uint32_t beats) {   /* 非 static: uart_cmd.c 外部引用 */
	uint32_t done = 0;
	while (beats) {
		uint32_t n = beats > DDRB_MAX_BEATS ? DDRB_MAX_BEATS : beats;
		DDRB_ADDR = blk + done;
		DDRB_CTRL = (n << 8) | (1u << 1);
		for (uint32_t i = 0; i < n * 8u; i++)
			DDRB_DATA = src[done * 8u + i];
		DDRB_CTRL = (n << 8) | (1u << 1) | 1u;
		uint32_t guard = 0;
		while (!(DDRB_STATUS & DDRB_DONE))
			if (++guard > 2000000u) return -1;
		DDRB_STATUS = DDRB_DONE;
		ddrb_gap();
		done += n; beats -= n;
	}
	return 0;
}

/* ---- overlay 框 ---- */
static void ovl_box(uint32_t n, uint32_t xmin, uint32_t ymin,
                    uint32_t xmax, uint32_t ymax, uint32_t rgb565) {
	OVL_MIN(n)   = ((ymin & 0xFFFu) << 16) | (xmin & 0xFFFu);
	OVL_MAX(n)   = ((ymax & 0xFFFu) << 16) | (xmax & 0xFFFu);
	OVL_COLOR(n) = rgb565;
}

#include "y3t.h"
#include "y3t_exec.h"     /* 整网执行器 (静态 job 表 + 逐层权重流式), 给 0x10 NET_RUN 用 */

/* ---- 0x10 NET_RUN 的薄封装 -------------------------------------------------
 * 真正的调度在 y3t_exec.c：表驱动、每层 wload_dma 搬权重、L11 自动拆 2 个子 job。
 * vals[1..5] 回填 {jobs, failed, wload_cyc, mac_cyc, total_cyc}; vals[0]=返回值由调用方填。
 * romblk==0 ⇒ identity 尺度（multM=1/bias=0），只能验"结构跑通 + 拍数"，不能验数值。 */
int demo_net_run(uint32_t wblk, uint32_t romblk, uint32_t *vals)
{
	y3t_exec_stats_t st;
	int ret;

	y3t_exec_rom_from_ddr(romblk);
	ret = y3t_exec_run(wblk ? wblk : Y3T_EXEC_WBASE_BLK, &st);
	if (vals) {
		vals[0] = 0u;
		vals[1] = st.jobs;
		vals[2] = st.failed;
		vals[3] = st.wload_cyc;
		vals[4] = st.mac_cyc;
		vals[5] = st.total_cyc;
	}
	return ret;
}

/* ---- ROI 帧差参数: cam0 帧中心 80×64 (10KB, 桥 320 块/帧 ≈ 0.5ms) ---- */
#define ROI_X0   280u
#define ROI_Y0   328u
#define ROI_W    80u
#define ROI_H    64u
#define ROI_ROW_BLKS 40u                    /* 640px 行 = 40 块 (256b) */
static uint8_t prev_gray[ROI_W * ROI_H];    /* 5KB 上一帧灰度 */

/* ---- 块级错位探针: 每块填块号标签 (0xB0000000|k), 读回定位位移 ----
 * 返回 0xFF = 8 块全对; 否则 = 第一个标签不符的字下标 (0..63), *fnd 带回实读值 */
static uint32_t blk_probe(uint32_t *rb, uint32_t *fnd) {
	uint32_t wbuf[64];
	for (int k = 0; k < 8; k++)
		for (int j = 0; j < 8; j++)
			wbuf[k * 8 + j] = 0xB0000000u | (uint32_t)k;
	if (ddrb_write(SELFTEST_BLK, wbuf, 8) != 0) return 0xFEu;
	if (ddrb_read(SELFTEST_BLK, rb, 8) != 0)    return 0xFDu;
	for (int i = 0; i < 64; i++)
		if (rb[i] != (0xB0000000u | (uint32_t)(i / 8))) { *fnd = rb[i]; return (uint32_t)i; }
	return 0xFFu;
}

/* ---- 读确定性探针 (ZIGUANG 2026-10-15 v4): 同址连读 8 次, 返回不一致的字数 ----
 * 返回 0xFFFFFFFF = 读超时; 否则 = 后 7 次相对第 1 次的不一致字数 (0..448)。
 * 【判据】若 >0 => DDR 桥读通路【非确定性】: 同一份静态数据每次读回都不同,
 *   则 [2] 的失配与主循环 detect 爬升都可能是"读抖动"造成的假象, 而非地址错位。 */
static uint32_t rd_stab_probe(uint32_t *ref) {
	static uint32_t t[64];
	uint32_t bad = 0;
	if (ddrb_read(SELFTEST_BLK, ref, 8) != 0) return 0xFFFFFFFFu;
	for (int r = 0; r < 7; r++) {
		if (ddrb_read(SELFTEST_BLK, t, 8) != 0) return 0xFFFFFFFFu;
		for (int i = 0; i < 64; i++) if (t[i] != ref[i]) bad++;
	}
	return bad;
}

/* ---- 忙等延时 (基于 mtime, 54.054MHz) ---- */
static void delay_ms(uint32_t ms) {
	uint64_t t0 = mtime();
	while (mtime() - t0 < (uint64_t)ms * 54054ull)
		;
}

/* ---- 帧内容缩略图 (ZIGUANG v5): 读 cam0 帧槽 0, 打 32x16 亮度缩略 + 统计 ----
 * 桥修好后这是第一次能"看见"帧缓冲内容。 */
#define FR_ROW_BLKS  40u          /* 640px 行 = 40 个 256bit 块 */
#define FR_ROW_WORDS 320u         /* 40 块 x 8 字 = 320 字 = 640 像素 */
static uint32_t fr_row[FR_ROW_WORDS];
static uint8_t  fr_prev[32];

static void frame_probe(uint32_t cam_base) {
	uint32_t white = 0, black = 0, px = 0;
	uint32_t rc_num = 0, rc_den = 0, ty, tx;
	const char *ramp = " .:-=+*#%@";
	uart_puts("[6] thumb cam_base="); uart_dec(cam_base); uart_puts("\n");
	for (ty = 0; ty < 16u; ty++) {
		uint32_t r = ty * 45u;                  /* 720 / 16 */
		if (r > 719u) r = 719u;
		if (ddrb_read(cam_base + r * FR_ROW_BLKS, fr_row, FR_ROW_BLKS) != 0) {
			uart_puts("[6] row TIMEOUT at r="); uart_dec(r); uart_puts("\n");
			return;
		}
		uart_puts("[6] r"); uart_dec(r); uart_puts(" |");
		for (tx = 0; tx < 32u; tx++) {
			uint32_t c = tx * 20u;              /* 640 / 32 */
			uint32_t w, p, l;
			if (c > 639u) c = 639u;
			w = fr_row[c >> 1];
			p = (c & 1u) ? (w >> 16) : (w & 0xFFFFu);
			l = ((((p >> 11) & 31u) * 4u + ((p >> 5) & 63u) * 8u + (p & 31u) * 4u) >> 4);
			if (p == 0xFFFFu) white++;
			if (p == 0x0000u) black++;
			px++;
			uart_putc(ramp[(l * 9u) / 255u]);
			if (ty > 0u) {
				uint32_t d = (l > fr_prev[tx]) ? (l - fr_prev[tx]) : (fr_prev[tx] - l);
				rc_den++;
				if (d <= 12u) rc_num++;
			}
			fr_prev[tx] = (uint8_t)l;
		}
		uart_puts("|\n");
	}
	uart_puts("[6] white="); uart_dec(white); uart_puts("/"); uart_dec(px);
	uart_puts(" black=");    uart_dec(black);
	uart_puts(" rowcorr=");  uart_dec(rc_den ? (rc_num * 100u / rc_den) : 0u);
	uart_puts("%  (高=像图, ~55%=像噪点)\n");

	/* ---- 6b. 4 个帧槽各自的指纹 (cam_base + 槽距) ----
	 * 判据: 若 4 槽都含"新数据"(head 非 0x55555555 且互不相同) => 写确实在轮转;
	 *       若只有 slot0 有新数据、slot1/2/3 仍是 0x55555555 => 写【只落在 slot0】,
	 *       那么显示侧有 3/4 的时间在显示空槽 —— 这正好解释"花屏"。 */
	{
		static const uint32_t off[4] = { 0u, 57600u, 115200u, 172800u };
		static uint32_t one[8];
		uint32_t k;
		uart_puts("[6b] slot fingerprints cam_base="); uart_dec(cam_base); uart_puts("\n");
		for (k = 0; k < 4u; k++) {
			uart_puts("     slot"); uart_dec(k);
			uart_puts(" base="); uart_dec(cam_base + off[k]);
			if (ddrb_read(cam_base + off[k], one, 1) == 0) {
				uart_puts("  blk0="); uart_hex(one[0]);
			} else uart_puts("  blk0=TIMEOUT");
			if (ddrb_read(cam_base + off[k] + 13137u, one, 1) == 0) {
				uart_puts("  roi="); uart_hex(one[0]);
				if (one[0] == 0x55555555u) uart_puts("(未写)");
			} else uart_puts("  roi=TIMEOUT");
			uart_puts("\n");
		}
	}

	/* ---- 6c. 【v10】原始像素转储 + 行距探测 ----
	 * v9 的缩略图行呈"亮/暗交替"(period 90 行), 两种解释: (i) 帧缓冲里是 Bayer
	 * 图案(奇偶行底色不同); (ii) DDR 行的步长不是 40 块, 我的采样在跨行。
	 * 下面直接打原始 32bit 字 + 若干块的 w0 —— 一看便知:
	 *   连续块 w0 若在 40 块处"跳变" => 步长=40(640px 行); 在 80 块处跳变 => 步长=80
	 *   (1280px 行)。字数若呈"字节周期性"(如 9fee9fee) => Bayer。
	 */
	{
		static uint32_t raw[8];
		static const uint32_t poff[8] = { 0u, 39u, 40u, 79u, 80u, 119u, 120u, 159u };
		uint32_t i;
		uart_puts("[6c] cam_base="); uart_dec(cam_base); uart_puts(" raw blk0 w0..w7:");
		if (ddrb_read(cam_base, raw, 1) == 0) {
			for (i = 0; i < 8u; i++) { uart_putc(' '); uart_hex(raw[i]); }
		} else uart_puts(" TIMEOUT");
		uart_puts("\n[6c] blk 0/39/40/79/80/119/120/159 w0:");
		for (i = 0; i < 8u; i++) {
			if (ddrb_read(cam_base + poff[i], raw, 1) == 0) { uart_putc(' '); uart_hex(raw[0]); }
			else uart_puts(" TO");
		}
		uart_puts("\n");
	}
}

/* ---- CPU 本地内存自检 (完全不碰 DDR/总线) ----
 * 若这里也报错 => CPU 侧存储 (TCM/主存/栈) 问题, 与 DDR 通路无关 */
static uint32_t cpu_mem_selftest(void) {
	static uint32_t a[128], b[128];
	for (int i = 0; i < 128; i++) a[i] = 0xC3000000u ^ (uint32_t)(i * 0x01010101u);
	for (int i = 0; i < 128; i++) b[i] = a[i];
	for (int i = 0; i < 128; i++) if (b[i] != a[i]) return (uint32_t)i + 1u;
	return 0u;
}

static uint32_t selftest_stages(uint32_t *rd) {
	uint32_t fails = 0;

	/* 【BUG-FIX 2026-10-22 17:00】这里原来写死 468 (= 54.054M/468 ≈ 115200),
	 *   把 RTL 的 UART_BAUD 参数完全覆盖掉了 —— 所以 fusion_top 里
	 *   把 UART_BAUD 改成 460800 后, 板上波特率**始终是 115200**。
	 *   写 0 让硬件回落到 BAUDDIV_RESET (= CLK_HZ/UART_BAUD - 1),
	 *   波特率从此由 RTL 参数单一决定, 软件不再插手。 */
	UART_BAUDDIV = 0;
	UART_CTRL = 0;
	/* ---- ZIGUANG (v8): 软复位不清的计数器 —— 统计重启次数
 *   .noinit 段不被 _start 的 .bss 清零覆盖; 上电时 BRAM 初值为 0,
 *   软复位(rst_n 掉一下)则保留 => boot_cnt 能数出重启次数。 */
static uint32_t boot_magic __attribute__((section(".noinit")));
static uint32_t boot_cnt   __attribute__((section(".noinit")));

/* v65: GPIO 外设初始化 (低 8 位输出, gpio_in[0] 上升沿中断源) —— 必须在
 *   banner 之前, 保证后面所有流程都能用 gpio_publish() 输出结果。 */
gpio_init();

uart_puts("\n\nOmniCore fusion bring-up " FW_VERSION " (soc_clk 54.000MHz)\n");
if (boot_magic != 0xB007C0DEu) { boot_magic = 0xB007C0DEu; boot_cnt = 0u; }
boot_cnt++;
/* ZIGUANG v15: 放开标准 CSR 计数器 (mcycle/minstret)。
 * 复位默认 mcountinhibit=1 "省电压制", 不写这句两个计数器恒 0; 
 * 开了之后 [9] 才能给出与 CoreMark 同源的真实 IPC。 */
perf_csr_release();

uart_puts("[boot #"); uart_dec(boot_cnt); uart_puts("]\n");

	/* ZIGUANG: 探针清单行 —— 板上看到这行才说明【新固件真的进了比特流】。
	 * v25 (2026-10-19): 逐通道尺度 (ROM 0/1024/2048 真实布局) + 29 job 整网表
	 *   (含 pool/up/concat) + wload_dma/0x11 W3T。版本号必须跟着改 ——
	 *   之前版本串一直是 v24, 旧片新片在串口上根本分不出来。 */
	uart_puts("[v] fw=" FW_VERSION " probes=0,2,3,3b,6,1(VER x4),2b,2c,4,4b,2d,5(id x2),9(PERF+CSR),10(LOAD) | cmd:04/0B/0C/0D/0E/10(NET_RUN)/11(W3T)/15(AUTO)/17(GPIO)/18(GPIO_IRQ)/19(SETTLE)");

	/* ZIGUANG (v14): 性能计数器从 [0] 之前开始计, 这样 [9] 统计的是整段 bring-up
	 * 真实负载 (含 cache 预热、总线争用、轮询空转)。从机不存在时只打印 absent。 */
	if (!perf_present()) perf_start();

	/* ---- 0. CPU 本地内存 ---- */
	{
		uint32_t bad = cpu_mem_selftest();
		if (bad) { uart_puts("[0] CPU RAM FAIL at word "); uart_dec(bad); uart_puts("\n"); fails |= 1; }
		else     { uart_puts("[0] CPU RAM ok\n"); }
	}

	/* ---- 0b. 帧内容探针【前置】(ZIGUANG v6): 放在最前, 用于判定
	 *   "CPU 是在某个时刻之后才坏"还是"一开始就坏" —— 同时拿到 cam0/cam1
	 *   帧数据对比, 一次出片同时回答两个问题。 */
	/* ---- 1. overlay 四角静态框 ---- */
	if (g_ovl_mask & 0x01u) {          /* bit0: 四角自检 */
	ovl_box(0,  10,  10, 149,  89, 0xF800u);
	ovl_box(1, 490,  10, 629,  89, 0x07E0u);
	ovl_box(2,  10, 620, 149, 699, 0x001Fu);
	ovl_box(3, 490, 620, 629, 699, 0xFFE0u);
	}
	OVL_EN = 0x0Fu;
	uart_puts("[1] overlay test boxes armed, VER=");
	uart_hex(OVL_VER);
	/* v6: 连读 4 次 —— 区分"读回值不稳"与"计数本身错" */
	uart_puts(" /"); uart_hex(OVL_VER);
	uart_puts(" /"); uart_hex(OVL_VER);
	uart_puts(" /"); uart_hex(OVL_VER);
	uart_puts("\n");

	/* ---- 2. DDR 桥自检 (块基址 = 600000, 已在帧区之外) ---- */
	uint32_t pat[64], rr[64];
	for (int i = 0; i < 64; i++) pat[i] = 0xA5000000u ^ (uint32_t)(i * 0x01010101u);
	if (ddrb_write(SELFTEST_BLK, pat, 8) != 0) {
		uart_puts("[2] DDR write TIMEOUT\n"); fails |= 2;
	} else if (ddrb_read(SELFTEST_BLK, rr, 8) != 0) {
		uart_puts("[2] DDR read TIMEOUT\n"); fails |= 2;
	} else {
		uint32_t m = 0;
		for (int i = 0; i < 64; i++) if (rr[i] != pat[i]) m++;
		uart_puts("[2] ddr blk=");  uart_dec(SELFTEST_BLK);
		uart_puts(" mismatch=");    uart_dec(m); uart_puts("/64\n");
		if (m) {
			uint32_t badbits = 0;
			for (int i = 0; i < 8; i++) {
				uint32_t x = rr[i] ^ pat[i];
				for (int b = 0; b < 32; b++) if (x & (1u << b)) badbits++;
			}
			uart_puts("[2] bits="); uart_dec(badbits);
			uart_puts(" wr="); uart_hex(pat[0]); uart_puts(" "); uart_hex(pat[1]);
			uart_puts(" rd="); uart_hex(rr[0]);  uart_puts(" "); uart_hex(rr[1]);
			uart_puts(" rd4="); uart_hex(rr[4]); uart_puts(" "); uart_hex(rr[5]);
			uart_puts("\n");

			/* ---- 2b. 块级错位探针: 直接量出位移了几块 ---- */
			{
				uint32_t fnd = 0u, idx = blk_probe(rr, &fnd);
				if (idx == 0xFEu)      uart_puts("[2b] blk probe write TIMEOUT\n");
				else if (idx == 0xFDu) uart_puts("[2b] blk probe read TIMEOUT\n");
				else if (idx == 0xFFu) uart_puts("[2b] blk probe 8/8 ok\n");
				else {
					uart_puts("[2b] blk probe first bad word="); uart_dec(idx);
					uart_puts(" got="); uart_hex(fnd);
					uart_puts(" (tag="); uart_dec(fnd & 0xFu); uart_puts(")\n");
				}
			}

			/* ---- 2c. 邻域探测: 判断自检区是否被外部写入 ---- */
			ddrb_read(SELFTEST_BLK - 8, rd, 1);
			uart_puts("[2] pre[-8]=");  uart_hex(rd[0]); uart_puts("\n");
			ddrb_read(SELFTEST_BLK + 8, rd, 1);
			uart_puts("[2] post[+8]="); uart_hex(rd[0]); uart_puts("\n");
			ddrb_read(SELFTEST_BLK + 4, rd, 1);
			uart_puts("[2] mid[+4]=");  uart_hex(rd[0]); uart_puts("\n");
			fails |= 2;
		} else { uart_puts("[2] DDR bridge rw selftest PASS\n"); }
	}

#if FASTBOOT
	/* 快速启动: 跳过后续重探针, 直接返回让主循环尽早跑起来 */
	return fails;
#endif

	/* ---- 2e. 帧内容探针【v6 前置】cam0 vs cam1 ----
	 * 放在 [2] 之后: DDR 已被 [2] 证实可用; 且早于 [5] (CPU 出错点)。
	 * cam0(基址 0) 与 cam1(基址 230400) 各打一份缩略图+统计, 直接对比:
	 *   若两份内容互为取反/颜色不同 => 反相在【数据侧】(cam1 传感器/配置);
	 *   若两份统计一致而只有屏幕反相 => 才回到显示侧。
	 */
	/* ---- 7. 未对齐访问模拟自检 (v12) -------------------------------------
	uart_puts("[7] start\n");
	 * 目的: 把「模拟器取数是否有错」与「CoreMark 端口的 printf 是否有错」分开。
	 * 用 demo 固件自己的 UART (已验证干净的打印路径) 输出每个宽度的错误数。
	 * 期望 8 个 0; 非 0 的那一位就是模拟器对应宽度的 bug。 */
	{
		static uint8_t mb[32] __attribute__((aligned(8)));
		uint32_t e[8] = {0u,0u,0u,0u,0u,0u,0u,0u};
		uint32_t off, k, exp, exp1, got, pat;
		static uint32_t f7_off[5], f7_exp[5], f7_got[5];   /* v13: 首个失配明细 */
		for (k = 0; k < 32u; k++) mb[k] = (uint8_t)(0x11u * (k + 1u));
		for (off = 1; off <= 3u; off++) {
			/* v13: 每次比对都留"首个失配"记录 (off/exp/got), 不再只数次数 */
			exp = (uint32_t)mb[off] | ((uint32_t)mb[off+1]<<8)
			    | ((uint32_t)mb[off+2]<<16) | ((uint32_t)mb[off+3]<<24);
			got = asm_lw(&mb[off]);
			if (got != exp) { e[0]++; if (e[0]==1u) { f7_off[0]=off; f7_exp[0]=exp; f7_got[0]=got; } }
			exp1 = (uint32_t)mb[off] | ((uint32_t)mb[off+1]<<8);
			got = asm_lhu(&mb[off]);
			if (got != exp1) { e[2]++; if (e[2]==1u) { f7_off[2]=off; f7_exp[2]=exp1; f7_got[2]=got; } }
			exp = (uint32_t)(int32_t)(int16_t)exp1;
			got = asm_lh(&mb[off]);
			if (got != exp) { e[1]++; if (e[1]==1u) { f7_off[1]=off; f7_exp[1]=exp; f7_got[1]=got; } }
			got = asm_lbu(&mb[off]);
			if (got != (uint32_t)mb[off]) { e[4]++; if (e[4]==1u) { f7_off[4]=off; f7_exp[4]=mb[off]; f7_got[4]=got; } }
			got = asm_lb(&mb[off]);
			if (got != (uint32_t)(int32_t)(int8_t)mb[off]) { e[3]++; if (e[3]==1u) { f7_off[3]=off; f7_exp[3]=(uint32_t)(int32_t)(int8_t)mb[off]; f7_got[3]=got; } }
		}
		pat = 0xA5B6C7D8u;
		for (off = 1; off <= 3u; off++) {
			for (k = 0; k < 32u; k++) mb[k] = 0u;
			asm_sw(&mb[off], pat);
			for (k = 0; k < 4u; k++)
				if (mb[off+k] != (uint8_t)(pat >> (8u*k))) e[5]++;
			for (k = 0; k < 32u; k++) mb[k] = 0u;
			asm_sh(&mb[off], 0x1234u);
			if (mb[off] != 0x34u || mb[off+1] != 0x12u) e[6]++;
			for (k = 0; k < 32u; k++) mb[k] = 0u;
			asm_sb(&mb[off], 0x5Au);
			if (mb[off] != 0x5Au) e[7]++;
		}
		uart_puts("[7] misaligned emu err(lw,lh,lhu,lb,lbu,sw,sh,sb)=");
		for (k = 0; k < 8u; k++) { uart_dec(e[k]); if (k < 7u) uart_putc(','); }
		uart_puts("  (期望 0,0,0,0,0,0,0,0)\n");
		for (k = 0; k < 5u; k++) {            /* v13: 打印首个失配明细 */
			if (!e[k]) continue;
			uart_puts("[7] first bad k="); uart_dec(k);
			uart_puts(" off=");  uart_dec(f7_off[k]);
			uart_puts(" exp=");  uart_hex(f7_exp[k]);
			uart_puts(" got=");  uart_hex(f7_got[k]);
			uart_puts("\n");
		}
	}

	/* ---- 8. Y3T 引擎端到端自检 (v13) ------------------------------------
	 * 硬件级 1×1 conv: y3tram CPU 口读写 + 引擎 job + 输出按 int16 精确比对。
	 * 仿真(tb_soc)与板上是同一段代码、同一个期望值 ⇒ 结果可直接互相印证。 */
	if (y3t_selftest(uart_puts, uart_dec, uart_hex)) fails |= 0x80000000u;

	/* ---- 9. 性能计数器 (v14) --------------------------------------------
	 * 从机 @0x4000_7000 (借 SEL_ERR 槽二次译码)。此时已完成 [0]~[8] 全部工作,
	 * 读到的是一段真实负载的统计: I$/D$ 命中率、分支预测错误率、stall 比例、IPC。
	 * 仅供报告取数, 不计入 fails (从机缺失时只打印一行 absent)。
	 * 注意: 写 CTRL 会清零, 所以 perf_dump 内部是"先读完全部计数器再停表"。 */
	(void)perf_dump(uart_puts, uart_dec, uart_hex);

	/* ---- 10. 在线下装通道自检 (v16): arena 写 -> fence.i -> 跳转执行 ----
	 * 目的: 在没有 PC 的情况下，把「0x0C 写 arena + 0x0D fence.i 后跳转」这条链路
	 *   用仿真/板级都能跑的确定性用例验证掉。
	 * 手法: 往 BOOT arena 起点手工塞 4 条 RV32I 指令，跳过去执行，
	 *   它把 42 写到 0x1000_5000；回来后读该地址校验。
	 *   blob 内容：addi a0,x0,42 / lui a1,0x10005 / sw a0,0(a1) / ret
	 * 注意: 这段代码在 BOOT(可缓存) 上，写完必须 fence.i 才能被执行到。
	 *
	 * ZIGUANG (2026-10-18 19:25) —— 【P0: 这里原本写 0x4000, 会踩掉固件自己】
	 *   本段是"绕过 arena_ok 直接写内存"的, 所以 uart_cmd.c 的窗口保护管不到它。
	 *   v24 的 .text/.rodata 已经长到 0x489B, 0x4000 落在 .rodata 内部的 poff.4
	 *   (池化偏移表, 0x3FEC..0x400C) 上 —— 每跑一次 [10] 就把这张表的后 16 字节
	 *   写成 4 条 RISC-V 指令, 整网推理结果随之出错。
	 *   arena 起点已统一抬到 0x8000, 与 uart_cmd.c / flash.py / arena.ld 保持一致。 */
	{
		#define ARENA_TEST_ADDR  0x00008000u
		#define ARENA_TEST_SCR   0x10005000u
		static const uint32_t blob[4] = {
			0x02A00513u,   /* addi a0, x0, 42          */
			0x100055B7u,   /* lui  a1, 0x10005         */
			0x00A5A023u,   /* sw   a0, 0(a1)           */
			0x00008067u    /* ret  (jalr x0, 0(ra))    */
		};
		volatile uint32_t *scr = (volatile uint32_t *)ARENA_TEST_SCR;
		uint32_t k;
		uint32_t okk;
		*scr = 0u;
		for (k = 0; k < 4u; k++)
			*(volatile uint32_t *)(ARENA_TEST_ADDR + k * 4u) = blob[k];
		__asm__ volatile ("fence.i" ::: "memory");
		((void (*)(void))ARENA_TEST_ADDR)();
		okk = (*scr == 42u) ? 1u : 0u;
		uart_puts("[10] online-load: arena wr + fence.i + jump = ");
		uart_puts(okk ? "PASS" : "FAIL");
		uart_puts("  (scratch="); uart_hex(*scr); uart_puts(", 期望 0000002a)\n");

		/* 再用【真实的 0x0C/0x0D 命令处理函数】跑一遍 (含 arena 拒绝测试),
		 * 保证 PC 侧 flash.py 走的那条协议路径也被覆盖。 */
		{
			extern int uart_cmd_selftest(void);
			uint32_t cb = (uint32_t)uart_cmd_selftest();
			uart_puts("[10] cmd-path (0x0C LOAD / 0x0D EXEC / 0x0E DDR2TCM / 0x0B WLOAD): ");
			uart_puts(cb == 0u ? "PASS" : "FAIL");
			uart_puts("  (bad="); uart_hex(cb);
			if (cb & 0x10u) {
				/* v19: 0x0B 段失败时给出子码 —— 1=清零写超时 2=回读超时 3=数据失配 */
				uart_puts(", 0B subcode="); uart_dec((cb >> 8) & 0xFFu);
			}
			if (cb & 0x20u) {
				/* 2026-10-18: 0x0B 大包段 (64 块/2048B) —— 4=清零写超时 5=回读超时 6=数据失配 */
				uart_puts(", 0B-bulk subcode="); uart_dec((cb >> 8) & 0xFFu);
			}
			uart_puts(")\n");
			if (cb) fails |= 0x40000000u;
		}
		if (!okk) fails |= 0x80000000u;
	}

	/* ---- 3. 相机帧序递增 ---- */
	uint32_t f0 = OVL_FRAME, f1 = f0;
	uint64_t t0 = mtime();
	/* ZIGUANG v4: 原窗口仅 3s —— 实测 detect 从 t≈7s 起才爬升 (相机在 ~7s 才开始
	   出帧), 3s 窗口必然报【假 STUCK】。窗口放宽到 15s, 并打印实际等待时长。 */
	while (f1 == f0 && mtime() - t0 < 54054000ull * 15ull)
		f1 = OVL_FRAME;
	uint32_t wait_s = (uint32_t)((mtime() - t0) / 54054000ull);
	if (f1 == f0) {
		uart_puts("[3] camera frame STUCK (waited ");
		uart_dec(wait_s); uart_puts("s)\n"); fails |= 4;
	} else {
		uart_puts("[3] camera streaming ");
		uart_hex(f0); uart_puts("->"); uart_hex(f1);
		uart_puts(" after "); uart_dec(wait_s); uart_puts("s\n");
	}

	/* ---- 3b. 帧序速率 (ZIGUANG v4): 1s 内 cam0 / cam1 帧序号各自变化次数 ----
	 * OVL_FRAME[1:0]=cam0 帧序, [3:2]=cam1 帧序。cam0 正常应 ≈ 相机帧率(如 30);
	 * 若明显偏高(如 ~110) => cam_vsync 上有毛刺(多次上升沿), 会连带把主循环
	 * detect 速率放大 4 倍 —— 这正好能解释 detect≈28/s 与"槽 4 帧轮转"的矛盾。
	 * cam1=0 => 右目无 vsync (与 LED2 常亮一致)。 */
	/* ZIGUANG v15: 原窗口只有 1 s —— 实测 cam1 比 cam0 启动晚 (1 s 内常读到 0,
	 * 而 [6b] 显示 cam1 槽内已有数据)。改为 1 s + 3 s 双采样: 1 s 给常规速率,
	 * 3 s 兜住启动慢的 cam1, 免得每次都要人工解释一遍。 */
	{
		uint32_t v0 = OVL_FRAME, p0 = v0 & 3u, p1 = (v0 >> 2) & 3u;
		uint32_t n0 = 0, n1 = 0, s0 = 0, s1 = 0, got1 = 0;
		uint64_t tt0 = mtime();
		for (;;) {
			uint64_t el = mtime() - tt0;
			if (el >= 3ull * 54054000ull) break;
			/* v20: 这 3s 里也要收命令 —— 否则上电后头 3 秒 PC 发的命令一律不理,
			 * 而 RX FIFO 只有 8 深, 连发两帧就丢。 */
			uart_cmd_poll();
			uint32_t c0 = OVL_FRAME;
			uint32_t i0 = c0 & 3u, i1 = (c0 >> 2) & 3u;
			if (i0 != p0) { n0++; p0 = i0; }
			if (i1 != p1) { n1++; p1 = i1; }
			if (!got1 && el >= 54054000ull) {
				got1 = 1u; s0 = n0; s1 = n1;
				uart_puts("[3b] /1s: cam0="); uart_dec(s0);
				uart_puts(" cam1=");       uart_dec(s1); uart_puts("\n");
			}
		}
		uart_puts("[3b] /3s: cam0="); uart_dec(n0);
		uart_puts(" cam1=");         uart_dec(n1);
		uart_puts("  (cam0 约 3x 帧率; cam1 仍为 0 才判右目无 vsync)\n");
	}

	/* ---- 3c. 帧内容探针【v9: 挪到相机出流之后】cam0 vs cam1 ----
	 * v8 的教训: [6] 放在 [2] 之后时相机还没写帧(slot0 仍是 55555555),
	 * 缩略图采到的是上电残留, 判不了反相。挪到 [3b] 之后, 相机已确认出流。
	 * cam0(基址 0) 与 cam1(基址 230400) 各打一份缩略图+统计, 直接对比:
	 *   若两份内容互为取反/颜色不同 => 反相在【数据侧】(cam1 传感器/配置);
	 *   若两份统计一致而只有屏幕反相 => 才回到显示侧。
	 */
	frame_probe(0u);        /* cam0 帧槽基址 */
	frame_probe(230400u);   /* cam1 帧槽基址 */

	/* ---- 4. 相机帧数据抽查 (cam0 帧槽 0 首块) ----
	 * 注意: 0x55555555 是 DDR3 上电初值(0101..), 而 nz 判定为 (v!=0 && v!=0xFFFFFFFF)
	 * => 初值会被计入 nz, 所以 "head=55555555 + nonzero=64/64" 的真实含义是
	 * 【该槽从未被写过】, 不是"有数据"。 */
	if (ddrb_read(CAM0_FRAME0_BLK, rr, 8) != 0) {
		uart_puts("[4] frame read TIMEOUT\n"); fails |= 8;
	} else {
		uint32_t nz = 0, init_word = 0;
		for (int i = 0; i < 64; i++) {
			if (rr[i] && rr[i] != 0xFFFFFFFFu) nz++;
			if (rr[i] == 0x55555555u) init_word++;
		}
		uart_puts("[4] frame nonzero=");
		uart_dec(nz); uart_puts("/64 head=");
		uart_hex(rr[0]); uart_puts(" initword=");
		uart_dec(init_word);
		if (init_word == 64u) uart_puts(" (== DDR 上电初值, 该槽从未写入)");
		uart_puts("\n");
	}

	/* ---- 4b. 相机写入活性 (ZIGUANG v4): 同址隔 200ms 再读, 变化即相机在写 DDR ----
	 * 静态数据在两次读之间不该变 => 若 changed>0, 说明确有外部(相机)在写该槽;
	 * 与 [2d] 交叉验证: [4b]>0 且 [2d]==0 时 "相机在写" 才成立。 */
	{
		uint32_t d = 0;
		if (ddrb_read(CAM0_FRAME0_BLK, rr, 8) == 0) {
			delay_ms(200);
			if (ddrb_read(CAM0_FRAME0_BLK, pat, 8) == 0)
				for (int i = 0; i < 64; i++) if (pat[i] != rr[i]) d++;
		}
		uart_puts("[4b] cam0 slot0 changed words /200ms=");
		uart_dec(d); uart_puts("/64 (0 => 该槽静态, 相机未写)\n");
	}

	/* ---- 2d. 读确定性 (ZIGUANG v4): SELFTEST_BLK 同址连读 8 次 ----
	 * 【判据】>0 => 桥读【非确定性】, 则 [2] 失配与主循环 detect 爬升都是读抖动假象;
	 *   0 => 桥读确定, 则 [2b] 量到的位移就是真实的地址错位, 必须修 RTL。 */
	{
		uint32_t bad = rd_stab_probe(rr);
		if (bad == 0xFFFFFFFFu) uart_puts("[2d] rd stability TIMEOUT\n");
		else {
			uart_puts("[2d] rd stability: "); uart_dec(bad);
			uart_puts("/448 word diffs over 7 rereads (0 => 桥读确定)\n");
		}
	}

	/* ---- 5. Y3T 在位 ---- */
	{
		uint32_t id = Y3T_ID;
		uart_puts("[5] Y3T ID=");
		uart_hex(id);
		if (id == 0x59335431u) uart_puts(" (ok)\n");
		else {
			/* v6: 第二次打印同一变量 —— 若与第一次不同, 证明 callee-saved
			 * 寄存器(s3)在 uart_hex() 调用过程中被破坏 (ABI 级故障) */
			uart_puts(" (UNEXPECTED) id2=");
			uart_hex(id);
			uart_puts("\n");
			fails |= 16;
		}
	}

	/* ---- 6. 帧内容缩略 (ZIGUANG v5): 读 cam0 帧槽0, 打 32x16 ASCII 亮度缩略图 ----
	 * 桥修好后终于能"看"帧缓冲了。判据:
	 *   - 缩略图出现可辨认的明暗结构(轮廓/渐变/大面积同色) => 帧缓冲里是【图像】
	 *     => 花屏出在显示读取/合成侧;
	 *   - 缩略图整体杂乱无结构, 且 white 占比很高(接近 100%) => 【不是图像】,
	 *     出在相机数据侧(最可能是 SCCB 没配成 RGB565 → 原始 Bayer 被当成 RGB565);
	 *   - rowcorr(相邻采样行亮度接近的比例) 高 => 像图; 接近 55% => 像噪点。
	 *   (v6: 探针已前置到 [0] 之后, 此处不再重复调用。) */

	return fails;
}

int demo_main(void) {
	uint32_t rd[64];
	uint32_t fails = selftest_stages(rd);

	uart_puts(fails ? "BRING-UP FAIL bitmask=" : "BRING-UP PASS, motion detect armed\n");
	if (fails) { uart_hex(fails); uart_puts("\n"); }

	/* ---- 6. 帧同步主循环: ROI 灰度帧差 → 运动检测框 ---- */
	uint32_t prev_f = OVL_FRAME, last_s = 0, tick = 0;
	uint32_t detect_cnt = 0, have_prev = 0;
	extern uint32_t uart_rx_pending(void);
	for (;;) {
		uart_cmd_poll();                       /* PC 命令解析 (非阻塞) */
		/* 【BUG-FIX 2026-10-22 15:25】正在收帧时不做相机/运动检测。
		 *   poll 每轮只排空 64B, 而相机分支含 ddrb_read(数百 µs~ms) ⇒
		 *   长传时 RX FIFO(512) 会满 ⇒ overrun ⇒ 整帧丢弃 ⇒ DDR 保持 0xFF 残留。
		 *   让出主循环给 poll, 上传才能可靠。代价: 上传期间相机掉帧(可接受)。 */
		if (uart_rx_pending()) continue;   /* 帧未收完 OR FIFO 还有待处理字节 */
		/* v37: 板上自动循环（0x15 AUTO_RUN 打开）——
		 *   相机帧→PP 预处理→整网→检测解码→OVL 画框。打开后取代运动检测分支;
		 *   命令解析在循环顶部已做过, 期间 PC 仍可发 0x15 关闭。 */
		{
			extern uint32_t g_auto_en;
			extern void y3t_auto_step(void);
			if (g_auto_en) { y3t_auto_step(); continue; }
		}
		/* v20: RX 诊断 —— 一收到新字节就报一次。
		 * 判据: rx 不增长 ⇒ 字节没进 CPU (引脚/波特率/RX FIFO);
		 *       rx 增长但 PC 端无响应 ⇒ 帧校验/协议解析问题。
		 * 放在循环顶部(而不是空闲分支): 仿真里每 1ms 一帧、CPU 一直不空闲,
		 *   放空闲分支就永远打不出来。 */
		{
			extern void uart_cmd_rxstat(uint32_t *cnt, uint8_t *last);
			static uint32_t rxp = 0;
			uint32_t rxc = 0; uint8_t rxl = 0;
			uart_cmd_rxstat(&rxc, &rxl);
			/* 节流: 每 16 个字节报一次 (曾经因为每个字节打一行, 日志被刷爆)
			 * ZIGUANG (2026-10-19 18:45, RX-PRINT-THROTTLE) —— 【P0: 打印本身杀死了长帧】
			 *   uart_puts 是逐字节忙等, "[rx] n=..." 约 24 字符 = 2.1ms 阻塞。
			 *   每 16 字节触发一次 ⇒ 接收上限 16B/(16/11520s + 2.1ms) ≈ 4.5KB/s
			 *   < 线速 11.5KB/s ⇒ 512B RX FIFO 在长帧 ~74ms 处必然溢出丢字节,
			 *   帧校验失败被丢 ⇒ 一切 >40B 的帧 (0x0B 权重/IN) 全部落地失败。
			 *   实测: 32B 帧 PASS, 256/512/1024/2048/4096B 帧 100% FAIL。
			 *   改为每 64K 字节报一次 (对 3MB 上传也只打 45 行), 诊断价值保留。 */
			if (rxc - rxp >= 65536u || rxc < rxp) {
				rxp = rxc;
				uart_puts("[rx] n="); uart_dec(rxc);
				uart_puts(" last="); uart_hex((uint32_t)rxl);
				uart_puts("\n");
			}
		}
		uint32_t f = OVL_FRAME;
		if (f == prev_f) {
			uint64_t now = mtime();
			uint32_t s = (uint32_t)(now / 54054000u);
			if (s != last_s) {
				last_s = s;
				uart_puts("t="); uart_dec(s);
				uart_puts("s detect="); uart_dec(detect_cnt);
				/* v20: 心跳里带 RX 统计 —— 发命令没响应时一眼分辨
				 * "字节没进来" 与 "进来了但解析错"。 */
				{
					extern void uart_cmd_rxstat(uint32_t *cnt, uint8_t *last);
					uint32_t rxc = 0; uint8_t rxl = 0;
					uart_cmd_rxstat(&rxc, &rxl);
					uart_puts(" rx="); uart_dec(rxc);
					uart_puts(" last="); uart_hex((uint32_t)rxl);
				}
				uart_puts("\n");
				GPIO_DATA_OUT = (GPIO_DATA_OUT & ~0x100u) | ((tick++ & 1u) << 8);
			}
			continue;
		}
		prev_f = f;

		/* --- 逐行读 ROI 并做帧差 --- */
		uint32_t xmin = ROI_W, ymin = ROI_H, xmax = 0, ymax = 0, changed = 0;
		for (uint32_t r = 0; r < ROI_H; r++) {
			uint32_t blk = (ROI_Y0 + r) * ROI_ROW_BLKS + ROI_X0 / 16u;
			if (ddrb_read(blk, rd, 8) != 0) break;
			for (uint32_t c = 0; c < ROI_W; c += 2) {
				uint32_t w = rd[c >> 1];
				uint32_t p0 = w & 0xFFFFu, p1 = w >> 16;
				uint32_t g0 = (((p0 >> 11) & 31u) * 4 + ((p0 >> 5) & 63u) * 8 + (p0 & 31u) * 4) >> 4;
				uint32_t g1 = (((p1 >> 11) & 31u) * 4 + ((p1 >> 5) & 63u) * 8 + (p1 & 31u) * 4) >> 4;
				uint8_t *pp = &prev_gray[r * ROI_W + c];
				if (have_prev) {
					uint32_t d0 = g0 > pp[0] ? g0 - pp[0] : pp[0] - g0;
					uint32_t d1 = g1 > pp[1] ? g1 - pp[1] : pp[1] - g1;
					if (d0 > 24u || d1 > 24u) {
						if (c < xmin)     xmin = c;
						if (c + 1 < xmin) xmin = c + 1;
						if (c + 1 > xmax) xmax = c + 1;
						if (r < ymin) ymin = r;
						if (r > ymax) ymax = r;
						changed++;
					}
				}
				pp[0] = (uint8_t)g0;
				pp[1] = (uint8_t)g1;
			}
		}
		have_prev = 1u;

		/* --- 检出运动 (屏坐标 = 左目 640 宽内 1:1) → 画框 4 --- */
		if ((g_ovl_mask & 0x02u) && changed > 40u && xmin <= xmax) {   /* bit1: 运动框 */
			ovl_box(4,
				ROI_X0 + xmin - 3u, ROI_Y0 + ymin - 3u,
				ROI_X0 + xmax + 3u, ROI_Y0 + ymax + 3u,
				0x07E0u);
			OVL_EN |= 0x10u;
			detect_cnt++;
		} else {
			OVL_EN &= ~0x10u;
		}
	}
	return 0;
}

void trap_dispatch(uint32_t *f) {
	/* 先尝试"未对齐访问模拟" (见 sw/trap_emu.h): 处理成功就直接返回继续执行 */
	/* 帧里的 mcause/mepc 由这里填写: 刚进异常时汇编侧 csrr 可能读到未提交值 */
	{ uint32_t m_, p_; __asm__ volatile("csrr %0, mcause":"=r"(m_));
	  __asm__ volatile("csrr %0, mepc":"=r"(p_)); f[31]=m_; f[32]=p_; }

	/* ================= v65: 中断分支 (赛题高阶②"完整异常/中断机制") =================
	 * mcause 最高位 = 1 表示中断 (RISC-V 规范)。本 SoC 只有外部中断一路接进 meip
	 * (ominicore_extirq 汇聚: bit0 uart / bit1 gpio / bit2 y3t_done / bit3 相机帧同步),
	 * 所以这里只需按"外部中断"处理:
	 *   ① 读 EXTIRQ.PENDING 记下是哪一路 (诊断价值: 确认真的进了 GPIO 分支);
	 *   ② 写 GPIO.IRQ_STATUS 做 W1C 清源 —— 规格要求"源状态在对应外设中清除";
	 *   ③ 记数/拉高 gpio_out[6] 指示, 然后 return (汇编侧 restore 后 mret 回原地)。
	 * 注意: 中断不需要 mepc += 4 (被中断的指令还没执行, 要重执行)。 */
	if ((f[31] & 0x80000000u) != 0u) {
		uint32_t pend = EXTIRQ_PENDING;
		g_irq_count++;
		g_irq_last_cause = f[31];
		g_irq_last_pend  = pend;
		GPIO_DATA_OUT |= GPIOO_ISR;                 /* 示波器观测点 */

		if (f[31] == 0x80000003u) {                 /* machine software interrupt (MSIP) */
			/* v66: 0x18 src=0 的确定性源改用 MSIP —— 板上 mtimecmp 坏 (读恒 0/
			 *   写不粘, mtip 恒 1, 开 MTIE 即风暴)。MSIP 是电平, ISR 必须写 0 清源,
			 *   否则 mret 后立刻重进 (风暴)。 */
			g_timer_evt++;
			TIMER_MSIP = 0u;
		}
		if (f[31] == 0x80000007u) {                 /* machine timer (MTIP) */
			/* MTIP 是电平, 必须重装 mtimecmp 才会回落; 否则一出中断立刻又进。
			 * v66: 必须用 64 位安全重装 (timer_rearm) —— 旧写法 HI=0 只在
			 *   上电 79.4s 内成立, 之后 mtimecmp < mtime => MTIP 永挂 => 风暴。 */
			g_timer_evt++;
			timer_rearm();
		}
		if (pend & 0x00000002u) {                   /* bit1 = GPIO */
			g_gpio_evt++;
			GPIO_IRQ_STATUS = 0x0000FFFFu;          /* W1C 清源 */
		}
		GPIO_DATA_OUT &= ~GPIOO_ISR;
		return;
	}

	/* v66: 访问故障 (AHB ERROR -> mcause 5=load / 7=store access fault)。
	 *   trap_emu 只处理非对齐 (4/6); 访问故障**不能重执行** (重执行还是错)。
	 *   策略: 按指令长度 (RV32/C) 跳过肇事指令继续跑 —— 防止一个坏地址探针把
	 *   整个固件停死。2026-10-27 实测: 越界读 0x40002020 (AHB ERROR) 走到这里
	 *   无人处理 -> TRAP 停机, 板子"挂死"。被跳过的 load 其 rd 保持旧值 (调用者
	 *   自担), 正常业务不应产生访问故障, 这只是保命兜底。 */
	if (f[31] == 5u || f[31] == 7u) {
		static uint32_t naf;
		uint32_t pc = f[32];
		uint32_t lo = ld16u(pc);
		uint32_t skip = ((lo & 3u) == 3u) ? 4u : 2u;
		f[32] = pc + skip;
		naf++;
		if (naf <= 4u || (naf & 0xFFu) == 0u) {
			uart_puts("[AF] access-fault #"); uart_dec(naf);
			uart_puts(" mcause="); uart_hex(f[31]);
			uart_puts(" pc="); uart_hex(pc);
			uart_puts(" -> skipped\n");
		}
		return;
	}

	if (trap_emu(f)) return;
	{
		static uint32_t ntr;
		uint32_t mv_;
		__asm__ volatile("csrr %0, mtval":"=r"(mv_));
		ntr++;
		uart_puts("TRAP#"); uart_dec(ntr);
		uart_puts(" mcause="); uart_hex(f[31]);
		uart_puts(" mepc=");   uart_hex(f[32]);
		uart_puts(" mtval=");  uart_hex(mv_);
		uart_puts(" ra=");     uart_hex(f[0]);
		uart_puts(" sp=");     uart_hex(f[1]);
		uart_puts("\n");
		/* v40: 全寄存器转储 (f[i-1]=x_i, i=1..31；f[0]=ra f[1]=sp f[2]=gp
		 *   f[3]=tp f[4]=t0 f[5]=t1 f[6]=t2 f[7]=s0 f[8]=s1 f[9,10]=a0,a1 ...) */
		for (uint32_t r = 1u; r <= 31u; r++) {
			uart_puts("x"); uart_dec(r); uart_puts("=");
			uart_hex(f[r - 1]);
			uart_puts((r & 3u) == 0u ? "\n" : " ");
		}
		uart_puts("\n");
	}
	uart_puts("TRAP mcause=");
	uart_hex(f[31]);
	uart_puts(" mepc=");
	uart_hex(f[32]);
	uart_puts(" - halted\n");
	for (;;) ;
}
