/*===========================================================================
 *  perf_probe.h — 性能计数器读取与 CPI 分解打印  (v14 起启用)
 *
 *  ★ 现状 (2026-10-17): 【尚未接入 demo.c】。
 *    原因: perf 从机 (source/soc/core/ominicore_perf.v) 已按 AHB5 从机风格重写
 *    并通过单元测试 (sim/run_perf.sh -> RESULT: PASS), 但【还没有挂到 SoC 总线上】。
 *    在 boardsoc_top 里挂好之前 include 本文件去读 0x4000_7000 会拿到总线错误。
 *
 *    挂载步骤见 BITSTREAM_V13_CHECKLIST.md §5.1。挂好 + 出片后:
 *      ① #include "perf_probe.h"
 *      ② 在 [8] 之后加:
 *             if (perf_dump(uart_puts, uart_dec, uart_hex)) fails |= 0x40000000u;
 *      ③ 版本号 v14
 *
 *  寄存器表 (见 ominicore_perf.v 头注释):
 *    0x0 ID  0x1 CTRL  0x2 IC_REQ  0x3 IC_HIT  0x4 IC_MISS  0x5 IC_ERR
 *    0x6 DC_REQ  0x7 DC_HIT  0x8 DC_MISS  0x9 DC_WB  0xA DC_BYPASS
 *    0xB BR_ALLOC  0xC BR_CLEAR   0xD CPU_STALL  0xE CPU_CYCLE  0xF SNAP
 *
 *  ★ 两条硬约束:
 *    ① **写 CTRL 会清零全部计数器**（bit0 同时是使能）；
 *    ② **读 0x2..0xE 返回的是快照值**, 而 13 个寄存器逐个读要 20+ 拍, 期间计数仍在跑
 *       ⇒ 必须先写 **SNAP(0xF)** 做原子锁存, 否则 hit+miss 会比 req 多出几个
 *       （v14 仿真实测多 2~4 个, 会打出假 WARN）。本文件 perf_dump 已内置这一步。
 *=========================================================================*/
#ifndef PERF_PROBE_H
#define PERF_PROBE_H

/* 与 boardsoc_top 的 perf 二次译码窗口必须一致 (0x4000_7000, 4KB) */
#define PERF_BASE 0x40007000u

#define PERF_REG(i) (*(volatile uint32_t *)(PERF_BASE + ((uint32_t)(i) << 2)))

#define PERF_ID         PERF_REG(0)
#define PERF_CTRL       PERF_REG(1)
#define PERF_IC_REQ     PERF_REG(2)
#define PERF_IC_HIT     PERF_REG(3)
#define PERF_IC_MISS    PERF_REG(4)
#define PERF_IC_ERR     PERF_REG(5)
#define PERF_DC_REQ     PERF_REG(6)
#define PERF_DC_HIT     PERF_REG(7)
#define PERF_DC_MISS    PERF_REG(8)
#define PERF_DC_WB      PERF_REG(9)
#define PERF_DC_BYPASS  PERF_REG(10)
#define PERF_BR_ALLOC   PERF_REG(11)
#define PERF_BR_CLEAR   PERF_REG(12)   /* ★btb_clear && !mispredict; 不是误预测! */
#define PERF_CPU_STALL  PERF_REG(13)
#define PERF_CPU_CYCLE  PERF_REG(14)
#define PERF_SNAP       PERF_REG(15)   /* 写任意值 = 原子锁存 0x2..0xE */

/* ---- 标准 CSR 计数器 (2026-10-17 板上复盘新增) -------------------------
 * 为什么必须加: [9] 原来那个 `ipc_milli` 是用 **I$ 请求数 / 周期数** 算的,
 *   它叫 "IPC" 是错的 —— 板上实测 71,874/2,617,597 = 27‰, 而真实 IPC 应在 0.5 左右
 *   (仿真同一构建的 I$ req=1.33M, 板上只有 71,874 ⇒ 该计数器在板上少计 18 倍,
 *    原因待查, 不能再作为评分/报告口径)。
 * 真实 IPC 用标准 CSR:  mcycle(0xB00) / minstret(0xB02),
 *   与 CoreMark 端口完全同源, 是本项目唯一可信的"流水线效率"数据源。
 * 注意: 复位后 mcountinhibit_cy/ir = 1 (csr 省电默认), 必须先写 0x320 清零才会计数,
 *   否则两个计数器恒 0 —— 见 ominicore_csr.v:704。 */
static void perf_csr_release(void) { __asm__ volatile ("csrw 0x320, zero"); }
static uint32_t perf_csr_mcycle(void)   { uint32_t v; __asm__ volatile ("csrr %0, mcycle"   : "=r"(v)); return v; }
static uint32_t perf_csr_minstret(void) { uint32_t v; __asm__ volatile ("csrr %0, minstret" : "=r"(v)); return v; }

typedef void (*perf_puts_t)(const char *);
typedef void (*perf_dec_t)(uint32_t);
typedef void (*perf_hex_t)(uint32_t);

/* 比例 (千分比, 无浮点)。d==0 时返回 0xFFFFFFFF 表示"无样本" */
static uint32_t perf_permille(uint32_t n, uint32_t d)
{
	if (d == 0u) return 0xFFFFFFFFu;
	return (n / d) * 1000u + ((n % d) * 1000u) / d;   /* 先除后乘, 防溢出 */
}

/* perf 从机是否挂在总线上: 0 = 正常, 1 = 不存在 */
static uint32_t perf_present(void)
{
	return (PERF_ID == 0x5A6C3001u) ? 0u : 1u;
}

/* 清零并开始计数 —— 必须在被测负载开始之前调用 */
static void perf_start(void)
{
	PERF_CTRL = 1u;     /* bit0=使能; 任意写同时清零全部计数器 */
}

/* 停表: 写 CTRL=0（**会清零**, 只在确定不再需要数据时调用） */
static void perf_stop(void)
{
	PERF_CTRL = 0u;
}

/* 打印快照。返回 0 = 成功; 1 = perf 从机不存在。
 * 顺序: 先写 SNAP 做**原子锁存**, 再逐个读 —— 这样 13 个值是同一时刻的样本。
 * 若直接读而不锁存, 读完要 20+ 拍, 期间计数仍在跑, hit+miss 会比 req 多出几个。 */
static uint32_t perf_dump(perf_puts_t p_puts, perf_dec_t p_dec, perf_hex_t p_hex)
{
	uint32_t ic_req, ic_hit, ic_miss, ic_err;
	uint32_t dc_req, dc_hit, dc_miss, dc_wb, dc_bypass;
	uint32_t br_alloc, br_mis, stall, cyc;
	uint32_t m;

	if (perf_present()) {
		p_puts("[9] PERF absent (PERF_ID != 0x5A6C3001)\n");
		return 1u;
	}

	PERF_SNAP = 1u;        /* 原子锁存全部计数器 (可多次取, 每次覆盖) */

	ic_req = PERF_IC_REQ;  ic_hit = PERF_IC_HIT;  ic_miss = PERF_IC_MISS;
	ic_err = PERF_IC_ERR;
	dc_req = PERF_DC_REQ;  dc_hit = PERF_DC_HIT;  dc_miss = PERF_DC_MISS;
	dc_wb  = PERF_DC_WB;   dc_bypass = PERF_DC_BYPASS;
	br_alloc = PERF_BR_ALLOC; br_mis = PERF_BR_CLEAR;
	stall  = PERF_CPU_STALL;  cyc = PERF_CPU_CYCLE;
	/* 注意: 这里**不**写 CTRL —— 写 CTRL 会把刚取到的统计清零 */

	p_puts("[9] PERF cyc=");      p_dec(cyc);
	p_puts(" stall=");            p_dec(stall);
	/* 注意: 这不是 IPC! 它是 **I$ 请求率**(每千周期发出多少次取指请求)。
	 * 真实 IPC 在下一行的 CSR 统计里(用 minstret/mcycle)。 */
	p_puts(" ireq_permille=");   p_dec(perf_permille(ic_req, cyc));
	p_puts("\n");

	{
		uint32_t mc = perf_csr_mcycle(), mi = perf_csr_minstret();
		p_puts("[9] CSR mcycle=");  p_dec(mc);
		p_puts(" minstret=");       p_dec(mi);
		p_puts(" ipc_milli=");      p_dec(perf_permille(mi, mc));
		p_puts("  (真实 IPC; 与 CoreMark 端口同源)\n");
	}

	p_puts("[9] I$ req=");        p_dec(ic_req);
	p_puts(" hit=");              p_dec(ic_hit);
	p_puts(" miss=");             p_dec(ic_miss);
	p_puts(" err=");              p_dec(ic_err);
	/* ZIGUANG (2026-10-17): 分母改用 hit+miss —— 即"只看 cacheable 访问"。
	 * 原用 req: req 含 bypass(不可缓存访问), 会稀释命中率。
	 * 实测反例: dc_req=645,943 里有 639,558 是 bypass, 用 req 算只有 9‰;
	 * 真实命中率是 6,327/(6,327+58) = 991‰。I$ 同理。 */
	p_puts(" hit_permille=");     p_dec(perf_permille(ic_hit, ic_hit + ic_miss));
	p_puts("\n");

	p_puts("[9] D$ req=");        p_dec(dc_req);
	p_puts(" hit=");              p_dec(dc_hit);
	p_puts(" miss=");             p_dec(dc_miss);
	p_puts(" wb=");               p_dec(dc_wb);
	p_puts(" bypass=");           p_dec(dc_bypass);
	p_puts(" hit_permille=");     p_dec(perf_permille(dc_hit, dc_hit + dc_miss));
	p_puts("\n");

	p_puts("[9] BR alloc=");      p_dec(br_alloc);
	p_puts(" clear=");             p_dec(br_mis);      /* ★BR_CLEAR, 不是误预测 */
	p_puts(" stall_permille=");   p_dec(perf_permille(stall, cyc));
	p_puts("\n");

	/* 自洽性检查: 命中+缺失不应超过请求数 (超了说明计数定义有重叠) */
	m = ic_hit + ic_miss;
	if (m > ic_req) {
		p_puts("[9] WARN I$ hit+miss > req (");
		p_dec(m); p_puts(" > "); p_dec(ic_req); p_puts(")\n");
	}
	m = dc_hit + dc_miss + dc_bypass;
	if (m > dc_req) {
		p_puts("[9] WARN D$ hit+miss+bypass > req (");
		p_dec(m); p_puts(" > "); p_dec(dc_req); p_puts(")\n");
	}

	return 0u;
}

#endif /* PERF_PROBE_H */
