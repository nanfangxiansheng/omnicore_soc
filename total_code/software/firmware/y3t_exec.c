/* ============================================================================
 * y3t_exec.c — G6 整网执行器实现（静态 job 表 + 逐层权重流式）
 * ========================================================================== */
#include <stdint.h>
#include "y3t_net_table.h"
#include "y3t_scale_table_blob.h"  /* 对齐版尺度偏移 (pack_scales_aligned.py) */
#include "y3t_exec.h"

/* DDR 桥批量读（定义在 demo.c，非 static）。rom_stream 与补末块都用。 */
extern int ddrb_read(uint32_t blk, uint32_t *dst, uint32_t beats);

/* ---- 寄存器（不 include y3t.h: 那里有非 static 函数定义，两个 .c 会重定义） ---- */
#define E_REG32(a)      (*(volatile uint32_t *)(a))
#define E_CTRL          E_REG32(0x40005000u)   /* W:start  R:{err,done} */
#define E_CFG0          E_REG32(0x40005004u)
#define E_HW            E_REG32(0x40005008u)
#define E_HWO           E_REG32(0x4000500Cu)
#define E_TILE          E_REG32(0x40005010u)
#define E_SH0           E_REG32(0x40005014u)
#define E_MULTY         E_REG32(0x40005018u)
#define E_FLAG          E_REG32(0x4000501Cu)
#define E_IN0           E_REG32(0x4000502Cu)
#define E_IN1           E_REG32(0x40005030u)
#define E_WB            E_REG32(0x40005034u)
#define E_OUT           E_REG32(0x40005038u)
#define E_ROM_ADDR      E_REG32(0x4000503Cu)
#define E_ROM_DATA      E_REG32(0x40005040u)
#define E_CYC_MAC       E_REG32(0x4000504Cu)
#define E_CYC_TOT       E_REG32(0x40005050u)

/* ---- 权重 DMA（0x9000_0000） ---- */
#define E_WLD_SRC       E_REG32(0x90000000u)
#define E_WLD_DST       E_REG32(0x90000004u)
#define E_WLD_LEN       E_REG32(0x90000008u)
#define E_WLD_CTRL      E_REG32(0x9000000Cu)
#define E_WLD_STATUS    E_REG32(0x9000000Cu)
#define E_WLD_DONE      0x1u

#define E_DONE          0x1u
#define E_ERR           0x2u
#define E_MTIME         E_REG32(0x40002000u)   /* mtime 低 32 位 @54.054MHz */

/* 片上权重区顶 = NET_WB_TOP(18432)；激活区从 18432 起（权威表口径） */
#define E_WB_TOP        NET_WB_TOP
#define E_JOB_TIMEOUT   200000000u             /* 单 job 上限（≈3.7s @54MHz，远大于最慢层） */

/* ---- 外部依赖（demo.c 提供，非 static） ---- */
extern int ddrb_read(uint32_t blk, uint32_t *dst, uint32_t beats);

static uint32_t g_rom_blk = 0u;    /* 0 = identity 模式 */

/* ==========================================================================
 * ROM 口
 *
 * 引擎侧 ROM 布局（source/soc/y3t/engine_y3t.v:49,97,98）—— **务必对齐**:
 *     cfg 写地址 0..2047    → multm_rom[0:2047]   （含 group 位: idx={g,ot[5:0],ch}）
 *     cfg 写地址 2048..3071 → bias_rom [0:1023]   （idx={ot[5:0],ch}）
 *
 * ZIGUANG (2026-10-19) —— 【P0 修复: identity 模式把 group 1 的 multM 清零了】
 *   原实现从地址 0 连写 1024 个 1 再写 512 个 0, 意图是 "multM=1, bias=0"。
 *   但地址不会自己跳到 2048 —— 那 512 个 0 实际落在 multm_rom[1024..1535],
 *   而 ng2 层（ng2 = split!=0 && split<cit，即 ic>16 的所有层）在第二组输入时
 *   用 midx={rq_g=1,ot,ch} 读的就是 1024 段 ⇒ **这些层的 group 1 贡献被整体
 *   乘成 0**, 输出严重失真。所谓 "identity 直通" 其实是残缺的。
 *   现按真实布局写: multM 写满 2048 个 1, bias 从 2048 起写 1024 个 0。
 * ======================================================================== */
#define ROM_MULTM_BASE  0u        /* multm_rom 在 cfg 地址空间的起点 */
#define ROM_MULTM_N     2048u     /* multm_rom 深度 (含 group 位) */
#define ROM_BIAS_BASE   2048u     /* bias_rom 在 cfg 地址空间的起点 */
#define ROM_BIAS_N      1024u     /* bias_rom 深度 (64 ot tile × 16 lane) */

void y3t_exec_rom_identity(void)
{
	uint32_t i;
	g_rom_blk = 0u;
	E_ROM_ADDR = ROM_MULTM_BASE;
	for (i = 0; i < ROM_MULTM_N; i++) E_ROM_DATA = 1u;   /* 全部通道组 multM = 1 */
	E_ROM_ADDR = ROM_BIAS_BASE;
	for (i = 0; i < ROM_BIAS_N; i++)  E_ROM_DATA = 0u;   /* 全部通道 bias  = 0 */
	E_MULTY = 1u;                                        /* ×1 */
	E_SH0   = 0u;                                        /* shM0/shM1/shY 全 0 = 直通 */
}

void y3t_exec_rom_from_ddr(uint32_t rom_blk)
{
	g_rom_blk = rom_blk;
	if (rom_blk == 0u) y3t_exec_rom_identity();
}

/* ZIGUANG (2026-10-19): 原 rom_layer_off() 在这里算每层的字偏移
 *   (off += 32*cot)。现改由 y3t_scale_table.h 的 g_scale[layer].w_g0/w_g1/w_bias
 *   直接给出 —— 因为多组层 (L17) 的 group1 段长度与 group0 不同, 用 32*cot 的
 *   固定公式会算错后续所有层的偏移。表由 tools/export_scales.py 生成。 */

/* 从 DDR 装一段 n 个 u32 到 ROM 窗口（起点 addr） */
extern void uart_puts(const char *s);
extern void uart_hex(uint32_t v);
extern void uart_dec(uint32_t v);
static uint8_t g_exec_dbg = 0u;          /* v46: 关闭插桩打印（打印占 ~190ms/帧） */

static void rom_stream(uint32_t ddr_blk, uint32_t addr, uint32_t n)
{
	static uint32_t buf[64];
	/* 【BUG-FIX 2026-10-24】尺度 blob 已重排成"每子段 64 字(256B)对齐"
	 *   （tools/pack_scales_aligned.py + y3t_scale_table_blob.h）。
	 *   DDR 桥的多 beat 读起始必须 256B 对齐，非对齐返回上电残留；
	 *   对齐后顺序读（每次 ≤8 块）天然全对齐。 */
	uint32_t done = 0u;
	if (g_exec_dbg) {
		uart_puts("[RS] a="); uart_hex(addr);
		uart_puts(" n="); uart_dec(n);
		uart_puts(" blk="); uart_hex(ddr_blk);
		uart_puts("\n");
	}
	E_ROM_ADDR = addr & 0xFFFu;
	while (done < n) {
		uint32_t i, nb = (n - done + 7u) / 8u;
		if (nb > 8u) nb = 8u;
		if (ddrb_read(ddr_blk + (done >> 3), buf, nb) != 0) return;
		for (i = 0; i < nb * 8u && done < n; i++) {
			E_ROM_DATA = buf[i];
			done++;
		}
	}
}


/* ==========================================================================
 * 权重搬运
 * ======================================================================== */
static int exec_wload(uint32_t src_blk, uint32_t dst_word, uint32_t words)
{
	uint32_t guard = 0;
	E_WLD_SRC  = src_blk;
	E_WLD_DST  = dst_word;
	E_WLD_LEN  = words;
	E_WLD_CTRL = 1u;
	/* 【BUG-FIX 2026-10-23 ★ NET_RUN 全错的根因】start 后不能立刻 poll done！
	 *   RTL (ominicore_wload_dma.v:705..748): start 只是置 start_pending，
	 *   要等 wfifo 排空才真正起步，**起步那一拍才无条件清 done_flag**；
	 *   W1C (写 bit1) 是死代码 —— 实测写 0x2 后 status 的 done 恒为 1。
	 *   所以上一笔留下的 done=1 会让下面的 poll **第一拍就退出**，
	 *   引擎随即开跑，而权重 DMA 还在往 y3tram 写 ⇒ 整网 29 层全错
	 *   （板上实测 NET_RUN 后 L0=40.92%，PC 路径重跑同一层=100%）。
	 *   修法：先等 done 归 0（证明本笔真的起步了），再等 done=1。
	 *   0x11 (uart_cmd.c) 一直"看起来正常"只是因为 PC 侧毫秒级延迟把 DMA 等完了。 */
	{
		uint32_t w;
		for (w = 0u; w < 100000u; ++w) {
			if (!(E_WLD_STATUS & E_WLD_DONE)) break;   /* 已被本笔 start 清掉 */
		}
	}
	while (!(E_WLD_STATUS & E_WLD_DONE))
		if (++guard > 200000000u) return -1;
	/* 【BUG-FIX 2026-10-23 ★★★ NET_RUN 全错的真正根因】
	 *   原来这里有 `E_WLD_STATUS = E_WLD_DONE;  /* W1C *​/` —— 但 wload_dma 的
	 *   CTRL 是**写 0x1 = start 脉冲**（RTL: start_pulse = ctrl_wr && hwdata[0]，
	 *   W1C 是 bit1=0x2 且已废弃）！这一行等于**把刚完成的传输原样再启动一遍**：
	 *   幽灵 DMA 与随后的引擎 job 并发跑，eng 口 mux 在写 burst 中途被切走
	 *   ⇒ 权重数据静默打烂（每通道不同比例畸变 + 大片 0，确定性复现）。
	 *   这解释了全部现象：v36/v37 同样错、0x11/0x05 路径（无此行）全对、
	 *   arena 调 y3t_exec_layer 直接挂死（mux 死锁）。
	 *   done_flag 无需清零 —— 下一次 start 会无条件清它（RTL :748）。 */

	/* 【BUG-FIX 2026-10-23】wload_dma 每笔任务**只写 words-1 个块**，末块不写，
	 *   目标处保留上一次搬运的残留。而每层权重都是**顶部对齐**放置
	 *   (wb = NET_WB_TOP - wsub = 32768 - ww)，所以每层的末块恒为 32767 ——
	 *   也就是「上一层留下的权重」⇒ 29 层全部用错一个 32B 权重块 ⇒
	 *   整网 ret=0 但输出全错（检测头饱和 / 大片为 0）。
	 *
	 *   板上已双向验证：把块 32767 清零后 L0 一致率 13% → 100%；
	 *   用 0x03 MEM_WRITE 补写末块后 L0/L1/L2 逐层 100%。
	 *
	 *   这里用 CPU 补：DDR 读 1 块(32B) → 直写 y3tram 8 个字。
	 *   每子 job 只多 1 次 DDR 读 + 8 次写，代价可忽略。
	 *   （RTL 侧的根治方案见 ominicore_wload_dma.v 的注释，需重综合。） */
	if (words >= 2u) {
		static uint32_t lb[8];
		volatile uint32_t *p = (volatile uint32_t *)
			(0x60000000u + ((dst_word + words - 1u) << 5));
		if (ddrb_read(src_blk + words - 1u, lb, 1u) == 0) {
			uint32_t k;
			for (k = 0u; k < 8u; k++) p[k] = lb[k];
		}
		/* v39 插桩：DMA+补丁后，CPU 读回首块/末块各 2 字与 DDR 源比对 */
		if (g_exec_dbg) {
			static uint32_t fb[8];
			volatile uint32_t *q = (volatile uint32_t *)(0x60000000u + (dst_word << 5));
			volatile uint32_t *r = (volatile uint32_t *)
				(0x60000000u + ((dst_word + words - 1u) << 5));
			uint32_t okf, okl;
			ddrb_read(src_blk, fb, 1u);
			okf = (q[0] == fb[0] && q[1] == fb[1]) ? 1u : 0u;
			okl = (r[6] == lb[6] && r[7] == lb[7]) ? 1u : 0u;
			uart_puts("[WL] dst="); uart_hex(dst_word);
			uart_puts(" n="); uart_dec(words);
			uart_puts(" f="); uart_hex(fb[0]); uart_puts("/"); uart_hex(q[0]);
			uart_puts(" l="); uart_hex(lb[6]); uart_puts("/"); uart_hex(r[6]);
			uart_puts(okf ? " FOK" : " FBAD");
			uart_puts(okl ? " LOK\n" : " LBAD\n");
		}
	}
	return 0;
}

/* ==========================================================================
 * 单个 job
 * ======================================================================== */
static int exec_job(uint32_t in0, uint32_t in1, uint32_t out, uint32_t wb,
                    const net_layer_t *L, uint32_t cot_sub, uint32_t *cyc_mac)
{
	uint32_t guard = 0, t0;

	if (g_exec_dbg) {
		uart_puts("[JOB] in0="); uart_hex(in0);
		uart_puts(" wb="); uart_hex(wb);
		uart_puts(" out="); uart_hex(out);
		uart_puts(" H="); uart_dec(L->H); uart_puts(" W="); uart_dec(L->W);
		uart_puts(" Ho="); uart_dec(L->Ho); uart_puts(" Wo="); uart_dec(L->Wo);
		uart_puts(" cit="); uart_dec(L->cit);
		uart_puts(" cot="); uart_dec(cot_sub);
		uart_puts(" split="); uart_dec(L->split);
		uart_puts(" k="); uart_dec(L->k);
		uart_puts(" mode="); uart_dec(L->mode);
		uart_puts(" act="); uart_dec(L->act);
		uart_puts(" b16="); uart_dec(L->bits16);
		uart_puts("\n");
	}

	/* CFG0 = {pad[25:24], s[21:20], k[17:16], mode[2:0]} —— 见 y3t_cfg_ahb.v:8 */
	E_CFG0 = ((L->pad & 3u) << 24) | ((L->s & 3u) << 20) |
	         ((L->k   & 3u) << 16) | (L->mode & 7u);
	E_HW   = ((L->W  & 0x3FFu) << 16) | (L->H  & 0x3FFu);
	E_HWO  = ((L->Wo & 0x3FFu) << 16) | (L->Ho & 0x3FFu);
	E_IN0  = in0;
	E_IN1  = in1;
	E_WB   = wb;
	E_OUT  = out;

	if (L->mode == NT_CONV) {
		/* TILE = {split[22:16], cot[14:8], cit[6:0]}  (y3t_cfg_ahb.v:10)
		 * split 非 0 才让引擎走双输入区 (concat 零拷贝) + ng2 两级尺度分组。
		 * 旧代码把 split 恒写 0、in1 恒等 in0 —— 那是 concat 从来没生效的根因。 */
		E_TILE = ((L->split & 0x7Fu) << 16) |
		         ((cot_sub  & 0x7Fu) << 8) | (L->cit & 0x7Fu);
		/* FLAG = {tiles[14:8], bits16[1], act[0]}  (y3t_cfg_ahb.v:13) */
		E_FLAG = ((L->bits16 & 1u) << 1) | (L->act & 1u);
	} else {
		/* POOL2(mode3) / UPSAMPLE(mode2): 引擎不做 requant, 也不读 TILE 的
		 * cit/cot —— 通道 tile 数走 FLAG.tiles (fw_gen.py 的 pool/up 分支同样写法)。 */
		E_TILE = 0u;
		E_FLAG = ((L->cot & 0x7Fu) << 8) |
		         ((L->bits16 & 1u) << 1) | (L->act & 1u);
	}

	t0 = E_MTIME;
	E_CTRL = 1u;                                           /* START 脉冲 */
	while (!(E_CTRL & E_DONE)) {
		if (E_CTRL & E_ERR) return -2;
		if (++guard > E_JOB_TIMEOUT) return -3;
	}
	if (cyc_mac) *cyc_mac += E_CYC_TOT;
	return 0;
}

/* ==========================================================================
 * 一层
 *
 * conv: 权重 > 权重区就按输出 tile 拆子 job (子 job 之间只改 wb/out/TILE.cot)
 * pool/upsample: 无权重、无尺度, 一个 job 完成
 * concat: 不在这里 —— 它在表生成阶段就折进了消费者 (L17) 的 in1 + split
 * ======================================================================== */
int y3t_exec_layer(uint32_t layer, uint32_t ddr_w_base_blk,
                   y3t_exec_stats_t *st)
{
	const net_layer_t *L;
	uint32_t nsub, cot_sub, wsub, sub, plane_out;
	uint32_t cyc;
	uint32_t wspace;
	int r;

	if (layer >= NET_LAYERS) return -1;
	L = &g_net[layer];
	plane_out = L->Ho * L->Wo;

	/* ---- 非卷积层: 一次 job ---- */
	if (L->mode != NT_CONV) {
		cyc = 0u;
		r = exec_job(L->in0, L->in1, L->out, 0u, L, L->cot, &cyc);
		if (r) {
			if (st) { st->failed = layer; st->jobs++; }
			return r;
		}
		if (st) { st->mac_cyc += cyc; st->jobs++; }
		return 0;
	}

	/* ---- conv: 权重放不下就按输出 tile 等分 ----
	 * ★ nsub 必须**整除** cot, 否则子 job 的输出 tile 数不整。所以不能简单地
	 *   用 ceil(w_words/wspace), 要取 cot 的因子。L11 (cot=32, 36,864 块) 在
	 *   14,336 的权重区下 ⇒ nsub=4 (每个 9,216 块), 而不是 3。 */
	wspace = E_WB_TOP - NET_WB_LO;
	nsub = 1u;
	while (nsub <= L->cot) {
		if ((L->cot % nsub) == 0u && (L->w_words / nsub) <= wspace) break;
		nsub <<= 1;
	}
	if (nsub > L->cot) return -1;                  /* 权重区太小, 表有问题 */
	cot_sub  = L->cot / nsub;
	wsub     = L->w_words / nsub;

	for (sub = 0; sub < nsub; sub++) {
		uint32_t wb    = E_WB_TOP - wsub;                  /* 顶部对齐 */
		uint32_t ddrb  = ddr_w_base_blk + L->w_src_blk + sub * wsub;
		uint32_t out   = L->out + sub * cot_sub * plane_out;
		uint32_t t_wl0, t_wl1;

		/* ① 权重 DDR → y3tram（硬件 DMA，CPU 不碰数据） */
		t_wl0 = E_MTIME;
		r = exec_wload(ddrb, wb, wsub);
		t_wl1 = E_MTIME;
		if (r) return -3;
		if (st) st->wload_cyc += (t_wl1 - t_wl0);

		/* ② 真尺度模式：按 y3t_scale_table.h 装该子 job 的 multM / bias 切片
		 *
		 * 三段各自落到引擎规定的 ROM 窗口 (engine_y3t.v:49):
		 *     group0 multM → ROM 0      （单组层只用这一段）
		 *     group1 multM → ROM 1024   （仅 L17 这类双组层, w_g1 != 0）
		 *     bias_q13     → ROM 2048
		 *
		 * 子 job 的切片: multM/bias 的索引是 {ot[5:0],ch}, ot 是 **job 内局部**
		 * 序号(engine_y3t.v:183 `rq_ot = ot`), 子 job 之间 ot 从 0 重新数,
		 * 所以每段都要再偏移 sub*nm —— 这正是 spl 的用处。
		 *
		 * ★ 段起点必须是 8 的倍数, rom_stream 才能用 `blk + (off>>3)` 精确落块:
		 *   w_g0/w_g1/w_bias 由导出脚本保证是 16 的倍数, nm=16*cot_sub 亦然。 */
		if (g_rom_blk) {
			const y3t_scale_t *S = &g_scale[layer];
			uint32_t nm  = 16u * cot_sub;
			uint32_t spl = sub * nm;
			rom_stream(g_rom_blk + ((S->w_g0   + spl) >> 3), ROM_MULTM_BASE, nm);
			if (S->w_g1)
				rom_stream(g_rom_blk + ((S->w_g1 + spl) >> 3), 1024u, nm);
			rom_stream(g_rom_blk + ((S->w_bias + spl) >> 3), ROM_BIAS_BASE, nm);
			/* 移位/输出乘子按层设置一次 (引擎侧是单寄存器, 层内必须均匀) */
			E_SH0   = ((uint32_t)S->shY << 16) |
			          ((uint32_t)S->shM1 << 8) | (uint32_t)S->shM0;
			E_MULTY = S->multY;
			if (g_exec_dbg) {
				uart_puts("[SH] shY="); uart_dec(S->shY);
				uart_puts(" shM0="); uart_dec(S->shM0);
				uart_puts(" multY="); uart_hex(S->multY);
				uart_puts(" w_g0="); uart_dec(S->w_g0);
				uart_puts(" w_bias="); uart_dec(S->w_bias);
				uart_puts("\n");
			}
		}

		/* ③ 发 job */
		cyc = 0u;
		r = exec_job(L->in0, L->in1, out, wb, L, cot_sub, &cyc);
		if (r) {
			if (st) { st->failed = layer; st->jobs++; }
			return r;
		}
		if (st) { st->mac_cyc += cyc; st->jobs++; }
	}
	return 0;
}

/* ==========================================================================
 * 整网
 * ======================================================================== */
int y3t_exec_run(uint32_t ddr_w_base_blk, y3t_exec_stats_t *st)
{
	uint32_t i, t0;
	int r;

	if (st) {
		st->jobs = 0u; st->failed = 0xFFFFFFFFu;
		st->wload_cyc = 0u; st->mac_cyc = 0u; st->total_cyc = 0u;
		st->rom_mode = g_rom_blk ? 1u : 0u;
	}
	t0 = E_MTIME;
	for (i = 0; i < NET_LAYERS; i++) {
		r = y3t_exec_layer(i, ddr_w_base_blk, st);
		if (r) {
			if (st) st->total_cyc = E_MTIME - t0;
			return r;
		}
	}
	if (st) st->total_cyc = E_MTIME - t0;
	return 0;
}
