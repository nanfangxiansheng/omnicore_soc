/* ============================================================================
 * y3t_selftest.h — Y3T 卷积引擎【硬件级】端到端自检 (ZIGUANG 2026-10-17)
 *
 * 为什么放在固件里跑: 板上跑 = 真 RTL 真时序; 仿真跑 = 同一个 TB (tb_soc) 就能出
 *   可对比的黄金结果, 不依赖上位机串口脚本 (y3t_smoke.py), 也不会把 RAM 变量的
 *   地址差误当成 y3tram 块地址 (v12 的教训)。
 *
 * 测的策略: 1×1 conv (k=1) 把权重寻址简化成
 *     w_addr = WB + ot*(cit*8) + cnt          (每 OCH tile 8 字 = 16ch×int16)
 *   激活 layout = tile-major: addr = base + tile*H*W + y*W + x
 *                          每字 256b = 16ch × int16 (通道 pad 到 16)
 *
 * requant 契约 (requant_pipe_y3t.v):
 *     y13 = RHU(acc × multM, shM) + bias_q13
 *     out = sat16( RHU(max(0,y13) × multY, shY) )
 *   本自检把 multM=1<<shM、multY=1<<shY、bias=0 全部显式写出 ⇒ 输出 = acc,
 *   于是期望值是【精确整数】, 比对无歧义。
 *
 * 三段:
 *   S1  y3tram 读写(经 CPU B 口 0x6000_0000): 写 16 个不同 256b 字再读回
 *   S2  端到端 conv 三个用例
 *        A: cit=1 cot=1 2×2   (最小, 4 点)
 *        B: cit=1 cot=1 8×8   (64 点, 查 tile 内 +y*W+x 步进)
 *        C: cit=2 cot=1 4×4   (2 输入 tile 求和, 查 wjob/通道 pad)
 *   S3  cyc_mac / cyc_tot (每 MAC 周期数素材)
 * ========================================================================== */
#ifndef Y3T_SELFTEST_H
#define Y3T_SELFTEST_H

#include <stdint.h>
#include "y3t.h"

/* ---- y3tram CPU 窗口 (boardsoc: sel_y3tmem = haddr[31:22]==10'h180) ----
 * 256b 字索引 = haddr[21:5], 32b lane = haddr[4:2]。 */
#define Y3RAM_W32(blk, lane) \
	(*(volatile uint32_t *)(0x60000000u + ((uint32_t)(blk) << 5) + ((uint32_t)(lane) << 2)))

/* 把 256b 字里某通道的 int16 写入 (保留其它通道) */
static void y3ram_set_ch(uint32_t blk, uint32_t ch, int32_t v)
{
	uint32_t w = Y3RAM_W32(blk, ch >> 1);
	uint32_t h = (uint32_t)v & 0xFFFFu;
	if (ch & 1u) w = (w & 0x0000FFFFu) | (h << 16);
	else         w = (w & 0xFFFF0000u) | h;
	Y3RAM_W32(blk, ch >> 1) = w;
}

/* 读 256b 字里某通道的 int16 (符号扩展) */
static int32_t y3ram_get_ch(uint32_t blk, uint32_t ch)
{
	uint32_t w = Y3RAM_W32(blk, ch >> 1);
	uint32_t h = (ch & 1u) ? (w >> 16) : (w & 0xFFFFu);
	return (h & 0x8000u) ? (int32_t)h - 65536 : (int32_t)h;
}

static void y3ram_fill(uint32_t blk, int32_t v)
{
	uint32_t l, h = ((uint32_t)v & 0xFFFFu) * 0x00010001u;
	for (l = 0; l < 8u; l++) Y3RAM_W32(blk, l) = h;
}

/* 权重区专用填充: 权重在 256b 字里是【16 个 int8】，不是 16 个 int16。
 * mac_array 按 in_w[o*ICH*8 + i*8 +: 8] 取字节 ⇒ 用 y3ram_fill 的 0x00010001
 * 会得到 01,00,01,00…（只有一半通道权重=1），累计结果恰好是全 1 的一半。
 * 本函数把 v 的低 8 位铺满 32 个字节 ⇒ 16 OCH × 16 ICH 全等于 v。 */
static void y3ram_fill_w(uint32_t blk, int32_t v)
{
	uint32_t l, b = (uint32_t)v & 0xFFu;
	l = b * 0x01010101u;
	for (uint32_t k = 0; k < 8u; k++) Y3RAM_W32(blk, k) = l;
}

/* ---- 自检主体: 返回 fail 计数 (0 = 全通) ----
 * puts/dec/hex 由调用方提供 (demo.c 有自己的 UART 实现)。 */
static uint32_t y3t_selftest(void (*p_puts)(const char *),
                             void (*p_dec)(uint32_t),
                             void (*p_hex)(uint32_t))
{
	uint32_t fails = 0, i, c, y, x, ot;
	/* 块地址: 全部选在权重区/激活区之外的低段 (0..4095), 不碰 ACT_A(20480)/ACT_B(24576) */
	const uint32_t SBASE  = 8u;
	const uint32_t IBASE  = 64u;
	const uint32_t WBASE  = 3072u;
	const uint32_t OUBASE = 3200u;

	p_puts("[8] Y3T selftest start (id="); p_hex(Y3T_ID); p_puts(")\n");

	/* ---------------- S1: y3tram CPU 口读写 ---------------- */
	for (i = 0; i < 16u; i++) y3ram_fill(SBASE + i, (int32_t)(0x100 + i));
	{
		uint32_t bad1 = 0, bblk = 0; int32_t be = 0, bg = 0;
		for (i = 0; i < 16u; i++) {
			int32_t got = y3ram_get_ch(SBASE + i, 3u);   /* 抽查通道 3 */
			if (got != (int32_t)(0x100 + i)) {
				if (!bad1) { bblk = SBASE + i; be = (int32_t)(0x100 + i); bg = got; }
				bad1++;
			}
		}
		if (bad1) {
			p_puts("[8] S1 y3tram rw FAIL n="); p_dec(bad1);
			p_puts(" blk="); p_dec(bblk);
			p_puts(" exp="); p_dec((uint32_t)be); p_puts(" got="); p_dec((uint32_t)bg);
			p_puts("\n");
			fails++;
		} else {
			p_puts("[8] S1 y3tram rw ok (16 blk x 256b)\n");
		}
	}

	/* ---------------- requant 单位化: multM=1<<shM, multY=1<<shY, shM=0 ---------------- */
	Y3T_MULTY = 1u;                       /* ×1 */
	/* SH0 = {shY[21:16], shM1[13:8], shM0[5:0]} —— shM 与 shY 全 0 = 直通 */
	Y3T_SH0   = 0u;
	/* multm_rom / bias_rom 是引擎内部 RAM，复位值 0（不是 1）——必须显式写。
	 * 契约: y13 = RHU(acc × multM, shM) + bias_q13；取 multM=1<<shM=1、bias=0
	 *       ⇒ y13 = acc；再 ×multY(1)>>shY(0) ⇒ out = acc。
	 * 索引: multm[idx] idx<2048 → {group[10], ot[9:4], ch[3:0]}；
	 *       bias[idx]  idx-2048 → {ot[9:4], ch[3:0]}。
	 * 为了覆盖所有可能用到的 ot/ch，前 512 项全写 1（ot 0..31 × ch 0..15）。 */
	Y3T_ROM_ADDR = 0u;
	for (i = 0; i < 1024u; i++) Y3T_ROM_DATA = 1u;      /* multm: 0..1023 = 1 */
	for (i = 0; i < 512u;  i++) Y3T_ROM_DATA = 0u;      /* bias : 1024..1535 = 0 */
	/* 校验 ROM 口读写通路（ROM_ADDR 可读回） */
	Y3T_ROM_ADDR = 0u;
	{
		uint32_t ra = Y3T_ROM_ADDR;
		p_puts("[8] S1b rom port ok="); p_dec(ra == 0u ? 1u : 0u);
		p_puts(" addr_rd="); p_dec(ra); p_puts("\n");
	}

	/* ---------------- S2: 端到端 conv ---------------- */
	{
		uint32_t tcit[3]  = { 1u, 1u, 2u };
		uint32_t tcot[3]  = { 1u, 1u, 1u };
		uint32_t tH[3]    = { 2u, 8u, 4u };
		uint32_t tW[3]    = { 2u, 8u, 4u };
		uint32_t tno;
		for (tno = 0; tno < 3u; tno++) {
			uint32_t cit = tcit[tno], cot = tcot[tno], H = tH[tno], W = tW[tno];
			uint32_t plane = H * W;
			uint32_t n_in  = cit * plane, n_out = cot * plane;
			uint32_t bad = 0, shown = 0;
			int32_t exp;

			/* 输入: 通道 ch (0..cit*16-1) 全平面置 (ch+1)*(tno+1) */
			for (c = 0; c < cit * 16u; c++)
				for (i = 0; i < plane; i++)
					y3ram_set_ch(IBASE + (c / 16u) * plane + i, c % 16u,
					             (int32_t)((c % 16u) + 1u) * (int32_t)(tno + 1u));
			/* 权重: 每 ot tile 的 cit*8 字全置 1 (int8 布局, 每通道权重 = 1) */
			for (i = 0; i < cot * cit * 8u; i++) y3ram_fill_w(WBASE + i, 1);
			/* 偏置: bias_rom 在引擎内, 由 cfg 口 0x40 写 —— 本轮全 0 (reset 值) */
			/* 输出区清零 */
			for (i = 0; i < n_out; i++) y3ram_fill(OUBASE + i, 0);

			{
				int r = y3t_job(IBASE, IBASE, WBASE, OUBASE,
				                W, H, W, H, 1u, 1u, 0u, 0u,
				                cit, cot, 0u, 1u, 20000000u);
				if (r != 0) {
					p_puts("[8] S2 case"); p_dec(tno);
					p_puts(" job FAIL ret="); p_dec((uint32_t)(-r));
					p_puts(" (2=err 1=timeout)\n");
					fails++; continue;
				}
			}
			/* 期望: acc = Σ_{it tile} Σ_{ch=0..15} in_ch   (pad 的 16-cit*16 通道不存在,
			 *   但 cit 的每 tile 恰好 16 通道都有值, 所以就是 cit * Σ_{ch}(ch+1)*(tno+1)) */
			{
				int32_t per_tile = 0;
				for (c = 0; c < 16u; c++)
					per_tile += (int32_t)(c + 1u) * (int32_t)(tno + 1u);
				exp = per_tile * (int32_t)cit;
			}
			for (ot = 0; ot < cot; ot++)
				for (y = 0; y < H; y++)
					for (x = 0; x < W; x++)
						for (c = 0; c < 16u; c++) {
							int32_t got = y3ram_get_ch(OUBASE + ot * plane + y * W + x, c);
							if (got != exp) {
								bad++;
								if (!shown) {
									shown = 1u;
									p_puts("[8] S2 case"); p_dec(tno);
									p_puts(" out(y="); p_dec(y); p_puts(",x="); p_dec(x);
									p_puts(",ch="); p_dec(c);
									p_puts(") exp="); p_dec((uint32_t)exp);
									p_puts(" got="); p_dec((uint32_t)(got & 0xFFFFu));
									p_puts("\n");
								}
							}
						}
			p_puts("[8] S2 case"); p_dec(tno);
			p_puts(" cit="); p_dec(cit); p_puts(" cot="); p_dec(cot);
			p_puts(" "); p_dec(H); p_puts("x"); p_dec(W);
			p_puts(" bad="); p_dec(bad); p_puts("/"); p_dec(n_out * 16u);
			p_puts(" cyc_mac="); p_dec(Y3T_CYC_MAC);
			p_puts(" cyc_tot="); p_dec(Y3T_CYC_TOT);
			p_puts("\n");
			if (bad) fails++;
		}
	}

	if (!fails) p_puts("[8] Y3T selftest PASS\n");
	else        { p_puts("[8] Y3T selftest FAIL n="); p_dec(fails); p_puts("\n"); }
	return fails;
}

#endif /* Y3T_SELFTEST_H */
