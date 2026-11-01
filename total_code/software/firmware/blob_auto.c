/* blob_auto.c —— arena 自治实时识别循环（相机→PP→整网→解码→画框）
 *
 * 一次性经 0x0C 下装到 arena(0x8000)，0x0D EXEC 后**板上无限循环**：
 *   arm PP → 等新帧 → 等写完 → 撤 PP → 整网(29 层) → 解码 → 画框 → 重复
 * PC 只在启动时下装 + 读串口状态行，**不在环路里**。
 * 参数邮箱 0x9F80: {u32 wblk, u32 romblk, u32 conf_thr(i16 原始分数阈值)}
 * 停止：复位板子（程序不返回）。
 */
#include <stdint.h>
#include "y3t_net_table.h"
#include "y3t_scale_table_blob.h"
#include "y3t_detect_tbl.h"
#include "blob_syms.h"

#define REG32(a)      (*(volatile uint32_t *)(a))
#define PP_CSR        REG32(0x40004000u)
#define PP_ARM        1u                    /* bit0: arm 控制位（写回时绝不能带状态位） */
#define PP_PAD_DONE   (1u << 8)             /* 本帧 letterbox+pad 完成 */
#define PP_FCN(st)    (((st) >> 10) & 0xFFu)/* 采集帧计数（回读域） */
#define SETTLE_CYC    1080000u              /* 撤 arm 后落盘等待 20ms @54MHz（同固件 g_settle_cyc 默认） */
#define OVL_BASE      0x40006000u
#define OVL_EN        REG32(0x40006060u)
/* v47 文字层: 0x70 {ch1,ch0} 0x74 {ch3,ch2} 0x78 RGB565 0x7C {en,y,x/4} */
#define TXT_CH01      REG32(0x40006070u)
#define TXT_CH23      REG32(0x40006074u)
#define TXT_COLOR     REG32(0x40006078u)
#define TXT_POS_EN    REG32(0x4000607Cu)
#define E_CTRL        REG32(0x40005000u)
#define E_CFG0        REG32(0x40005004u)
#define E_HW          REG32(0x40005008u)
#define E_HWO         REG32(0x4000500Cu)
#define E_TILE        REG32(0x40005010u)
#define E_SH0         REG32(0x40005014u)
#define E_MULTY       REG32(0x40005018u)
#define E_FLAG        REG32(0x4000501Cu)
#define E_IN0         REG32(0x4000502Cu)
#define E_IN1         REG32(0x40005030u)
#define E_WB          REG32(0x40005034u)
#define E_OUT         REG32(0x40005038u)
#define E_ROM_ADDR    REG32(0x4000503Cu)
#define E_ROM_DATA    REG32(0x40005040u)
#define E_CYC_TOT     REG32(0x40005050u)
#define E_MTIME       REG32(0x40002000u)

/* ---- GPIO (v65 gpio_ctl.h 同款, 2026-10-30 移植进 blob) ----
 * 输出编码 (one-hot, 一类一灯): bottle→LED3 mouse→LED4 cup→LED5
 *   toy→LED6 screen→LED7 (gpio_out[0..4] = E18/A17/A18/C17/B17, 高电平亮);
 *   无目标 = 5 灯全灭。bit5 = 帧心跳翻转(未引出, 示波器可测) */
#define GPIO_BASE      0x40001000u
#define GPIO_DATA_OUT  REG32(GPIO_BASE + 0x04u)
#define GPIO_DIR       REG32(GPIO_BASE + 0x08u)
#define GPIOO_ONEHOT   0x0000001Fu
#define GPIOO_HEARTBEAT 0x00000020u
#define E_WLD_SRC     REG32(0x90000000u)
#define E_WLD_DST     REG32(0x90000004u)
#define E_WLD_LEN     REG32(0x90000008u)
#define E_WLD_CTRL    REG32(0x9000000Cu)
#define E_WLD_STATUS  REG32(0x9000000Cu)
#define E_WLD_DONE    0x1u
#define E_DONE        0x1u
#define E_ERR         0x2u
#define TIMEOUT       200000000u
#define ROM_MULTM_BASE 0u
#define ROM_BIAS_BASE  2048u
#define MAILBOX        0xFC00u   /* 2026-10-29 从 0x9F80 搬到 arena 顶, 给镜像让空间 */
#define Y3T_BASE       0x60000000u
#define UART_STATUS    REG32(0x40000004u)
#define UART_DATA      REG32(0x40000000u)

#define DDRB_READ  ((int (*)(uint32_t, uint32_t *, uint32_t))DDRB_READ_ADDR)
#define PUTS       ((void (*)(const char *))UART_PUTS_ADDR)
#define UHEX       ((void (*)(uint32_t))UART_HEX_ADDR)
#define UDEC       ((void (*)(uint32_t))UART_DEC_ADDR)

static uint32_t g_rom_blk;
static uint32_t g_conf_thr = (uint32_t)(-32768);   /* 0xFFFF8000 = 用表内默认 */

/* ---------------- 整网执行（与 netrun_blob 同款） ---------------- */
static int exec_wload(uint32_t src_blk, uint32_t dst_word, uint32_t words)
{
	uint32_t guard = 0, w;
	E_WLD_SRC = src_blk; E_WLD_DST = dst_word; E_WLD_LEN = words;
	E_WLD_CTRL = 1u;
	for (w = 0u; w < 100000u; ++w)
		if (!(E_WLD_STATUS & E_WLD_DONE)) break;
	while (!(E_WLD_STATUS & E_WLD_DONE))
		if (++guard > TIMEOUT) return -1;
	if (words >= 2u) {
		uint32_t lb[8];
		volatile uint32_t *p = (volatile uint32_t *)
			(Y3T_BASE + ((dst_word + words - 1u) << 5));
		if (DDRB_READ(src_blk + words - 1u, lb, 1u) == 0) {
			uint32_t k;
			for (k = 0u; k < 8u; k++) p[k] = lb[k];
		}
	}
	return 0;
}

static void rom_stream(uint32_t ddr_blk, uint32_t addr, uint32_t n)
{
	uint32_t buf[64];
	uint32_t done = 0u;
	E_ROM_ADDR = addr & 0xFFFu;
	while (done < n) {
		uint32_t i, nb = (n - done + 7u) / 8u;
		if (nb > 8u) nb = 8u;
		if (DDRB_READ(ddr_blk + (done >> 3), buf, nb) != 0) return;
		for (i = 0; i < nb * 8u && done < n; i++) {
			E_ROM_DATA = buf[i];
			done++;
		}
	}
}

static int exec_job(uint32_t in0, uint32_t in1, uint32_t out, uint32_t wb,
                    const net_layer_t *L, uint32_t cot_sub)
{
	uint32_t guard = 0;
	E_CFG0 = ((L->pad & 3u) << 24) | ((L->s & 3u) << 20) |
	         ((L->k   & 3u) << 16) | (L->mode & 7u);
	E_HW  = ((L->W  & 0x3FFu) << 16) | (L->H  & 0x3FFu);
	E_HWO = ((L->Wo & 0x3FFu) << 16) | (L->Ho & 0x3FFu);
	E_IN0 = in0; E_IN1 = in1; E_WB = wb; E_OUT = out;
	if (L->mode == 0u) {
		E_TILE = ((L->split & 0x7Fu) << 16) |
		         ((cot_sub  & 0x7Fu) << 8) | (L->cit & 0x7Fu);
		E_FLAG = ((L->bits16 & 1u) << 1) | (L->act & 1u);
	} else {
		E_TILE = 0u;
		E_FLAG = ((L->cot & 0x7Fu) << 8) |
		         ((L->bits16 & 1u) << 1) | (L->act & 1u);
	}
	E_CTRL = 1u;
	while (!(E_CTRL & E_DONE)) {
		if (E_CTRL & E_ERR) return -2;
		if (++guard > TIMEOUT) return -3;
	}
	return 0;
}

static int exec_layer(uint32_t layer, uint32_t ddr_w_base_blk)
{
	const net_layer_t *L;
	uint32_t nsub, cot_sub, wsub, sub, plane_out, wspace;
	int r;

	if (layer >= NET_LAYERS) return -1;
	L = &g_net[layer];
	plane_out = L->Ho * L->Wo;
	if (L->mode != 0u)
		return exec_job(L->in0, L->in1, L->out, 0u, L, L->cot);
	wspace = 32768u - 18432u;
	nsub = 1u;
	while (nsub <= L->cot) {
		if ((L->cot % nsub) == 0u && (L->w_words / nsub) <= wspace) break;
		nsub <<= 1;
	}
	if (nsub > L->cot) return -1;
	cot_sub = L->cot / nsub;
	wsub = L->w_words / nsub;
	for (sub = 0u; sub < nsub; sub++) {
		uint32_t wb = 32768u - wsub;
		uint32_t ddrb = ddr_w_base_blk + L->w_src_blk + sub * wsub;
		uint32_t out = L->out + sub * cot_sub * plane_out;
		r = exec_wload(ddrb, wb, wsub);
		if (r) return -3;
		if (g_rom_blk) {
			const y3t_scale_t *S = &g_scale[layer];
			uint32_t nm = 16u * cot_sub;
			uint32_t spl = sub * nm;
			rom_stream(g_rom_blk + ((S->w_g0   + spl) >> 3), ROM_MULTM_BASE, nm);
			if (S->w_g1)
				rom_stream(g_rom_blk + ((S->w_g1 + spl) >> 3), 1024u, nm);
			rom_stream(g_rom_blk + ((S->w_bias + spl) >> 3), ROM_BIAS_BASE, nm);
			E_SH0 = ((uint32_t)S->shY << 16) |
			        ((uint32_t)S->shM1 << 8) | (uint32_t)S->shM0;
			E_MULTY = S->multY;
		}
		r = exec_job(L->in0, L->in1, out, wb, L, cot_sub);
		if (r) return r;
	}
	return 0;
}

static int run_net(uint32_t wblk)
{
	uint32_t i;
	for (i = 0u; i < NET_LAYERS; i++) {
		int r = exec_layer(i, wblk);
		if (r) return r;
	}
	return 0;
}

/* ---------------- 检测解码（与 y3t_detect.c 同权） ---------------- */
typedef struct { int32_t cx96_q8, cy96_q8, w96_q8, h96_q8; uint16_t conf_q; uint8_t cls, scale; } dbox_t;

static inline int16_t act_get(const int16_t *base, uint32_t H, uint32_t W,
                              uint32_t y, uint32_t x, uint32_t c)
{
	uint32_t g = c >> 4;                 /* 16 通道一组 */
	uint32_t l = c & 15u;
	uint32_t widx = g * (H * W) + y * W + x;
	uint32_t idx = widx * 16u + l;
	/* 【BUG-FIX 2026-10-25 ★ det_decode 崩溃根因】CPU 的 y3tram 窗口(0x60000000)
	 *   只支持 **32 位** 访问：用 int16_t* 读会生成 `lh`（HSIZE=16），
	 *   总线直接回 ERROR ⇒ mcause=5 访问错误、固件挂死（v38 自动循环崩溃即此）。
	 *   改为 32 位读 + 取高/低半字。 */
	{
		const volatile uint32_t *w = (const volatile uint32_t *)
			((const char *)base + (idx & ~1u) * 2u);
		uint32_t v = *w;
		return (idx & 1u) ? (int16_t)(v >> 16) : (int16_t)(v & 0xFFFFu);
	}
}

/* Q24 乘 int16 -> **Q8 网格单位**（四舍五入）
 *
 * 【Q8-FIX 2026-10-28】与固件 y3t_detect.c 同步修：原来返回"整数网格单位"，
 * 而 l/r/t/b 是 0~1 的亚格量（box 头 s_out≈7.8e-5，一格≈12800 计数）⇒ 取整后
 * 只剩 0/1，宽度只能是 0 或 16/32 px。旧模型实测几何偏差 13.48 px，改后 0.057 px。
 * arena blob 是独立编译单元，必须单独改，否则屏上的框尺寸还是错的。 */
static inline int32_t qmul_q8(int32_t a_q24, int32_t b)
{
	int64_t t = (int64_t)a_q24 * (int64_t)b;
	if (t >= 0) t += (1 << 15);
	else        t -= (1 << 15);
	return (int32_t)(t >> 16);
}

static int det_decode(const int16_t *y3t, int max_out, dbox_t *out)
{
	dbox_t cand[24];                 /* 先全收，再 NMS 去重 */
	int ncand = 0;
	int n = 0;
	for (uint32_t sc = 0; sc < DET_NSCALE; sc++) {
		const det_head_t *hb = &g_det[sc][0];
		const det_head_t *hs = &g_det[sc][1];
		const uint32_t H = hb->H, W = hb->W;
		const uint32_t stride = 96u / H;
		const int32_t sq = hb->s_out_q24;
		const int16_t *bb = y3t + (int32_t)hb->out_blk * 16;
		const int16_t *bs = y3t + (int32_t)hs->out_blk * 16;
		const uint32_t ncls = hs->oc;
		for (uint32_t y = 0; y < H; y++) {
			for (uint32_t x = 0; x < W; x++) {
				int32_t best = -32768; uint32_t bestc = 0;
				for (uint32_t c = 0; c < ncls; c++) {
					int32_t v = act_get(bs, H, W, y, x, c);
					if (v > best) { best = v; bestc = c; }
				}
				int32_t thr = (int32_t)g_conf_thr;
				if (thr == -32768) thr = hs->thr_q;
				if (best < thr) continue;
				if (ncand >= (int)(sizeof cand / sizeof cand[0])) break;
				{
					int32_t ax = (int32_t)x * 256 + 128;
					int32_t ay = (int32_t)y * 256 + 128;
					int32_t l = qmul_q8(sq, act_get(bb, H, W, y, x, 0));
					int32_t t = qmul_q8(sq, act_get(bb, H, W, y, x, 1));
					int32_t rr = qmul_q8(sq, act_get(bb, H, W, y, x, 2));
					int32_t bt = qmul_q8(sq, act_get(bb, H, W, y, x, 3));
					/* l/r/t/b 已是 Q8 网格, ax/ay 也是 Q8 ⇒ 直接加减, 不再乘 128/256 */
					cand[ncand].cx96_q8 = (ax + (rr - l) / 2) * (int32_t)stride;
					cand[ncand].cy96_q8 = (ay + (bt - t) / 2) * (int32_t)stride;
					cand[ncand].w96_q8  = (l + rr) * (int32_t)stride;
					cand[ncand].h96_q8  = (t + bt) * (int32_t)stride;
					cand[ncand].cls = (uint8_t)bestc;
					cand[ncand].scale = (uint8_t)sc;
					cand[ncand].conf_q = (int16_t)best;
					ncand++;
				}
			}
			if (ncand >= (int)(sizeof cand / sizeof cand[0])) break;
		}
	}

	/* ---- NMS（贪心，按类别去重）----
	 * 【2026-10-29】原来没有 NMS：同一物体横跨多个网格 ⇒ 每个过阈值的格子
	 * 都出一个框, 一个物体上叠最多 8 个嵌套矩形（屏上"重复标签/框"的来源之一）。
	 * 按置信度降序取框, 与已保留的同类别框 IoU>0.35 的丢弃。全整数运算。
	 * 注意: -nostdlib 下不能用 ={0} 初始化和结构体赋值 —— GCC 会生成
	 * memset/memcpy 调用导致链接失败, 必须显式循环/逐字段。 */
	char kept[ sizeof cand / sizeof cand[0] ];
	for (int i = 0; i < (int)(sizeof kept); i++) kept[i] = 0;
	for (int pass = 0; pass < ncand; pass++) {
		int bi = -1;
		for (int i = 0; i < ncand; i++)
			if (!kept[i] && (bi < 0 || cand[i].conf_q > cand[bi].conf_q)) bi = i;
		if (bi < 0) break;
		kept[bi] = 1;
		if (n < max_out) {
			out[n].cx96_q8 = cand[bi].cx96_q8;
			out[n].cy96_q8 = cand[bi].cy96_q8;
			out[n].w96_q8  = cand[bi].w96_q8;
			out[n].h96_q8  = cand[bi].h96_q8;
			out[n].conf_q  = cand[bi].conf_q;
			out[n].cls     = cand[bi].cls;
			out[n].scale   = cand[bi].scale;
			n++;
		}
		int64_t x1 = (int64_t)cand[bi].cx96_q8 - cand[bi].w96_q8 / 2;
		int64_t y1 = (int64_t)cand[bi].cy96_q8 - cand[bi].h96_q8 / 2;
		int64_t x2 = x1 + cand[bi].w96_q8, y2 = y1 + cand[bi].h96_q8;
		for (int i = 0; i < ncand; i++) {
			if (kept[i] || cand[i].cls != cand[bi].cls) continue;
			int64_t a1 = (int64_t)cand[i].cx96_q8 - cand[i].w96_q8 / 2;
			int64_t b1 = (int64_t)cand[i].cy96_q8 - cand[i].h96_q8 / 2;
			int64_t a2 = a1 + cand[i].w96_q8, b2 = b1 + cand[i].h96_q8;
			int64_t iw = (a2 < x2 ? a2 : x2) - (a1 > x1 ? a1 : x1);
			int64_t ih = (b2 < y2 ? b2 : y2) - (b1 > y1 ? b1 : y1);
			if (iw <= 0 || ih <= 0) continue;
			int64_t inter = iw * ih;
			int64_t uni = cand[i].w96_q8 * cand[i].h96_q8
			            + cand[bi].w96_q8 * cand[bi].h96_q8 - inter;
			if (uni > 0 && inter * 100 > uni * 35) kept[i] = 1;   /* IoU>0.35 抑制 */
		}
	}
	return n;
}

/* ---------------- overlay 画框 ---------------- */
static void ovl_box(uint32_t n, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, uint32_t rgb)
{
	uint32_t b = OVL_BASE + n * 12u;
	REG32(b + 0u) = ((y0 & 0xFFFu) << 16) | (x0 & 0xFFFu);
	REG32(b + 4u) = ((y1 & 0xFFFu) << 16) | (x1 & 0xFFFu);
	REG32(b + 8u) = rgb & 0xFFFFu;
}

static volatile int32_t g_db0x0, g_db0y0, g_db0x1, g_db0y1;   /* 观测用 */

static void draw_boxes(const dbox_t *d, int n)
{
	static const uint32_t COL[5] = { 0x07E0u, 0x07FFu, 0xFFE0u, 0xF81Fu, 0xF800u };
	uint32_t used = 0u;
	for (int i = 0; i < n && i < 8; i++) {
		/* 96 坐标 Q8 -> 帧 Q8: ×1000/75; y=(v-21*256)*1000/75
		 * overlay 的 x_cnt 是整条 1280 宽合成流（左半 cam0 / 右半 cam1），
		 * 但 cam0 画面存 640x720 后 1:1 贴左半屏 ⇒ **帧坐标要 ÷2**（sx/hw 用 >>9）。 */
		int32_t xf = (d[i].cx96_q8 * 1000) / 75;
		int32_t yf = ((d[i].cy96_q8 - 21 * 256) * 1000) / 75;
		int32_t wf = (d[i].w96_q8 * 1000) / 75;
		int32_t hf = (d[i].h96_q8 * 1000) / 75;
		int32_t sx = xf >> 9, sy = yf >> 8;          /* 帧÷2 贴左半屏, 720 直通 */
		int32_t hw = wf >> 9, hh = hf >> 8;
		int32_t x0 = sx - hw / 2, y0 = sy - hh / 2;
		int32_t x1 = sx + hw / 2, y1 = sy + hh / 2;
		uint32_t col = (d[i].cls < 5) ? COL[d[i].cls] : 0xFFFFu;
		if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
		if (x1 > 639) x1 = 639; if (y1 > 719) y1 = 719;   /* 只画在左半屏(cam0) */
		if (x1 <= x0 || y1 <= y0) continue;
		if (i == 0) { g_db0x0 = x0; g_db0y0 = y0; g_db0x1 = x1; g_db0y1 = y1; }
		ovl_box((uint32_t)i, (uint32_t)x0, (uint32_t)y0, (uint32_t)x1, (uint32_t)y1, col);
		used |= (1u << i);
	}
	if (n == 0) { g_db0x0 = g_db0y0 = g_db0x1 = g_db0y1 = -1; }
	for (uint32_t k = 0; k < 8u; k++)
		if (!(used & (1u << k))) ovl_box(k, 0, 0, 0, 0, 0);
	OVL_EN = (OVL_EN & ~0xFFu) | (n ? 0xFFu : 0u);
}

/* ---------------- v47 文字层：top-1 类别名 ---------------- *
 * 只在最高置信度那个框的左上角打 4 字符类别缩写；无框则关闭。
 * 位置编码: reg[31]=en, reg[22:12]=y, reg[11:2]=x>>2（RTL 里 x = reg[11:2]<<2）。
 */
/* conf_q(原始 logit 分数, i16) × 该头 s_out → 置信度百分数 2..98。
 * 17 项 LUT 近似 sigmoid（步长 0.5，覆盖 logit ±4）+ 线性插值（误差 <1 个点），
 * 全程整数——固件无浮点。 */
static uint8_t conf_pct(int32_t conf_q, int32_t s_out_q24)
{
	static const uint8_t LUT[17] = {
		2, 3, 5, 8, 12, 18, 27, 38, 50,
		62, 73, 82, 88, 92, 95, 97, 98 };
	int32_t lg = (int32_t)((int64_t)conf_q * s_out_q24 >> 12) + 16384; /* q12, 偏移到 ≥0 */
	int32_t idx, frac, a, b;
	if (lg < 0)     lg = 0;                 /* 防御 */
	if (lg > 32768) lg = 32768;             /* logit=+4 封顶 */
	idx  = lg >> 11;
	if (idx > 16) idx = 16;
	frac = lg & 2047;                        /* 档内插值位置 */
	a = LUT[idx];
	b = (idx < 16) ? LUT[idx + 1] : a;
	return (uint8_t)(a + ((b - a) * frac >> 11));
}

/* FPS 显示：帧周期(周期数) → fps×10 → "12.3" 格式（4 字符, '.' 字库已有）。
 * 固定屏幕左上角 (8,8)，白色。E_MTIME 时基 = 54.054 MHz。 */
/* 发布一帧识别结果到 GPIO/LED（2026-10-30, one-hot 一类一灯版）。
 * cls = 类别 0..4 (bottle,mouse,cup,toy,screen), -1/无目标 = 全灭:
 *   bottle→LED3  mouse→LED4  cup→LED5  toy→LED6  screen→LED7 */
static void gpio_publish(int cls, int valid)
{
	uint32_t v = GPIO_DATA_OUT;
	v &= ~GPIOO_ONEHOT;
	if (valid && cls >= 0 && cls < 5)
		v |= (1u << cls);                 /* one-hot: 识别到哪类亮哪个灯 */
	v ^= GPIOO_HEARTBEAT;                 /* 每帧翻转 (bit5 未引出, 不影响灯) */
	GPIO_DATA_OUT = v;
}

static void draw_fps(uint32_t period_cyc)
{
	char t[4];
	uint32_t fx10 = (uint32_t)((uint64_t)54054000u * 10u /
	                           (period_cyc ? period_cyc : 1u));
	if (fx10 > 999) fx10 = 999;
	t[0] = (fx10 >= 100) ? (char)('0' + fx10 / 100) : ' ';
	t[1] = (char)('0' + (fx10 / 10) % 10);
	t[2] = '.';
	t[3] = (char)('0' + fx10 % 10);
	TXT_CH01   = ((uint32_t)(uint8_t)t[1] << 8) | (uint8_t)t[0];
	TXT_CH23   = ((uint32_t)(uint8_t)t[3] << 8) | (uint8_t)t[2];
	TXT_COLOR  = 0xFFFFu;
	TXT_POS_EN = 0x80000000u | (8u << 12) | 8u;   /* en, y=8, x=8 */
}

static void draw_label(const dbox_t *d, int n)
{
	/* NM 顺序 = 训练侧 classes.txt（bottle,mouse,cup,toy,screen）*/
	static const char *const NM[5] = { "BOTT", "MOUS", "CUP ", "TOY ", "SCRN" };
	static const uint32_t COL[5] = { 0x07E0u, 0x07FFu, 0xFFE0u, 0xF81Fu, 0xF800u };
	int best = -1;
	int k;
	if (n <= 0) { TXT_POS_EN = 0u; return; }        /* 无框 => 关文字 */
	for (k = 0; k < n; k++)
		if (best < 0 || d[k].conf_q > d[best].conf_q) best = k;
	{
		char t[4];   /* "B85%" = 类别首字母 + 两位百分数 + '%'（文字层共 4 字符） */
		uint32_t  col;
		const det_head_t *hs = &g_det[d[best].scale][1];  /* 该尺度的 score 头 */
		uint8_t pct = conf_pct(d[best].conf_q, hs->s_out_q24);
		if (d[best].cls < 5) {
			t[0] = NM[d[best].cls][0];      /* B/M/C/T/S 首字母恰好互不相同 */
			col = COL[d[best].cls];
		} else {
			t[0] = ' ';
			col = 0xFFFFu;
		}
		t[1] = (char)('0' + pct / 10);
		t[2] = (char)('0' + pct % 10);
		t[3] = '%';
		int32_t xf = (d[best].cx96_q8 * 1000) / 75;
		int32_t yf = ((d[best].cy96_q8 - 21 * 256) * 1000) / 75;
		int32_t wf = (d[best].w96_q8 * 1000) / 75;
		int32_t hf = (d[best].h96_q8 * 1000) / 75;
		int32_t sx = xf >> 9, sy = yf >> 8;
		int32_t hw = wf >> 9, hh = hf >> 8;
		int32_t tx = sx - hw / 2;
		int32_t ty = sy - hh / 2 - 9;               /* 框左上角上方一行字高 */
		if (tx < 0) tx = 0;
		if (tx > 560) tx = 560;
		if (ty < 0) ty = 0;
		if (ty > 700) ty = 700;
		TXT_CH01   = ((uint32_t)(uint8_t)t[1] << 8) | (uint8_t)t[0];
		TXT_CH23   = ((uint32_t)(uint8_t)t[3] << 8) | (uint8_t)t[2];
		TXT_COLOR  = col;
		TXT_POS_EN = 0x80000000u | ((uint32_t)ty << 12) |
		             (((uint32_t)tx & ~3u));
	}
}

/* ---------------- 入口：无限循环 ---------------- */
__attribute__((section(".text.entry")))
void entry(void)
{
	uint32_t wblk = REG32(MAILBOX);
	uint32_t romblk = REG32(MAILBOX + 4u);
	uint32_t nframe = 0u, nrun = 0u, nfail = 0u;
	uint32_t tprev = 0u;                 /* 上帧时间戳 → 帧周期 → FPS */
	int32_t thr = (int32_t)REG32(MAILBOX + 8u);
	static dbox_t bx[8];
	uint32_t t0, tlast;

	if (thr != 0) g_conf_thr = (uint32_t)thr;
	g_rom_blk = romblk;
	/* 2026-10-30: GPIO/LED 上电配置——bit0..5 输出, 全零起步
	 * (y3t_auto 的 gpio_init 同款; blob 是独立程序必须自己配一次,
	 *   否则 DIR=0 全输入, 写 DATA_OUT 灯不亮) */
	GPIO_DIR      = 0x0000003Fu;
	GPIO_DATA_OUT = 0x00000000u;
	PUTS("[LOOP] start wblk="); UHEX(wblk);
	PUTS(" romblk="); UHEX(romblk);
	PUTS(" thr="); UHEX(g_conf_thr);
	PUTS("\n");
	tlast = E_MTIME;

	/* ★★★ 时序：arm → 等新帧(帧号变化+pad_done) → 撤 arm → settle → 推理 → 解码 → 画框 → 重 arm ★★★
	 * 撤 arm 的原因（2026-10-25 实测）：检测头输出就在 y3tram 的**前 96 块**
	 *   (H_cv20_k2@块0 / H_cv30_k2@36 / H_cv21_k2@72 / H_cv31_k2@81)，
	 *   而 IN 区也是从块 0 开始的 [0,9216)。若让 PP 一直开着（30fps 连续写 IN），
	 *   引擎算完写好的检测头会在下一帧立刻被 PP 覆盖 ⇒ 解码读到残值，
	 *   表现为"框冻住不动 / 只有一个框 / 不跟物体"。
	 *   所以推理期间必须撤 arm，跟固件 v47 的 y3t_auto 保持一致。 */
	/* ★ 采集策略 v2（2026-10-29）：照搬固件 y3t_auto 的事件驱动——PP 常 arm，
	 *   轮询「帧号变化 且 pad_done(bit8)」，替代旧版 80+400+50ms 固定死等
	 *   （旧版帧率被压到 ~2fps，注释自称"每帧输入完整"）。
	 *   pad_done 残留误判由「帧号变化」条件天然消除：残留时帧号必未变。
	 *   超时 4000 轮（每轮 1ms）放行，避免 PP 异常时死等。 */
	uint32_t s_wait    = 0u;
	uint32_t s_last_ok = E_MTIME;           /* 2s 看门狗时间戳 */
	PP_CSR = PP_ARM;
	/* ★ last_fc 必须锁存 arm 时刻的当前 fc：等的是"下一个 fc"（=一次完整发射结束）。
	 *   【2026-10-29 教训】若置 0xFF 强制立刻处理，会在主帧发射中途就 de-arm，
	 *   frame_cnt 从此永远不走 ⇒ 2s 看门狗自锁循环（每 2s 处理一个残帧）。 */
	uint8_t  last_fc   = PP_FCN(PP_CSR);
	for (;;) {
		uint32_t st, fc;
		/* ★ 串口命令（单字节, 每帧轮询, 2026-10-30 运行中热调）:
		 *   'q'      退出循环（回固件）
		 *   '0'..'9' 置信度十档 0.15/0.20/0.25/0.30/0.35/0.40/0.50/0.60/0.70/0.80
		 *   '+'/'-'  原始阈值 ±128 微调（i16 分数域）
		 *   'r'      恢复表内默认（conf≈0.30, 逐头精确值）
		 * 单字节协议防丢: 每帧(~100ms)轮询一次, UART FIFO 足以暂存。 */
		while (UART_STATUS & 1u) {
			uint8_t cmd = (uint8_t)UART_DATA;
			if (cmd == 'q') {
				/* ★ 字节碰撞防护(2026-10-30): 退出需魔数确认 'q' 0x55 0xAA 三连。
				 * 教训: 静态评测的图像裸流中 RGB=113(0x71)='q' 很常见(灰像素),
				 * 旧版单 'q' 即退出 ⇒ 一次收图出错后残留数据流就能把 blob 打死。 */
				uint32_t tconf = E_MTIME;
				uint8_t ok = 0u;
				while ((uint32_t)(E_MTIME - tconf) < 2700000u) {   /* 50ms 等确认 */
					if (UART_STATUS & 1u) {
						uint8_t c2 = (uint8_t)UART_DATA;
						if (ok == 0u && c2 == 0x55u)      ok = 1u;
						else if (ok == 1u && c2 == 0xAAu) { ok = 2u; break; }
						else                              { ok = 0u; break; }
					}
				}
				if (ok == 2u) {
					PP_CSR = 0u;
					PUTS("[LOOP] exit by uart, f="); UDEC(nframe); PUTS("\n");
					return;
				}
				PUTS("[THR] q 未确认(需 q 55 AA), 忽略\n");
			} else if (cmd >= '0' && cmd <= '9') {
				/* 十档 conf 的 logit(Q12) 常量: ln(c/(1-c))×4096 */
				static const int32_t LOGIT_Q12[10] = {
					-7106, -5679, -4500, -3471, -2536,
					-1661, 0, 1661, 3471, 5679 };
				int32_t lq = LOGIT_Q12[cmd - '0'];
				/* thr = logit / s_out, 定点: (logit_q12<<12)/s_out_q24
				 * 以 scale0 分数头(H_cv30_k2, s_out=4.588e-4)折算, 0.3 档≈-1847 */
				g_conf_thr = (uint32_t)((int64_t)lq * 4096 /
				                        (int64_t)g_det[0][1].s_out_q24);
				PUTS("[THR] conf档"); UDEC((uint32_t)(cmd - '0'));
				PUTS(" thr="); UDEC(g_conf_thr & 0xFFFFu); PUTS("\n");
			} else if (cmd == '+') {
				g_conf_thr += 128u;
				PUTS("[THR] +128 -> "); UDEC(g_conf_thr & 0xFFFFu); PUTS("\n");
			} else if (cmd == '-') {
				g_conf_thr -= 128u;
				PUTS("[THR] -128 -> "); UDEC(g_conf_thr & 0xFFFFu); PUTS("\n");
			} else if (cmd == 'r') {
				g_conf_thr = (uint32_t)(-32768);   /* 哨兵 = 逐头用表内默认 */
				PUTS("[THR] 恢复表内默认(≈0.30)\n");
			} else if (cmd == 'D') {
				/* ★ 静态评测模式 (2026-10-30): 接收 96x96x3 RGB u8 裸流 (27648B)
				 * + 1 字节异或校验 → 量化写 y3tram IN → 跑整网 → 回传 [DET] 行。
				 * 魔数头 0xDE 0xAD 确认后才收图(防数据流误触发, 同 'q' 碰撞教训)。
				 * 量化与训练同口径: q = round(v/255/s_in) = round(v*128.5)。
				 * y3tram 窗口只支持 32 位访问 (act_get 同款教训), 按 u32 组块写。 */
				{
					uint32_t tm = E_MTIME;
					uint8_t m = 0u;
					while ((uint32_t)(E_MTIME - tm) < 2700000u) {   /* 50ms 等魔数 */
						if (UART_STATUS & 1u) {
							uint8_t c = (uint8_t)UART_DATA;
							if (m == 0u && c == 0xDEu)      m = 1u;
							else if (m == 1u && c == 0xADu) { m = 2u; break; }
							else                            break;   /* 魔数不符, 丢弃该 D */
						}
					}
					if (m != 2u)
						goto d_skip;
				}
				static uint8_t rowbuf[288];        /* 一行 96px*3ch, 流式处理省 RAM */
				uint32_t y, x, got, xorv = 0u, idle = 0u;
				int err = 0;
				PP_CSR = 0u;                        /* 撤 arm: 相机不得覆盖 IN */
				for (y = 0u; y < 96u && !err; y++) {
					got = 0u;
					while (got < 288u) {
						if (UART_STATUS & 1u) {
							rowbuf[got] = (uint8_t)UART_DATA;
							xorv ^= rowbuf[got];
							got++;
							idle = 0u;
						} else if (++idle > 270000000u) {   /* ~5s 无数据超时 */
							err = 1;
							break;
						}
					}
					for (x = 0u; x < 96u; x++) {
						int32_t rr = rowbuf[x * 3u + 0u];
						int32_t gg = rowbuf[x * 3u + 1u];
						int32_t bb = rowbuf[x * 3u + 2u];
						int32_t qr = rr * 128 + ((rr + 1) >> 1);   /* round(v*128.5) */
						int32_t qg = gg * 128 + ((gg + 1) >> 1);
						int32_t qb = bb * 128 + ((bb + 1) >> 1);
						volatile uint32_t *w = (volatile uint32_t *)
							(Y3T_BASE + (y * 96u + x) * 32u);
						w[0] = (uint32_t)(uint16_t)qr | ((uint32_t)(uint16_t)qg << 16);
						w[1] = (uint32_t)(uint16_t)qb;             /* lane2, lane3=0 */
						w[2] = 0u; w[3] = 0u; w[4] = 0u;
						w[5] = 0u; w[6] = 0u; w[7] = 0u;            /* lane4..15 = 0 */
					}
				}
				/* 校验字节 */
				idle = 0u;
				while (!(UART_STATUS & 1u)) {
					if (++idle > 54000000u) { err = 2; break; }   /* 1s */
				}
				if (!err && (uint8_t)UART_DATA != (uint8_t)xorv) err = 3;
				if (err) {
					PUTS("[DET] ERR "); UDEC((uint32_t)err); PUTS("\n");
					/* ★ 出错后冲刷串口 2s: PC 端可能还在发剩余图像字节,
					 *   残留流若进命令层会误触发(含旧 'q' 退出)。读而丢弃。 */
					{
						uint32_t td = E_MTIME;
						while ((uint32_t)(E_MTIME - td) < 108000000u) {
							if (UART_STATUS & 1u) (void)UART_DATA;
						}
					}
				} else {
					int nd;
					(void)run_net(wblk);            /* 结果留在 y3tram 头区 */
					nd = det_decode((const int16_t *)Y3T_BASE, 8, bx);
					PUTS("[DET] n="); UDEC((uint32_t)nd);
					for (int i = 0; i < nd; i++) {
						PUTS(" c"); UDEC(bx[i].cls);
						PUTS("/q"); UDEC((uint32_t)(uint16_t)bx[i].conf_q);
						PUTS("/x"); UDEC((uint32_t)bx[i].cx96_q8);
						PUTS("/y"); UDEC((uint32_t)bx[i].cy96_q8);
						PUTS("/w"); UDEC((uint32_t)bx[i].w96_q8);
						PUTS("/h"); UDEC((uint32_t)bx[i].h96_q8);
					}
					PUTS("\n");
				}
				PP_CSR = PP_ARM;                    /* 恢复相机循环 */
				d_skip: ;                           /* 魔数不符时跳到此处 */
			}
		}
		st = PP_CSR;
		fc = PP_FCN(st);
		if ((uint8_t)fc == last_fc) {
			/* 无新帧。2s 看门狗：连续无成功帧 => 撤arm→1ms→arm→强制重处理（自愈） */
			if (E_MTIME - s_last_ok > 108108000u) {  /* 2s @54MHz */
				PP_CSR = 0u;
				{
					uint32_t tw = E_MTIME;
					while ((uint32_t)(E_MTIME - tw) < 54054u) { }   /* 1ms @54MHz */
				}
				PP_CSR = PP_ARM;
				last_fc = PP_FCN(PP_CSR); s_wait = 0u; s_last_ok = E_MTIME;
				PUTS("[WD] kick last_fc="); UHEX(last_fc); PUTS("\n");
			}
			continue;                        /* 快速轮询，不占帧周期 */
		}
		/* 新帧到了：等 letterbox+PAD 完成（~127ms，约 127 轮），超时放行防死等 */
		if (!(st & PP_PAD_DONE)) {
			uint32_t tw = E_MTIME;
			while ((uint32_t)(E_MTIME - tw) < 54054u) { }   /* 1ms @54MHz */
			if (++s_wait < 4000u)
				continue;
		}
		s_wait = 0u;
		last_fc = (uint8_t)fc;
		/* 推理期间撤 arm：输入不被下一帧覆盖、检测头不被 PP 冲掉 */
		PP_CSR = 0u;
		{
			uint32_t tw = E_MTIME;
			while ((uint32_t)(E_MTIME - tw) < SETTLE_CYC) { }   /* 20ms 落盘 */
		}
		t0 = E_MTIME;
		if (run_net(wblk) == 0) {
			int nd = det_decode((const int16_t *)Y3T_BASE, 8, bx);
			uint32_t now = E_MTIME;
			uint32_t period = now - tprev;   /* 完整帧周期（含 PP 等待） */
			tprev = now;
			draw_boxes(bx, nd);
			/* 2026-10-30: 结果发布到 GPIO/LED —— top-1 类别码亮 LED3..6,
			 *             LED7=有目标, bit5 每帧翻转(心跳)。无目标则类别码清 0。 */
			{
				int gcls = -1, gi, gbest = -32768;
				for (gi = 0; gi < nd; gi++) {
					if ((int)bx[gi].conf_q > gbest) {
						gbest = (int)bx[gi].conf_q;
						gcls  = (int)bx[gi].cls;
					}
				}
				gpio_publish(gcls, nd > 0);
			}
			/* 4 字符窗口复用: 每 4 帧前 3 帧显示 类别+置信度, 第 4 帧显示 FPS。
			 * 【2026-10-29】撤销闪烁占空比（恢复标签常亮）。 */
			if ((nframe & 3u) != 3u) draw_label(bx, nd);
			else                     draw_fps(period);
			nrun++;
			s_last_ok = E_MTIME;             /* 成功帧刷新看门狗时间戳（否则每 2s 空踢一次） */
			if ((nframe & 3u) == 0u) {           /* 每 4 帧打一行 */
				const volatile uint32_t *inw =
					(const volatile uint32_t *)(Y3T_BASE + 3000u * 32u);
				uint32_t ck = 0u;
				for (uint32_t z = 0u; z < 8u; z++) ck += inw[z * 32u];
				PUTS("[AUTO] f="); UDEC(nframe);
				PUTS(" det="); UDEC((uint32_t)nd);
				PUTS(" net="); UDEC(E_MTIME - t0);
				PUTS(" tot="); UDEC(period);
				PUTS(" ck="); UHEX(ck);
				PUTS(" b0="); UDEC((uint32_t)g_db0x0); PUTS(",");
				UDEC((uint32_t)g_db0y0); PUTS(",");
				UDEC((uint32_t)g_db0x1); PUTS(",");
				UDEC((uint32_t)g_db0y1);
				PUTS(" thr="); UHEX(g_conf_thr);
				PUTS(" st="); UHEX(st);
				PUTS("\n");
			}
		} else {
			nfail++;
			PP_CSR = 1u;                    /* 重新请求下一帧 */
			PUTS("[AUTO] FAIL f="); UDEC(nframe);
			PUTS(" nfail="); UDEC(nfail); PUTS("\n");
		}
		PP_CSR = 1u;                        /* 重新 arm，等下一帧 */
		nframe++;
		(void)tlast;
	}
}
