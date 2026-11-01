/* =====================================================================
 * y3t_auto.c —— 板上自动循环（v37 新增, 2026-10-23）
 *
 *   相机帧 → PP 预处理(RTL, letterbox 96x96) → 整网推理 → 检测解码 → OVL 画框
 *
 * 用法:
 *   PC 发 0x15 AUTO_RUN {u32 en, u32 wblk, u32 romblk}
 *     en=1 启动循环, en=0 停止; wblk/romblk 与 0x10 同义 (默认 480000/580000)。
 *   前提: DDR 里已有整网权重(wblk)与尺度(romblk), 与 0x10 完全相同。
 *
 * 设计要点:
 *   1. 帧同步用 PP 的 frame_cnt (每写完一帧 IN 区自增), 不用 OVL_FRAME ——
 *      后者反映相机输出帧序, PP 可能因 arm/授权而跳帧。
 *   2. 推理期间必须 **撤 PP arm**: 否则 PP 会在引擎读 IN 的同时写下一帧
 *      (eng 口虽有仲裁不撞 burst, 但数据内容会被覆盖) ⇒ 输入被污染。
 *      代价: 推理 + 解码 + 画框期间掉帧, 推理 45ms ⇒ 约 10~15 fps。
 *   3. 检测框用 OVL 框 5..7 (0..3 四角自检, 4 运动框已被占用)。
 *      坐标链: 96 网格(Q8) --det_to_frame_q8--> 1280x720 帧(Q8)
 *              --÷2--> 640 宽屏坐标 (屏是 1280 帧的半宽 1:1 高)。
 *   4. 每秒一行状态 (frame/推理/框数/周期), 不刷屏。
 * ===================================================================== */
#include <stdint.h>
#include "y3t_detect.h"
#include "y3t_detect_tbl.h"  /* v72: conf_pct 反量化系数 g_det[..][1].s_out_q24 */
#include "gpio_ctl.h"      /* v65: 结果发布到 GPIO */

/* ---- PP (预处理 RTL) CSR @0x4000_4000 --------------------------------
 * 读回布局 (boardsoc_top.v pp_hrdata):
 *   {14'd0, frame_cnt[7:0], fifo_ovf, pad_done, 7'd0, arm}
 *   bit0=arm, bit8=pad_done, bit9=fifo_ovf, bit[17:10]=frame_cnt */
#define PP_CSR        (*(volatile uint32_t *)0x40004000u)
#define PP_ARM        0x1u
#define PP_FCN(st)    (((st) >> 10) & 0xFFu)
#define PP_PAD_DONE   (1u << 8)   /* v64: 本帧 letterbox+pad 已完成 */

/* ---- overlay (与 demo.c 同口径) ---- */
#define OVL_BASE     0x40006000u
#define REG32(a)     (*(volatile uint32_t *)(a))
#define OVL_MIN(n)   REG32(OVL_BASE + (n) * 12u)
#define OVL_MAX(n)   REG32(OVL_BASE + (n) * 12u + 4u)
#define OVL_COLOR(n) REG32(OVL_BASE + (n) * 12u + 8u)
#define OVL_EN       REG32(OVL_BASE + 0x60u)
/* v47 文字层寄存器: 0x70 {ch1,ch0} 0x74 {ch3,ch2} 0x78 色 0x7C {en,y,x} */
#define TXT_CH01     REG32(OVL_BASE + 0x70u)
#define TXT_CH23     REG32(OVL_BASE + 0x74u)
#define TXT_COLOR    REG32(OVL_BASE + 0x78u)
#define TXT_POS_EN   REG32(OVL_BASE + 0x7Cu)
#define OVL_BOX_EN   0xE0u                     /* 框 5..7 */

/* ---- 串口 (demo.c 提供) ---- */
extern void uart_puts(const char *s);
extern void uart_dec(uint32_t v);
extern void uart_hex(uint32_t v);
extern int  demo_net_run(uint32_t wblk, uint32_t romblk, uint32_t *vals6);
extern volatile uint32_t g_ovl_mask;

/* demo.c 的 mtime() 是 static, 这里直接读 mtime 低 32 位寄存器 */
#define MTIME_LO     REG32(0x40002000u)

/* ---- 循环状态 (0x15 AUTO_RUN 可改) ---- */
uint32_t g_auto_en     = 0u;
uint32_t g_auto_wblk   = 480000u;
uint32_t g_auto_romblk = 580000u;
/* v66: 撤 arm 后落盘等待拍数, 0x19 SET_SETTLE 运行时可调 (默认 20ms 不变行为)。
 *   扫参下限 2ms: PP 在途写入经仲裁最深的块数有限, 20ms 是保守值。 */
uint32_t g_settle_cyc  = 1080000u;

/* v67: 以下状态变量去 static —— 供 0x1A AUTO_DUMP 转储 (uart_cmd.c extern) */
uint8_t  s_last_fc   = 0xFFu;   /* 上次处理的帧号 (0xFF=强制先跑一次) */
uint32_t s_wait      = 0u;      /* v64: 等 pad_done 的轮次计数 */
uint32_t s_same      = 0u;      /* 无新帧轮次计数 (自愈用) */
static uint32_t s_run_cnt   = 0;
static uint32_t s_last_rep  = 0;       /* 上次状态打印时刻 (秒) */
/* v67: 帧看门狗 —— 2026-10-27 实测 PP 偶发掉 arm 后楔死 (对 arm 写不敏感,
 *   旧自愈 20000 轮补写 arm=1 被 wedge 态无视, 数分钟不恢复; wedge 自清后
 *   同样的写立即生效)。连续 2s 无成功处理帧 => 强制恢复序列。 */
static uint32_t s_last_ok   = 1u;      /* 上次成功处理帧的 MTIME_LO (1=未初始化) */
uint32_t        s_wd_cnt    = 0u;      /* 看门狗踢的次数 (诊断, 0x1A 转储) */

#define NDET 8                  /* overlay 硬件共 8 个框槽, 全部用于检测框 */
static det_box_t s_box[NDET];

/* ---- v72 (2026-10-29): 置信度百分数 + FPS 角标, 从 blob_auto.c 同款移植 ---- */
static uint32_t s_frame_cnt    = 0u;    /* 处理帧计数 (FPS 轮显分频用) */
static uint32_t s_frame_period = 0u;    /* 上帧完整周期 (MTIME 拍, 54.054MHz) */
static uint32_t s_prev_ok      = 0u;    /* 上上次成功帧时刻 (差分出周期) */

/* conf_q(原始 logit 分数, i16) × 该头 s_out → 置信度百分数 2..98。
 * 17 项 LUT 近似 sigmoid（步长 0.5, 覆盖 logit ±4）+ 线性插值（误差 <2 点）。 */
static uint8_t conf_pct(int32_t conf_q, int32_t s_out_q24)
{
	static const uint8_t LUT[17] = {
		2, 3, 5, 8, 12, 18, 27, 38, 50,
		62, 73, 82, 88, 92, 95, 97, 98 };
	int32_t lg = (int32_t)((int64_t)conf_q * s_out_q24 >> 12) + 16384;
	int32_t idx, frac, a, b;
	if (lg < 0)     lg = 0;
	if (lg > 32768) lg = 32768;
	idx  = lg >> 11;
	if (idx > 16) idx = 16;
	frac = lg & 2047;
	a = LUT[idx];
	b = (idx < 16) ? LUT[idx + 1] : a;
	return (uint8_t)(a + ((b - a) * frac >> 11));
}

/* FPS 显示: 帧周期(拍) → fps×10 → "12.3" 固定屏幕左上角 (8,8), 白色。
 * '.' 与数字 0..9 均在 overlay_boxes.v 字库中。 */
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

static void ovl_box(uint32_t n, uint32_t xmin, uint32_t ymin,
                    uint32_t xmax, uint32_t ymax, uint32_t rgb565)
{
	OVL_MIN(n)   = ((ymin & 0xFFFu) << 16) | (xmin & 0xFFFu);
	OVL_MAX(n)   = ((ymax & 0xFFFu) << 16) | (xmax & 0xFFFu);
	OVL_COLOR(n) = rgb565;
}

void y3t_auto_step(void)
{
	uint32_t st  = PP_CSR;
	uint32_t fc  = PP_FCN(st);
	uint32_t vals[6];
	int nb, i;
	uint32_t now, drawn = 0;

	if ((uint8_t)fc == s_last_fc) {
		/* 没有新帧：若连续 ~1s 无新帧且 PP 未 arm，主动重新 arm（自愈） */
		uint32_t now2 = MTIME_LO;
		/* v67: 2s 帧看门狗 —— 连续 2s 无成功处理帧 => 强制恢复序列:
		 *   撤 arm -> 1ms -> arm -> s_last_fc=0xFF 强制重处理当前帧。 */
		if ((uint32_t)(now2 - s_last_ok) > 108108000u) {
			PP_CSR = 0u;
			{
				uint32_t t3 = MTIME_LO;
				while ((uint32_t)(MTIME_LO - t3) < 54054u) { }   /* 1ms */
			}
			PP_CSR = PP_ARM;
			s_last_fc = 0xFFu;
			s_wait    = 0u;
			s_same    = 0u;
			s_last_ok = now2;
			s_wd_cnt++;
			if (s_wd_cnt <= 4u || (s_wd_cnt & 0x3Fu) == 0u) {
				uart_puts("[AUTO] watchdog kick #");
				uart_dec(s_wd_cnt);
				uart_puts("\n");
			}
			return;
		}
		if (++s_same > 20000u) {
			s_same = 0u;
			if (!(st & PP_ARM)) PP_CSR = PP_ARM;
		}
		s_wait = 0u;
		return;
	}
	/* v64 ★ 必须等本帧 padding 完成(bit8) 再撤 arm。
	 *   原因: frame_cnt 在"帧开始"就变，而 letterbox+PAD 要 127ms 才完成；
	 *   撤早了 IN 还是半成品 —— 实测同一场景 toy 置信度从 0.89 掉到 0.4 以下，
	 *   表现为"能出框但类别乱/漏检"。
	 *   超时 4000 轮放行，避免 PP 异常时死等。 */
	if (!(st & PP_PAD_DONE) && ++s_wait < 4000u)
		return;
	s_wait = 0u;
	s_last_fc = (uint8_t)fc;

	/* 1. 推理期间撤 arm: PP 不再写 IN, 输入不被下一帧覆盖
	 *    ★ 只写控制位 0/1 —— 绝不能把回读的帧号/pad_done/ovf 一起写回，
	 *      否则 PP 状态机卡死（2026-10-25 实测：写回后 arm 写 1 也不再响应）。 */
	PP_CSR = 0u;
	/* ★ 撤 arm 后 PP 可能还有经仲裁的在途写入，必须等它落盘再跑整网，
	 *   否则整网读到半成品。默认 20ms @54MHz = 1,080,000 拍；
	 *   v66: 拍数可经 0x19 在线调 (g_settle_cyc), 扫参压帧率。 */
	{
		uint32_t t0 = MTIME_LO;
		while ((uint32_t)(MTIME_LO - t0) < g_settle_cyc) { }
	}

	/* 2. 整网 (与 0x10 同一条路径: 权重 DMA + 逐层尺度 ROM) */
	if (demo_net_run(g_auto_wblk, g_auto_romblk, vals) != 0) {
		uart_puts("[AUTO] net FAIL ret=");
		uart_dec(vals[0]);
		uart_puts("\n");
		PP_CSR = PP_ARM;                          /* 恢复, 下帧再试 */
		return;
	}

	/* 3. 解码 (全整数, 直接读 y3tram 检测头) */
	/* v65: gpio_in[1] (KEY2) = 阈值模式 —— 高电平=宽(conf 0.15), 低=严(conf 0.25)。
	 *   thr_q = ln(c/(1-c)) / s_out,  s_out0=3.3176e-4, s_out1=3.7205e-4:
	 *     0.25 -> (-3311, -2953)   0.15 -> (-5229, -4663)
	 *   只在模式变化时写一次, 避免每帧重复做除法。 */
	{
		static int32_t s_thr_mode = -1;
		int32_t mode = (GPIO_DATA_IN & 0x00000002u) ? 1 : 0;
		if (mode != s_thr_mode) {
			s_thr_mode = mode;
			g_det_thr_ovr[0] = mode ? -5229 : -3311;
			g_det_thr_ovr[1] = mode ? -4663 : -2953;
		}
	}

	nb = det_decode((const int16_t *)0x60000000u, NDET, s_box);

	/* v65: 把本帧结果发布到 GPIO (对外执行器接口, 见 sw/gpio_ctl.h)。
	 *   取置信度最高的那个框; 无框时 valid=0。心跳位每帧翻转, 外部可判"系统在跑"。 */
	{
		int      gcls = -1;
		int32_t  gconf = 0;
		uint32_t gi;
		for (gi = 0; gi < (uint32_t)nb; gi++) {
			if ((int32_t)s_box[gi].conf_q > gconf) {
				gconf = (int32_t)s_box[gi].conf_q;
				gcls  = (int)s_box[gi].cls;
			}
		}
		gpio_publish(gcls, nb > 0 ? 1 : 0);
	}

	/* 4. 画框: overlay 硬件共 8 个框槽 (0..7), 全部用于检测框。
	 *    ★ 每帧先把没被本轮用到的框槽清成 (0,0)-(0,0)，否则上一帧的旧框
	 *      会残留在屏幕上（"这帧 3 个框、下帧 1 个框"时多出的 2 个不消失）。
	 *    自动模式下相机帧差分支不跑，槽 4(运动框) 不会被固件重画，可安全占用。 */
	{
		uint32_t used = 0;
		for (i = 0; i < nb; i++) {
			int32_t xf_q8, yf_q8;
			uint32_t n = (uint32_t)i;
			det_to_frame_q8(s_box[i].cx96_q8, s_box[i].cy96_q8, &xf_q8, &yf_q8);
			{
				/* 帧(Q8, 1280x720) -> 屏幕(1280x720，overlay 接的是整条合成流)
				 * ★BUG-FIX 2026-10-25 两处：
				 *  ① 宽高原来写成 w96_q8>>9 / h96_q8>>8（把"96 格 Q8"当像素），
				 *     少乘了 96格→1280帧 的 1000/75；
				 *  ② 坐标原来又对 640 半屏折半（>>9），但 `video_rect_read_data_inst1`
				 *     是级联在 inst0 后面的，overlay 拿到的是**整条 1280 宽合成流**
				 *     （左半 cam0 / 右半 cam1）⇒ 必须用 1280 空间。
				 *   两处叠加导致框"又小又偏、不在物体上"。 */
				int32_t wf_q8 = (s_box[i].w96_q8 * 1000) / 75;
				int32_t hf_q8 = (s_box[i].h96_q8 * 1000) / 75;
				/* overlay 的 x_cnt 是整条 1280 宽合成流（左半 cam0 / 右半 cam1），
				 * 但 cam0 画面存成 640x720 后 1:1 贴在左半屏 ⇒
				 * **帧坐标必须 ÷2 才是屏幕坐标**（宽也一样）。 */
				int32_t sx = (xf_q8 >> 9);            /* 1280 -> 屏幕左半 640 */
				int32_t sy = (yf_q8 >> 8);            /* 720 直通 */
				int32_t hw = (wf_q8 >> 9);
				int32_t hh = (hf_q8 >> 8);
				int32_t x0 = sx - hw / 2, y0 = sy - hh / 2;
				int32_t x1 = sx + hw / 2, y1 = sy + hh / 2;
				if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
				if (x1 > 639) x1 = 639; if (y1 > 719) y1 = 719;   /* 只画在左半屏(cam0) */
				if (x1 <= x0 || y1 <= y0) continue;
				ovl_box(n, (uint32_t)x0, (uint32_t)y0, (uint32_t)x1, (uint32_t)y1,
				        (s_box[i].scale == 0u) ? 0xF800u : 0x07E0u);
				used |= (1u << n);
				drawn++;
			}
		}
		/* 清掉本轮没用到的槽（包括 nb 超过 8 时溢出的），防止旧框残留 */
		for (i = 0; i < 8; i++)
			if (!(used & (1u << i)))
				ovl_box((uint32_t)i, 0, 0, 0, 0, 0);
	}
	/* 有框才点亮 0..7 (没框时全灭, 避免残留) */
	{
		uint32_t en = OVL_EN & ~0xFFu;
		if (drawn && (g_ovl_mask & 0x04u))
			en |= 0xFFu;
		OVL_EN = en;
	}

	/* 4b. v47 文字层: 置信度最高的框 → 4 字符标签
	 * 【v72 2026-10-29, 同 blob_auto】格式 "B85%" = 类别首字母+置信度百分数+%；
	 * 每 4 帧第 4 帧改显 FPS（固定左上角, 白色）。NM 顺序=训练 classes.txt。 */
	if (nb > 0) {
		static const char *const nm[5] = { "BOTT", "MOUS", "CUP ", "TOY ", "SCRN" };
		int best = 0;
		uint32_t k2;
		for (k2 = 1u; k2 < (uint32_t)nb; k2++)
			if (s_box[k2].conf_q > s_box[best].conf_q) best = (int)k2;
		{
			char t[4];
			uint8_t c = (uint8_t)s_box[best].cls;
			uint32_t col = (s_box[best].scale == 0u) ? 0xF800u : 0x07E0u;
			uint8_t pct = conf_pct(s_box[best].conf_q,
			                       g_det[s_box[best].scale][1].s_out_q24);
			if (c < 5u) t[0] = nm[c][0];       /* B/M/C/T/S 首字母互不相同 */
			else        t[0] = ' ';
			t[1] = (char)('0' + pct / 10);
			t[2] = (char)('0' + pct % 10);
			t[3] = '%';
			if ((s_frame_cnt & 3u) != 3u) {    /* 3/4 帧显示 类别+置信度 */
				TXT_CH01 = ((uint32_t)(uint8_t)t[1] << 8) | (uint8_t)t[0];
				TXT_CH23 = ((uint32_t)(uint8_t)t[3] << 8) | (uint8_t)t[2];
				TXT_COLOR = col;
				TXT_POS_EN = 0x80000000u | (16u << 12) | (8u << 2);
			} else {                            /* 1/4 帧显示 FPS */
				draw_fps(s_frame_period);
			}
		}
	} else if ((s_frame_cnt & 3u) == 3u) {
		draw_fps(s_frame_period);               /* 无框的第 4 帧也显示 FPS */
	} else {
		TXT_POS_EN = 0u;                                  /* 无框则关文字 */
	}

	/* 5. 恢复 PP, 等下一帧 */
	PP_CSR = PP_ARM;
	s_last_ok = MTIME_LO;      /* v67: 本帧成功处理, 喂看门狗 */
	/* v72: 帧周期统计（FPS 角标用）——喂狗点即"帧成功"的唯一定义 */
	s_frame_period = s_last_ok - s_prev_ok;
	s_prev_ok      = s_last_ok;
	s_frame_cnt++;

	s_run_cnt++;
	now = MTIME_LO / 54054000u;
	if (now != s_last_rep) {
		s_last_rep = now;
		uart_puts("[AUTO] t=");
		uart_dec(now);
		uart_puts("s frame=");
		uart_dec(fc);
		uart_puts(" run=");
		uart_dec(s_run_cnt);
		uart_puts(" box=");
		uart_dec(drawn);
		uart_puts(" mac=");
		uart_dec(vals[4]);
		uart_puts(" tot=");
		uart_dec(vals[5]);
	if (PP_CSR & 0x200u) uart_puts(" FIFO_OVF");
	/* v66: 附最高置信框的 原始分(q, ln 域 int16) 与尺度号 —— 上位机换算:
	 *   conf = 1/(1+exp(-q*s_out)), s_out0=3.3176e-4, s_out1=3.7205e-4 */
	if (nb > 0) {
		int b = 0;
		uint32_t k2;
		for (k2 = 1u; k2 < (uint32_t)nb; k2++)
			if (s_box[k2].conf_q > s_box[b].conf_q) b = (int)k2;
		uart_puts(" top=");
		uart_dec((uint32_t)s_box[b].cls);
		uart_puts("/q");
		uart_hex((uint32_t)(int32_t)s_box[b].conf_q);
		uart_puts("/s");
		uart_dec((uint32_t)s_box[b].scale);
	}
	uart_puts("\n");
	}
}
