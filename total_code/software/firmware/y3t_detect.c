/* =====================================================================
 * y3t_detect.c —— Y3T 检测头解码（全整数, 不依赖 FPU / libm）
 *
 * 为什么全整数
 *   RISC-V 核是 rv32imac（无 F/D 扩展）, 编译用 -nostdlib 也没有 libm。
 *   而解码只需要"某个位置的分数是否过阈值", 于是:
 *       sigmoid(q·s) >= thr   <=>   q·s >= ln(thr/(1-thr))   <=>   q >= ln(...)/s
 *   右侧全是编译期常量 ⇒ 运行期只做 int16 比较, 连一次乘法都不需要。
 *   （阈值在 y3t_detect_tbl.h 里已折成 thr_q）
 *
 * 数据布局（与 y3t_mem_ahb 一致）
 *   激活区按 256b 字存, 每个字放 16 个 int16。
 *   通道按 16 对齐分组: 通道 c 位于「组 g = c/16, 组内 lane l = c%16」,
 *   组 g 的字基址 = g * (H*W), 具体字 = g*(H*W) + y*W + x, 再取该字的第 l 个 int16。
 *
 * 坐标约定（与 tools/det_read.py 完全一致, 便于交叉验证）
 *   lt, rb = box[0..1], box[2..3]
 *   ax = x + 0.5, ay = y + 0.5
 *   x1 = ax - lt, y1 = ay - lt_y, x2 = ax + rb_x, y2 = ay + rb_y
 *   xywh96 = [(x1+x2)/2*stride, (y1+y2)/2*stride, (x2-x1)*stride, (y2-y1)*stride]
 *   这里用 Q8 定点算中心与宽高, 避免浮点。
 *
 * 输出
 *   det_box_t 数组, 按 conf_q 降序（conf_q = 命中的 int16 分数, 越大越可信）。
 *   调用者负责 NMS（本文件不做, 保持解码层纯粹）。
 * ===================================================================== */
#include "y3t_detect.h"
#include "y3t_detect_tbl.h"

#define Y3TMEM_WORDS_PER_BLK  (256u / 16u)   /* 1 字 = 16 个 int16 */

/* 取 (y, x, c) 处的 int16 激活值 */
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
 *   s_out 量级 ~1e-4, 用 Q16.16 只能表示成 4 (误差 5%), 故用 Q24 = s_out*2^24
 * 【Q8-FIX 2026-10-28, patch01】原实现返回整数网格单位, 而 l/r/t/b 是 0~1 的
 *   亚格量（box 头 s_out≈7.8e-5, 一格≈12800 计数）⇒ 取整后只剩 0/1,
 *   宽度只能是 0 或 16/32 px。改返回 Q8(×256) 后几何偏差 13.48px → 0.057px。 */
static inline int32_t qmul_q8(int32_t a_q24, int32_t b)
{
	int64_t t = (int64_t)a_q24 * (int64_t)b;
	if (t >= 0) t += (1 << 15);      /* 半个 Q8 LSB */
	else        t -= (1 << 15);
	return (int32_t)(t >> 16);       /* 2^24 / 2^8 = 2^16 */
}

/* 运行时阈值覆盖 (0x16 SET_THR 可改): -32768 = 用表内默认 (conf_thr=0.30)。
 * 换算: conf=c 的分数阈值 thr_q = ln(c/(1-c)) / s_out, PC 侧 tools/det_thr.py 会算好发来 */
int32_t g_det_thr_ovr[DET_NSCALE] = { -32768, -32768 };

int det_decode(const int16_t *y3t, int max_out, det_box_t *out)
{
	det_box_t cand[24];                 /* 先全收, 再 NMS 去重（同 blob_auto v2） */
	int ncand = 0;
	int n = 0;

	for (uint32_t sc = 0; sc < DET_NSCALE; sc++) {
		const det_head_t *hb = &g_det[sc][0];   /* box  : 4ch ltrb */
		const det_head_t *hs = &g_det[sc][1];   /* score: oc ch    */
		const uint32_t H = hb->H, W = hb->W;
		const uint32_t stride = 96u / H;        /* 网格步长 (96 / 网格高) */
		const int32_t  sq = hb->s_out_q24;      /* 框头反量化 Q24 */
		const int16_t *bb = y3t + (int32_t)hb->out_blk * Y3TMEM_WORDS_PER_BLK;
		const int16_t *bs = y3t + (int32_t)hs->out_blk * Y3TMEM_WORDS_PER_BLK;
		const uint32_t ncls = hs->oc;

		for (uint32_t y = 0; y < H; y++) {
			for (uint32_t x = 0; x < W; x++) {
				/* ---- 1) 找该格最大分数的类别 ---- */
				int32_t  best = -32768;
				uint32_t bestc = 0;
				for (uint32_t c = 0; c < ncls; c++) {
					int32_t v = act_get(bs, H, W, y, x, c);
					if (v > best) { best = v; bestc = c; }
				}
				/* ---- 2) 阈值比较（整数, 不用 sigmoid）----
			 * 0x16 SET_THR 的运行时覆盖优先, -32768 表示用表内默认 */
				if (best < ((g_det_thr_ovr[sc] != -32768)
				            ? g_det_thr_ovr[sc] : hs->thr_q)) continue;
				if (ncand >= (int)(sizeof cand / sizeof cand[0])) break;

				/* ---- 3) ltrb -> 中心/宽高（Q8 定点）---- */
				/* 先把网格中心化成 Q8: (x+0.5)*256 = x*256 + 128 */
				int32_t ax_q8 = (int32_t)x * 256 + 128;
				int32_t ay_q8 = (int32_t)y * 256 + 128;
				int32_t l = qmul_q8(sq, act_get(bb, H, W, y, x, 0));
				int32_t t = qmul_q8(sq, act_get(bb, H, W, y, x, 1));
				int32_t r = qmul_q8(sq, act_get(bb, H, W, y, x, 2));
				int32_t b = qmul_q8(sq, act_get(bb, H, W, y, x, 3));
				/* l/r/t/b 已是 Q8 网格, ax/ay 也是 Q8 ⇒ 直接加减（Q8-FIX） */
				int32_t cx_q8 = ax_q8 + ((r - l) / 2);
				int32_t cy_q8 = ay_q8 + ((b - t) / 2);
				int32_t w_q8  = l + r;
				int32_t h_q8  = t + b;

				/* ---- 4) 乘 stride 得到 96x96 坐标（Q8）---- */
				cand[ncand].cx96_q8 = cx_q8 * (int32_t)stride;
				cand[ncand].cy96_q8 = cy_q8 * (int32_t)stride;
				cand[ncand].w96_q8  = w_q8  * (int32_t)stride;
				cand[ncand].h96_q8  = h_q8  * (int32_t)stride;
				cand[ncand].cls     = (uint8_t)bestc;
				cand[ncand].scale   = (uint8_t)sc;
				cand[ncand].conf_q  = (int16_t)best;
				ncand++;
			}
			if (ncand >= (int)(sizeof cand / sizeof cand[0])) break;
		}
	}

	/* ---- NMS（贪心, 按类别去重）【2026-10-29, 同 blob_auto】----
	 * 无 NMS 时同一物体横跨多个网格 ⇒ 每格一框, 叠最多 8 个嵌套矩形。
	 * 按置信度降序保留, 与已保留同类别框 IoU>0.35 的丢弃。全整数。
	 * 注意: -nostdlib 禁 ={0}/结构体赋值（GCC 会生成 memset/memcpy）。 */
	{
		char kept[sizeof cand / sizeof cand[0]];
		int  i, pass;
		for (i = 0; i < (int)(sizeof kept); i++) kept[i] = 0;
		for (pass = 0; pass < ncand; pass++) {
			int bi = -1;
			for (i = 0; i < ncand; i++)
				if (!kept[i] && (bi < 0 || cand[i].conf_q > cand[bi].conf_q)) bi = i;
			if (bi < 0) break;
			kept[bi] = 1;
			if (n < max_out) {
				out[n].cx96_q8 = cand[bi].cx96_q8;
				out[n].cy96_q8 = cand[bi].cy96_q8;
				out[n].w96_q8  = cand[bi].w96_q8;
				out[n].h96_q8  = cand[bi].h96_q8;
				out[n].cls     = cand[bi].cls;
				out[n].scale   = cand[bi].scale;
				out[n].conf_q  = cand[bi].conf_q;
				n++;
			}
			{
				int64_t x1 = (int64_t)cand[bi].cx96_q8 - cand[bi].w96_q8 / 2;
				int64_t y1 = (int64_t)cand[bi].cy96_q8 - cand[bi].h96_q8 / 2;
				int64_t x2 = x1 + cand[bi].w96_q8, y2 = y1 + cand[bi].h96_q8;
				for (i = 0; i < ncand; i++) {
					int64_t a1, b1, a2, b2, iw, ih, inter, uni;
					if (kept[i] || cand[i].cls != cand[bi].cls) continue;
					a1 = (int64_t)cand[i].cx96_q8 - cand[i].w96_q8 / 2;
					b1 = (int64_t)cand[i].cy96_q8 - cand[i].h96_q8 / 2;
					a2 = a1 + cand[i].w96_q8; b2 = b1 + cand[i].h96_q8;
					iw = (a2 < x2 ? a2 : x2) - (a1 > x1 ? a1 : x1);
					ih = (b2 < y2 ? b2 : y2) - (b1 > y1 ? b1 : y1);
					if (iw <= 0 || ih <= 0) continue;
					inter = iw * ih;
					uni = (int64_t)cand[i].w96_q8 * cand[i].h96_q8
					    + (int64_t)cand[bi].w96_q8 * cand[bi].h96_q8 - inter;
					if (uni > 0 && inter * 100 > uni * 35) kept[i] = 1;
				}
			}
		}
	}
	return n;
}

/* 96x96 坐标 -> 1280x720 帧坐标（Q8）
 * letterbox: r = min(96/720, 96/1280) = 0.075 ; 缩放后 96x54, 上下各 pad 21
 *   x_frame = x96 / 0.075            -> x96 * 1000 / 75
 *   y_frame = (y96 - 21) / 0.075     -> (y96 - 21*256) * 1000 / 75
 */
void det_to_frame_q8(int32_t x96_q8, int32_t y96_q8, int32_t *xf_q8, int32_t *yf_q8)
{
	const int32_t PAD_Q8 = 21 * 256;
	*xf_q8 = (x96_q8 * 1000) / 75;
	*yf_q8 = ((y96_q8 - PAD_Q8) * 1000) / 75;
}
