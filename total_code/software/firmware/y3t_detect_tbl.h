/* y3t_detect_tbl.h —— 自动生成（gen_det_tbl / deploy_newmodel），勿手改
 *   源: y3t_params.json
 *       y3t_net_table.h
 *   conf_thr=0.30 -> logit=-0.847298
 *   阈值折成整数: 判据 qs16 >= thr_q (省掉 sigmoid/exp/浮点)
 *   反量化用 Q24: s_out 量级 1e-4, Q16.16 只能表示成个位数(误差 5%), Q24 误差 <0.01%
 */
#ifndef Y3T_DETECT_TBL_H
#define Y3T_DETECT_TBL_H
#include <stdint.h>

#define DET_NSCALE  2u
#define DET_NOBJ    2u          /* 每尺度: box(4ch) + score(nc) */
#define DET_NCLS    5u

typedef struct {
	uint32_t out_blk;   /* y3tram 内起始块 (256b 字) */
	uint32_t H, W;      /* 网格高宽 */
	uint32_t oc;        /* 通道数 */
	int32_t  thr_q;     /* 命中阈值 (qs16 >= thr_q) */
	int32_t  s_out_q24; /* 反量化系数 Q24 */
} det_head_t;

static const det_head_t g_det[DET_NSCALE][DET_NOBJ] = {
	{ /* scale 0: 6x6 stride=16 */
	  {     0u,  6u,  6u,  4u, -10795,     1317 },  /* H_cv20_k2 box (s_out=7.848765e-05) */
	  {    36u,  6u,  6u,  5u,  -1847,     7697 },  /* H_cv30_k2 score (s_out=0.0004587941) */
	},
	{ /* scale 1: 3x3 stride=32 */
	  {    72u,  3u,  3u,  4u, -10734,     1324 },  /* H_cv21_k2 box (s_out=7.893582e-05) */
	  {    81u,  3u,  3u,  5u,  -2530,     5618 },  /* H_cv31_k2 score (s_out=0.0003348883) */
	},
};

#endif
