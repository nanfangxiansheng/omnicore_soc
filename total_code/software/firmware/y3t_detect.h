/* =====================================================================
 * y3t_detect.h —— Y3T 检测头解码接口
 *   全整数实现, 供固件在 y3t 整网推理完成后调用。
 *   坐标系: 96x96 网络坐标, Q8 定点（值 = 真实值 * 256）。
 * ===================================================================== */
#ifndef Y3T_DETECT_H
#define Y3T_DETECT_H

#include <stdint.h>

/* 单个检测框（96x96 坐标, Q8 定点） */
typedef struct {
	int32_t cx96_q8;    /* 中心 x (96 坐标 × 256) */
	int32_t cy96_q8;    /* 中心 y */
	int32_t w96_q8;     /* 宽 */
	int32_t h96_q8;     /* 高 */
	int16_t conf_q;     /* 命中分数（int16 原始值, 越大越可信）*/
	uint8_t cls;        /* 类别索引 */
	uint8_t scale;      /* 来自哪个尺度 (0=6x6 stride16, 1=3x3 stride32) */
} det_box_t;

/* 解码 y3tram 里的 2 个尺度检测头, 返回写出的框数（按扫描顺序, 未排序未 NMS）。
 *   y3t   : y3tram 基址（0x60000000 映射后的 int16 指针）
 *   max_out: out 数组容量
 * 返回 <0 表示参数错误。
 */
int det_decode(const int16_t *y3t, int max_out, det_box_t *out);

/* 96x96 坐标 -> 1280x720 帧坐标（Q8）, letterbox 反变换。
 *   r = 0.075, 上下各 pad 21 行
 */
void det_to_frame_q8(int32_t x96_q8, int32_t y96_q8, int32_t *xf_q8, int32_t *yf_q8);

/* 运行时置信度阈值覆盖 (0x16 SET_THR 写入)。
 * 每尺度一个原始分数阈值; -32768 = 用表内默认 (conf_thr=0.30)。
 * conf=c 对应的分数阈值 = ln(c/(1-c)) / s_out —— PC 侧 tools/det_thr.py 会算好。 */
extern int32_t g_det_thr_ovr[2];

#endif /* Y3T_DETECT_H */
