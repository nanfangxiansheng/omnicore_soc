/* ==================================================================
 * y3t_scale_table.h —— 逐通道 requant 尺度表 (由 tools/export_scales.py 生成, 勿手改)
 *   源: y3t_params.json 的 groups[*].multM / bias_q13 / shiftY / multY
 *   blob: engine_scales.bin, 上传到 DDR 后把块号传给 rom_blk
 *   引擎 ROM 布局: multM group0 -> 0, group1 -> 1024, bias -> 2048
 *   注意 identity 若从地址 0 连写 bias 会踩掉 group1 的 multM (见脚本头注释)
 * ================================================================== */
#ifndef Y3T_SCALE_TABLE_H
#define Y3T_SCALE_TABLE_H

#include <stdint.h>

#define Y3T_SCALE_LAYERS   29u
#define Y3T_SCALE_WORDS    4736u   /* bin 总字数 (x4 = 字节) */
#define Y3T_SCALE_BLK_DEF  580000u /* 建议的 DDR 落点 (权重区之后, 空闲) */

typedef struct {
	uint16_t n;        /* 每段字数 (= cot*16) */
	uint16_t shM0, shM1, shY;
	uint32_t w_g0;     /* group0 multM 在 blob 内的字偏移 */
	uint32_t w_g1;     /* group1 multM 字偏移, 0 = 本层只有 1 组 */
	uint32_t w_bias;   /* bias_q13 字偏移 */
	uint32_t multY;    /* 输出乘子 (S4 级) */
} y3t_scale_t;

static const y3t_scale_t g_scale[Y3T_SCALE_LAYERS] = {
	{   16u, 37u, 37u, 32u,      0u,      0u,     16u, 1220777901u }, /* L0 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L1 */
	{   16u, 36u, 36u, 33u,     32u,      0u,     48u, 1853283485u }, /* L2 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L3 */
	{   32u, 37u, 37u, 32u,     64u,      0u,     96u, 1311336565u }, /* L4 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L5 */
	{   64u, 38u, 38u, 32u,    128u,      0u,    192u, 1575876203u }, /* L6 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L7 */
	{  128u, 38u, 38u, 31u,    256u,      0u,    384u, 1164623188u }, /* L8 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L9 */
	{  256u, 40u, 40u, 31u,    512u,      0u,    768u, 1851205252u }, /* L10 */
	{  512u, 41u, 41u, 30u,   1024u,      0u,   1536u, 1433901798u }, /* L11 */
	{  128u, 41u, 41u, 30u,   2048u,      0u,   2176u, 1709518756u }, /* L12 */
	{  256u, 41u, 41u, 30u,   2304u,      0u,   2560u, 1773384727u }, /* L13 */
	{   64u, 40u, 40u, 29u,   2816u,      0u,   2880u, 1349566785u }, /* L14 */
	{    0u,  0u,  0u,  0u,      0u,      0u,      0u,          1u }, /* L15 */
	{  128u, 42u, 40u, 31u,   2944u,   3072u,   3200u, 1814405777u }, /* L17 */
	{   32u, 40u, 40u, 30u,   3328u,      0u,   3360u, 1370994266u }, /* H_cv20_k0 */
	{   32u, 41u, 41u, 29u,   3392u,      0u,   3424u, 1089889982u }, /* H_cv20_k1 */
	{   16u, 41u, 41u, 30u,   3456u,      0u,   3472u, 1669969777u }, /* H_cv20_k2 */
	{  128u, 39u, 39u, 31u,   3488u,      0u,   3616u, 1693722236u }, /* H_cv30_k0 */
	{  128u, 40u, 40u, 31u,   3744u,      0u,   3872u, 1612914956u }, /* H_cv30_k1 */
	{   16u, 39u, 39u, 32u,   4000u,      0u,   4016u, 1142752211u }, /* H_cv30_k2 */
	{   32u, 42u, 42u, 29u,   4032u,      0u,   4064u, 1212743130u }, /* H_cv21_k0 */
	{   32u, 40u, 40u, 30u,   4096u,      0u,   4128u, 2015346111u }, /* H_cv21_k1 */
	{   16u, 41u, 41u, 30u,   4160u,      0u,   4176u, 1660488139u }, /* H_cv21_k2 */
	{  128u, 42u, 42u, 30u,   4192u,      0u,   4320u, 1562400013u }, /* H_cv31_k0 */
	{  128u, 40u, 40u, 30u,   4448u,      0u,   4576u, 2037609879u }, /* H_cv31_k1 */
	{   16u, 41u, 41u, 32u,   4704u,      0u,   4720u, 1565560708u }, /* H_cv31_k2 */
};

#endif /* Y3T_SCALE_TABLE_H */
