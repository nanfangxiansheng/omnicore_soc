/* ==================================================================
 * y3t_net_table.h —— 整网执行表 (由 tools/gen_net_table.py 生成, 勿手改)
 *   源: y3t_params.json 的 jobs/tmeta + engine_layout.json 的权重块偏移
 *   含 23 conv + 5 pool + 1 up = 29 个可执行 job;
 *   concat(L16) 不单独执行, 折叠为 L17 的 in1 + split (双输入区零拷贝)。
 *   激活区复用: 按生命周期首次适配, 高水位 18432 块。
 *   权重区 [18432, 32768): 每层现搬 wb = 32768 - w_words。
 * ================================================================== */
#ifndef Y3T_NET_TABLE_H
#define Y3T_NET_TABLE_H

#include <stdint.h>

#define NET_WB_TOP   32768u
#define NET_WB_LO    18432u
#define NET_LAYERS   29u

/* types */
#define NT_CONV  0u
#define NT_POOL  3u   /* 引擎 cfg_mode=3: 2x2 maxpool s2 */
#define NT_UPS   2u   /* 引擎 cfg_mode=2: 2x2 最近邻 */

typedef struct {
	uint32_t w_src_blk;  /* DDR 内权重块偏移 (engine_weights.bin 口径), 非 conv 为 0 */
	uint32_t w_words;    /* 本层权重 256b 字数 */
	uint32_t in0;        /* 输入激活块 (concat 的第一个来源) */
	uint32_t in1;        /* 第二个输入激活块; 仅 concat 消费者用, 其余 = in0 */
	uint32_t out;        /* 输出激活块 */
	uint32_t split;      /* 输入通道分组: g1 起点 (cit 个数); 无 concat 时 = cit */
	uint32_t k, s, pad;
	uint32_t cit, cot;   /* 通道 tile 数 (1 tile = 16 ch) */
	uint32_t H, W, Ho, Wo;
	uint32_t mode;      /* 引擎 cfg_mode: 0 conv / 2 upsample / 3 pool2 */
	uint32_t act, bits16;
	uint32_t ich, och;   /* 真实通道数 (仅打印/校验) */
	const char *name;
} net_layer_t;

static const net_layer_t g_net[NET_LAYERS] = {
	{      0u,     72u,      0u,      0u,   9216u,    1u,  2u,  1u,  1u,    1u,    1u,  96u,  96u,  96u,  96u,  0u,  1u,  1u,    3u,    8u, "L0" },
	{      0u,      0u,   9216u,   9216u,      0u,    1u,  2u,  2u,  0u,    1u,    1u,  96u,  96u,  48u,  48u,  3u,  0u,  1u,    8u,    8u, "L1" },
	{     72u,     72u,      0u,      0u,   2304u,    1u,  2u,  1u,  1u,    1u,    1u,  48u,  48u,  48u,  48u,  0u,  1u,  1u,    8u,   16u, "L2" },
	{      0u,      0u,   2304u,   2304u,      0u,    1u,  2u,  2u,  0u,    1u,    1u,  48u,  48u,  24u,  24u,  3u,  0u,  1u,   16u,   16u, "L3" },
	{    144u,    144u,      0u,      0u,    576u,    1u,  2u,  1u,  1u,    1u,    2u,  24u,  24u,  24u,  24u,  0u,  1u,  1u,   16u,   32u, "L4" },
	{      0u,      0u,    576u,    576u,      0u,    2u,  2u,  2u,  0u,    2u,    2u,  24u,  24u,  12u,  12u,  3u,  0u,  1u,   32u,   32u, "L5" },
	{    288u,    576u,      0u,      0u,    288u,    2u,  2u,  1u,  1u,    2u,    4u,  12u,  12u,  12u,  12u,  0u,  1u,  1u,   32u,   64u, "L6" },
	{      0u,      0u,    288u,    288u,      0u,    4u,  2u,  2u,  0u,    4u,    4u,  12u,  12u,   6u,   6u,  3u,  0u,  1u,   64u,   64u, "L7" },
	{    864u,   2304u,      0u,      0u,    144u,    4u,  2u,  1u,  1u,    4u,    8u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,   64u,  128u, "L8" },
	{      0u,      0u,    144u,    144u,      0u,    8u,  2u,  2u,  0u,    8u,    8u,   6u,   6u,   3u,   3u,  3u,  0u,  1u,  128u,  128u, "L9" },
	{   3168u,   9216u,      0u,      0u,    432u,    8u,  2u,  1u,  1u,    8u,   16u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  128u,  256u, "L10" },
	{  12384u,  36864u,    432u,    432u,    576u,   16u,  2u,  1u,  1u,   16u,   32u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  256u,  512u, "L11" },
	{  49248u,   2048u,    576u,    576u,      0u,   32u,  1u,  1u,  0u,   32u,    8u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  512u,  128u, "L12" },
	{  51296u,   9216u,      0u,      0u,    432u,    8u,  2u,  1u,  1u,    8u,   16u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  128u,  256u, "L13" },
	{  60512u,    256u,      0u,      0u,     72u,    8u,  1u,  1u,  0u,    8u,    4u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  128u,   64u, "L14" },
	{      0u,      0u,     72u,     72u,    576u,    4u,  1u,  1u,  0u,    4u,    4u,   3u,   3u,   6u,   6u,  2u,  0u,  1u,   64u,   64u, "L15" },
	{  60768u,   6912u,    576u,    144u,    720u,    4u,  2u,  1u,  1u,   12u,    8u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,  192u,  128u, "L17" },
	{  67680u,   1152u,    720u,    720u,      0u,    8u,  2u,  1u,  1u,    8u,    2u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,  128u,   32u, "H_cv20_k0" },
	{  68832u,    288u,      0u,      0u,     72u,    2u,  2u,  1u,  1u,    2u,    2u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,   32u,   32u, "H_cv20_k1" },
	{  69120u,     16u,     72u,     72u,      0u,    2u,  1u,  1u,  0u,    2u,    1u,   6u,   6u,   6u,   6u,  0u,  0u,  1u,   32u,    4u, "H_cv20_k2" },
	{  69136u,   4608u,    720u,    720u,     36u,    8u,  2u,  1u,  1u,    8u,    8u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,  128u,  128u, "H_cv30_k0" },
	{  73744u,   4608u,     36u,     36u,    576u,    8u,  2u,  1u,  1u,    8u,    8u,   6u,   6u,   6u,   6u,  0u,  1u,  1u,  128u,  128u, "H_cv30_k1" },
	{  78352u,     64u,    576u,    576u,     36u,    8u,  1u,  1u,  0u,    8u,    1u,   6u,   6u,   6u,   6u,  0u,  0u,  1u,  128u,    5u, "H_cv30_k2" },
	{  78416u,   2304u,    432u,    432u,     72u,   16u,  2u,  1u,  1u,   16u,    2u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  256u,   32u, "H_cv21_k0" },
	{  80720u,    288u,     72u,     72u,     90u,    2u,  2u,  1u,  1u,    2u,    2u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,   32u,   32u, "H_cv21_k1" },
	{  81008u,     16u,     90u,     90u,     72u,    2u,  1u,  1u,  0u,    2u,    1u,   3u,   3u,   3u,   3u,  0u,  0u,  1u,   32u,    4u, "H_cv21_k2" },
	{  81024u,   9216u,    432u,    432u,     81u,   16u,  2u,  1u,  1u,   16u,    8u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  256u,  128u, "H_cv31_k0" },
	{  90240u,   4608u,     81u,     81u,    153u,    8u,  2u,  1u,  1u,    8u,    8u,   3u,   3u,   3u,   3u,  0u,  1u,  1u,  128u,  128u, "H_cv31_k1" },
	{  94848u,     64u,    153u,    153u,     81u,    8u,  1u,  1u,  0u,    8u,    1u,   3u,   3u,   3u,   3u,  0u,  0u,  1u,  128u,    5u, "H_cv31_k2" },
};

#endif /* Y3T_NET_TABLE_H */
