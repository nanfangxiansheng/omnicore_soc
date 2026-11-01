/* y3t_scale_table_blob.h —— 对齐版尺度偏移（pack_scales_aligned.py 生成）
 *   每层段起始 64 字对齐，配合 rom_stream 的对齐读。数值与 y3t_scale_table.h 同源。 */
#ifndef Y3T_SCALE_TABLE_BLOB_H
#define Y3T_SCALE_TABLE_BLOB_H
#include <stdint.h>
#define Y3T_SCALE_LAYERS   29u
#define Y3T_SCALE_WORDS    5584u
typedef struct {
	uint16_t n;
	uint16_t shM0, shM1, shY;
	uint32_t w_g0, w_g1, w_bias, multY;
} y3t_scale_t;
static const y3t_scale_t g_scale[Y3T_SCALE_LAYERS] = {
	{   16u,  37u,  37u,  32u,      0u,      0u,     64u, 1220777901u }, /* L0 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L1 */
	{   16u,  36u,  36u,  33u,    128u,      0u,    192u, 1853283485u }, /* L2 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L3 */
	{   32u,  37u,  37u,  32u,    256u,      0u,    320u, 1311336565u }, /* L4 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L5 */
	{   64u,  38u,  38u,  32u,    384u,      0u,    448u, 1575876203u }, /* L6 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L7 */
	{  128u,  38u,  38u,  31u,    512u,      0u,    640u, 1164623188u }, /* L8 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L9 */
	{  256u,  40u,  40u,  31u,    768u,      0u,   1024u, 1851205252u }, /* L10 */
	{  512u,  41u,  41u,  30u,   1280u,      0u,   1792u, 1433901798u }, /* L11 */
	{  128u,  41u,  41u,  30u,   2304u,      0u,   2432u, 1709518756u }, /* L12 */
	{  256u,  41u,  41u,  30u,   2560u,      0u,   2816u, 1773384727u }, /* L13 */
	{   64u,  40u,  40u,  29u,   3072u,      0u,   3136u, 1349566785u }, /* L14 */
	{    0u,   0u,   0u,   0u,      0u,      0u,      0u, 1u }, /* L15 */
	{  128u,  42u,  40u,  31u,   3200u,   3328u,   3456u, 1814405777u }, /* L17 */
	{   32u,  40u,  40u,  30u,   3584u,      0u,   3648u, 1370994266u }, /* H_cv20_k0 */
	{   32u,  41u,  41u,  29u,   3712u,      0u,   3776u, 1089889982u }, /* H_cv20_k1 */
	{   16u,  41u,  41u,  30u,   3840u,      0u,   3904u, 1669969777u }, /* H_cv20_k2 */
	{  128u,  39u,  39u,  31u,   3968u,      0u,   4096u, 1693722236u }, /* H_cv30_k0 */
	{  128u,  40u,  40u,  31u,   4224u,      0u,   4352u, 1612914956u }, /* H_cv30_k1 */
	{   16u,  39u,  39u,  32u,   4480u,      0u,   4544u, 1142752211u }, /* H_cv30_k2 */
	{   32u,  42u,  42u,  29u,   4608u,      0u,   4672u, 1212743130u }, /* H_cv21_k0 */
	{   32u,  40u,  40u,  30u,   4736u,      0u,   4800u, 2015346111u }, /* H_cv21_k1 */
	{   16u,  41u,  41u,  30u,   4864u,      0u,   4928u, 1660488139u }, /* H_cv21_k2 */
	{  128u,  42u,  42u,  30u,   4992u,      0u,   5120u, 1562400013u }, /* H_cv31_k0 */
	{  128u,  40u,  40u,  30u,   5248u,      0u,   5376u, 2037609879u }, /* H_cv31_k1 */
	{   16u,  41u,  41u,  32u,   5504u,      0u,   5568u, 1565560708u }, /* H_cv31_k2 */
};
#endif
