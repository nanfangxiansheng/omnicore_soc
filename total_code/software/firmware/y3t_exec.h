/* ============================================================================
 * y3t_exec.h — G6 整网执行器 (Route B: DDR 权重逐层流式 + 静态 job 表)
 * ========================================================================== */
#ifndef Y3T_EXEC_H
#define Y3T_EXEC_H

#include <stdint.h>

/* DDR 里 engine_weights.bin 的基块（与 tools/upload_weights.py 的 WDDR_BASE_BLK 一致） */
#define Y3T_EXEC_WBASE_BLK   480000u

typedef struct {
	uint32_t jobs;        /* 实际下发的 job 数（含拆分） */
	uint32_t failed;      /* 首个失败 job 的层下标, 0xFFFFFFFFu = 全过 */
	uint32_t wload_cyc;   /* 权重搬运累计拍 */
	uint32_t mac_cyc;     /* 引擎累计拍 */
	uint32_t total_cyc;   /* 整网累计拍 */
	uint32_t rom_mode;    /* 0 = identity(直通), 1 = 从 DDR 装真尺度 */
} y3t_exec_stats_t;

/* 尺度/偏置 ROM 的两种模式 ------------------------------------------------ */
/* identity: multM=1 / bias=0 / multY=1 / 移位全 0 —— 与 [8] 自检同口径,
 *   用于**结构联调**（能跑通、能看 cyc，但数值不是网络输出）。 */
void y3t_exec_rom_identity(void);

/* 真尺度: 从 DDR 基块 rom_blk 起按层顺序取 {multM[16*cot], bias[16*cot]},
 *   层偏移按 g_net[] 的 cot 累加。镜像由模型导出工具生成（当前工程还没有，
 *   见 docs 里的说明）；blk==0 表示未提供，退回 identity。 */
void y3t_exec_rom_from_ddr(uint32_t rom_blk);

/* 整网执行一次。ddr_w_base_blk = engine_weights.bin 在 DDR 的基块。
 * 返回 0 = 全部 job 成功；-1 = 权重区装不下且无法等分；-2 = 引擎报错；-3 = 超时 */
int  y3t_exec_run(uint32_t ddr_w_base_blk, y3t_exec_stats_t *st);

/* 跑一层（供仿真/单层对拍使用）；layer 为 g_net[] 下标, st 可为 NULL */
int  y3t_exec_layer(uint32_t layer, uint32_t ddr_w_base_blk, y3t_exec_stats_t *st);

#endif /* Y3T_EXEC_H */
