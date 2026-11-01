/* ============================================================================
 * y3t_pipeline.h — Y3T 分层推理调度器 (Route B: DDR 分层流式)
 *
 * y3tram 1MB 内存布局 (256b 字地址):
 *   [0           .. WB_BASE-1]              权重区 (wload_dma 按层装载)
 *   [WB_BASE     .. WB_BASE+ACT_SIZE-1]    激活 A (乒乓面 0)
 *   [WB_BASE+ACT_SIZE .. +2*ACT_SIZE-1]    激活 B (乒乓面 1)
 *   [其余]                                  结果/暂存
 *
 * 每层执行:
 *   ① wload_dma: DDR 层权重块 → y3tram 权重区 (引擎空闲, 独占)
 *   ② Y3T job:   引擎读 A(当前面) + W → 写 B(另一面)
 *   ③ 乒乓面切换 → 下一层
 *
 * CPU 只做: 写 wload_dma 寄存器 + 写 Y3T cfg 寄存器 + 轮询 done。
 * 全程不碰像素数据 (DMA 直搬)。
 * ========================================================================== */
#ifndef Y3T_PIPELINE_H
#define Y3T_PIPELINE_H

#include <stdint.h>

#ifndef Y3T_REG32_DEFINED
#define Y3T_REG32_DEFINED
#define Y3T_REG32(a) (*(volatile uint32_t *)(a))
#endif
#define REG32(a) Y3T_REG32(a)

/* ---- y3tram 布局常量 (256b 字地址) ---- */
#define Y3RAM_TOTAL     32768u              /* 1MB / 32B */
#define WB_BASE         20480u              /* 权重区顶 (2.5K 词 × 32B = 80KB 预留头) */
#define ACT_SIZE        4096u               /* 每面激活区 4096 词 = 128KB */

/* ---- 权重 DMA 寄存器 (0x9000_0000) ---- */
#define WLD_SRC         REG32(0x90000000u)
#define WLD_DST         REG32(0x90000004u)
#define WLD_LEN         REG32(0x90000008u)
#define WLD_CTRL        REG32(0x9000000Cu)
#define WLD_STATUS      REG32(0x9000000Cu)
#define WLD_DONE        0x1u

/* ---- Y3T cfg 寄存器 ---- */
#define Y3T_CTRL        REG32(0x40005000u)
#define Y3T_STATUS      REG32(0x40005000u)
#define Y3T_CFG0        REG32(0x40005004u)
#define Y3T_HW          REG32(0x40005008u)
#define Y3T_HWO         REG32(0x4000500Cu)
#define Y3T_TILE        REG32(0x40005010u)
#define Y3T_FLAG        REG32(0x4000501Cu)
#define Y3T_IN0         REG32(0x4000502Cu)
#define Y3T_IN1         REG32(0x40005030u)
#define Y3T_WB          REG32(0x40005034u)
#define Y3T_OUT         REG32(0x40005038u)

/* ---- 层描述符 (固件根据 G6 模型填充) ---- */
typedef struct {
    uint32_t w_src_blk;     /* 权重在 DDR 中的起始 256b 块地址 */
    uint32_t w_words;       /* 该层权重字数 (256b) */
    uint32_t in_base;       /* 输入激活 y3tram 块地址 */
    uint32_t out_base;      /* 输出激活 y3tram 块地址 */
    uint32_t wb_base;       /* y3tram 内偏置基址 (本层) */
    uint32_t k;             /* 卷积核尺寸 (3=3×3) */
    uint32_t s;             /* 步长 */
    uint32_t pad;           /* 填充 */
    uint32_t mode;          /* 引擎模式 */
    uint32_t W, H;          /* 输入 W×H */
    uint32_t Wo, Ho;        /* 输出 Wo×Ho */
    uint32_t act;           /* 激活函数 */
    uint32_t bits16;        /* 16bit 激活 */
} layer_desc_t;

/* ---- G6 29 层描述符表 (由离线工具生成, 此处为框架) ---- */
/* 实际部署时: python eval_y3t.py --export-c > layer_table.h */
/* #include "layer_table.h" */

#ifndef LAYER_COUNT
#define LAYER_COUNT 29
#endif

/* ---- L11 通道拆分 (方案 A: 256→512 3×3 conv 超出 y3tram) ----
 * L11 引擎交织权重 = 36,864 词 > y3tram 32,768 词
 * 拆成 2 个子 job, 每个处理 256 个输出通道 (cot=16 tiles × 16ch):
 *   子job1: 权重前半 (18,432 词), 写 y3tram OUT 区前半
 *   子job2: 权重后半 (18,432 词), 写 y3tram OUT 区后半
 * TILE.cot 控制输出通道 tile; 输出通道独立无累加, 天然可拆。 */
#define L11_SPLIT_COT   16u     /* 每子 job 的 cot tile 数 (16×16=256ch) */
#define L11_SPLIT_JOBS  2u

#endif /* Y3T_PIPELINE_H */
