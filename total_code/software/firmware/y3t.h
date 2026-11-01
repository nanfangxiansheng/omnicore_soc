/* ============================================================================
 * y3t.h — Y3T 引擎 cfg 口驱动 (AHB @0x4000_5000)
 * 寄存器映射与 y3t_cfg_ahb.v 一致; 全部基址为 256bit 字 (y3tram 内)。
 * 典型发 job 流程: y3t_job(...) → 轮询 y3t_done() → 读结果区。
 * ========================================================================== */
#ifndef Y3T_H
#define Y3T_H

#include <stdint.h>

#ifndef Y3T_REG32_DEFINED
#define Y3T_REG32_DEFINED
#define Y3T_REG32(a) (*(volatile uint32_t *)(a))
#endif

#define Y3T_CFG_BASE  0x40005000u
#define Y3T_CTRL      Y3T_REG32(Y3T_CFG_BASE + 0x00u)   /* W:start  R:{err,done} */
#define Y3T_CFG0      Y3T_REG32(Y3T_CFG_BASE + 0x04u)
#define Y3T_HW        Y3T_REG32(Y3T_CFG_BASE + 0x08u)
#define Y3T_HWO       Y3T_REG32(Y3T_CFG_BASE + 0x0Cu)
#define Y3T_TILE      Y3T_REG32(Y3T_CFG_BASE + 0x10u)
#define Y3T_SH0       Y3T_REG32(Y3T_CFG_BASE + 0x14u)
#define Y3T_MULTY     Y3T_REG32(Y3T_CFG_BASE + 0x18u)
#define Y3T_FLAG      Y3T_REG32(Y3T_CFG_BASE + 0x1Cu)
#define Y3T_IN0       Y3T_REG32(Y3T_CFG_BASE + 0x2Cu)
#define Y3T_IN1       Y3T_REG32(Y3T_CFG_BASE + 0x30u)
#define Y3T_WB        Y3T_REG32(Y3T_CFG_BASE + 0x34u)
#define Y3T_OUT       Y3T_REG32(Y3T_CFG_BASE + 0x38u)
#define Y3T_ROM_ADDR  Y3T_REG32(Y3T_CFG_BASE + 0x3Cu)
#define Y3T_ROM_DATA  Y3T_REG32(Y3T_CFG_BASE + 0x40u)
#define Y3T_STATUS    Y3T_REG32(Y3T_CFG_BASE + 0x00u)   /* 读复用: [1]=err [0]=done */
#define Y3T_CYC_MAC   Y3T_REG32(Y3T_CFG_BASE + 0x4Cu)
#define Y3T_CYC_TOT   Y3T_REG32(Y3T_CFG_BASE + 0x50u)
#define Y3T_ID        Y3T_REG32(Y3T_CFG_BASE + 0xFCu)

#define Y3T_DONE      0x1u
#define Y3T_ERR       0x2u

/* mode[2:0]: 0=conv 1=depthwise...(与 engine_y3t 解码一致, v2 只用卷积) */

void y3t_rom_write(uint32_t addr12, const uint32_t *words, uint32_t n) {
    Y3T_ROM_ADDR = addr12 & 0xFFFu;
    for (uint32_t i = 0; i < n; i++) Y3T_ROM_DATA = words[i];  /* 地址自增 */
}

/* 发一个卷积 job 并等待完成。返回 0=ok。blk* 均为 y3tram 256b 块基址。 */
int y3t_job(uint32_t in0, uint32_t in1, uint32_t wb, uint32_t out,
                          uint32_t W, uint32_t H, uint32_t Wo, uint32_t Ho,
                          uint32_t k, uint32_t s, uint32_t pad, uint32_t mode,
                          uint32_t cit, uint32_t cot, uint32_t act, uint32_t bits16,
                          uint32_t timeout_loops) {
    Y3T_CFG0 = ((pad & 3u) << 24) | ((s & 3u) << 20) | ((k & 3u) << 16) | (mode & 7u);
    /* HW  = {W[25:16],  H[9:0]}   HWO = {Wo[25:16], Ho[9:0]}  (各 10bit) */
    Y3T_HW  = ((W  & 0x3FFu) << 16) | (H  & 0x3FFu);
    Y3T_HWO = ((Wo & 0x3FFu) << 16) | (Ho & 0x3FFu);
    /* TILE = {split[22:16], cot[14:8], cit[6:0]}; 每 tile = 16 通道
     * ZIGUANG 修复 (2026-10-16): 原为 Y3T_TILE = 0 (注释 cit/cot 默认 0),
     *   等于把 cit/cot 两个入参丢弃 -> 引擎按 0 通道算, 真 job 结果必然不对。
     *   现按 y3t_routeb.c 约定填入 (cit/cot 均为 tile 数, 1 tile = 16 通道; split=0)。
     *   注: L11 的拆分模式 (split=1) 另有路径, 不走本函数。 */
    Y3T_TILE = ((cot & 0x7Fu) << 8) | (cit & 0x7Fu);
    Y3T_FLAG = (act & 1u) | ((bits16 & 1u) << 1);
    Y3T_IN0  = in0;  Y3T_IN1 = in1;  Y3T_WB = wb;  Y3T_OUT = out;
    Y3T_CTRL = 1u;                                   /* START 脉冲, 清粘滞 */
    uint32_t guard = 0;
    while (!(Y3T_STATUS & Y3T_DONE)) {
        if (Y3T_STATUS & Y3T_ERR) return -2;
        if (++guard > timeout_loops) return -1;
    }
    return 0;
}

#endif /* Y3T_H */
