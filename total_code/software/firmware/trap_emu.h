/* ============================================================================
 * trap_emu.h — 陷阱"未对齐访问"模拟 (ZIGUANG 2026-10-16)
 *
 * 背景: 本 SoC 的 RV32I 核【不支持】非对齐 load/store —— 触发 mcause=4/6。
 *   CoreMark 的链表算法(core_list_join)会按 list_head 指针步进访问内存,
 *   一旦地址非 4 字节对齐就异常停机 ⇒ 这是 CoreMark 一直跑不完的直接原因。
 *
 * 做法(不改 EEMBC 源码, 只改固件侧): 在陷阱入口保存【完整寄存器现场】后,
 *   解码故障指令 → 用【逐字节】访问完成它 → 写回 rd → mepc += 指令长度 → mret。
 *   注: 逐字节访问是必须的 —— 用 uint16_t 或 uint32_t 指针访问非对齐地址会再次触发
 *   同一个异常, 形成死循环。
 *
 * 帧布局 (由 boot.S 的 trap_entry 建立, 160 字节):
 *   f[i-1] = x_i   (i = 1..31)          f[31] = mcause      f[32] = mepc
 *
 * 本文件同时被 demo.c 与 coremark_port/core_portme.c 包含, 两处陷阱派发共用同一实现。
 * ========================================================================== */
#ifndef TRAP_EMU_H
#define TRAP_EMU_H

#include <stdint.h>
#include <stddef.h>

/* ---- 逐字节访存助手: 必须用 inline asm --------------------------------
 * 实测教训 (2026-10-16): 普通的 `*(volatile uint8_t*)p` 循环会被 GCC 的
 *   load-merge 优化【还原成一条 lw】—— 那样对非对齐地址会再次触发同一异常,
 *   变成嵌套陷阱死循环。lbu/sb 是字节访问, 任何地址都不会未对齐, 且 asm 化后
 *   编译器无法再合并。 */
static uint32_t ld8u(uintptr_t a){ uint32_t v; __asm__ volatile("lbu %0, 0(%1)" : "=r"(v) : "r"(a)); return v; }
static uint32_t ld16u(uintptr_t a){ uint32_t v; __asm__ volatile("lhu %0, 0(%1)" : "=r"(v) : "r"(a)); return v; }
static void st8(uintptr_t a, uint32_t v){ __asm__ volatile("sb %1, 0(%0)" :: "r"(a), "r"(v) : "memory"); }


/* 返回 1 = 已模拟并处理(可以 mret 返回); 0 = 不是我能处理的陷阱(交给调用方) */
static int trap_emu(uint32_t *f)
{
    uint32_t mcause = f[31];
    uint32_t pc, lo, insn, op, f3, r1, r2, rd, imm, addr, v, n, i;
    int      is_comp = 0;
    int      is_load;
    static uint32_t last_pc = 0, same_cnt = 0;

    if (mcause != 4u && mcause != 6u)          /* 4=load misaligned, 6=store misaligned */
        return 0;

    pc = f[32];

    /* 死循环护栏: 同一 pc 反复异常说明模拟没起作用, 直接放行让上层停机并打印 */
    if (pc == last_pc) {
        if (++same_cnt > 8u) { same_cnt = 0; return 0; }
    } else {
        last_pc = pc; same_cnt = 0;
    }

    /* 取指: 先读 2 字节(压缩指令 2 对齐, 永远安全), 再用低两位判断是否 32 位指令 */
    lo = ld16u(pc);                      /* 2 对齐, 永不未对齐 */
    if ((lo & 3u) == 3u) {
        insn = lo | (ld16u(pc + 2u) << 16);
    } else {
        is_comp = 1;
        insn = lo;
    }

    if (is_comp) {
        /* ---- RVC 压缩访存: c.lw / c.sw / c.lwsp / c.swsp (RV32) ----
         * -march=rv32imac 下 CoreMark 会大量使用压缩 load/store, 必须处理。
         * 立即数是打散的, 按 RVC 规范逐位抽。 */
        uint32_t q  = insn & 3u;             /* 0 或 2 */
        uint32_t cf = (insn >> 13) & 7u;
        uint32_t a, rda, rsa, m;
        if (q == 0u && cf == 2u) {           /* c.lw  rd'=[4:2] rs1'=[9:7] */
            rda = 8u + ((insn >> 2) & 7u);
            rsa = 8u + ((insn >> 7) & 7u);
            m   = (((insn >> 5) & 1u) << 6) | (((insn >> 10) & 7u) << 3) | (((insn >> 6) & 1u) << 2);
            a   = f[rsa - 1] + m;
            v = 0;
            for (i = 0; i < 4u; i++) v |= ld8u(a + i) << (8u * i);
            f[rda - 1] = v;
            f[32] = pc + 2u; return 1;
        }
        if (q == 0u && cf == 6u) {           /* c.sw  rs2'=[4:2] rs1'=[9:7] */
            uint32_t rs2a = 8u + ((insn >> 2) & 7u);
            rsa = 8u + ((insn >> 7) & 7u);
            m   = (((insn >> 5) & 1u) << 6) | (((insn >> 10) & 7u) << 3) | (((insn >> 6) & 1u) << 2);
            a   = f[rsa - 1] + m;
            v   = f[rs2a - 1];
            for (i = 0; i < 4u; i++) st8(a + i, (v >> (8u * i)) & 0xFFu);
            f[32] = pc + 2u; return 1;
        }
        if (q == 2u && cf == 2u) {           /* c.lwsp rd=[11:7], uimm=[3:2|12|6:4] */
            rda = (insn >> 7) & 0x1Fu;
            m   = (((insn >> 2) & 3u) << 6) | (((insn >> 12) & 1u) << 5) | (((insn >> 4) & 7u) << 2);
            a   = f[1] + m;                  /* x2 = sp */
            v = 0;
            for (i = 0; i < 4u; i++) v |= ld8u(a + i) << (8u * i);
            if (rda) f[rda - 1] = v;
            f[32] = pc + 2u; return 1;
        }
        if (q == 2u && cf == 6u) {           /* c.swsp rs2=[6:2], uimm=[8:7|12:9] */
            uint32_t rs2a = (insn >> 2) & 0x1Fu;
            m   = (((insn >> 7) & 3u) << 6) | (((insn >> 9) & 0xFu) << 2);
            a   = f[1] + m;
            v   = f[rs2a - 1];
            for (i = 0; i < 4u; i++) st8(a + i, (v >> (8u * i)) & 0xFFu);
            f[32] = pc + 2u; return 1;
        }
        return 0;   /* 其它压缩指令: 交回上层停机打印 */
    }

    op = insn & 0x7Fu;
    f3 = (insn >> 12) & 7u;
    r1 = (insn >> 15) & 0x1Fu;
    rd = (insn >>  7) & 0x1Fu;
    r2 = (insn >> 20) & 0x1Fu;

    if (op == 0x03u) {                                   /* ---- load (I 型) ---- */
        is_load = 1;
        imm = (uint32_t)((int32_t)insn >> 20);
    } else if (op == 0x23u) {                            /* ---- store (S 型) ---- */
        is_load = 0;
        imm = (((insn >> 25) & 0x7Fu) << 5) | ((insn >> 7) & 0x1Fu);
        if (imm & 0x800u) imm |= 0xFFFFF000u;             /* 12 位符号扩展 */
    } else {
        return 0;
    }

    addr = (r1 ? f[r1 - 1] : 0u) + imm;

    if (is_load) {
        switch (f3) {
        case 0: n = 1; break;   /* lb  */
        case 1: n = 2; break;   /* lh  */
        case 2: n = 4; break;   /* lw  */
        case 4: n = 1; break;   /* lbu */
        case 5: n = 2; break;   /* lhu */
        default: return 0;
        }
        v = 0;
        for (i = 0; i < n; i++)
            v |= ld8u(addr + i) << (8u * i);
        if (f3 == 0 && (v & 0x80u))   v |= 0xFFFFFF00u;   /* lb  符号扩展 */
        if (f3 == 1 && (v & 0x8000u)) v |= 0xFFFF0000u;   /* lh  符号扩展 */
        if (rd) f[rd - 1] = v;                             /* rd=0 丢弃 */
    } else {
        switch (f3) {
        case 0: n = 1; break;   /* sb */
        case 1: n = 2; break;   /* sh */
        case 2: n = 4; break;   /* sw */
        default: return 0;
        }
        v = (r2 ? f[r2 - 1] : 0u);                         /* store 的源是 rs2 */
        for (i = 0; i < n; i++)
            st8(addr + i, (v >> (8u * i)) & 0xFFu);
    }

    f[32] = pc + 4u;                    /* 跳过故障指令, 继续执行 */
    return 1;
}

#endif /* TRAP_EMU_H */
