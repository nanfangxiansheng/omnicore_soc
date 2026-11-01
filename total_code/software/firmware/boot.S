/* boot.S — OmniCore 板级启动 (spec §14.2 启动顺序的板级最小实现)
 * 复位向量 0x0 (MODE-01), 位于 Boot BRAM (uncached)。
 * .data 由 BRAM 初始化文件预载, 仅需清 .bss 后跳主程序。
 */

	.section .boot, "ax", %progbits
	.globl _start
_start:
	/* 栈顶在 MAIN 窗口 (board.ld 提供) */
	la		sp, __stack_top

	/* mtvec 指向自检的陷阱入口 (direct 模式) */
	la		t0, trap_entry
	csrw	mtvec, t0

	/* 关全局中断, 清 mie (自检按阶段自行开启) */
	csrw	mie, zero
	li		t0, 8
	csrc	mstatus, t0

	/* 清 .bss (spec SW-02) */
	la		t0, __bss_start
	la		t1, __bss_end
1:
	bgeu	t0, t1, 2f
	sw		zero, 0(t0)
	addi	t0, t0, 4
	j		1b
2:
	/* ZIGUANG (2026-10-18): 清 UART 大包接收缓冲 (board.ld 的 .uart_rx, NOLOAD)。
	 *   BOOT BRAM 上电内容不是 0(out-of-context 综合时是未初始化), 而大包路径
	 *   依赖"缓冲里没有残留就当成没收到过"。只清 4KB, 循环成本可忽略。 */
	la		t0, UART_RXBUF
	la		t1, UART_RXEND
3:
	bgeu	t0, t1, 4f
	sw		zero, 0(t0)
	addi	t0, t0, 4
	j		3b
4:
	/* ZIGUANG 加固 (2026-10-17): 早期 UART 初始化。
	 * 任何发生在 demo_main 内部初始化之前的陷阱(非法指令/取指异常), 也需要能打印,
	 * 否则板上表现就是"完全静默", 无从定位 —— 本轮 CoreMark 片就栽在这里:
	 * banner 在 UART_BAUDDIV 仍为复位值 0 时打印, 卡死在等 TX_READY, 串口一个字都没有。
	 * 468 = 54.054MHz/115200 - 1, 与 demo.c / core_portme.c 的取值一致(幂等)。 */
	li		t0, 0x40000000
	li		t1, 468
	sw		t1, 12(t0)			/* UART_BAUDDIV */
	sw		zero, 8(t0)			/* UART_CTRL   */

	/* 跳入主程序 (MAIN 窗口, I$ cacheable 路径) */
#ifdef ARENA_BUILD
	call	arena_main
#else
	call	demo_main
#endif

	/* 不返回; 若返回则停机 */
5:
	wfi
	j		5b

/* ------------------------------------------------------------------ */
/* 陷阱入口: 保存全部整数现场, C 派发, 恢复后 mret                      */
/* ------------------------------------------------------------------ */
	.section .text
	.align 2
	.globl trap_entry
trap_entry:
	/* ZIGUANG (2026-10-16): 改为保存【完整】整数现场(x1..x31) —— 未对齐访问模拟
	 * 需要能写回任意 rd, 只存 a 系与 t 系寄存器是不够的。帧布局与 sw/trap_emu.h 严格一致:
	 *   f[i-1] = x_i (i=1..31)   f[31] = mcause   f[32] = mepc   (帧 160 字节)
	 * 注意: x2(原始 sp) 不显式恢复, 退出时用 addi sp,sp,160 还原。 */
	addi	sp, sp, -160
	sw	ra,   0(sp)
	sw	t0,  16(sp)
	addi	t0, sp, 160
	sw	t0,   4(sp)
	sw	gp,   8(sp)
	sw	tp,  12(sp)
	sw	t1,  20(sp)
	sw	t2,  24(sp)
	sw	s0,  28(sp)
	sw	s1,  32(sp)
	sw	a0,  36(sp)
	sw	a1,  40(sp)
	sw	a2,  44(sp)
	sw	a3,  48(sp)
	sw	a4,  52(sp)
	sw	a5,  56(sp)
	sw	a6,  60(sp)
	sw	a7,  64(sp)
	sw	s2,  68(sp)
	sw	s3,  72(sp)
	sw	s4,  76(sp)
	sw	s5,  80(sp)
	sw	s6,  84(sp)
	sw	s7,  88(sp)
	sw	s8,  92(sp)
	sw	s9,  96(sp)
	sw	s10,100(sp)
	sw	s11,104(sp)
	sw	t3, 108(sp)
	sw	t4, 112(sp)
	sw	t5, 116(sp)
	sw	t6, 120(sp)
	/* ZIGUANG (2026-10-16 v2): 【不在汇编里读 CSR】—— 实测刚进异常时 csrr 可能
	 * 读到未提交的陈旧值, 连读两次也不够稳 (随代码布局变化)。改为由 C 侧
	 * trap_dispatch 进入后填写 f[31]=mcause / f[32]=mepc (那时早已提交, 实测稳定)。 */
	sw	zero, 124(sp)
	sw	zero, 128(sp)
	mv	a0, sp
	call	trap_dispatch
	lw	t0, 128(sp)
	csrw	mepc, t0
	lw	ra,   0(sp)
	lw	t0,  16(sp)
	lw	t1,  20(sp)
	lw	t2,  24(sp)
	lw	s0,  28(sp)
	lw	s1,  32(sp)
	lw	a0,  36(sp)
	lw	a1,  40(sp)
	lw	a2,  44(sp)
	lw	a3,  48(sp)
	lw	a4,  52(sp)
	lw	a5,  56(sp)
	lw	a6,  60(sp)
	lw	a7,  64(sp)
	lw	s2,  68(sp)
	lw	s3,  72(sp)
	lw	s4,  76(sp)
	lw	s5,  80(sp)
	lw	s6,  84(sp)
	lw	s7,  88(sp)
	lw	s8,  92(sp)
	lw	s9,  96(sp)
	lw	s10,100(sp)
	lw	s11,104(sp)
	lw	t3, 108(sp)
	lw	t4, 112(sp)
	lw	t5, 116(sp)
	lw	t6, 120(sp)
	lw	gp,   8(sp)
	lw	tp,  12(sp)
	addi	sp, sp, 160
	mret
