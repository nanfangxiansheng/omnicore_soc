/* board.ld — OmniCore 板级链接脚本 (spec §7.1 地址映射)
 * BOOT @0x0000_0000 (64KB, 复位向量 0x0, MEM-02/MODE-01)
 * MAIN @0x1000_0000 (cacheable 窗口, 容量与 boardsoc_top MAIN_WORDS 对齐)
 */

MEMORY
{
	BOOT (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
	MAIN (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
}

ENTRY(_start)

SECTIONS
{
	.boot : {
		KEEP(*(.boot))
	} > BOOT

	/* ZIGUANG (2026-10-17): .text/.rodata 由 MAIN 移到 BOOT。
	 *   原因: MAIN 只有 32KB, 而全展开 CoreMark 的 text 已是 29,218, 再加
	 *   bss 10,288 + 栈 8,192 必然溢出 (实测溢出 6,704 字节); 同时 BOOT 64KB
	 *   此前只用了 72 字节 (.boot 段), 白白空着。
	 *   配套: boardsoc_top 的 ZG_CACHE_BASE 已改为 0x0, 让 BOOT 可缓存 ——
	 *   否则代码落在 uncached 区, 每次取指都走总线, 性能会崩。
	 *   容量: BOOT 64KB 装 text+rodata (全展开约 30KB);
	 *         MAIN 32KB 装 data(24) + bss(10,288) + noinit(16) + stack(8,192) ≈ 18.5KB。 */
	.text : {
		*(.text*)
		*(.init)
		*(.fini)
	} > BOOT

	/* ========================================================================
	 * ZIGUANG (2026-10-18 21:05) —— 大包接收缓冲 (BOOT 内预留, 不进 RAM)
	 *
	 * 为什么放在这里而不是 .bss:
	 *   UART 权重上传的死穴是**停等往返**: 一条 262B 的 0x0B 只带 256B 数据,
	 *   主机却必须等 18B 响应才能发下一条; 11,864 条 × 347ms ≈ 9 小时。
	 *
	 *   把单帧载荷上限从 500B 提到 4096B 后, 一条覆盖 128 块 (4KB), 停等次数
	 *   降到 1/16; 再配合"大包默认静默不回 ACK", 往返彻底离开关键路径。
	 *
	 *   代价是 4KB 缓冲。放 .bss 的话 MAIN(32KB) 只剩 5KB 余量, 而栈已占 8KB,
	 *   固件再长一点就会栈撞 .bss。放这里则是:
	 *     ① 段标 NOLOAD ⇒ 不占 .bin/hex, 不进比特流;
	 *     ② 在 BOOT 里, BOOT 还有十几 KB 余量;
	 *     ③ 地址由链接器分配 (符号 UART_RXBUF), 固件变长自动跟着后移。
	 *
	 *   ★ 但 ld 会把 NOLOAD 段**插在第一个 LOAD 段的 VMA 之后**, 这里就是
	 *     .text 末尾 —— 所以它实际位于 .text 与 .rodata 之间, 不占固件镜像。
	 *     实测 (2026-10-18): UART_RXBUF=0x35BA..0x45C0, .rodata 被推到 0x45C0,
	 *     __fw_boot_end=0x5A68, 离 arena(0x8000) 还有 9.4KB。
	 *   约束 (文件末尾有 ASSERT): UART_RXEND <= 0x8000, 且 __fw_boot_end <= 0x8000。
	 * ======================================================================== */
	.uart_rx (NOLOAD) : {
		. = ALIGN(16);
		KEEP(*(.uart_rx))
		/* 连续两个赋值 = 在段内"占用"4096 字节并把首尾地址导出成符号。
		 *   不用静态数组是因为那是 .bss(→MAIN), 而这里要的正是 BOOT。 */
		UART_RXBUF = .;  . += 4096;
		UART_RXEND = .;
	/* ★ 位置合同: 这块缓冲必须整体落在 arena 之下, 否则自检 [10] /
	 *   UART 在线下装写 0x8000 就会踩到接收缓冲。
	 *   断言写在 SECTIONS 之外(文件末尾) —— ld 不允许 ASSERT 出现在段定义内部。 */
	} > BOOT

	.rodata : {
		*(.rodata*)
		*(.srodata*)
		/* ZIGUANG (2026-10-18 19:25): 导出固件本体结束位置。
		 *   用途: ① uart_cmd.c 的 arena 保护据此核对, 不再靠手写常量;
		 *         ② tools/check_fw_hex.py 出片前断言 __fw_boot_end <= arena 起点。
		 *   背景: v24 加入 y3t_exec.c 的 g_net(1.8KB)/poff 后 .rodata 涨到 0x489B,
		 *         而 BOOT arena 当时硬编码在 0x4000 —— 固件把自己的 arena 吃掉 2,203 B,
		 *         自检 [10] 写 0x4000 会踩 poff.4, UART 下装更会连 g_net 一起踩。
		 *         加这个符号后固件无法"悄悄长大": 超了校验直接 FAIL。 */
		. = ALIGN(4);
		__fw_boot_end = .;
	} > BOOT

	.data : ALIGN(4) {
		__data_start = .;
		*(.data*)
		*(.sdata*)
		__data_end = .;
	} > MAIN

	.bss : ALIGN(4) {
		__bss_start = .;
		/* ZIGUANG (2026-10-16): 把 CoreMark 的 static_memblk 强制 8 字节对齐。
		 *   【注意】实测该符号原本就已 16 字节对齐(0x100042b0), 所以它
		 *   并不是 CoreMark 触发 misaligned load 的原因 —— 保留此规则只是
		 *   良好的防御性做法(需配合 -fdata-sections 才生效)。 */
		. = ALIGN(8);
		*(.bss.static_memblk)
		. = ALIGN(8);
		*(.bss.static_memblk)
		. = ALIGN(8);
		*(.bss*)
		*(.sbss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end = .;
		/* ZIGUANG (2026-10-18): 给 ARENA 版程序用的 bss 边界别名。
		 * arena 版 CoreMark 没有 crt0, 靠 __arena_bss_start/end 自己清 bss;
		 * 在标准布局下这两个符号指向普通 .bss, 清零两次无害。 */
		__arena_bss_start = __bss_start;
		__arena_bss_end   = __bss_end;
	} > MAIN

	/* ZIGUANG (v8): 软复位不清的本区 —— 用于统计重启次数 (诊断复位回路) */
	.noinit (NOLOAD) : ALIGN(4) {
		__noinit_start = .;
		KEEP(*(.noinit))
		. = . + 16;	/* 预留, 防空段被丢弃 */
		__noinit_end = .;
	} > MAIN

	.stack (NOLOAD) : ALIGN(16) {
		/* ZIGUANG (2026-10-16): 4K -> 8K。实测 demo_main 内联后帧很大,
		 *   4K 栈会向下长进 .bss (从数组尾部开始踩), 表现为 [0] CPU RAM 自检在
		 *   word 126/127 失败 —— 书上板暂未暴露只因当时固件更小。 */
		. = . + 8K;
		__stack_top = .;
	} > MAIN

	/DISCARD/ : {
		*(.eh_frame*)
		*(.comment*)
		*(.riscv.attributes*)
	}
}

/* ============================================================================
 * 布局合同 (链接期断言; ASSERT 只能写在 SECTIONS 之外, 否则 ld 报 syntax error)
 *
 *   ① UART 大包缓冲必须在 BOOT arena 之下。
 *      UART 在线下装 / 自检 [10] 会往 0x8000 写代码, 若缓冲压到 arena 里,
 *      表现是"上传大包后偶发权重错", 且只在板上复现。
 *   ② 固件本体 (__fw_boot_end) 同理, 且 tools/check_fw_hex.py 出片前还会再查一遍
 *      (那边查的是**嵌进比特流的那一份**, 这一条只保证链接期不会悄悄错位)。
 * ========================================================================== */
ASSERT(UART_RXEND <= 0x00008000,
       "UART 大包缓冲越界进入 BOOT arena(0x8000): 压缩固件或把 RFC_BULK 调小")
ASSERT(UART_RXBUF >= 0x00000000,
       "UART_RXBUF 未分配到 BOOT 区: 检查 .uart_rx 段的 > BOOT 标记")

