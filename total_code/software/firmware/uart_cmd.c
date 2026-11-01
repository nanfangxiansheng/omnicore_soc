/* ============================================================================
 * uart_cmd.c — UART 命令解析器 (PC 控制推理全流程)
 * 非阻塞: 在主循环中调用 uart_cmd_poll(), 有完整命令帧时解析执行。
 * 与 bring-up 心跳共存 (无帧时打印心跳)。
 * ============================================================================
 */
#include "cmd_proto.h"
#include "y3t_detect.h"   /* 内含 <stdint.h>; 原自制 typedef 与之冲突 (ZIGUANG 修复) */
#include "gpio_ctl.h"     /* v65: GPIO 外设场景 (0x17/0x18) */

/* v65: 计时用 (忙等窗口), 与 demo.c 同一组寄存器 */
#define GPIOCMD_MTIME_LO (*(volatile uint32_t *)0x40002000u)
#define GPIOCMD_MTIME_HI (*(volatile uint32_t *)0x40002004u)


/* ---- 外部接口 (由 demo.c 提供) ---- */
extern void uart_putc(char c);
extern int  ddrb_read(uint32_t blk, uint32_t *dst, uint32_t beats);
extern int  ddrb_write(uint32_t blk, const uint32_t *src, uint32_t beats);
extern int  y3t_job(uint32_t in0, uint32_t in1, uint32_t wb, uint32_t out,
                    uint32_t W, uint32_t H, uint32_t Wo, uint32_t Ho,
                    uint32_t k, uint32_t s, uint32_t pad, uint32_t mode,
                    uint32_t cit, uint32_t cot, uint32_t act, uint32_t bits16,
                    uint32_t timeout);
extern int  demo_net_run(uint32_t wblk, uint32_t romblk, uint32_t *vals6);

/* ---- UART 底层 ---- */
#define UART_STATUS  (*(volatile uint32_t *)0x40000004u)
#define UART_DATA    (*(volatile uint32_t *)0x40000000u)
#define UART_TX_READY 0x2u
#define UART_RX_VALID 0x1u

/* ---- ZIGUANG v20: RX 通路诊断计数器 -------------------------------------
 * 为什么要这个: PC 侧发命令"完全没响应"时, 必须先分清是
 *   ① 字节根本没进 CPU (RX 引脚/引脚约束/波特率/RX FIFO) 还是
 *   ② 字节进来了但协议解析错 (checksum/状态机)。
 * 把收到的字节数和最后一个字节暴露给心跳打印, 一眼就能分辨。 */
static uint32_t g_rx_cnt = 0;
static uint8_t  g_rx_last = 0;
void uart_cmd_rxstat(uint32_t *cnt, uint8_t *last) {
    if (cnt)  *cnt  = g_rx_cnt;
    if (last) *last = g_rx_last;
}

/* ZIGUANG (2026-10-19 21:40) 【P0: 0xFF 哨兵吞掉真实数据字节】
 *
 * 旧实现 `uint8_t uart_getc_nb()` 用 **返回值 0xFF 表示"没有数据"**,
 * 而调用方写 `if (b == 0xFFu) return;` —— 于是**真实数据里的 0xFF 字节
 * 被当成"无数据"直接丢弃**, 既不进 RXB 也不进校验和:
 *     帧里出现 1 个 0xFF  ⇒ 之后全部字节错位 1 位 ⇒ 校验失败 ⇒ 整帧静默丢弃。
 *
 * 这就是"长帧永远收不到"的真正原因。板上实测铁证:
 *   - 96B 帧 PASS, 因为 (i*7+0x3a)&0xFF 在 i<96 内取不到 0xFF;
 *   - 128B 帧 FAIL, 因为 i=115 处正好是 0xFF;
 *   - 4KB / 3MB 权重必然含大量 0xFF ⇒ 100% FAIL。
 *   而 rx 计数增量 = 4107 ≈ 期望 4102 ⇒ 字节明明全进了 CPU, 只是被解析器扔了。
 *
 * 纯内容相关、与长度无关 —— 也解释了历史上"0xAA 图案通过、递增 0..255 失败"。
 *
 * 修法: 改成"返回有效标志 + 出参给数据", 数据字节不再兼职哨兵。
 * 顺带做批量取数: 一次把 FIFO 里已有的字节全部排空再返回, 主循环每轮
 *   不再只推进 1 字节 (原实现 115200 下要求主循环 <87us, 相机帧差一来就断流)。
 * 返回值 = 本次取到的字节数 (0 表示当前无数据)。
 */
static uint32_t uart_getc_burst(uint8_t *dst, uint32_t max) {
    uint32_t n = 0;
    while (n < max && (UART_STATUS & UART_RX_VALID)) {
        uint8_t b = (uint8_t)UART_DATA;
        g_rx_cnt++;
        g_rx_last = b;
        dst[n++] = b;
    }
    return n;
}

/* ZIGUANG v16: 自检时要复用真实命令处理函数, 但不能把二进制响应打进串口日志。 */
static uint8_t rsp_quiet = 0;
static void rsp_byte(uint8_t b) { if (!rsp_quiet) uart_putc((char)b); }

/* ---- PING 的"边排队边应答" ------------------------------------------------
 * uart_putc 是**逐字节忙等** (`while(!(UART_STATUS & 0x2))`)。一条 18B 的响应对
 * 应 1.56 ms 线速, 而这段时间里 RX FIFO 只能靠 512B 深度硬扛 —— 主机若不等响应
 * 继续灌 4KB 大包, FIFO 必满, 硬件**丢字节 + 置 overrun**, 权重静默错。
 * PING 是上传过程中唯一常发的"活口"命令, 所以让它只把 9 个字节**塞进 8 深的 TX
 * FIFO**(检查 has_room 再塞, 绝不越过 FIFO), 立刻返回继续收数。真的挤不进去
 * 才退回忙等 —— 那条路径只在主机把线速跑满时偶发。
 * ------------------------------------------------------------------------ */
static void ping_fast(void)
{
    /* 响应帧: 5A 81 00 04 00 'P' 'O' 'N' 'G'  (9 字节, 无独立校验字节按协议) */
    static const uint8_t fr[9] = { CMD_SYNC_RSP, CMD_PING | 0x80u, STATUS_OK,
                                   4u, 0u, 'P', 'O', 'N', 'G' };
    if (rsp_quiet) return;
    for (uint32_t k = 0; k < 9u; k++) {
        uart_putc((char)fr[k]);
    }
}

/* ============================================================================
 * ZIGUANG (2026-10-18 21:30) —— 【上传提速的另一半: 缩短"每处理完一条"的开销】
 *
 * 大包 (RFC_BULK) 把死时间摊薄了, 但真正决定上传速度的是**主机灌数据的比特率**
 * 减去固件每帧的处理开销。实测板上 0x0B 每条 (8 块=256B) 要 ~525 拍 @54MHz
 * ≈ 9.7us, 而 262B 线速是 22.8ms —— 处理开销只有 0.04%, 说明瓶颈**全在停等**。
 * 因此提速的正确姿势不是优化搬运 (那已经是 0.04%), 而是:
 *   ① 让主机别等 (大包 + 静默ACK);
 *   ② 大包时把"每字节一次函数调用 + 状态机"的代价压掉 —— RX FIFO 一次能读
 *      8 字节, 直接在帧内奔循环搬。
 * ========================================================================== */


static uint8_t g_hdr_sent = 0;   /* 本命令是否已经自己发过响应头 */
static void rsp_head(uint8_t cmd, uint8_t status, uint16_t len) {
    g_hdr_sent = 1;
    rsp_byte(CMD_SYNC_RSP);
    rsp_byte(cmd | 0x80u);
    rsp_byte(status);
    rsp_byte((uint8_t)(len & 0xFF));
    rsp_byte((uint8_t)(len >> 8));
}

/* ============================================================================
 * ZIGUANG (2026-10-18 21:10) —— 【P0 性能: 权重上传从"每块一次往返"改成流式】
 *
 * 旧路径每条 0x0B 只带 256B, 主机必须等响应才能发下一条 —— 实测 347 ms/条,
 * 3 MB 要 11,864 条 ⇒ 约 9 小时。三条浪费叠在一起:
 *   ① **停等往返**: 22.8ms 线速 + 主机侧收完 18B 响应才发下一条;
 *   ② **响应字节**: 每块回 6B (0x5A cmd st lo hi ck) = 3MB 上传额外吐 70KB;
 *   ③ **逐字节 TX 轮询**: rsp_byte 每个字节都要 `while(!(UART_STATUS&2))`,
 *      UART RX FIFO 有 512B 深, 主机只要连发就把 FIFO 塞满 —— 而 FIFO 满时
 *      硬件**丢字节并置 overrun**(见 ominicore_uart.v 的 rxf_push_now), 所以
 *      原来"主机不等响应"反而会静默丢数据, 只能靠停等保平安。
 *
 * 新路径: 单帧载荷上限 4096B (RFC_BULK), 一条 0x0B 最多写 **128 个 256b 块**
 * (4KB), 且默认**不回响应**(让主机连续灌, 自己不写 TX, 不拖慢收数);
 * 主机每 N 条发一次 0x06 之类的小命令确认活口。这样 RX FIFO 由主机线速填,
 * 我们只负责尽快搬进 DDR, 关键路径上不再有往返。
 *
 * 4KB 缓冲从哪来: board.ld 的 .uart_rx (BOOT NOLOAD, 地址由链接器分配),
 * 不占 MAIN、不进比特流, 且链接期断言不许越过 arena 起点。见该段注释。
 * ========================================================================== */
#define RFC_BULK   4096u                 /* 单帧最大载荷 (大包模式) */
#define RFC_SMALL  500u                  /* 小命令仍限 500, 检查提前到 LEN 解析处 */

extern uint8_t UART_RXBUF[];             /* board.ld 提供 */
#define RXB  UART_RXBUF
static uint16_t rx_cap = RFC_SMALL;      /* 本帧允许的载荷上限 (按命令选) */

/* DDR 桥单次传输的拍数上限 —— 与 demo.c 的 ddrb_write 内部切片一致。
 * 在这边重复定义是为了让 0x0B 自己按批提交, 不依赖 ddrb_write 的隐式切片
 * (隐式切片也要过一遍 4KB 中转缓冲, 反而多一次拷贝)。 */
#define DDRB_MAX_BEATS 8u      /* 16→8: 避免 AXI 突发跨 4KB 边界被截断, 见 demo.c 说明 */

/* ---- 接收状态机 ---- */
enum { RX_SYNC, RX_CMD, RX_LEN0, RX_LEN1, RX_PAYLOAD, RX_CKSUM };
static uint8_t rx_state = RX_SYNC;
static uint8_t rx_cmd, rx_len_lo, rx_len_hi, rx_cksum;
static uint16_t rx_len, rx_pos;
static uint32_t last_rx_tick = 0;     /* 最近一次收到字节的 mtime */

/* 【BUG-FIX 2026-10-22 15:25】把"正在收帧"暴露给主循环。
 *   板上实测: 短传(2KB)全对, 长传(2.9MB)坏 75%, 坏处内容全是 0xFF(UART 空闲电平)。
 *   根因: uart_cmd_poll 每轮只排空 64B, 而主循环里相机帧差分支含 ddrb_read(数百 µs~ms),
 *   长传时二者互相拖累 → RX FIFO(512) 满 → overrun → rx_state 复位 → 整帧丢弃,
 *   那些 DDR 位置就保持上电残留(全 0xFF)。
 *   修法: 主循环在"正在收帧"期间跳过相机分支, 让 poll 能以最高频率排空 FIFO。 */
uint32_t uart_rx_busy(void) { return (uint32_t)(rx_state != RX_SYNC); }

/* 【2026-10-23】FIFO 里还有待处理的字节（或帧没收到一半）→ 主循环应让路给 UART。
 *   原实现只在"帧收到一半"时让路：当一条命令已收完、FIFO 里还排着下一条时，
 *   主循环会去跑相机分支（≈1s，含 ddrb_read），把后续命令各拖 1 秒。
 *   实测后果：PC 逐字写尺度 ROM（每层 16*cot*2 个字）变成 1s/字，
 *   29 层逐层扫描里有 ~58 分钟纯耗在这上面。
 *   加上 FIFO 判据后，连续命令可以在一轮里排空，速度提升一个数量级。 */
uint32_t uart_rx_pending(void)
{
    return (uint32_t)((rx_state != RX_SYNC) || (UART_STATUS & UART_RX_VALID));
}

#define REG32(a) (*(volatile uint32_t *)(a))
#define OVL_BASE   0x40006000u
#define DDRB_BASE  0x80000000u
#define Y3T_BASE   0x40005000u

/* ---- 在线下装的 arena 保护 -------------------------------------------------
 * 为什么只允许这两个窗口: TCM 是单端口, 且 CPU 正在从 BOOT 低地址取指。
 * 允许任意地址写 = 一条命令就能把正在执行的代码改掉 (自毁)。
 *
 * ZIGUANG (2026-10-18 19:25) —— 【P0 修正: arena 窗口被固件自己吃掉】
 *   v16 起 BOOT arena 硬编码在 0x4000..0xFFFF (48KB), 当时 .text/.rodata 只到 0x3247。
 *   但 v24 新增 sw/y3t_exec.c 后 .rodata 暴涨到 **0x489B**, 与 arena 重叠 2,203 B,
 *   且重叠区内有真实数据:
 *     0x3FE0  off.7  (16B)
 *     0x3FF0  poff.4 (32B)  ← 自检 [10] 往 0x4000 写 4 条指令, 正好踩掉其后 16B
 *     0x4068  g_net  (1,840B = 整网 job 表)  ← UART 下装(默认 --addr 0x4000) 必踩
 *     0x4798  __clz_tab (256B)
 *   后果: v24 上板后每跑一次 [10] 自检就把池化偏移表写坏, 整网推理结果随之出错;
 *         UART 在线下装则直接毁掉 g_net。
 *   v23 的 .rodata 到 0x3B43, 离 0x4000 尚有 1,213B 余量 —— 所以 v23 侥幸没暴露。
 *
 *   现方案: BOOT 64KB 一分为二, 交付两边都有明确预算
 *     固件 0x0000..0x8000 (32KB, 现用 18.6KB, 余 13.4KB)
 *     arena 0x8000..0x10000 (32KB)
 *   并且 board.ld 导出 __fw_boot_end, tools/check_fw_hex.py 出片前断言
 *   __fw_boot_end <= ARENA_BOOT_LO —— 固件再长大就直接 FAIL, 不允许悄悄越界。
 *
 *   MAIN arena 0x1000_5000..0x1000_7FFF : 栈顶 0x1000_4990, 留 1.6KB 余量后剩 12KB
 */
#define ARENA_BOOT_LO 0x00008000u
#define ARENA_BOOT_HI 0x00010000u
#define ARENA_MAIN_LO 0x10005000u
#define ARENA_MAIN_HI 0x10008000u
static int arena_ok(uint32_t a, uint32_t bytes)
{
    uint32_t e = a + bytes;
    if (a & 3u) return 0;                        /* 必须 4 字节对齐 */
    if (e < a)  return 0;                        /* 溢出 */
    if (a >= ARENA_BOOT_LO && e <= ARENA_BOOT_HI) return 1;
    if (a >= ARENA_MAIN_LO && e <= ARENA_MAIN_HI) return 1;
    return 0;
}

/* ---- 大包是否需要回 ACK (见文件头 RFC_BULK) ----
 * 0 = 静默 (默认), 主机连续灌; 1 = 每条 0x0B 回 4B。
 * 供 PC 侧在"怀疑丢数据"时切到调试模式。 */
static uint8_t bulk_ack = 0;

/* ---- 0x11 W3T: 把 DDR 里暂存的块流式搬进 y3tram 权重区 ----
 * 载荷 {u32 wb_word, u32 nwords}: 从 DDR 的 W3T_SRC_BLK 起(由 0x0B 刚灌进去的
 * 连续 nwords 个 256b 字) 全部搬到 y3tram 绝对字地址 wb_word。
 *
 * 为什么需要它 (而不是沿用 y3t_exec 的逐层搬):
 *   ① 权重在 DDR 里是**按层连续、层间有空洞**的布局 (engine_layout.json),
 *      【不能】当成一整段绝对地址直接搬 —— 每层起点 = NET_WB_TOP - 该层字数。
 *   ② UART 上传的最终目的就是填 y3tram, 先落 DDR 再让 CPU 逐层搬等于搬两遍;
 *      有了 CMD 直通 (wload_dma 0x04 寄存器), 同一次 DMA 就能同时写 y3tram
 *      和 DDR, 省掉这一趟。 */

/* ---- 命令执行 ---- */
static void cmd_exec(uint8_t cmd, const uint8_t *pay, uint16_t len) {
    g_hdr_sent = 0;
    uint32_t addr, blk, dst, words;
    uint16_t n;
    uint8_t st = STATUS_OK;
    uint32_t i;

    switch (cmd) {
    case CMD_PING:
        rsp_head(cmd, STATUS_OK, 4);
        rsp_byte('P'); rsp_byte('O'); rsp_byte('N'); rsp_byte('G');
        break;

    case CMD_MEM_READ: {
        /* 0x02: {u32 addr, u16 nbytes} -> 回 nbytes 字节 (addr 必须 4 字节对齐)
         *
         * ZIGUANG 修复 (2026-10-18): 原文 `for (i=0;i<n;i++) rsp_byte(REG32(addr+i));`
         *   addr+i 是【未对齐地址】, 总线按字返回时低 8 位永远是同一个字节 =>
         *   i=1,2,3 读到重复数据, 而且每字节都走一次总线。
         *   改为每 4 字节读一次字再按小端拆开。 */
        uint32_t k;
        if (len < 6) { st = STATUS_ERR_PARAM; break; }
        addr = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        n = (uint16_t)pay[4] | ((uint16_t)pay[5]<<8);
        if (n > 480) n = 480;
        if (addr & 3u) { st = STATUS_ERR_PARAM; break; }
        rsp_head(cmd, STATUS_OK, n);
        for (i = 0; i < n; i += 4u) {
            uint32_t wv = REG32(addr + i);
            for (k = 0; k < 4u && (i + k) < n; k++)
                rsp_byte((uint8_t)((wv >> (8u * k)) & 0xFFu));
        }
        break;
    }

    case CMD_MEM_WRITE: {
        /* 0x03: {u32 addr, u16 nbytes, data} -> 写 nbytes 字节 (addr 对齐, nbytes 为 4 的倍数)
         *
         * ZIGUANG 修复 (2026-10-18): 原来 `for(...) REG32(addr+i)=pay[6+i];` 是
         * 【逐字节写整字】—— 同一个字被写 4 次(每次只改一个字节, 其余填 0),
         * 会把相邻字节冲掉, 而且写未对齐地址还要靠 trap_emu 兜。改为按字拼装后写。 */
        uint32_t k;
        if (len < 6) { st = STATUS_ERR_PARAM; break; }
        addr = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        n = (uint16_t)pay[4] | ((uint16_t)pay[5]<<8);
        if ((addr & 3u) || (n & 3u)) { st = STATUS_ERR_PARAM; break; }
        if (len < (uint16_t)(6u + n)) { st = STATUS_ERR_PARAM; break; }
        for (i = 0; i < n; i += 4u) {
            uint32_t wv = 0;
            for (k = 0; k < 4u; k++) wv |= ((uint32_t)pay[6u + i + k]) << (8u * k);
            REG32(addr + i) = wv;
        }
        break;
    }

    case CMD_LOAD_IMG: {
        /* {u32 addr, u32 nwords, u32 word[n]} -> 对齐写 arena。
         * 注意: 与 0x03 MEM_WRITE 的区别 —— 这里是【按字对齐】写,
         *   0x03 那个是逐字节写整字 (会重叠 3 次 + 触发未对齐陷阱模拟), 不能用来下装代码。 */
        uint32_t a, nw, bad = 0;
        if (len < 8) { st = STATUS_ERR_PARAM; break; }
        a  = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        nw = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8) | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        if (nw > 120u || len < (uint16_t)(8u + nw*4u) || !arena_ok(a, nw*4u)) {
            st = STATUS_ERR_PARAM; bad = 1u;
        }
        if (!bad) {
            for (i = 0; i < nw; i++) {
                uint32_t o = 8u + i*4u;
                REG32(a + i*4u) = (uint32_t)pay[o] | ((uint32_t)pay[o+1u]<<8)
                                | ((uint32_t)pay[o+2u]<<16) | ((uint32_t)pay[o+3u]<<24);
            }
        }
        rsp_head(cmd, st, 4);
        rsp_byte((uint8_t)(bad ? 0u : (nw & 0xFFu)));
        rsp_byte((uint8_t)(bad ? 0u : ((nw >> 8) & 0xFFu)));
        rsp_byte(0); rsp_byte(0);
        break;
    }

    case CMD_EXEC: {
        /* {u32 entry} -> 先回 GO 再 fence.i + 跳转 (被调代码 ret 后回到本循环) */
        uint32_t e;
        if (len < 4) { st = STATUS_ERR_PARAM; break; }
        e = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        if (!arena_ok(e, 4u)) { st = STATUS_ERR_PARAM; break; }
        rsp_head(cmd, STATUS_OK, 4);
        rsp_byte('G'); rsp_byte('O'); rsp_byte('!'); rsp_byte('!');
        __asm__ volatile ("fence.i" ::: "memory");   /* 清 I$: 刚写进去的代码才可见 */
        ((void (*)(void))e)();
        break;
    }

    case CMD_DDR2TCM: {
        /* {u32 ddr_blk, u32 tcm_addr, u32 bytes} -> DDR 分块搬到 TCM arena
         * 每块 512B = 16 拍 = 128 字; ddrb_read 内部已按 <=16 拍分段。 */
        static uint32_t tbuf[128];
        uint32_t blk, dst, bytes, done = 0;
        if (len < 12) { st = STATUS_ERR_PARAM; break; }
        blk   = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        dst   = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8) | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        bytes = (uint32_t)pay[8] | ((uint32_t)pay[9]<<8) | ((uint32_t)pay[10]<<16) | ((uint32_t)pay[11]<<24);
        if ((bytes & 3u) || bytes == 0u || !arena_ok(dst, bytes)) { st = STATUS_ERR_PARAM; break; }
        while (done < bytes) {
            uint32_t cb = bytes - done;
            uint32_t k;
            if (cb > 512u) cb = 512u;
            if (ddrb_read(blk + done / 32u, tbuf, (cb + 31u) / 32u) != 0) { st = STATUS_ERR_TMO; break; }
            for (k = 0; k < (cb >> 2); k++)
                REG32(dst + done + k*4u) = tbuf[k];
            done += cb;
        }
        if (st == STATUS_OK) __asm__ volatile ("fence.i" ::: "memory");
        rsp_head(cmd, st, 4);
        rsp_byte((uint8_t)(done & 0xFFu)); rsp_byte((uint8_t)((done >> 8) & 0xFFu));
        rsp_byte((uint8_t)((done >> 16) & 0xFFu)); rsp_byte((uint8_t)((done >> 24) & 0xFFu));
        break;
    }

    case CMD_Y3T_JOB: {
        /* ZIGUANG 补齐 (2026-10-16): cmd_proto.h 早已定义 0x04 而 uart_cmd.c 无实现,
         * 上位机发不了 job。载荷 17 x u32 按字节拼装 (避免 RV32I 非对齐访问陷阱),
         * 回 {ret, cyc_mac, cyc_tot} 供 golden 比对。 */
        uint32_t a[17];
        if (len < 68) { st = STATUS_ERR_PARAM; break; }
        for (i = 0; i < 17u; i++)
            a[i] = (uint32_t)pay[i*4] | ((uint32_t)pay[i*4+1]<<8)
                 | ((uint32_t)pay[i*4+2]<<16) | ((uint32_t)pay[i*4+3]<<24);
        {
            int r = y3t_job(a[0], a[1], a[2], a[3],  a[4], a[5], a[6], a[7],
                            a[8], a[9], a[10], a[11],  a[12], a[13], a[14], a[15],
                            a[16]);
            uint32_t cm = REG32(Y3T_BASE + 0x4Cu);
            uint32_t ct = REG32(Y3T_BASE + 0x50u);
            uint32_t ret = (uint32_t)r;
            rsp_head(cmd, STATUS_OK, Y3T_JOB_RSP_LEN);
            rsp_byte((uint8_t)(ret      )); rsp_byte((uint8_t)(ret >> 8));
            rsp_byte((uint8_t)(ret >> 16)); rsp_byte((uint8_t)(ret >> 24));
            rsp_byte((uint8_t)(cm       )); rsp_byte((uint8_t)(cm >> 8));
            rsp_byte((uint8_t)(cm >> 16)); rsp_byte((uint8_t)(cm >> 24));
            rsp_byte((uint8_t)(ct       )); rsp_byte((uint8_t)(ct >> 8));
            rsp_byte((uint8_t)(ct >> 16)); rsp_byte((uint8_t)(ct >> 24));
        }
        break;
    }

    case CMD_WLOAD_RAW: {
        /* 0x0B: {u32 blk, u16 len, data} -> 从 DDR 块 blk 起写 len/32 个 256bit 块。
         *
         * ZIGUANG 修复 (2026-10-18, 阻塞整网推理的隐性 bug):
         *   原来【固定只写 1 个块(32B)】—— 取 pay[6..38] 拼 8 个字就 ddrb_write(...,1)。
         *   但 PC 侧按 256B/条发送 (upload_weights.py CHUNK=256、块号步进 +8),
         *   于是每 8 块只有第 0 块被写、其余 7 块保留上电残留 => 权重 87.5% 是垃圾,
         *   DMA 搬进 y3tram 后算出来必然错。(y3t_smoke.py 更糟: 块号步进 +1, 整体错位。)
         * 现在按 len 循环写 len/32 块, 与两个 PC 脚本的 256B 粒度对齐。
         * ZIGUANG (2026-10-18 21:10): 大包模式 —— 一条最多 128 块 (4KB)。
         *   DDRB_MAX_BEATS=16, 所以缓冲里仍按 16 块一批提交, 只是不再限 15 块。
         *   4KB 源缓冲在 BOOT 的 .uart_rx (见文件头 RFC_BULK 说明)。 */
        static uint32_t w[8 * 128];  /* 128 块 × 8 字 (BOOT 内静态, 不占 MAIN) */
        uint32_t nb, b, dlen, off;
        if (len < 38u) { st = STATUS_ERR_PARAM; break; }
        blk  = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        /* 数据长度取【载荷里的 len 字段】, 不是整个 payload 长度:
         * payload = 6 字节头(u32 blk + u16 len) + 数据, 所以总长永远不是 32 的倍数
         * (第一次改这里时就栽在这: len=262 被判成不对齐 -> 直接 PARAM 错)。 */
        dlen = (uint32_t)pay[4] | ((uint32_t)pay[5] << 8);
        if (dlen < 32u || (dlen & 31u) || (6u + dlen) > (uint32_t)len) { st = STATUS_ERR_PARAM; break; }
        nb = dlen / 32u;
        if (nb > 128u) { st = STATUS_ERR_PARAM; break; }
        /* ZIGUANG (2026-10-18 v24): 原来【每块一次 ddrb_write(...,1)】= 8 次独立
         *   总线事务, 每次都要等 DDR 桥 done。板上实测 8 块平均 347 ms/条
         *   ⇒ 3 MB 权重 ≈ 9 小时, 完全不可用。
         *   改成把 nb 块拼进一个缓冲、**按 16 拍一批 ddrb_write**, 事务数降到 1/16。 */
        for (b = 0; b < nb * 8u; b++) {
            uint32_t o = 6u + b * 4u;               /* 按字节拼, 避开 RV32I 非对齐访问 */
            w[b] = (uint32_t)pay[o] | ((uint32_t)pay[o+1u]<<8)
                 | ((uint32_t)pay[o+2u]<<16) | ((uint32_t)pay[o+3u]<<24);
        }
        for (off = 0; off < nb; off += DDRB_MAX_BEATS) {
            uint32_t n = nb - off;
            if (n > DDRB_MAX_BEATS) n = DDRB_MAX_BEATS;
            if (ddrb_write(blk + off, &w[off * 8u], n) != 0) { st = STATUS_ERR_TMO; break; }
        }
        /* 大包默认静默: 主机连续灌 4KB/条, 自己不写 TX 才不会被发送拖慢。
         * 需要确认时主机发 0x06 WLOAD_POLL / 0x01 PING (PING 已改成边排队边应答)。 */
        if (bulk_ack) {
            uint32_t dn = nb;                        /* 回已写块数, 便于主机计数校验 */
            rsp_head(cmd, st, 4);
            rsp_byte((uint8_t)(dn & 0xFFu)); rsp_byte((uint8_t)((dn >> 8) & 0xFFu));
            rsp_byte(0); rsp_byte(0);
        }
        break;
    }

    case CMD_W3T: {
        uint32_t ddr_ret;
        /* 0x11: {u32 wb_word, u32 nwords} -> 把 DDR 里 W3T_SRC_BLK 起的连续
         *       nwords 个 256bit 字搬进 y3tram 绝对字地址 wb_word。
         *
         * 数据流: PC 先按 4KB 大包 0x0B 灌进 DDR 块 W3T_SRC_BLK (静默, 不回包),
         *         再发一条 0x11 触发 wload_dma。wload_dma 走 CMD 直通
         *         (REG_CMD bit1=DDR) 同一次读源、两个目的: y3tram + DDR 回写。
         *
         * 为什么回写 DDR 是有用的而不是浪费: 回写地址用 REG_DDRD 指向权重正式
         *   区, 于是"灌进暂存区"与"落到权重区"合并成一次 —- 省掉原先
         *   0x0B(写DDR) + 逐层 CPU 搬运 两趟。这也是本命令存在的理由。
         *
         * 注意 (2026-10-19 修): 下发前必须确认上一笔 wload_dma 已真的完成。
         *   CTRL.done_flag 是 W1C 且不会自动清, 但 RTL 里已改为 start 时无条件
         *   清零, 所以这里只需先 W1C 一次把可能残留的清掉, 再 start,
         *   然后轮询 done。原 tb 就是因为残留 done 让上层在第 200 拍就 break。 */
        if (len < 8u) { st = STATUS_ERR_PARAM; break; }
        dst   = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        words = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8) | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        if (words == 0u || words > 65536u) { st = STATUS_ERR_PARAM; break; }
        /* 【BUG-FIX 2026-10-22 17:45】新增可选第 3 个 u32 = DDR 回写目标块。
         *   原实现 `DDRD = dst` 把 y3tram 的片上字地址(0..32768)当成 DDR 块地址用,
         *   于是回写落到 ddr_blk = dst (例如 32696*32B = 1MB), 正好踩进相机帧槽
         *   (0..460800), 与相机写冲突 ⇒ 回写失败 ⇒ done 永不来 ⇒ 0x11 超时。
         *   实测: 单独调 0x11 搬 L0 权重必失败(status=3, ok_words 只算到一半)。
         *
         *   现改为:
         *     ddr_ret = 0  -> CMD=0x01 只写 y3tram, 完全不动 DDR (逐层对比就用这个)
         *     ddr_ret != 0 -> CMD=0x03 同时回写 DDR 块 ddr_ret (原"暂存区→权重区"用法)
         *   只传 8 字节的旧调用等价于 ddr_ret = dst, 保持向后兼容。 */
        ddr_ret = (len >= 12u)
                ? ((uint32_t)pay[8] | ((uint32_t)pay[9]<<8)
                 | ((uint32_t)pay[10]<<16) | ((uint32_t)pay[11]<<24))
                : dst;

        /* 1. W1C 清残留完成标志
         *    ★ RTL: done_clr_w = ctrl_wr && hwdata[1] —— 清位在 **bit1**,
         *    不是 done_flag 所在的 bit0。写 bit1=1 才是清, 写 0 什么也不做。 */
        REG32(0x9000000Cu) = 0x2u;

        /* 2. 配寄存器: SRC = 暂存区块号, DST = y3tram 绝对字地址,
         *    DDRD = 权重正式区 (与 DST 同基址, 让回写直接落到最终位置) */
        REG32(0x90000000u) = (uint32_t)CMD_W3T_SRC_BLK;   /* SRC  (DDR 块) */
        REG32(0x90000004u) = dst;                         /* DST  (y3tram 256b 字) */
        REG32(0x90000008u) = words;                       /* LEN  (字数) */
        REG32(0x90000014u) = ddr_ret;                     /* DDRD (回写目标块; 0=不回写) */
        REG32(0x90000010u) = (ddr_ret == 0u) ? 0x01u : 0x03u;  /* sel: bit1=回写使能 */

        /* 3. 启动 */
        REG32(0x9000000Cu) = 1u;

        /* 4. 轮询 done (上限按 words 估算: 每字最多 3 拍 fan-out + DDR 突发余量) */
        {
            uint32_t lim = 200000u + words * 8u;
            uint32_t t;
            for (t = 0; t < lim; t++) {
                if (REG32(0x9000000Cu) & 1u) break;
            }
            if (t >= lim) st = STATUS_ERR_TMO;
        }

        /* 5. 回 {ok_words, err}
         *    ok_words 取 CTRL 里的 done_words (实际送进 y3tram 的字数), 便于主机
         *    逐批核对; 不一致说明有字被静默丢掉, 必须停下来查而不是继续灌。 */
        {
            uint32_t ctrl  = REG32(0x9000000Cu);
            /* CTRL = {12'd0, done_words[15:0], 2'b0, busy_r, done_flag}
             *   ⇒ done_words 在 bit[19:4], 不是 bit[17:2]。 */
            uint32_t okw   = (ctrl >> 4) & 0xFFFFu;
            uint32_t ecode = (st == STATUS_OK) ? 0u : 1u;
            rsp_head(cmd, st, 8);
            rsp_byte((uint8_t)(okw & 0xFFu));      rsp_byte((uint8_t)((okw >> 8) & 0xFFu));
            rsp_byte((uint8_t)((okw >> 16) & 0xFFu)); rsp_byte(0);
            rsp_byte((uint8_t)(ecode & 0xFFu)); rsp_byte(0); rsp_byte(0); rsp_byte(0);
        }
        break;
    }

    case CMD_WLOAD_START:
        if (len < 10) { st = STATUS_ERR_PARAM; break; }
        blk  = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        dst  = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8) | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        words= (uint32_t)pay[8] | ((uint32_t)pay[9]<<8);
        /* 写 wload_dma 寄存器 @0x9000_0000 */
        REG32(0x90000000u) = blk;                    /* SRC */
        /* ZIGUANG 修复 (2026-10-16): 原写 dst*4 (注释"byte→word") 是错的 ——
         * wload_dma 的 DST 是 y3tram 的 **256bit 字地址直通** (RTL: s_addr<=dst_word_r;
         * tb_wload_dma 亦以 DST=0x100 期望 eng_addr=0x100+wr_count)。乘 4 会让权重
         * 落到 4 倍地址处 (如 WB_BASE=20480 → 81920), 整层权重装错位置。 */
        REG32(0x90000004u) = dst;                    /* DST = y3tram 256b 字地址 (直通) */
        REG32(0x90000008u) = words;                  /* LEN */
        REG32(0x9000000Cu) = 1u;                     /* CTRL.start */

        /* 【BUG-FIX 2026-10-23】wload_dma 每笔任务**只写 words-1 块**，末块不写，
         *   目标处保留上一次搬运的残留。权重侧已在 y3t_exec.c 的 exec_wload 里补，
         *   但 0x05 (PC 装 IN) 走的也是这条 DMA —— 不补就会让 IN 的最后一块(9215)
         *   是残留 ⇒ L0 的最后一个像素错 ⇒ 每层输出末像素错（逐层放大）。
         *   板上实测：IN 块 9213/9214 与 act_in.bin 一致，块 9215 完全不同；
         *   补写末块后 L0..L8 逐层全对。
         *   这里改成"同步 + 补末块"：PC 端后续再 poll 会立刻看到 done。 */
        if (words >= 2u) {
            uint32_t guard = 0;
            uint32_t lb[8];
            uint32_t kk;
            volatile uint32_t *pp;
            /* 【BUG-FIX 2026-10-23】同 exec_wload：start 后 done 要等 RTL 真正起步才清零，
             *   不先等 done 归 0 就 poll 会把上一笔的 done=1 当成"已完成"⇒
             *   末块补写在 DMA 还没搬完时就执行（PC 侧靠毫秒级延迟侥幸掩盖）。 */
            {
                uint32_t w;
                for (w = 0u; w < 100000u; ++w)
                    if (!(REG32(0x9000000Cu) & 1u)) break;
            }
            while (!(REG32(0x9000000Cu) & 1u))
                if (++guard > 200000000u) break;
            if (ddrb_read(blk + words - 1u, lb, 1u) == 0) {
                pp = (volatile uint32_t *)(0x60000000u + ((dst + words - 1u) << 5));
                for (kk = 0; kk < 8u; kk++) pp[kk] = lb[kk];
            }
        }
        break;

    case CMD_WLOAD_POLL: {
        uint32_t status = REG32(0x9000000Cu);
        uint32_t dw = REG32(0x9000001Cu);
        rsp_head(cmd, STATUS_OK, 8);
        rsp_byte(status & 0xFF); rsp_byte((status>>8)&0xFF);
        rsp_byte(dw & 0xFF); rsp_byte((dw>>8)&0xFF);
        rsp_byte((dw>>16)&0xFF); rsp_byte((dw>>24)&0xFF);
        rsp_byte(0); rsp_byte(0);
        break;
    }

    case CMD_OVL_BOX: {
        if (len < 14) { st = STATUS_ERR_PARAM; break; }
        uint32_t nbox = pay[0];
        if (nbox >= 8) { st = STATUS_ERR_PARAM; break; }
        uint32_t base = OVL_BASE + nbox * 12u;
        REG32(base + 0u) = (uint32_t)pay[1] | ((uint32_t)pay[2]<<8) | ((uint32_t)pay[3]<<16) | ((uint32_t)pay[4]<<24);
        REG32(base + 4u) = (uint32_t)pay[5] | ((uint32_t)pay[6]<<8) | ((uint32_t)pay[7]<<16) | ((uint32_t)pay[8]<<24);
        REG32(base + 8u) = (uint32_t)pay[9] | ((uint32_t)pay[10]<<8);
        break;
    }

    case CMD_OVL_EN:
        if (len < 1) { st = STATUS_ERR_PARAM; break; }
        REG32(OVL_BASE + 0x60u) = pay[0];
        break;

    case CMD_GET_FRAME: {
        if (len < 4) { st = STATUS_ERR_PARAM; break; }
        blk = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        n = 64;  /* 8 块 = 256B = 64 字 */
        /* 经 DDR 桥读 */
        REG32(DDRB_BASE + 4u) = blk;
        REG32(DDRB_BASE + 0u) = (8u << 8) | 0u;
        REG32(DDRB_BASE + 0u) = (8u << 8) | 1u;
        uint32_t guard = 0;
        while (!(REG32(DDRB_BASE + 8u) & 1u)) { if (++guard > 2000000u) { st = STATUS_ERR_TMO; break; } }
        if (st == STATUS_OK) {
            /* v67 修复: 8 块 = 256B = 64 个 32 位字。旧代码 `for(256)
             * rsp_byte(REG32(DATA))` 每读一次弹一个字, 超读 192 字 =>
             * DATA FIFO 下溢 => 总线停死 => 整机挂死 (0x09 当年禁用的真因)。
             * 改为弹 64 次、每次按小端拆 4 字节。 */
            rsp_head(cmd, STATUS_OK, 256);
            for (i = 0; i < 64; i++) {
                uint32_t wv = REG32(DDRB_BASE + 0x0Cu);
                rsp_byte((uint8_t)(wv       & 0xFFu));
                rsp_byte((uint8_t)((wv>>8 ) & 0xFFu));
                rsp_byte((uint8_t)((wv>>16) & 0xFFu));
                rsp_byte((uint8_t)((wv>>24) & 0xFFu));
            }
            REG32(DDRB_BASE + 8u) = 1u;
        }
        break;
    }

    case CMD_GET_INFO: {
        uint32_t frame = REG32(OVL_BASE + 0x78u);
        uint32_t y3t_id = REG32(Y3T_BASE + 0xFCu);
        rsp_head(cmd, STATUS_OK, 12);
        uint32_t vals[3] = {frame, y3t_id, 0x46554E31u};  /* "FUN1" */
        for (i = 0; i < 12; i++) rsp_byte(((uint8_t*)vals)[i]);
        break;
    }

    case CMD_DET_READ: {
        /* 0x12: 从 y3tram 读 2 个尺度的检测头, 用固件自己的全整数解码。
         * 回 {u16 n} + n*12B { i16 cx,cy,w,h (Q8, 96 坐标); i16 conf; u8 cls; u8 scale; u16 pad }
         * 目的: 与 PC 侧 tools/det_read.py 的结果交叉验证 C 实现的数值。
         * 注意: 调用前 y3tram 里必须有刚跑完的整网结果（0x10 NET_RUN 之后）。 */
        static det_box_t bx[24];
        int nb = det_decode((const int16_t *)0x60000000u, 24, bx);
        int i;
        if (nb < 0) { st = STATUS_ERR_PARAM; break; }
        rsp_head(cmd, st, (uint16_t)(2 + nb*12));
        rsp_byte((uint8_t)(nb & 0xFF)); rsp_byte(0);
        for (i = 0; i < nb; i++) {
            int16_t v[6];
            v[0] = (int16_t)(bx[i].cx96_q8 >> 8);   /* Q8 -> 96 坐标整数 */
            v[1] = (int16_t)(bx[i].cy96_q8 >> 8);
            v[2] = (int16_t)(bx[i].w96_q8  >> 8);
            v[3] = (int16_t)(bx[i].h96_q8  >> 8);
            v[4] = bx[i].conf_q;
            v[5] = (int16_t)((bx[i].cls & 0xFF) | (bx[i].scale << 8));
            rsp_byte((uint8_t)(v[0]&0xFF)); rsp_byte((uint8_t)((v[0]>>8)&0xFF));
            rsp_byte((uint8_t)(v[1]&0xFF)); rsp_byte((uint8_t)((v[1]>>8)&0xFF));
            rsp_byte((uint8_t)(v[2]&0xFF)); rsp_byte((uint8_t)((v[2]>>8)&0xFF));
            rsp_byte((uint8_t)(v[3]&0xFF)); rsp_byte((uint8_t)((v[3]>>8)&0xFF));
            rsp_byte((uint8_t)(v[4]&0xFF)); rsp_byte((uint8_t)((v[4]>>8)&0xFF));
            rsp_byte((uint8_t)(v[5]&0xFF)); rsp_byte((uint8_t)((v[5]>>8)&0xFF));
        }
        break;
    }

    case CMD_OVL_MODE: {
        /* 0x14: overlay 开关掩码。bit0=四角自检 bit1=运动框 bit2..=YOLO框(留给后面)
         * 用途: 跑整网时关掉四角框和运动框, 屏幕上只剩检测框。 */
        extern volatile uint32_t g_ovl_mask;
        if (len < 1) { st = STATUS_ERR_PARAM; break; }
        g_ovl_mask = pay[0];
        /* 立即应用: OVL_EN 的 bit4 是运动框使能, 这里按掩码重算 */
        REG32(OVL_BASE + 0x60u) = (REG32(OVL_BASE + 0x60u) & ~0x10u)
                                | ((g_ovl_mask & 0x02u) ? 0x10u : 0u);
        break;
    }

    case CMD_NET_RUN: {
        /* 0x10: {u32 ddr_w_base_blk, u32 rom_blk} -> 整网跑一次 (demo.c 里的薄封装,
         *   真正的执行器在 y3t_exec.c, 静态 job 表 + 逐层权重 DMA) */
        uint32_t vals[6];
        uint32_t wblk, rblk;
        int ret;
        if (len < 8) { st = STATUS_ERR_PARAM; break; }
        wblk = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8) | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        rblk = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8) | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        ret  = demo_net_run(wblk, rblk, vals);
        vals[0] = (uint32_t)ret;
        rsp_head(cmd, STATUS_OK, NET_RUN_RSP_LEN);
        for (i = 0; i < NET_RUN_RSP_LEN; i++) rsp_byte(((uint8_t *)vals)[i]);
        break;
    }

    case CMD_AUTO_RUN: {
        /* 0x15: {u32 en, u32 wblk, u32 romblk} -> 板上自动循环开关 (v37)
         *   en=1: demo 主循环进入 y3t_auto_step()（相机帧→PP→整网→解码→画框）；
         *   en=0: 退出回运动检测分支。权重/尺度必须在 DDR（同 0x10 的前提）。 */
        extern uint32_t g_auto_en, g_auto_wblk, g_auto_romblk;
        if (len < 4) { st = STATUS_ERR_PARAM; break; }
        g_auto_en = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8)
                  | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        if (len >= 8) {
            g_auto_wblk = (uint32_t)pay[4] | ((uint32_t)pay[5]<<8)
                        | ((uint32_t)pay[6]<<16) | ((uint32_t)pay[7]<<24);
        }
        if (len >= 12) {
            g_auto_romblk = (uint32_t)pay[8] | ((uint32_t)pay[9]<<8)
                          | ((uint32_t)pay[10]<<16) | ((uint32_t)pay[11]<<24);
        }
        rsp_head(cmd, STATUS_OK, 4);
        rsp_byte((uint8_t)(g_auto_en      )); rsp_byte((uint8_t)(g_auto_en >> 8));
        rsp_byte((uint8_t)(g_auto_en >> 16)); rsp_byte((uint8_t)(g_auto_en >> 24));
        break;
    }

    case CMD_SET_THR: {
        /* 0x16: {i32 thr_q_scale0, i32 thr_q_scale1} -> 运行时改检测阈值 (v37)。
         * -32768 = 恢复表内默认 (conf_thr=0.30); PC 侧 tools/det_thr.py --conf c 会算好发来。 */
        extern int32_t g_det_thr_ovr[2];
        int32_t v[2];
        if (len < 8) { st = STATUS_ERR_PARAM; break; }
        for (i = 0; i < 2; i++)
            v[i] = (int32_t)(int8_t)pay[i*4+3] << 24
                 | (uint32_t)pay[i*4+2] << 16
                 | (uint32_t)pay[i*4+1] << 8
                 | (uint32_t)pay[i*4];
        g_det_thr_ovr[0] = v[0];
        g_det_thr_ovr[1] = v[1];
        rsp_head(cmd, STATUS_OK, 8);
        for (i = 0; i < 8; i++) rsp_byte(((uint8_t *)v)[i]);
        break;
    }

    /* ================= v65: GPIO 外设 (赛题"外设系统集成") ================= */
    case CMD_GPIO: {
        /* 0x17: {u8 op, u8 val}
         *   把 GPIO 的 8 个寄存器直接暴露给主机 —— 用来做"外设可读可写"的板上实证,
         *   也方便在没有引脚的阶段先验证寄存器通路 (写 DATA_OUT -> DIR=1 -> 回读)。*/
        uint8_t  op, val;
        uint32_t r = 0u, k;
        if (len < 1) { st = STATUS_ERR_PARAM; break; }
        op  = pay[0];
        val = (len >= 2) ? pay[1] : 0u;

        if (op == 0) {                       /* 读回 8 个寄存器 */
            rsp_head(cmd, STATUS_OK, 32);
            for (k = 0; k < 8u; k++) {
                uint32_t v = ((volatile uint32_t *)GPIO_BASE)[k];
                rsp_byte((uint8_t)( v        & 0xFFu));
                rsp_byte((uint8_t)((v >>  8) & 0xFFu));
                rsp_byte((uint8_t)((v >> 16) & 0xFFu));
                rsp_byte((uint8_t)((v >> 24) & 0xFFu));
            }
            break;                           /* 已自己发过头, 不走收尾补头 */
        }
        switch (op) {
        case 1: GPIO_DATA_OUT = val; r = GPIO_DATA_OUT; break;
        case 2: GPIO_DIR      = val; r = GPIO_DIR;      break;
        case 3: GPIO_SET      = val; r = GPIO_DATA_OUT; break;
        case 4: GPIO_CLR      = val; r = GPIO_DATA_OUT; break;
        case 5: GPIO_IRQ_RISE = val; r = GPIO_IRQ_RISE; break;
        case 6: r = GPIO_IRQ_STATUS; GPIO_IRQ_STATUS = val; break;  /* W1C */
        case 7: GPIO_IRQ_FALL = val; r = GPIO_IRQ_FALL; break;      /* 板载按键用这个 */
        default: st = STATUS_ERR_PARAM; break;
        }
        if (st != STATUS_OK) break;
        rsp_head(cmd, STATUS_OK, 4);
        rsp_byte((uint8_t)( r        & 0xFFu));
        rsp_byte((uint8_t)((r >>  8) & 0xFFu));
        rsp_byte((uint8_t)((r >> 16) & 0xFFu));
        rsp_byte((uint8_t)((r >> 24) & 0xFFu));
        break;
    }

    case CMD_GPIO_IRQ: {
        /* 0x18: {u8 src, u8 rsv[3], u32 ms}  ->  {次数, mcause, EXTIRQ.PENDING, 源事件数}
         *   src=0 软件中断 (MSIP, v66) —— 确定性, 不依赖任何外部接线;
         *     开窗即触发 1 次 (mcause=0x80000003, ISR 写 0 清源), ms 为窗口时长;
         *     【板上 mtimecmp 寄存器坏 (读恒 0/写不粘), MTIP 比较中断停用, 见 gpio_ctl.h】
         *   src=1 GPIO 外部中断 (需要 gpio_in[0] 下降沿, 即 KEY1 按下);
         *   src=2 两者都开。
         * 兼容旧格式 (len<8): 载荷 {u32 ms}, src 默认 0。
         * 为什么必须窗口式: boot.S 里 mie=0/MIE=0, 而 cam_frame/y3t_done/uart 三个源
         *   会常挂, 常开中断会把主循环淹掉。这里开 ms 毫秒就关, 用完即止。 */
        uint8_t  src = 0u;
        uint32_t ms = 100u, c0, e0, t0, n, ev;
        if (len >= 8) {
            src = pay[0];
            ms  = (uint32_t)pay[4] | ((uint32_t)pay[5] << 8)
                | ((uint32_t)pay[6] << 16) | ((uint32_t)pay[7] << 24);
        } else if (len >= 4) {
            ms  = (uint32_t)pay[0] | ((uint32_t)pay[1] << 8)
                | ((uint32_t)pay[2] << 16) | ((uint32_t)pay[3] << 24);
        }
        if (ms > 5000u) ms = 5000u;

        c0 = g_irq_count;
        e0 = (src == 1u) ? g_gpio_evt : g_timer_evt;

        if (src == 0u || src == 2u) msip_irq_window(1);
        if (src == 1u || src == 2u) gpio_irq_window(1);

        t0 = GPIOCMD_MTIME_LO;
        while ((uint32_t)(GPIOCMD_MTIME_LO - t0) < (ms * 54054u)) {
            /* 空转等中断 */
        }

        if (src != 1u) msip_irq_window(0);
        if (src != 0u) gpio_irq_window(0);

        n  = g_irq_count - c0;
        ev = ((src == 1u) ? g_gpio_evt : g_timer_evt) - e0;

        rsp_head(cmd, STATUS_OK, 16);
        for (i = 0; i < 4u; i++) {
            uint32_t v = (i == 0) ? n
                       : (i == 1) ? g_irq_last_cause
                       : (i == 2) ? g_irq_last_pend
                                  : ev;
            rsp_byte((uint8_t)( v        & 0xFFu));
            rsp_byte((uint8_t)((v >>  8) & 0xFFu));
            rsp_byte((uint8_t)((v >> 16) & 0xFFu));
            rsp_byte((uint8_t)((v >> 24) & 0xFFu));
        }
        break;
    }

    case CMD_SET_SETTLE: {
        /* 0x19: {u32 cyc} -> y3t_auto 撤 arm 后落盘等待拍数 (默认 1,080,000=20ms)。
         *   钳制 [108000 (2ms), 5405400 (100ms)]。回 4B: 生效值。
         *   用途: 自动循环在线扫参 —— 逐步调小, det 变差/框乱即回退一档。 */
        extern uint32_t g_settle_cyc;
        uint32_t cyc;
        if (len < 4) { st = STATUS_ERR_PARAM; break; }
        cyc = (uint32_t)pay[0] | ((uint32_t)pay[1]<<8)
            | ((uint32_t)pay[2]<<16) | ((uint32_t)pay[3]<<24);
        if (cyc < 108000u)   cyc = 108000u;
        if (cyc > 5405400u)  cyc = 5405400u;
        g_settle_cyc = cyc;
        rsp_head(cmd, STATUS_OK, 4);
        rsp_byte((uint8_t)(cyc      )); rsp_byte((uint8_t)(cyc >>  8));
        rsp_byte((uint8_t)((cyc >> 16) & 0xFFu)); rsp_byte((uint8_t)(cyc >> 24));
        break;
    }

    case CMD_AUTO_DUMP: {
        /* 0x1A: 无载荷 -> 29B 内部状态 (卡死诊断, 见 cmd_proto.h) */
        extern uint32_t g_auto_en, g_settle_cyc, s_wait, s_same, s_wd_cnt;
        extern uint8_t  s_last_fc;
        uint32_t vals[7] = {
            (*(volatile uint32_t *)0x40004000u),   /* PP_CSR */
            g_auto_en,
            (uint32_t)s_last_fc,
            s_wait, s_same, g_settle_cyc,
            GPIOCMD_MTIME_LO
        };
        rsp_head(cmd, STATUS_OK, 29);
        rsp_byte((uint8_t)s_wd_cnt);
        for (i = 0; i < 7; i++) {
            rsp_byte((uint8_t)(vals[i]       & 0xFFu));
            rsp_byte((uint8_t)((vals[i]>>8 ) & 0xFFu));
            rsp_byte((uint8_t)((vals[i]>>16) & 0xFFu));
            rsp_byte((uint8_t)((vals[i]>>24) & 0xFFu));
        }
        break;
    }

    default:
        st = STATUS_ERR_PARAM;
        break;
    }
    /* 收尾补头: 只在【case 里没自己发过】时才补, 否则会发出两个响应帧。
     * ZIGUANG 修复 (2026-10-18): 原来用 `cmd != GET_FRAME && cmd != PING` 判断,
     *   但 MEM_READ/LOAD_IMG/EXEC/DDR2TCM/Y3T_JOB/WLOAD_POLL/GET_INFO 都在
     *   case 内发过 rsp_head ⇒ 每次都多发一个 `5A cmd st 00 00` 的空响应,
     *   上位机下一次收帧会撞上这个残留头, 表现为"命令偶发无响应/数据错位"。 */
    if (!g_hdr_sent) {
        rsp_head(cmd, st, 0);
    }
}

/* ---- 非阻塞轮询: 在主循环中调用 ---- */
/* ---- ZIGUANG v16: 命令通道自检 (给 demo 的 [10] 用) ---------------------
 * 直接调用 cmd_exec(), 走【真实的 0x0C/0x0D 解析 + arena 校验 + 写入 + fence.i 跳转】,
 * 而不是绕过命令层直接写内存 —— 这样仿真/板级都能覆盖新加的协议代码。
 * 返回 0 = PASS, 非 0 = 失败位图。
 *   bit0: arena 校验逻辑不对 (允许了代码区 / 拒绝了 arena)
 *   bit1: 0x0C 写入后回读不符
 *   bit2: 0x0D 跳转执行后便签不是 42
 */
int uart_cmd_selftest(void)
{
    static const uint32_t blob[4] = {
        0x02A00513u,   /* addi a0, x0, 42  */
        0x100055B7u,   /* lui  a1, 0x10005 */
        0x00A5A023u,   /* sw   a0, 0(a1)   */
        0x00008067u    /* ret              */
    };
    uint8_t pl[8 + 16];
    uint8_t pe[4];
    uint32_t k, v;
    int bad = 0;

    /* --- ① arena 校验逻辑本身 ---
     * ZIGUANG (2026-10-18 19:25): 全部改用 ARENA_BOOT_LO 常量。
     *   原来 0x4000 是硬编码 —— 一旦 arena 起点改动 (本次 0x4000→0x8000),
     *   这几条自检就全变成"测旧常量", 表面 PASS 实则什么都没验到。
     *   另: 原 0x4001 那条既越界又未对齐, 分不清是哪个条件拒的;
     *   改 ARENA_BOOT_LO+1 才是纯粹的"未对齐"用例。 */
    if (arena_ok(ARENA_BOOT_LO - 0x1000u, 4u)) bad |= 1;   /* 固件代码区: 必须拒 */
    if (arena_ok(0x10000000u, 4u))             bad |= 1;   /* MAIN 低区(栈): 必须拒 */
    if (arena_ok(ARENA_BOOT_LO + 1u, 4u))      bad |= 1;   /* 未对齐: 必须拒 */
    if (arena_ok(ARENA_BOOT_HI, 4u))           bad |= 1;   /* 窗口上界: 必须拒 */
    if (!arena_ok(ARENA_BOOT_LO, 4u))          bad |= 1;   /* arena 起点: 必须允许 */
    if (!arena_ok(ARENA_BOOT_HI - 4u, 4u))     bad |= 1;   /* 窗口末字: 必须允许 */
    if (bad) return bad;

    /* --- ② 0x0C LOAD_IMG: {addr, nwords, words...} --- */
    *(volatile uint32_t *)0x10005000u = 0u;
    pl[0]=(uint8_t)(ARENA_BOOT_LO      ); pl[1]=(uint8_t)(ARENA_BOOT_LO >>  8);
    pl[2]=(uint8_t)(ARENA_BOOT_LO >> 16); pl[3]=(uint8_t)(ARENA_BOOT_LO >> 24);
    pl[4]=0x04; pl[5]=0x00; pl[6]=0x00; pl[7]=0x00;      /* nwords= 4      */
    for (k = 0; k < 4u; k++) {
        pl[8+k*4+0] = (uint8_t)(blob[k]);
        pl[8+k*4+1] = (uint8_t)(blob[k] >> 8);
        pl[8+k*4+2] = (uint8_t)(blob[k] >> 16);
        pl[8+k*4+3] = (uint8_t)(blob[k] >> 24);
    }
    rsp_quiet = 1;
    cmd_exec(CMD_LOAD_IMG, pl, 8u + 16u);
    rsp_quiet = 0;
    for (k = 0; k < 4u; k++) {                            /* 回读校验 */
        v = *(volatile uint32_t *)(ARENA_BOOT_LO + k*4u);
        if (v != blob[k]) bad |= 2;
    }
    if (bad) return bad;

    /* --- ③ 0x0D EXEC: {entry} -> fence.i + 跳转 --- */
    pe[0]=(uint8_t)(ARENA_BOOT_LO      ); pe[1]=(uint8_t)(ARENA_BOOT_LO >>  8);
    pe[2]=(uint8_t)(ARENA_BOOT_LO >> 16); pe[3]=(uint8_t)(ARENA_BOOT_LO >> 24);
    rsp_quiet = 1;
    cmd_exec(CMD_EXEC, pe, 4u);
    rsp_quiet = 0;
    if (*(volatile uint32_t *)0x10005000u != 42u) bad |= 4;

    /* --- ④ 0x0E DDR2TCM: 先把镜像写进 DDR 暂存区, 再搬进 arena 执行 --- */
    {
        static uint32_t blob8[8];        /* 一个 256bit 块 = 32B = 8 字 */
        uint8_t pd[12];
        for (k = 0; k < 8u; k++) blob8[k] = (k < 4u) ? blob[k] : 0u;
        /* ZIGUANG 修正 (2026-10-18, v19): 测试块 50000 改 480000。
         *   原来注释说"50000 落在 cam0 slot0 之后"——错的。实际帧缓冲:
         *     cam0 槽 0..172800 块, cam1 槽 230400..460800 块, 每槽 57600 块
         *     (1280×720×2B = 1,843,200B / 32B = 57,600), 合计占 0..460,800 块。
         *   50000 正好在 cam0 slot0 (0..57,599) 里面, 相机 30fps 持续往里写像素,
         *   "写完→回读"之间被覆盖 ⇒ 板上 [10] 间歇 FAIL(bad=00000010), 仿真(无相机)却通过。
         *   仍不能选 480000 —— 那是【整网权重区】起始块, 万一软复位重跑 demo_main,
         *   会把 L0 前 16 个权重块清零+写图案(静默毁掉第一层)。故用 620000(19.84MB), 
         *   与帧缓冲(0..460800)、权重区(480000..574912)、下装暂存(600000) 全不重叠。 */
        if (ddrb_write(620000u, blob8, 1u) != 0) return 8;
        *(volatile uint32_t *)0x10005000u = 0u;
        for (k = 0; k < 4u; k++) *(volatile uint32_t *)(ARENA_BOOT_LO + k*4u) = 0u;
        /* pd = {blk=620000, dst=ARENA_BOOT_LO, bytes=16}; 620000 = 0x000975E0 */
        pd[0]=0xE0; pd[1]=0x75; pd[2]=0x09; pd[3]=0x00;
        pd[4]=(uint8_t)(ARENA_BOOT_LO      ); pd[5]=(uint8_t)(ARENA_BOOT_LO >>  8);
        pd[6]=(uint8_t)(ARENA_BOOT_LO >> 16); pd[7]=(uint8_t)(ARENA_BOOT_LO >> 24);
        pd[8]=0x10; pd[9]=0x00; pd[10]=0x00; pd[11]=0x00;
        rsp_quiet = 1;
        cmd_exec(CMD_DDR2TCM, pd, 12u);
        rsp_quiet = 0;
        for (k = 0; k < 4u; k++)
            if (*(volatile uint32_t *)(ARENA_BOOT_LO + k*4u) != blob[k]) bad |= 8;
        if (bad) return bad;
        rsp_quiet = 1;
        cmd_exec(CMD_EXEC, pe, 4u);
        rsp_quiet = 0;
        if (*(volatile uint32_t *)0x10005000u != 42u) bad |= 8;
    }

    /* --- ⑤ 0x0B WLOAD_RAW: 一条命令写 8 个 256bit 块 + 回读比对 (v17) ---
     * 为什么要这条: 原来 0x0B 固定只写 1 块(32B), 而 PC 侧按 256B/条发 =>
     *   每 8 块只写 1 块, 权重 87.5% 是残留值 —— 整网推理会算出垃圾。
     * 手法: 先把 DDR 块 480010 起 8 块清零, 再用一条 0x0B 写 256B, 再逐字回读。
     *   清零是为了排除"残留值碰巧相等"的假通过。
     *   ZIGUANG 修正 (2026-10-18, v19): 测试块 50010 → 620010, 理由同上 ——
     *   50010 在 cam0 slot0(0..57,599) 内, 会被相机写通路持续覆盖。
     *   失败原因编码: 返回值 bit4 = 本段失败, 高 8 位 = 子码
     *     (1=清零写超时 2=回读超时 3=数据失配), 便于一次定位到底哪步坏。 */
    {
        static uint8_t  pl[6 + 256];
        static uint8_t  pat[256];
        static uint32_t rd[64];
        static uint32_t z64[64];
        uint32_t k2, b2 = 0;
        const uint32_t TBLK = 620010u;

        for (k2 = 0; k2 < 64u; k2++) z64[k2] = 0u;
        if (ddrb_write(TBLK, z64, 8u) != 0) b2 = 1u;        /* 清零 8 块 */

        for (k2 = 0; k2 < 256u; k2++) pat[k2] = (uint8_t)(k2 * 7u + 3u);
        pl[0] = (uint8_t)(TBLK & 0xFFu); pl[1] = (uint8_t)((TBLK >> 8) & 0xFFu);
        pl[2] = (uint8_t)((TBLK >> 16) & 0xFFu); pl[3] = 0u;
        pl[4] = 0x00u; pl[5] = 0x01u;                        /* len = 256 */
        for (k2 = 0; k2 < 256u; k2++) pl[6u + k2] = pat[k2];

        rsp_quiet = 1;                                      /* 别把二进制响应打进日志 */
        cmd_exec(CMD_WLOAD_RAW, pl, (uint16_t)(6u + 256u));
        rsp_quiet = 0;

        if (!b2) {
            if (ddrb_read(TBLK, rd, 8u) != 0) b2 = 2u;
            else {
                for (k2 = 0; k2 < 64u; k2++) {
                    uint32_t exp = (uint32_t)pat[k2*4+0]
                                 | ((uint32_t)pat[k2*4+1] << 8)
                                 | ((uint32_t)pat[k2*4+2] << 16)
                                 | ((uint32_t)pat[k2*4+3] << 24);
                    if (rd[k2] != exp) { b2 = 3u; break; }
                }
            }
        }
        if (b2) bad |= 16 | (int)(b2 << 8);                 /* bit4=失败, 高 8 位=子码 */
    }

    /* --- ⑥ 0x0B 大包: 一条写 64 块 (2048B) + 逐字回读 (2026-10-18 21:30) ---
     * 为什么必须要这条: 大包路径把 len 上限从 500 提到 4096, 并且 DDR 提交从
     *   "一次 <=15 块"改成"按 16 块分批循环"。这两处任一写错, 现场表现都是
     *   "上传完了但整网算错", 极难定位 —— 所以让 [10] 在板上把它跑一遍。
     *   64 块 = 4 批 x 16 块, 正好覆盖分批循环的边界; 载荷 2054B > 512,
     *   小缓冲路径根本收不下, 能过就说明 4KB 缓冲与 LEN 上限都生效了。
     * 子码编码: bit5 = 本段失败, 高 8 位 += 4 (清零) / 5 (回读) / 6 (失配) */
    {
        const uint32_t TBLK2 = 621000u;                      /* 与 620010 及权重区都不重叠 */
        const uint32_t NB2   = 64u;
        static uint32_t z512[64 * 8];                        /* 清零用: 64 块 x 8 字 */
        uint32_t k2, b3 = 0;
        uint16_t n2 = (uint16_t)(6u + NB2 * 32u);

        for (k2 = 0; k2 < NB2 * 8u; k2++) z512[k2] = 0u;
        if (ddrb_write(TBLK2, z512, NB2) != 0) b3 = 4u;      /* 只用来排除"残留碰巧相等" */

        /* 载荷直接就地构造: 每个字节 = (块内偏移 * 7 + 3) ^ 块号, 逐字可验 */
        {
            uint32_t blk, by;
            uint8_t *pb = RXB;
            pb[0] = (uint8_t)(TBLK2 & 0xFFu); pb[1] = (uint8_t)((TBLK2 >> 8) & 0xFFu);
            pb[2] = (uint8_t)((TBLK2 >> 16) & 0xFFu); pb[3] = (uint8_t)(TBLK2 >> 24);
            pb[4] = (uint8_t)((NB2 * 32u) & 0xFFu);
            pb[5] = (uint8_t)(((NB2 * 32u) >> 8) & 0xFFu);
            for (blk = 0; blk < NB2; blk++)
                for (by = 0; by < 32u; by++)
                    pb[6u + blk * 32u + by] = (uint8_t)((by * 7u + 3u) ^ (blk & 0xFFu));
        }
        rsp_quiet = 1;
        cmd_exec(CMD_WLOAD_RAW, RXB, n2);                     /* 注意: 载荷在 RXB, 不是小缓冲 */
        rsp_quiet = 0;

        if (!b3) {
            /* 分批回读: 一次 16 块 = 128 字, 正好对上 ddrb_read 的突发上限 */
            for (k2 = 0; k2 < NB2; k2 += 16u) {
                uint32_t w128[128], q;
                if (ddrb_read(TBLK2 + k2, w128, 16u) != 0) { b3 = 5u; break; }
                for (q = 0; q < 128u; q++) {
                    uint32_t by = q & 7u, blk = k2 + (q >> 3);
                    uint32_t exp = 0u, r;
                    for (r = 0; r < 4u; r++) {
                        uint32_t off = (by * 4u + r) & 31u;
                        exp |= (uint32_t)(uint8_t)((off * 7u + 3u) ^ (blk & 0xFFu)) << (r * 8u);
                    }
                    if (w128[q] != exp) { b3 = 6u; break; }
                }
                if (b3) break;
            }
        }
        if (b3) bad |= 32 | (int)(b3 << 8);                   /* bit5=失败, 高 8 位=子码 */
    }
    return bad;
}

/* 单个字节喂给解析状态机 (原 uart_cmd_poll 的主体, 拆出来供批量调用) */
static void rx_feed(uint8_t b) {
    last_rx_tick = REG32(0x40002000u);

    switch (rx_state) {
    case RX_SYNC:
        if (b == CMD_SYNC_REQ) rx_state = RX_CMD;
        break;
    case RX_CMD:
        rx_cmd = b; rx_cksum = b; rx_state = RX_LEN0;
        /* 只有 0x0B/0x11 走大包缓冲, 其余仍限 500B。
         * 为什么要提前决定: 4KB 缓冲在 BOOT 的 .uart_rx, 而小命令的载荷理论上
         * 可以更大 —— 在 LEN 阶段就按命令选容量, 超了直接丢帧回 SYNC, 不越界。 */
        rx_cap = (b == CMD_WLOAD_RAW || b == CMD_W3T) ? RFC_BULK : RFC_SMALL;
        break;
    case RX_LEN0:
        rx_len_lo = b; rx_cksum ^= b; rx_state = RX_LEN1;
        break;
    case RX_LEN1:
        rx_len_hi = b; rx_cksum ^= b;
        rx_len = (uint16_t)rx_len_lo | ((uint16_t)rx_len_hi << 8);
        if (rx_len > rx_cap) { rx_state = RX_SYNC; break; }
        rx_pos = 0;
        rx_state = rx_len ? RX_PAYLOAD : RX_CKSUM;
        if (rx_state == RX_CKSUM) rx_state = RX_PAYLOAD;  /* len=0 也进 payload 流程 */
        if (rx_len == 0) { rx_state = RX_SYNC; cmd_exec(rx_cmd, RXB, 0); break; }
        break;
    case RX_PAYLOAD:
        RXB[rx_pos++] = b;
        rx_cksum ^= b;
        if (rx_pos >= rx_len) rx_state = RX_CKSUM;
        break;
    case RX_CKSUM:
        if (b == rx_cksum) cmd_exec(rx_cmd, RXB, rx_len);
        rx_state = RX_SYNC;
        break;
    }
}

void uart_cmd_poll(void) {
    /* ZIGUANG (2026-10-18): 溢出后自愈。
     * FIFO 已加深到 512 并带满保护, 理论上不会溢出; 但一旦发生(PC 突刺/新命令),
     * 必须把解析器拉回同步, 否则会永久错位 —— 板上出现过"发过长帧后连 PING 都不响应"。 */
    if (UART_STATUS & 0x4u) {
        rx_state = RX_SYNC;
        REG32(0x40000010u) = 0x4u;        /* W1C: 清 st_rx_overrun */
    }
    /* 批量排空 FIFO (每次最多 64B, 66us @115200)。
     * 为什么不再"每轮只取 1 字节": 那样要求主循环周期 <87us 才跟得上线速,
     *   而相机帧差分支里一次 ddrb_read 就要几百 us~ms ⇒ FIFO 必满 ⇒ 丢字节。
     *   一次取干能把"主循环慢"的影响从"必丢"降成"只要 FIFO 没满就不丢"。 */
    {
        uint8_t buf[64];
        uint32_t n = uart_getc_burst(buf, (uint32_t)sizeof buf);
        if (n == 0) {
            /* ZIGUANG (2026-10-18 v23): 帧内超时自愈。
             * 板上现象: 一条 256B 的 0x0B 有时收到一半就断(PC 侧看是"无响应"),
             *   之后解析器若停在 RX_PAYLOAD 等一个永远不来的字节, 长帧会**连续失败**,
             *   直到下一次重启。这里用 mtime 做 50ms 帧内超时: 只要半途断流就把状态机
             *   拉回 RX_SYNC, 保证一条坏帧不会"毒死"后续所有长帧。 */
            if (rx_state != RX_SYNC) {
                uint32_t now = REG32(0x40002000u);        /* mtime 低 32 位 */
                if (now - last_rx_tick > 2700000u) {      /* 50ms @54MHz */
                    rx_state = RX_SYNC;
                }
            }
            return;
        }
        for (uint32_t k = 0; k < n; k++) rx_feed(buf[k]);
    }
}
