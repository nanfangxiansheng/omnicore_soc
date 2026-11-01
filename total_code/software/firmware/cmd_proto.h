/* ============================================================================
 * cmd_proto.h — UART 命令协议 (PC ↔ OmniCore)
 *
 * 物理层: UART 115200-8-N-1
 * 帧格式: [0xA5] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...] [CKSUM]
 *   CKSUM = CMD ^ LEN_LO ^ LEN_HI ^ (PAYLOAD 字节异或和)
 * 响应: [0x5A] [CMD|0x80] [STATUS] [LEN_LO] [LEN_HI] [PAYLOAD...] [CKSUM]
 *   STATUS: 0=OK, 1=BUSY, 2=ERR_PARAM, 3=ERR_TIMEOUT
 *
 * 命令集:
 *   0x01 PING        — 无负载, 回 "PONG"
 *   0x02 MEM_READ    — 载荷 {u32 addr, u16 len} → 回 len 字节
 *   0x03 MEM_WRITE   — 载荷 {u32 addr, u16 len, data...} → 回 OK
 *   0x04 Y3T_JOB     — 载荷 {y3t_job_params_t} → 完成后回结果
 *   0x05 WLOAD_START — 载荷 {u32 src_blk, u32 dst_word, u16 words} → 启动权重 DMA
 *   0x06 WLOAD_POLL  — 无负载 → 回 {done, busy}
 *   0x07 OVL_BOX     — 载荷 {u8 n, u16 xmin, u16 ymin, u16 xmax, u16 ymax, u16 color}
 *   0x08 OVL_EN      — 载荷 {u8 mask}
 *   0x09 GET_FRAME   — 载荷 {u32 blk, u16 blocks} → 回帧数据 (经 DDR 桥)
 *   0x0A GET_INFO    — 无载荷 → 回 {frame_cnt, detect_cnt, y3t_id, ver}
 * ========================================================================== */
#ifndef CMD_PROTO_H
#define CMD_PROTO_H

#include <stdint.h>

#define CMD_SYNC_REQ  0xA5u
#define CMD_SYNC_RSP  0x5Au

#define CMD_PING        0x01u
#define CMD_MEM_READ    0x02u
#define CMD_MEM_WRITE   0x03u
#define CMD_Y3T_JOB     0x04u
#define CMD_WLOAD_START 0x05u
#define CMD_WLOAD_POLL  0x06u
#define CMD_OVL_BOX     0x07u
#define CMD_OVL_EN      0x08u
#define CMD_GET_FRAME   0x09u
#define CMD_GET_INFO    0x0Au

/* 0x0B WLOAD_RAW — 上位机 tools/upload_weights.py 在用: 载荷 {u32 blk, u16 len, data...},
 *   data 恰好 1 个 256bit 块 (CHUNK=256B) -> 固件写 DDR 块 blk。原缺此 handler, 权重传不上去。 */
#define CMD_WLOAD_RAW   0x0Bu

/* 0x0C LOAD_IMG — 【在线下装】载荷 {u32 addr, u32 nwords, u32 word[n]}, 回 {u32 done}
 *   addr 必须 4 字节对齐, 且只能落在 arena 区 (见 uart_cmd.c 的 arena_ok):
 *     BOOT arena 0x0000_8000..0x0000_FFFF (32 KB)
 *     MAIN arena 0x1000_5000..0x1000_7FFF (12 KB)
 *   限制在 arena 是【物理保护】: 写到别处会覆盖正在执行的代码, 直接写死。
 *   ⚠ 2026-10-18: BOOT arena 起点由 0x4000 抬到 0x8000 —— v24 的 .text/.rodata
 *     已长到 0x489B, 从 0x4000 起会把 poff.4(池化偏移) 和 g_net(整网 job 表)踩掉。
 *     详见 uart_cmd.c 的 "arena 窗口被固件自己吃掉" 注释。
 *   nwords 上限 120 (受 rx_buf[512] 限制: 8 + 120*4 = 488 <= 512)。 */
#define CMD_LOAD_IMG    0x0Cu

/* 0x0D EXEC — 载荷 {u32 entry}, 回 {u32 'GO!!'} 后 fence.i 并跳转执行。
 *   entry 同样必须落在 arena 区; 被执行代码 ret 后自然回到命令循环。 */
#define CMD_EXEC        0x0Du

/* 0x0E DDR2TCM — {u32 ddr_blk, u32 tcm_addr, u32 bytes}, 回 {u32 done}
 *   把 staging 在 DDR 里的镜像搬进 TCM arena (512B/块, 走 ddrb_read 16 拍突发)。
 *   为什么要有它: ① PC 可以先 0x0B 把镜像写进 DDR 并回读校验, 再一次性提交进 TCM;
 *   ② DDR 可暂存比 arena 更大的东西, 分多次搬; ③ 搬完统一 fence.i, 避免边收边写。
 *   约束: tcm_addr 必须落在 arena (见 arena_ok), bytes 必须 4 的倍数, 且 <= arena 余量。 */
#define CMD_DDR2TCM     0x0Eu

/* 0x10 NET_RUN — 整网推理一次 (固件内静态 job 表 + 逐层权重流式, 见 sw/y3t_exec.c)
 *   载荷 {u32 ddr_w_base_blk, u32 rom_blk}
 *     ddr_w_base_blk : engine_weights.bin 在 DDR 的基块 (常规 480000)
 *     rom_blk        : 尺度/偏置镜像基块; 0 = 用 identity(直通) 尺度, 只验结构不验数值
 *   回 24 字节: {i32 ret, u32 jobs, u32 failed, u32 wload_cyc, u32 mac_cyc, u32 total_cyc}
 *   ret: 0=全过, -1=表不能等分, -2=引擎报错, -3=超时
 *   注意: 权重必须先经 0x0B 上传到 DDR, 否则引擎读到的是上电残留。 */
#define CMD_NET_RUN     0x10u
/* 0x12: 固件自解码检测头, 回 {u16 n}+n*12B。与 PC 侧 tools/det_read.py 双路对照用 */
#define CMD_DET_READ    0x12u
/* 0x14: overlay 开关掩码 (1B): bit0=四角自检 bit1=运动框 bit2=YOLO框 */
#define CMD_OVL_MODE    0x14u
#define NET_RUN_RSP_LEN 24u

/* 0x15 AUTO_RUN (v37, 2026-10-23) —— 板上自动循环开关
 *   载荷 {u32 en, u32 wblk, u32 romblk}
 *     en=1 : 主循环进入自动模式（相机帧→PP 预处理→整网→解码→OVL 画框）
 *     en=0 : 退出自动模式（回到运动检测分支）
 *     wblk / romblk : 与 0x10 相同（权重 / 尺度在 DDR 的基块）
 *   前提与 0x10 一致: DDR 里必须先有整网权重与尺度镜像。
 *   回 4 字节: {u32 en}（回显当前状态, 供 PC 确认） */
#define CMD_AUTO_RUN    0x15u

/* 0x16 SET_THR (v37) —— 运行时改检测置信度阈值
 *   载荷 {i32 thr_q_scale0, i32 thr_q_scale1} (原始分数阈值, 每尺度一个)
 *   值 -32768 = 恢复表内默认 (conf_thr=0.30)。
 *   conf=c 的换算: thr_q = ln(c/(1-c)) / s_out，PC 侧 tools/det_thr.py --conf 0.5 会算好。
 *   回 8 字节: 回显两个生效值。 */
#define CMD_SET_THR     0x16u

/* 0x17 GPIO (v65) —— GPIO 外设直接读写 (赛题"外设系统集成"实证)
 *   载荷 {u8 op, u8 val}
 *     op=0 读回 8 个寄存器 (32B): DATA_IN/DATA_OUT/DIR/SET/CLR/RISE/FALL/STATUS
 *     op=1 DATA_OUT = val        op=2 DIR = val
 *     op=3 SET = val             op=4 CLR = val
 *     op=5 IRQ_RISE_EN = val     op=6 IRQ_STATUS W1C = val (回清前值)
 *   回 32B (op=0/6) 或 4B (其它, 回写后的 DATA_OUT)
 *   地址 0x4000_1000, 位定义见 sw/gpio_ctl.h。 */
#define CMD_GPIO        0x17u

/* 0x18 GPIO_IRQ (v65) —— GPIO 边沿中断闭环验证 (赛题高阶②"完整异常/中断机制")
 *   载荷 {u32 ms}: 开一个 ms 毫秒的中断窗口, 只放行 GPIO 源 (EXTIRQ.ENABLE=bit1),
 *   期间 gpio_in[0] 每来一个上升沿就进一次 ISR (mcause=0x8000_0001)。
 *   窗口结束自动关闭 (mstatus.MIE=0, mie&=~MEIE, EXTIRQ.ENABLE=0)。
 *   回 16B: {u32 进入次数, u32 最近 mcause, u32 最近 EXTIRQ.PENDING, u32 GPIO 事件数}
 *   ★ 用窗口而不是常开: cam_frame / y3t_done / uart 三个源会常挂, 常开会把主循环淹掉。 */
#define CMD_GPIO_IRQ    0x18u

/* 0x19 SET_SETTLE (v66) —— 自动循环"撤 arm 后落盘等待"运行时可调
 *   载荷 {u32 cyc}: y3t_auto 撤 arm 后等 PP 在途写入落盘的拍数 (默认 1,080,000 = 20ms)。
 *   目的: 出片后在线扫参找最小安全值, 把自动循环从 10fps 往 12fps 压 (不重出片)。
 *   范围钳制 [108000 (2ms), 5405400 (100ms)]。回 4B: 生效的 cyc。 */
#define CMD_SET_SETTLE  0x19u

/* 0x1A AUTO_DUMP (v67) —— 自动循环内部状态转储 (卡死诊断)
 *   无载荷。回 29B: {u8 s_wd_cnt, u32 PP_CSR, u32 g_auto_en, u8 s_last_fc,
 *                    u32 s_wait, u32 s_same, u32 g_settle_cyc, u32 MTIME_LO}。 */
#define CMD_AUTO_DUMP   0x1Au

/* 0x11 W3T — 流式权重装载 (2026-10-18 21:10 新增, 上传提速主通道)
 *   载荷 {u32 wb_word, u32 nwords}
 *     wb_word : 这批权重在 y3tram 的目标 **256bit 字地址** (绝对, 见 engine_layout.json)
 *     nwords  : 本批字数 (<= 65536, 但建议 <= 2048, 受 DDR scratch 容量限制)
 *   数据来源: PC 先用大包 0x0B 把这一批写进 DDR 块 W3T_SRC_BLK, 再发 0x11 触发
 *             wload_dma (CMD 直通: 同时写 y3tram 与 DDR)。
 *   回 8 字节: {u32 ok_words, u32 err}
 *   为什么不做成"0x0B 直接进 y3tram": 0x0B 的载荷在 DDR 块号空间里可能不连续
 *   (PC 侧按 4KB 大包切), 而 wload_dma 的 CMD 直通要求入口是**连续 DDR 段**,
 *   所以中间要一次连续暂存 —— 暂存在 DDR 擦写零成本, 且能顺带做完整性校验。 */
#define CMD_W3T         0x11u
#define CMD_W3T_SRC_BLK 620000u

/* ---- 0x04 Y3T_JOB 载荷: 17 x u32 (小端), 与 y3t.h 的 y3t_job() 参数一一对应 ----
 * 注意: 不要在固件里对该载荷做 4 字节指针对齐读取 (RV32I 非对齐访问会触发陷阱);
 *       固件按字节拼装, PC 侧可直接 memcpy 本结构体 (RV32 小端, 无填充)。 */
typedef struct {
    uint32_t in0, in1, wb, out;      /* y3tram 256b 块基址 (输入乒乓面/权重/输出) */
    uint32_t W, H, Wo, Ho;           /* 输入/输出 宽高 (各 10bit) */
    uint32_t k, s, pad, mode;        /* 卷积核/步长/填充/模式 */
    uint32_t cit, cot, act, bits16;  /* 输入通道/输出通道/激活/16bit 标志 */
    uint32_t timeout_loops;          /* 完成轮询上限 */
} y3t_job_params_t;

/* 0x04 响应载荷 (12 字节): {int32 ret, u32 cyc_mac, u32 cyc_tot}
 *   ret: 0=ok, -1=超时, -2=引擎报错 —— 用 golden 比对时以 cyc_mac/cyc_tot 佐证真的跑了卷积 */
#define Y3T_JOB_RSP_LEN  12u

#define STATUS_OK        0u
#define STATUS_BUSY      1u
#define STATUS_ERR_PARAM 2u
#define STATUS_ERR_TMO   3u

#endif
