/* ============================================================================
 * gpio_ctl.h — GPIO 外设使用场景 (v65, 2026-10-27)
 *
 * 背景: SoC 里早有 ominicore_gpio (0x4000_1000, 8 个寄存器), 顶层也有
 *   gpio_in[15:0] / gpio_out[15:0] 端口, 但**固件从未真正用过它**
 *   (只有 demo.c 里一行把 bit8 当心跳翻转过)。赛题"外设系统集成 (UART/GPIO)"
 *   这一项因此是空的。本文件给 GPIO 定一个真实用途:
 *
 * ---------------------------- 场景: AI 结果的对外接口 ----------------------------
 *   输出 (gpio_out) —— 把识别结果直接送给外部执行器 (机械臂/分拣机构/指示灯):
 *     [3:0]  类别码   0=无 1=cup 2=bottle 3=mouse 4=toy 5=screen  -> LED3..LED6
 *     [4]    det_valid  本帧有目标                              -> LED7
 *     [5]    帧心跳     每处理完一帧翻转一次 (外部可判断系统在跑)
 *     [6]    ISR 指示   进中断服务时拉高, 退出时拉低 (示波器可测中断延迟)
 *     [15:7] 保留
 *   板上实际引出 (《MES2L676-200HP 硬件使用手册 v1.2》§3.4):
 *     LED3=E18  LED4=A17  LED5=A18  LED6=C17  LED7=B17   (高电平亮)
 *     KEY1=C23  KEY2=B22  (低有效: 未按=高, 按下=低)
 *   输入 (gpio_in):
 *     [0]    外部触发   KEY1 (C23)。按键低有效 ⇒ 按下是**下降沿**, 用下降沿中断
 *     [1]    阈值模式   KEY2 (B22)。0 = 严 (0.25), 1 = 宽 (0.15)
 *     [15:2] 保留
 *
 * ---------------------------- 中断链路 (赛题高阶②实证) ----------------------------
 *   GPIO 边沿检测 -> gpio_irq -> ominicore_extirq bit1 -> core mip.meip
 *     -> mcause = 0x8000_0001 -> trap_entry -> trap_dispatch -> ISR
 *     -> 读 EXTIRQ.PENDING 确认源 -> 写 GPIO.IRQ_STATUS (W1C) 清源 -> mret
 *   注意: boot.S 默认 `csrw mie, zero` 且清 mstatus.MIE, 即**全局中断默认关闭**;
 *     所以本 SoC 的异常(ecall/未对齐)一直在用, 而中断从未被触发过。
 *     0x18 命令按 "开 MEIE -> 等 -> 关 MEIE" 的窗口式做法, 用完立刻关闭,
 *     不会让 cam_frame/y3t_done/uart 这些常挂的源淹掉主循环。
 * ========================================================================== */
#ifndef GPIO_CTL_H
#define GPIO_CTL_H

#include <stdint.h>

/* ---- 寄存器映射 (规格 §11.2) ---- */
#define GPIO_BASE        0x40001000u
#define GPIO_DATA_IN     (*(volatile uint32_t *)(GPIO_BASE + 0x00u))  /* RO  同步后输入  */
#define GPIO_DATA_OUT    (*(volatile uint32_t *)(GPIO_BASE + 0x04u))  /* RW  输出锁存    */
#define GPIO_DIR         (*(volatile uint32_t *)(GPIO_BASE + 0x08u))  /* RW  1=输出      */
#define GPIO_SET         (*(volatile uint32_t *)(GPIO_BASE + 0x0Cu))  /* WO  置位        */
#define GPIO_CLR         (*(volatile uint32_t *)(GPIO_BASE + 0x10u))  /* WO  清位        */
#define GPIO_IRQ_RISE    (*(volatile uint32_t *)(GPIO_BASE + 0x14u))  /* RW  上升沿使能  */
#define GPIO_IRQ_FALL    (*(volatile uint32_t *)(GPIO_BASE + 0x18u))  /* RW  下降沿使能  */
#define GPIO_IRQ_STATUS  (*(volatile uint32_t *)(GPIO_BASE + 0x1Cu))  /* RO/W1C 边沿挂起 */

/* ---- 外部中断控制器 (规格 §11.4) ---- */
#define EXTIRQ_BASE      0x40003000u
#define EXTIRQ_PENDING   (*(volatile uint32_t *)(EXTIRQ_BASE + 0x00u)) /* RO   bit0=uart bit1=gpio bit2=y3t bit3=cam */
#define EXTIRQ_ENABLE    (*(volatile uint32_t *)(EXTIRQ_BASE + 0x04u)) /* RW */
#define EXTIRQ_ACTIVE    (*(volatile uint32_t *)(EXTIRQ_BASE + 0x08u)) /* RO */

/* ---- Timer (规格 §11.3, 0x4000_2000) —— 确定性中断源 ---- */
#define TIMER_MTIME_LO      (*(volatile uint32_t *)0x40002000u)
#define TIMER_MTIME_HI      (*(volatile uint32_t *)0x40002004u)
#define TIMER_MTIMECMP_LO   (*(volatile uint32_t *)0x40002008u)
#define TIMER_MTIMECMP_HI   (*(volatile uint32_t *)0x4000200Cu)
#define TIMER_MSIP          (*(volatile uint32_t *)0x40002010u)

#define IRQ_PERIOD_CYC      54054u    /* 1ms @54.054MHz */

/* ---- 输出位定义 ---- */
#define GPIOO_CLS_MASK   0x0000000Fu
#define GPIOO_VALID      0x00000010u
#define GPIOO_HEARTBEAT  0x00000020u
#define GPIOO_ISR        0x00000040u

/* ---- 中断计数 (0x18 命令与 ISR 共用) ---- */
extern volatile uint32_t g_irq_count;       /* 进入中断的总次数 */
extern volatile uint32_t g_irq_last_cause;  /* 最近一次 mcause */
extern volatile uint32_t g_irq_last_pend;   /* 最近一次 EXTIRQ.PENDING */
extern volatile uint32_t g_gpio_evt;        /* 外部触发次数 (ISR 里累加) */
extern volatile uint32_t g_timer_evt;       /* 定时器中断次数 (ISR 里累加) */

/* 初始化: 低 8 位输出, 高 8 位输入; 使能 gpio_in[0] 上升沿 (源先不接到 meip)
 *   必须在主流程开头调用一次。 */
static inline void gpio_init(void)
{
    GPIO_DIR      = 0x000000FFu;   /* bit0..7 输出 */
    GPIO_DATA_OUT = 0x00000000u;
    /* 板载按键低有效 ⇒ "按下"是下降沿。若外部信号是高有效脉冲, 用 0x17 op=5/7 改。 */
    GPIO_IRQ_RISE = 0x00000000u;
    GPIO_IRQ_FALL = 0x00000001u;   /* gpio_in[0] 下降沿 -> pending */
    GPIO_IRQ_STATUS = 0x0000FFFFu; /* W1C 清掉上电残留 */
}

/* 发布一帧的识别结果。cls = 类别索引 0..4 (0=cup 1=bottle 2=mouse 3=toy 4=screen),
 *   -1 表示本帧无目标；对外编码 = cls+1, 于是 0 天然表示"无"。 */
static inline void gpio_publish(int cls, int valid)
{
    uint32_t v = GPIO_DATA_OUT;
    v &= ~(GPIOO_CLS_MASK | GPIOO_VALID | GPIOO_HEARTBEAT);
    if (valid && cls >= 0) {
        v |= (((uint32_t)cls + 1u) & GPIOO_CLS_MASK) | GPIOO_VALID;
    }
    v ^= GPIOO_HEARTBEAT;          /* 每帧翻转, 外部可测"系统在跑" */
    GPIO_DATA_OUT = v;
}

/* 中断使能窗口 (0x18 用): on=1 只放行 GPIO 源, 打开全局中断 */
static inline void gpio_irq_window(int on)
{
    if (on) {
        GPIO_IRQ_STATUS = 0x0000FFFFu;     /* 清残留 */
        EXTIRQ_ENABLE   = 0x00000002u;     /* 只放行 bit1=GPIO */
        __asm__ volatile("csrs mie, %0" :: "r"(1u << 11));       /* MEIE */
        __asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));    /* MIE  */
    } else {
        __asm__ volatile("csrc mstatus, %0" :: "r"(1u << 3));    /* 关 MIE  */
        __asm__ volatile("csrc mie, %0" :: "r"(1u << 11));       /* 关 MEIE */
        EXTIRQ_ENABLE   = 0x00000000u;
    }
}

/* ---- 64 位安全重装 mtimecmp (2026-10-27 风暴 bug 修复) --------------------
 * 旧代码 (开窗与 ISR 重装两处) 写:
 *     MTIMECMP_HI = 0xFFFFFFFF -> LO = MTIME_LO + 周期 -> HI = 0;
 * 即把 mtimecmp 高 32 位硬编码为 0 —— 只在 mtime < 2^32 (上电 79.4s 内) 成立。
 * 之后 mtimecmp 被压到 mtime 以下, mtip = (mtime >= mtimecmp) 永远为高,
 * mret 一返回立刻重进中断 => **中断风暴**, 主循环饿死, UART 全静默。
 * 板上实锤: 2026-10-27 0x18 src=0 在 uptime>79.4s 后必挂, 79.4s 内存活;
 *           src=1 (不碰 mtimecmp) 任何时刻都正常。
 * 修法: 64 位算目标值, hi-lo-hi 三读防低字回绕撕裂。 */
static inline void timer_rearm(void)
{
	uint32_t hi1, lo, hi2;
	uint64_t t;
	do {
		hi1 = TIMER_MTIME_HI;
		lo  = TIMER_MTIME_LO;
		hi2 = TIMER_MTIME_HI;
	} while (hi1 != hi2);
	t = ((uint64_t)hi1 << 32) | (uint64_t)lo;
	t += IRQ_PERIOD_CYC;
	TIMER_MTIMECMP_HI = 0xFFFFFFFFu;       /* 防瞬态契约 (§11.3): 先抬 HI */
	TIMER_MTIMECMP_LO = (uint32_t)t;
	TIMER_MTIMECMP_HI = (uint32_t)(t >> 32);
}

/* v66: MSIP 软件中断窗口 —— 0x18 src=0 的确定性中断源。
 *   板上实测 (2026-10-27): 同一 slave 的 mtimecmp (0x40002008/0C) 读恒 0、写不粘,
 *   mtime 复位值丢失 => mtip=(mtime>=mtimecmp) 从上电起恒 1, 开 MTIE 即中断风暴;
 *   而 MSIP (0x40002010) 写读正常 (写 1 读回 1)。故 src=0 改用 MSIP:
 *     开窗: MSIP=1 -> 立即触发一次 mcause=0x80000003, ISR 里写 0 清源 (单次);
 *     关窗: MSIP=0 兜底 + 关 MSIE。
 *   (定时器比较中断留待 mtimecmp 的 FPGA 问题解决后恢复, timer_rearm 保留。) */
static inline void msip_irq_window(int on)
{
	if (on) {
		TIMER_MSIP = 1u;                                       /* 电平置位 */
		__asm__ volatile("csrs mie, %0" :: "r"(1u << 3));      /* MSIE */
		__asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));  /* MIE  */
	} else {
		__asm__ volatile("csrc mstatus, %0" :: "r"(1u << 3));
		__asm__ volatile("csrc mie, %0" :: "r"(1u << 3));
		TIMER_MSIP = 0u;                                       /* 兜底清源 */
	}
}

/* 定时器比较中断窗口 —— v66 起 0x18 不再使用 (板上 mtimecmp 坏, 见上)。
 * 逻辑保留: 64 位安全重装已修 (旧写法 HI=0 只在 mtime<2^32 即上电 79.4s 内成立),
 * 等 FPGA 侧 mtimecmp 问题定位后可恢复。 */
static inline void timer_irq_window(int on)
{
	if (on) {
		timer_rearm();
		__asm__ volatile("csrs mie, %0" :: "r"(1u << 7));        /* MTIE */
		__asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));    /* MIE  */
	} else {
		__asm__ volatile("csrc mstatus, %0" :: "r"(1u << 3));
		__asm__ volatile("csrc mie, %0" :: "r"(1u << 7));
		TIMER_MTIMECMP_HI = 0xFFFFFFFFu;
		TIMER_MTIMECMP_LO = 0xFFFFFFFFu;
	}
}

#endif /* GPIO_CTL_H */
