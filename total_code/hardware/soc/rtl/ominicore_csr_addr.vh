
// OmniCore 实现的 CSR 地址列表，包括自定义 CSR。

// ----------------------------------------------------------------------------
// M 模式 CSR

// 机器信息寄存器（RO）
localparam MVENDORID      = 12'hf11; // 供应商 ID。
localparam MARCHID        = 12'hf12; // 体系结构 ID。
localparam MIMPID         = 12'hf13; // 实现 ID。
localparam MHARTID        = 12'hf14; // 硬件线程 ID。
localparam MCONFIGPTR     = 12'hf15; // 指向配置数据结构的指针。

// 机器陷阱设置（RW）
localparam MSTATUS        = 12'h300; // 机器状态寄存器。
localparam MSTATUSH       = 12'h310; // 自 priv-1.12 起即使恒接 0 也必须存在。
localparam MISA           = 12'h301; // ISA 与扩展
localparam MEDELEG        = 12'h302; // 机器异常委托寄存器。
localparam MIDELEG        = 12'h303; // 机器中断委托寄存器。
localparam MIE            = 12'h304; // 机器中断使能寄存器。
localparam MTVEC          = 12'h305; // 机器陷阱处理程序基址。
localparam MCOUNTEREN     = 12'h306; // 机器计数器使能。

// 机器陷阱处理（RW）
localparam MSCRATCH       = 12'h340; // 机器陷阱处理程序暂存寄存器。
localparam MEPC           = 12'h341; // 机器异常程序计数器。
localparam MCAUSE         = 12'h342; // 机器陷阱原因。
localparam MTVAL          = 12'h343; // 机器出错地址或指令。
localparam MIP            = 12'h344; // 机器中断挂起。

// 机器内存保护（RW）
localparam PMPCFG0        = 12'h3a0; // 物理内存保护配置。
localparam PMPCFG1        = 12'h3a1; // 物理内存保护配置，仅 RV32。
localparam PMPCFG2        = 12'h3a2; // 物理内存保护配置。
localparam PMPCFG3        = 12'h3a3; // 物理内存保护配置，仅 RV32。
localparam PMPADDR0       = 12'h3b0; // 物理内存保护地址寄存器。
localparam PMPADDR1       = 12'h3b1; // ...
localparam PMPADDR2       = 12'h3b2;
localparam PMPADDR3       = 12'h3b3;
localparam PMPADDR4       = 12'h3b4;
localparam PMPADDR5       = 12'h3b5;
localparam PMPADDR6       = 12'h3b6;
localparam PMPADDR7       = 12'h3b7;
localparam PMPADDR8       = 12'h3b8;
localparam PMPADDR9       = 12'h3b9;
localparam PMPADDR10      = 12'h3ba;
localparam PMPADDR11      = 12'h3bb;
localparam PMPADDR12      = 12'h3bc;
localparam PMPADDR13      = 12'h3bd;
localparam PMPADDR14      = 12'h3be;
localparam PMPADDR15      = 12'h3bf;

localparam MSECCFG        = 12'h747;
localparam MSECCFGH       = 12'h757;

// 性能计数器（RW）
localparam MCYCLE         = 12'hb00; // 上电以来的原始周期数
localparam MINSTRET       = 12'hb02; // 上电以来的指令退休计数
localparam MHPMCOUNTER3   = 12'hb03; // WARL（我们恒接 0）
localparam MHPMCOUNTER4   = 12'hb04; // WARL（我们恒接 0）
localparam MHPMCOUNTER5   = 12'hb05; // WARL（我们恒接 0）
localparam MHPMCOUNTER6   = 12'hb06; // WARL（我们恒接 0）
localparam MHPMCOUNTER7   = 12'hb07; // WARL（我们恒接 0）
localparam MHPMCOUNTER8   = 12'hb08; // WARL（我们恒接 0）
localparam MHPMCOUNTER9   = 12'hb09; // WARL（我们恒接 0）
localparam MHPMCOUNTER10  = 12'hb0a; // WARL（我们恒接 0）
localparam MHPMCOUNTER11  = 12'hb0b; // WARL（我们恒接 0）
localparam MHPMCOUNTER12  = 12'hb0c; // WARL（我们恒接 0）
localparam MHPMCOUNTER13  = 12'hb0d; // WARL（我们恒接 0）
localparam MHPMCOUNTER14  = 12'hb0e; // WARL（我们恒接 0）
localparam MHPMCOUNTER15  = 12'hb0f; // WARL（我们恒接 0）
localparam MHPMCOUNTER16  = 12'hb10; // WARL（我们恒接 0）
localparam MHPMCOUNTER17  = 12'hb11; // WARL（我们恒接 0）
localparam MHPMCOUNTER18  = 12'hb12; // WARL（我们恒接 0）
localparam MHPMCOUNTER19  = 12'hb13; // WARL（我们恒接 0）
localparam MHPMCOUNTER20  = 12'hb14; // WARL（我们恒接 0）
localparam MHPMCOUNTER21  = 12'hb15; // WARL（我们恒接 0）
localparam MHPMCOUNTER22  = 12'hb16; // WARL（我们恒接 0）
localparam MHPMCOUNTER23  = 12'hb17; // WARL（我们恒接 0）
localparam MHPMCOUNTER24  = 12'hb18; // WARL（我们恒接 0）
localparam MHPMCOUNTER25  = 12'hb19; // WARL（我们恒接 0）
localparam MHPMCOUNTER26  = 12'hb1a; // WARL（我们恒接 0）
localparam MHPMCOUNTER27  = 12'hb1b; // WARL（我们恒接 0）
localparam MHPMCOUNTER28  = 12'hb1c; // WARL（我们恒接 0）
localparam MHPMCOUNTER29  = 12'hb1d; // WARL（我们恒接 0）
localparam MHPMCOUNTER30  = 12'hb1e; // WARL（我们恒接 0）
localparam MHPMCOUNTER31  = 12'hb1f; // WARL（我们恒接 0）

localparam MCYCLEH        = 12'hb80; // 各计数器的高半部
localparam MINSTRETH      = 12'hb82;
localparam MHPMCOUNTER3H  = 12'hb83;
localparam MHPMCOUNTER4H  = 12'hb84;
localparam MHPMCOUNTER5H  = 12'hb85;
localparam MHPMCOUNTER6H  = 12'hb86;
localparam MHPMCOUNTER7H  = 12'hb87;
localparam MHPMCOUNTER8H  = 12'hb88;
localparam MHPMCOUNTER9H  = 12'hb89;
localparam MHPMCOUNTER10H = 12'hb8a;
localparam MHPMCOUNTER11H = 12'hb8b;
localparam MHPMCOUNTER12H = 12'hb8c;
localparam MHPMCOUNTER13H = 12'hb8d;
localparam MHPMCOUNTER14H = 12'hb8e;
localparam MHPMCOUNTER15H = 12'hb8f;
localparam MHPMCOUNTER16H = 12'hb90;
localparam MHPMCOUNTER17H = 12'hb91;
localparam MHPMCOUNTER18H = 12'hb92;
localparam MHPMCOUNTER19H = 12'hb93;
localparam MHPMCOUNTER20H = 12'hb94;
localparam MHPMCOUNTER21H = 12'hb95;
localparam MHPMCOUNTER22H = 12'hb96;
localparam MHPMCOUNTER23H = 12'hb97;
localparam MHPMCOUNTER24H = 12'hb98;
localparam MHPMCOUNTER25H = 12'hb99;
localparam MHPMCOUNTER26H = 12'hb9a;
localparam MHPMCOUNTER27H = 12'hb9b;
localparam MHPMCOUNTER28H = 12'hb9c;
localparam MHPMCOUNTER29H = 12'hb9d;
localparam MHPMCOUNTER30H = 12'hb9e;
localparam MHPMCOUNTER31H = 12'hb9f;

localparam MCOUNTINHIBIT  = 12'h320; // mcycle/minstret 的计数禁止寄存器
localparam MHPMEVENT3     = 12'h323; // WARL（我们恒接 0）
localparam MHPMEVENT4     = 12'h324; // WARL（我们恒接 0）
localparam MHPMEVENT5     = 12'h325; // WARL（我们恒接 0）
localparam MHPMEVENT6     = 12'h326; // WARL（我们恒接 0）
localparam MHPMEVENT7     = 12'h327; // WARL（我们恒接 0）
localparam MHPMEVENT8     = 12'h328; // WARL（我们恒接 0）
localparam MHPMEVENT9     = 12'h329; // WARL（我们恒接 0）
localparam MHPMEVENT10    = 12'h32a; // WARL（我们恒接 0）
localparam MHPMEVENT11    = 12'h32b; // WARL（我们恒接 0）
localparam MHPMEVENT12    = 12'h32c; // WARL（我们恒接 0）
localparam MHPMEVENT13    = 12'h32d; // WARL（我们恒接 0）
localparam MHPMEVENT14    = 12'h32e; // WARL（我们恒接 0）
localparam MHPMEVENT15    = 12'h32f; // WARL（我们恒接 0）
localparam MHPMEVENT16    = 12'h330; // WARL（我们恒接 0）
localparam MHPMEVENT17    = 12'h331; // WARL（我们恒接 0）
localparam MHPMEVENT18    = 12'h332; // WARL（我们恒接 0）
localparam MHPMEVENT19    = 12'h333; // WARL（我们恒接 0）
localparam MHPMEVENT20    = 12'h334; // WARL（我们恒接 0）
localparam MHPMEVENT21    = 12'h335; // WARL（我们恒接 0）
localparam MHPMEVENT22    = 12'h336; // WARL（我们恒接 0）
localparam MHPMEVENT23    = 12'h337; // WARL（我们恒接 0）
localparam MHPMEVENT24    = 12'h338; // WARL（我们恒接 0）
localparam MHPMEVENT25    = 12'h339; // WARL（我们恒接 0）
localparam MHPMEVENT26    = 12'h33a; // WARL（我们恒接 0）
localparam MHPMEVENT27    = 12'h33b; // WARL（我们恒接 0）
localparam MHPMEVENT28    = 12'h33c; // WARL（我们恒接 0）
localparam MHPMEVENT29    = 12'h33d; // WARL（我们恒接 0）
localparam MHPMEVENT30    = 12'h33e; // WARL（我们恒接 0）
localparam MHPMEVENT31    = 12'h33f; // WARL（我们恒接 0）

// 其他标准 M 模式 CSR：
localparam MENVCFG        = 12'h30a;
localparam MENVCFGH       = 12'h31a;

// 自定义 M 模式 CSR：
localparam PMPCFGM0       = 12'hbd0; // 使 PMP 区域进入 M 模式且不加锁
//                              bd1  // (为超过 32 个区域保留)

localparam MEIEA          = 12'hbe0; // 外部中断挂起数组
localparam MEIPA          = 12'hbe1; // 外部中断使能数组
localparam MEIFA          = 12'hbe2; // 外部中断强制数组
localparam MEIPRA         = 12'hbe3; // 外部中断优先级数组
localparam MEINEXT        = 12'hbe4; // 下一个外部中断
localparam MEICONTEXT     = 12'hbe5; // 外部中断上下文寄存器

localparam MSLEEP         = 12'hbf0; // M 模式睡眠控制寄存器
localparam H3MISA         = 12'hbf1; // OmniCore M 模式 ISA 标识寄存器

// ----------------------------------------------------------------------------
// U 模式 CSR

// M 模式计数器 CSR 的只读别名：
localparam CYCLE          = 12'hc00;
localparam TIME           = 12'hc01;
localparam INSTRET        = 12'hc02;
localparam CYCLEH         = 12'hc80;
localparam TIMEH          = 12'hc81;
localparam INSTRETH       = 12'hc82;

// 自定义 U 模式 CSR
localparam SLEEP          = 12'h8f0; // M 模式睡眠控制的 U 模式子集

// ----------------------------------------------------------------------------
// 触发模块

localparam TSELECT       = 12'h7a0;
localparam TDATA1        = 12'h7a1;
localparam TDATA2        = 12'h7a2;
localparam TDATA3        = 12'h7a3;
localparam TINFO         = 12'h7a4;
localparam TCONTROL      = 12'h7a5;
localparam MCONTEXT      = 12'h7a8;

// ----------------------------------------------------------------------------
// D 模式 CSR

localparam DCSR           = 12'h7b0;
localparam DPC            = 12'h7b1;
localparam DMDATA0        = 12'hbff; // 自定义读/写
