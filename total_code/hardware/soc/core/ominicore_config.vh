// ===========================================================================
// ominicore_config.vh — 紫光竞赛核心的 RTL 侧配置
// ===========================================================================
// 格式：供 #(...) 使用的参数表主体。由 ziguang_core.v、ziguang_frontend.v
// 与 ziguang_cpu_2port_cached.v 在其参数头中 include。与 cfg/config_ziguang.vh
// （localparam 版本，供 testbench 模块体使用）保持同步。
// ===========================================================================

parameter RESET_VECTOR        = 32'h80000040,
parameter MTVEC_INIT          = 32'h80000000,
parameter EXTENSION_A         = 0,
parameter EXTENSION_C         = 1,
parameter EXTENSION_E         = 0,
parameter EXTENSION_M         = 1,
parameter EXTENSION_ZBA       = 1,
parameter EXTENSION_ZBB       = 1,
parameter EXTENSION_ZBC       = 0,
parameter EXTENSION_ZBKB      = 1,
parameter EXTENSION_ZBKX      = 0,
parameter EXTENSION_ZBS       = 1,
parameter EXTENSION_ZCB       = 0,
parameter EXTENSION_ZCLSD     = 0,
parameter EXTENSION_ZCMP      = 1,
parameter EXTENSION_ZIFENCEI  = 1,
parameter EXTENSION_ZILSD     = 0,
parameter EXTENSION_XH3BEXTM  = 0,
parameter EXTENSION_XH3IRQ    = 1,
parameter EXTENSION_XH3PMPM   = 0,
parameter EXTENSION_XH3POWER  = 1,
parameter CSR_M_MANDATORY     = 1,
parameter CSR_M_TRAP          = 1,
parameter CSR_COUNTER         = 1,
parameter U_MODE              = 0,
parameter PMP_REGIONS         = 0,
parameter PMP_GRAIN           = 0,
parameter PMP_MATCH_NAPOT     = 1,
parameter PMP_MATCH_TOR       = 0,
// ZIGUANG: 复制拼接次数先用普通参数算好 -- ADS 对复制符内嵌三目表达式
// 报 "hasn't been initialized" (4112), 拆成两步即可。语义不变。
parameter PMP_REGIONS_GE1     = (PMP_REGIONS > 0) ? PMP_REGIONS : 1,
parameter PMP_HARDWIRED       = {PMP_REGIONS_GE1{1'b0}},
parameter PMP_HARDWIRED_ADDR  = {PMP_REGIONS_GE1{32'h0}},
parameter PMP_HARDWIRED_CFG   = {PMP_REGIONS_GE1{8'h00}},
parameter DEBUG_SUPPORT       = 1,
parameter BREAKPOINT_TRIGGERS = 0,
parameter NUM_IRQS            = 16,
parameter IRQ_PRIORITY_BITS   = 4,
parameter IRQ_INPUT_BYPASS    = {NUM_IRQS{1'b0}},
parameter MVENDORID_VAL       = 32'h0,
parameter MCONFIGPTR_VAL      = 32'h0,
parameter REDUCED_BYPASS      = 0,
parameter MULDIV_UNROLL       = 2,
parameter MUL_FAST            = 1,
parameter MUL_FASTER          = 1,
parameter MULH_FAST           = 1,
parameter FAST_BRANCHCMP      = 1,
parameter RESET_REGFILE       = 0,
parameter BRANCH_PREDICTOR    = 1,
parameter MTVEC_WMASK         = 32'hfffffffd,

// ---------------------------------------------------------------------------
// 紫光新增项（由 ziguang_frontend / ziguang_cpu_2port_cached 使用）
// ---------------------------------------------------------------------------
parameter ZG_BTB_ENTRIES      = 64,             // 2 的幂 (BP-OPT: 128 项实测无增益)
parameter ZG_ICACHE_SETS      = 128,            // 2 路 x 每路 8 字 x 128 组 = 8 KB (B3: 32B 行)
parameter ZG_DCACHE_SETS      = 128,            // 8 KB (容量决策: 真实 miss 0.1%, 16KB 无收益)
parameter ZG_CACHE_LINE_WORDS = 8,              // B3/CACHE-01: 每行字数
parameter ZG_ICACHE_EN        = 1,              // 0 = I 侧按上游 2port 直通（generate 剪除 cache 模块）
parameter ZG_DCACHE_EN        = 1,              // 1 = load 走 2 路组相联; 0 = 上游直通
parameter ZG_CACHE_BASE       = 32'h80000000,   // 可缓存窗口 [BASE, LIMIT)
parameter ZG_CACHE_LIMIT      = 32'hc0000000,   // (仿真: RAM @0x80000000 走 cache, IO @0xc0000000 旁路)
parameter ZG_WRAP_FILL        = 0,              // B3: 32B 行用 INCR8 突发 (AHB 无 WRAP8)

// ---------------------------------------------------------------------------
// 端口宽度参数（勿改动）——上游子模块所要求
// ---------------------------------------------------------------------------
parameter W_ADDR              = 32,
parameter W_DATA              = 32
