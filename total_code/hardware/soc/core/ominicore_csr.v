


`default_nettype none

// 控制与状态寄存器（CSR）
// 还包含 CSR 相关逻辑，如中断使能/屏蔽、
// 陷阱向量计算。

module ominicore_csr #(
	parameter XLEN            = 32,   // 必须为 32
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input  wire               clk,
	input  wire               clk_always_on,
	input  wire               rst_n,

	// 调试信号
	output reg                debug_mode,

	input  wire               dbg_req_halt,
	input  wire               dbg_req_halt_on_reset,
	input  wire               dbg_req_resume,

	output wire               dbg_instr_caught_exception,
	output wire               dbg_instr_caught_ebreak,

	input  wire [W_DATA-1:0]  dbg_data0_rdata,
	output wire [W_DATA-1:0]  dbg_data0_wdata,
	output wire               dbg_data0_wen,

	output wire [W_ADDR-1:0]  debug_dpc_wdata,
	output wire               debug_dpc_wen,
	input  wire [W_ADDR-1:0]  debug_dpc_rdata,

	// 读端口是组合逻辑。
	// 写端口是同步的，写效果在下一时钟拍可见。
	// *_soon 选通是不经内核停顿信号门控的版本。
	// 需要它们的原因:
	// - 内核停顿是总线停顿的函数
	// - 非法 CSR 访问会产生陷阱进入
	// - 陷阱进入（未必由 CSR 访问引起）会门控向外的总线访问
	// - 从 hready 到 htrans 之类的直通路径对时序/实现是麻烦
	input  wire [11:0]        addr,
	input  wire [XLEN-1:0]    wdata,
	input  wire               wen,
	input  wire               wen_soon, // 某个停顿条件解除后 wen 即置起
	input  wire               wen_raw,  // 原始的、未门控的指令译码信号，供 D 模式寄存器使用
	input  wire [1:0]         wtype,
	output reg  [XLEN-1:0]    rdata,
	input  wire               ren,
	input  wire               ren_soon, // 某个停顿条件解除后 ren 即置起
	output wire               illegal,
	output wire               write_is_fetch_ordered,

	// 陷阱信号
	// 由*我们*告知内核正在进入陷阱及目标地址，依据:
	// - 来自内核的同步异常输入
	// - 外部 IRQ 信号
	// - 基于 mie 等 CSR 状态的屏蔽等
	//
	// 我们置起 trap_enter_vld 并保持，直到 trap_enter_rdy
	// 变高。trap_addr 携带陷阱目标地址的绝对值。
	// 一旦 trap_enter_vld && _rdy，mepc_in 被复制进 mepc，并设置其他陷阱状态。
	//
	// 注意异常输入可能消失，例如流水线被冲刷。这种情况下
	// 我们撤销 trap_enter_vld。
	output wire [XLEN-1:0]     trap_addr,
	output wire                trap_is_irq,
	output wire                trap_enter_vld,
	input  wire                trap_enter_rdy,
	// 即将陷入陷阱、但正在等待某条已产生或
	// 可能产生异常的指令先清空 M 级时为真。X 级的指令
	// 被抑制，X 的 PC 不自增但仍跟踪异常
	// 地址。
	output wire                trap_enter_soon,
	// 我们需要知道处于数据相的 load/store，因为其异常
	// 状态未知，进入调试模式前要把它们隔离。
	input  wire                loadstore_dphase_pending,
	// 第 2 级指令无法被取消时（例如为保持 AHB 地址相
	// 稳定），或无法发出取指请求时
	// （例如等待从 wfi 状态唤醒）置起。
	input  wire                delay_irq_entry,
	input  wire [XLEN-1:0]     mepc_in,

	// 电源控制信号
	output wire                pwr_allow_clkgate,
	output wire                pwr_allow_power_down,
	output wire                pwr_allow_sleep_on_block,
	output wire                pwr_wfi_wakeup_req,

	// 以下各操作可以在与其他操作不同的特权级执行:
	output wire                m_mode_execution,
	output wire                m_mode_trap_entry,
	output wire                m_mode_loadstore,

	// 异常不得是总线停顿的函数。
	input  wire [W_EXCEPT-1:0] except,
	input  wire                except_to_d_mode,

	// 电平触发中断源
	input  wire [NUM_IRQS-1:0] irq,
	input  wire                irq_software,
	input  wire                irq_timer,

	// PMP 配置接口
	output wire [11:0]         pmp_cfg_addr,
	output reg                 pmp_cfg_wen,
	output wire [W_DATA-1:0]   pmp_cfg_wdata,
	input  wire [W_DATA-1:0]   pmp_cfg_rdata,

	// 触发单元配置接口
	output wire [11:0]         trig_cfg_addr,
	output reg                 trig_cfg_wen,
	output wire [W_DATA-1:0]   trig_cfg_wdata,
	input  wire [W_DATA-1:0]   trig_cfg_rdata,
	output wire                trig_m_en,

	// 中断/异常触发器事件
	output wire                trig_event_interrupt,
	output wire                trig_event_exception,
	output wire [3:0]          trig_event_trap_cause,

	// 其他 CSR 专属信号
	output wire                clear_excl_on_trap_exit,
	output wire                trap_wfi,
	input  wire                instr_ret,
	input  wire [W_DATA-1:0]   mhartid_val,
	input  wire [3:0]          eco_version,

	// ZIGUANG (规格 §10, PMU-01~04): 自定义性能计数事件脉冲, 单拍高有效。
	// 每个事件唯一计数点, 定义见 PROGRESS-06 PMU 文档
	input  wire                zg_pmu_branch_ret,   // 退休条件分支
	input  wire                zg_pmu_bp_miss,      // 分支预测错误
	input  wire                zg_pmu_icaccess,     // I-Cache 查询
	input  wire                zg_pmu_icmiss,       // I-Cache miss
	input  wire                zg_pmu_dcaccess,     // D-Cache cacheable 查询
	input  wire                zg_pmu_dcmiss,       // D-Cache load miss
	input  wire                zg_pmu_stall,        // 数据/结构冲突停顿拍
	input  wire                zg_pmu_buswait       // 下游总线等待拍
);

`include "ominicore_ops.vh"
`include "ominicore_csr_addr.vh"

localparam X0 = {XLEN{1'b0}};

// ----------------------------------------------------------------------------
// ZIGUANG: 自定义性能监控单元 (规格 §10, CSR 0x7C0–0x7D0, PMU-01~04)
// 8 组 64-bit 回绕计数器, 复位 0; ENABLE=0 保持; 写清除优先于同周期事件。
// 计数点 (每个事件唯一, 不重复计数):
//   mbranch   = 退休条件分支 (退休脉冲 && 条件分支指令)
//   mbpmiss   = 分支预测错误 (退休条件分支且实际方向 != 预测方向)
//   micaccess = I$ 查询 (I$ 核心侧地址相受理)
//   micmiss   = I$ miss (查询且未命中)
//   mdcaccess = D$ cacheable 查询 (D 侧 cacheable 地址相受理; D$=0 即
//               cacheable 访问数)
//   mdcmiss   = D$ load miss (cacheable load 受理; D$=0 时全部 miss)
//   mstall    = 数据/结构冲突停顿拍 (X 级停顿且当拍非总线等待、非休眠/停机)
//   mbuswait  = 下游总线等待拍 (I$/D 任一端口数据相在途且 hready 未到,
//               同拍双口等待只计一次; 与 mstall 互斥定义)
// ----------------------------------------------------------------------------
localparam [11:0] ZPMU_MBRANCH    = 12'h7c0;
wire wen_m_mode;   // (2026-10-09 前移: vlog 严格先声明后使用)
localparam [11:0] ZPMU_MBRANCHH   = 12'h7c1;
localparam [11:0] ZPMU_MBPMISS    = 12'h7c2;
localparam [11:0] ZPMU_MBPMISSH   = 12'h7c3;
localparam [11:0] ZPMU_MICACCESS  = 12'h7c4;
localparam [11:0] ZPMU_MICACCESSH = 12'h7c5;
localparam [11:0] ZPMU_MICMISS    = 12'h7c6;
localparam [11:0] ZPMU_MICMISS_H  = 12'h7c7;
localparam [11:0] ZPMU_MDCACCESS  = 12'h7c8;
localparam [11:0] ZPMU_MDCACCESSH = 12'h7c9;
localparam [11:0] ZPMU_MDCMISS    = 12'h7ca;
localparam [11:0] ZPMU_MDCMISS_H  = 12'h7cb;
localparam [11:0] ZPMU_MSTALL     = 12'h7cc;
localparam [11:0] ZPMU_MSTALLH    = 12'h7cd;
localparam [11:0] ZPMU_MBUSWAIT   = 12'h7ce;
localparam [11:0] ZPMU_MBUSWAITH  = 12'h7cf;
localparam [11:0] ZPMU_CTRL       = 12'h7d0;

reg         zg_pmu_en;
reg [63:0]  zg_pmu_branch_r;
reg [63:0]  zg_pmu_bpmiss_r;
reg [63:0]  zg_pmu_icaccess_r;
reg [63:0]  zg_pmu_icmiss_r;
reg [63:0]  zg_pmu_dcaccess_r;
reg [63:0]  zg_pmu_dcmiss_r;
reg [63:0]  zg_pmu_stall_r;
reg [63:0]  zg_pmu_buswait_r;

// ----------------------------------------------------------------------------
// CSR 状态 + 更新逻辑
// ----------------------------------------------------------------------------
// 命名为 (寄存器)_(字段)

// 对整个 CSR 进行写/置位/清零的通用更新逻辑:
wire [XLEN-1:0] wdata_update =
		wtype == CSR_WTYPE_C ? rdata & ~wdata :
		wtype == CSR_WTYPE_S ? rdata |  wdata : wdata;

// ----------------------------------------------------------------------------
// ZIGUANG: PMU 计数器更新 (规格 §10)。写在 wdata_update 定义之后。
// 写清除: mpmuctrl 有效写入值 bit1 = 1; 清除优先于同周期事件; ENABLE=0 保持
// ----------------------------------------------------------------------------
wire zg_pmu_clear = wen_m_mode && addr == ZPMU_CTRL && wtype != CSR_WTYPE_C
	&& wdata_update[1];

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		zg_pmu_en       <= 1'b0;
		zg_pmu_branch_r   <= 64'h0;
		zg_pmu_bpmiss_r   <= 64'h0;
		zg_pmu_icaccess_r <= 64'h0;
		zg_pmu_icmiss_r   <= 64'h0;
		zg_pmu_dcaccess_r <= 64'h0;
		zg_pmu_dcmiss_r   <= 64'h0;
		zg_pmu_stall_r    <= 64'h0;
		zg_pmu_buswait_r  <= 64'h0;
	end else begin
		if (wen_m_mode && addr == ZPMU_CTRL)
			zg_pmu_en <= wdata_update[0];
		if (zg_pmu_clear) begin
			zg_pmu_branch_r   <= 64'h0;
			zg_pmu_bpmiss_r   <= 64'h0;
			zg_pmu_icaccess_r <= 64'h0;
			zg_pmu_icmiss_r   <= 64'h0;
			zg_pmu_dcaccess_r <= 64'h0;
			zg_pmu_dcmiss_r   <= 64'h0;
			zg_pmu_stall_r    <= 64'h0;
			zg_pmu_buswait_r  <= 64'h0;
		end else if (zg_pmu_en) begin
			if (zg_pmu_branch_ret)
				zg_pmu_branch_r  <= zg_pmu_branch_r  + 64'h1;
			if (zg_pmu_bp_miss)
				zg_pmu_bpmiss_r  <= zg_pmu_bpmiss_r  + 64'h1;
			if (zg_pmu_icaccess)
				zg_pmu_icaccess_r <= zg_pmu_icaccess_r + 64'h1;
			if (zg_pmu_icmiss)
				zg_pmu_icmiss_r  <= zg_pmu_icmiss_r  + 64'h1;
			if (zg_pmu_dcaccess)
				zg_pmu_dcaccess_r <= zg_pmu_dcaccess_r + 64'h1;
			if (zg_pmu_dcmiss)
				zg_pmu_dcmiss_r  <= zg_pmu_dcmiss_r  + 64'h1;
			if (zg_pmu_stall)
				zg_pmu_stall_r   <= zg_pmu_stall_r   + 64'h1;
			if (zg_pmu_buswait)
				zg_pmu_buswait_r <= zg_pmu_buswait_r + 64'h1;
		end
	end
end

function [XLEN-1:0] update_nonconst;
	input [XLEN-1:0] prev;
	input [XLEN-1:0] nonconst;
begin
		update_nonconst = (wdata_update & nonconst) | (prev & ~nonconst) ;
end
endfunction

wire enter_debug_mode;
wire exit_debug_mode;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		debug_mode <= 1'b0;
	end else if (DEBUG_SUPPORT) begin
		debug_mode <= (debug_mode && !exit_debug_mode) || enter_debug_mode;
	end
end

// 内核执行状态，1 -> M 模式，0 -> U 模式（若实现）
reg  m_mode;
assign wen_m_mode = wen && (m_mode || debug_mode);
wire ren_m_mode = ren && (m_mode || debug_mode);

// ----------------------------------------------------------------------------
// 标准陷阱处理 CSR

wire debug_suppresses_trap_update = DEBUG_SUPPORT && (debug_mode || enter_debug_mode);

wire trapreg_update = trap_enter_vld && trap_enter_rdy && !debug_suppresses_trap_update;
wire trapreg_update_enter = trapreg_update && except != EXCEPT_MRET && except != EXCEPT_REFETCH;
wire trapreg_update_exit = trapreg_update && except == EXCEPT_MRET;

// 本地监视器标志在陷阱退出时清除（但调试模式退出时不清除，
// 以便单步调试 LR/SC 序列）:
assign clear_excl_on_trap_exit = trapreg_update_exit;

reg mstatus_mpie;
reg mstatus_mie;
reg mstatus_mpp; // 仅实现最高位（MSB）
reg mstatus_mprv;
reg mstatus_tw;

// 规范原文（priv-1.12）:
// “MRET 或 SRET 指令分别用于从 M 模式或
//  S 模式的陷阱返回。执行 xRET 指令时，设 xPP
//  的值为 y，则 xIE 被置为 xPIE; 特权级改为
//  y; xPIE 置为 1; xPP 置为所支持的最低特权级
// （实现了 U 模式则为 U，否则为 M）。若 xPP=M，xRET 还将
//  MPRV 置 0。”

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		m_mode <= 1'b1;
		mstatus_mpie <= 1'b0;
		mstatus_mie <= 1'b0;
		mstatus_mpp <= 1'b1;
		mstatus_mprv <= 1'b0;
		mstatus_tw <= 1'b0;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量; 避免综合器对
		// 对 clk 不敏感的触发器报警告。应与上方 rst_n 分支一致，
		// 使触发器被常量传播优化掉。
		m_mode <= 1'b1;
		mstatus_mpie <= 1'b0;
		mstatus_mie <= 1'b0;
		mstatus_mpp <= 1'b1;
		mstatus_mprv <= 1'b0;
		mstatus_tw <= 1'b0;
	end else begin
		if (trapreg_update_exit) begin
			mstatus_mpie <= 1'b1;
			mstatus_mie <= mstatus_mpie;
			if (U_MODE) begin
				m_mode <= mstatus_mpp;
				mstatus_mpp <= 1'b0;
				if (!mstatus_mpp) begin
					mstatus_mprv <= 1'b0;
				end
			end
		end else if (trapreg_update_enter) begin
			mstatus_mpie <= mstatus_mie;
			mstatus_mie <= 1'b0;
			if (U_MODE) begin
				m_mode <= 1'b1;
				mstatus_mpp <= m_mode;
			end
		end else if (wen_m_mode && addr == MSTATUS) begin
			mstatus_mpie <= wdata_update[7];
			mstatus_mie  <= wdata_update[3];
			mstatus_mprv <= wdata_update[17];
			// 注意 MPP 只实现最高位。读回为 11 或 00。
			mstatus_mpp  <= wdata_update[12] || !U_MODE;
			mstatus_tw   <= wdata_update[21] && U_MODE;
		end else if (wen_raw && debug_mode && addr == DCSR && U_MODE && DEBUG_SUPPORT) begin
			// 调试器可以通过 dcsr.prv 直接改变/观察内核状态
			// （这不影响调试器操作，因为内核
			// 停机调试期间，所有操作的等效特权级
			// 都是 M 模式。）
			m_mode <= wdata_update[1];
		end
		if (!U_MODE) begin
			// 显式接常量; 某些综合工具不喜欢
			// 只对复位敏感的变量。
			m_mode <= 1'b1;
			mstatus_mpp <= 1'b1;
		end
	end
end

// 若 timeout 位置起，所有 U 模式 WFI 都陷入陷阱。注意调试规范
// 说程序缓冲执行期间 "wfi 指令等价于 nop"，
// 而 nop 不陷入异常，因此在调试模式中我们允许 WFI
// 但立即将其唤醒。
assign trap_wfi = mstatus_tw && !(debug_mode || m_mode);

reg [XLEN-1:0] mscratch;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mscratch <= X0;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量
		mscratch <= X0;
	end else begin
		if (wen_m_mode && addr == MSCRATCH) begin
			mscratch <= wdata_update;
		end
	end
end

// 陷阱向量基址
reg  [XLEN-1:0] mtvec_reg;
wire [XLEN-1:0] mtvec = mtvec_reg & ({XLEN{1'b1}} << 2);
wire            irq_vector_enable = mtvec_reg[0];

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mtvec_reg <= MTVEC_INIT;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量
		mtvec_reg <= MTVEC_INIT;
	end else begin
		if (wen_m_mode && addr == MTVEC) begin
			mtvec_reg <= update_nonconst(mtvec_reg, MTVEC_WMASK);
		end
	end
end

// 异常程序计数器
reg [XLEN-1:0] mepc;
// mepc 只保存与指令对齐粒度对齐的值
localparam MEPC_MASK = {{XLEN-2{1'b1}}, |EXTENSION_C, 1'b0};

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mepc <= X0;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量
		mepc <= X0;
	end else begin
		if (trapreg_update_enter) begin
			mepc <= mepc_in & MEPC_MASK;
		end else if (wen_m_mode && addr == MEPC) begin
			mepc <= wdata_update & MEPC_MASK;
		end
	end
end

// 中断使能（保留位接 0）
reg [XLEN-1:0] mie;
localparam MIE_WMASK = 32'h00000888; // meie, mtie, msie

wire meicontext_clearts;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mie <= X0;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量
		mie <= X0;
	end else begin
		if (wen_m_mode && addr == MIE) begin
			mie <= update_nonconst(mie, MIE_WMASK);
		end else if (wen_m_mode && addr == MEICONTEXT) begin
			// meicontext.clearts/mtiesave/msiesave 可用于清除和
			// 保存标准定时器与软件中断使能，同时
			// 保存外部中断上下文。
			mie[7] <= (mie[7] || wdata_update[3]) && !meicontext_clearts;
			mie[3] <= (mie[3] || wdata_update[2]) && !meicontext_clearts;
		end
	end
end

// 中断未决寄存器（稍后赋值）。在本实现中该
// 寄存器完全只读。
wire [XLEN-1:0] mip;

// 陷阱原因寄存器。非常量位可由软件写，
// 并在陷阱进入时自动更新。`code` 字段只需大到能
// 覆盖硬件会置起的原因，本实现为 4 位。
// （注意这与 scause 不同，后者硬性最少
// 5 位。）
reg        mcause_irq;
reg  [3:0] mcause_code;
wire       mcause_irq_next;
wire [3:0] mcause_code_next;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mcause_irq <= 1'b0;
		mcause_code <= 4'h0;
	end else if (!CSR_M_TRAP) begin
		// 未实现时显式接常量
		mcause_irq <= 1'b0;
		mcause_code <= 4'h0;
	end else begin
		if (trapreg_update_enter) begin
			mcause_irq <= mcause_irq_next;
			mcause_code <= mcause_code_next;
		end else if (wen_m_mode && addr == MCAUSE) begin
			{mcause_irq, mcause_code} <= {wdata_update[31], wdata_update[3:0]};
		end
	end
end

// ----------------------------------------------------------------------------
// 标准标识（ID）CSR



// 若本 RTL 尚未发布到稳定分支则置 1:
localparam [0:0] MIMPID_PRERELEASE = 1'b0;

// 主版本号，例如 v1.2.3 中的 1:
localparam [3:0] MIMPID_MAJOR      = 4'd1;
// 次版本号，例如 v1.2.3 中的 2:
localparam [7:0] MIMPID_MINOR      = 8'd1;
// 修订号，例如 v1.2.3 中的 3:
localparam [7:0] MIMPID_PATCH      = 8'd1;

// 注意 eco_version 字段外部连线 —— 在 ASIC 上它可以
// 接到 tie 单元，以便金属层 ECO 后递增。

// misa 为 WARL, 已实现的 CSR 写入触发非法指令反而是架构违规; 本项目规格
// CSR-01 中"写入非法"的表述按架构正确性处理并在变更记录中说明。

wire [31:0] mimpid_rdata = 32'h0001_0000;   // OmniCore v1.0（规格固定值）

localparam [31:0] MISA_VAL = 32'h4000_1104; // RV32IMC，无 A/F/D/X 位（规格固定值）

// ----------------------------------------------------------------------------
// 自定义外部中断控制器

wire [XLEN-1:0] irq_ctrl_rdata;
wire            external_irq_pending;

generate
if (|EXTENSION_XH3IRQ) begin: have_irq_ctrl

	ominicore_irq_ctrl #(
	`include "ominicore_config_inst.vh"
	) irq_ctrl (
		.clk                  (clk),
		.clk_always_on        (clk_always_on),
		.rst_n                (rst_n),

		.addr                 (addr),
		.wtype                (wtype),
		.wen_m_mode           (wen_m_mode),
		.ren_m_mode           (ren_m_mode),
		.wdata_raw            (wdata),
		.wdata                (wdata_update),
		.rdata                (irq_ctrl_rdata),

		.trapreg_update_enter (trapreg_update_enter),
		.trapreg_update_exit  (trapreg_update_exit),
		.trap_entry_is_eirq   (mcause_irq_next && mcause_code_next == 4'hb),

		.meicontext_clearts   (meicontext_clearts),
		.mie_mtie             (mie[7]),
		.mie_msie             (mie[3]),

		.irq                  (irq),
		.external_irq_pending (external_irq_pending)
	);

end else begin: no_irq_ctrl

	reg external_irq_pending_r;

	always @ (posedge clk_always_on or negedge rst_n) begin
		if (!rst_n) begin
			external_irq_pending_r <= 1'b0;
		end else begin
			external_irq_pending_r <= |irq;
		end
	end

	assign irq_ctrl_rdata = {W_DATA{1'b0}};
	assign meicontext_clearts = 1'b0;
	assign external_irq_pending = external_irq_pending_r;

end
endgenerate

// ----------------------------------------------------------------------------
// 自定义休眠/电源控制 CSR

reg msleep_sleeponblock;
reg msleep_powerdown;
reg msleep_deepsleep;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		msleep_sleeponblock <= 1'b0;
		msleep_powerdown    <= 1'b0;
		msleep_deepsleep    <= 1'b0;
	end else if (wen_m_mode && addr == MSLEEP) begin
		msleep_sleeponblock <= wdata_update[2] && |EXTENSION_XH3POWER;
		msleep_powerdown    <= wdata_update[1] && |EXTENSION_XH3POWER;
		msleep_deepsleep    <= wdata_update[0] && |EXTENSION_XH3POWER;
	end
end

assign pwr_allow_sleep_on_block = msleep_sleeponblock;
assign pwr_allow_power_down = msleep_powerdown;
assign pwr_allow_clkgate = msleep_deepsleep;

// ----------------------------------------------------------------------------
// 自定义标识（ID）CSR

// 推导隐含的扩展:

// B 是 ZbaZbbZbs 的简写:
localparam [0:0]   EXTENSION_B     = |EXTENSION_ZBA && |EXTENSION_ZBB && |EXTENSION_ZBS;
// RV32I 与 RV32E 互斥:
localparam [0:0]   EXTENSION_I     = ~|EXTENSION_E;
// Zbkc 是 Zbc 的子集:
localparam [0:0]   EXTENSION_ZBKC  = |EXTENSION_ZBC;
// Zca 在指令层面是 C 的子集:
localparam [0:0]   EXTENSION_ZCA   = |EXTENSION_C;
// 除在顺序乘除电路上执行的 mulh、mulhsu 外，
// 所有 Zkt 指令均满足数据无关时序:
localparam [0:0]   EXTENSION_ZKT   = !(|EXTENSION_M && !(|MUL_FAST && |MULH_FAST));
// Zmmul 在指令层面是 M 的子集:
localparam [0:0]   EXTENSION_ZMMUL = |EXTENSION_M;

localparam [31:0]  h3misa_standard_len = 32'd77;

localparam [127:0] h3misa_standard_extensions =
	({127'd0, |EXTENSION_A       } << 0 ) |
	({127'd0, |EXTENSION_B       } << 1 ) |
	({127'd0, |EXTENSION_C       } << 2 ) |
	({127'd0, |EXTENSION_E       } << 4 ) |
	({127'd0, |EXTENSION_I       } << 8 ) |
	({127'd0, |EXTENSION_M       } << 12) |
	({127'd0, |EXTENSION_ZBA     } << 27) |
	({127'd0, |EXTENSION_ZBB     } << 28) |
	({127'd0, |EXTENSION_ZBC     } << 29) |
	({127'd0, |EXTENSION_ZBKB    } << 30) |
	({127'd0, |EXTENSION_ZBC     } << 31) |
	({127'd0, |EXTENSION_ZBKX    } << 32) |
	({127'd0, |EXTENSION_ZBS     } << 33) |
	({127'd0, |EXTENSION_ZKT     } << 46) |
	({127'd0, |EXTENSION_C       } << 66) |
	({127'd0, |EXTENSION_ZCB     } << 67) |
	({127'd0, |EXTENSION_ZILSD   } << 72) |
	({127'd0, |EXTENSION_ZCLSD   } << 73) |
	({127'd0, |EXTENSION_ZCMP    } << 74) |
	({127'd0, |EXTENSION_ZIFENCEI} << 75) |
	({127'd0, |EXTENSION_ZMMUL   } << 76) |
	128'd0;

localparam [31:0]  h3misa_custom_len = 32'd5;

localparam [255:0] h3misa_custom_extensions = {
	32'd0,                                       // 保留
	32'd0,                                       // 保留
	32'd0,                                       // 保留
	32'h01_00_00_00 & {32{|EXTENSION_XH3BEXTM}}, // Xh3bextm
	32'h01_00_00_00 & {32{|EXTENSION_XH3POWER}}, // Xh3power
	32'h01_00_00_00 & {32{|EXTENSION_XH3PMPM}},  // Xh3pmpm
	32'h01_00_00_00 & {32{|EXTENSION_XH3IRQ}},   // Xh3irq
	32'h01_00_00_00 & {32{|CSR_M_MANDATORY}}     // Xh3misa
};

wire [63:0] h3misa_info = {
	h3misa_custom_len,
	h3misa_standard_len
};

wire [31:0]  h3misa_rdata =
	!wdata[10] ? h3misa_standard_extensions[32 * wdata[1:0] +: 32] :
	!wdata[8]  ? h3misa_info               [32 * wdata[  0] +: 32] :
	             h3misa_custom_extensions  [32 * wdata[2:0] +: 32];

// ----------------------------------------------------------------------------
// 计数器

reg mcountinhibit_cy;
reg mcountinhibit_ir;

reg [XLEN-1:0] mcycleh;
reg [XLEN-1:0] mcycle;
reg [XLEN-1:0] minstreth;
reg [XLEN-1:0] minstret;

// 对任一半的写都会抑制整个 64 位寄存器的自增。
// 而不只是替换某一个 32 位半的值。参见:
//   https://github.com/riscv/riscv-isa-manual/issues/1255
wire mcycle_stopped = mcountinhibit_cy || debug_mode || wen_m_mode && (
	addr == MCYCLEH || addr == MCYCLE
);

wire minstret_stopped = mcountinhibit_ir || debug_mode || wen_m_mode && (
	addr == MINSTRETH || addr == MINSTRET
);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mcycleh <= X0;
		mcycle <= X0;
		minstreth <= X0;
		minstret <= X0;
		// 计数器默认被抑制以省电
		mcountinhibit_cy <= 1'b1;
		mcountinhibit_ir <= 1'b1;
	end else begin
		if (!mcycle_stopped) begin
			{mcycleh, mcycle} <=
				({mcycleh, mcycle} + 64'd1) & {2*XLEN{|CSR_COUNTER}};
		end
		if (instr_ret && !minstret_stopped) begin
			{minstreth, minstret} <=
				({minstreth, minstret} + 64'd1) & {2*XLEN{|CSR_COUNTER}};
		end
		if (wen_m_mode) begin
			if (addr == MCYCLEH) begin
				mcycleh <= wdata_update & {XLEN{|CSR_COUNTER}};
			end else if (addr == MCYCLE) begin
				mcycle <= wdata_update & {XLEN{|CSR_COUNTER}};
			end else if (addr == MINSTRETH) begin
				minstreth <= wdata_update & {XLEN{|CSR_COUNTER}};
			end else if (addr == MINSTRET) begin
				minstret <= wdata_update & {XLEN{|CSR_COUNTER}};
			end else if (addr == MCOUNTINHIBIT) begin
				{mcountinhibit_ir, mcountinhibit_cy} <= {wdata_update[2], wdata_update[0]} | {2{~|CSR_COUNTER}};
			end
		end
	end
end

// 注意尽管 time/timeh CSR 未实现，
// 我们仍实现 tm 位。tm 位提供一种标准方式，
// 用于配置 M 模式软件是否允许被陷入的访问到达定时器寄存器。
// （这实际上是 RVM-CSI 平台规范所要求的。）
reg mcounteren_cy;
reg mcounteren_tm;
reg mcounteren_ir;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mcounteren_cy <= 1'b0;
		mcounteren_tm <= 1'b0;
		mcounteren_ir <= 1'b0;
	end else if (wen_m_mode && addr == MCOUNTEREN) begin
		mcounteren_cy <= wdata_update[0] && |CSR_COUNTER && |U_MODE;
		mcounteren_tm <= wdata_update[1] && |CSR_COUNTER && |U_MODE;
		mcounteren_ir <= wdata_update[2] && |CSR_COUNTER && |U_MODE;
	end
end

// ----------------------------------------------------------------------------
// 调试模式 CSR

// 以下 DCSR 位可正常读写:
// - ebreakm（位 15）
// - ebreaku（位 12），若支持 U 模式
// - step（位 2）
// 以下为只读易变（volatile）:
// - cause（位 8:6）
// 其余均为硬连线常量。

reg dcsr_ebreakm;
reg dcsr_ebreaku;
reg dcsr_step;
reg [2:0] dcsr_cause;
wire [2:0] dcause_next;

localparam DCSR_CAUSE_EBREAK  = 3'h1;
localparam DCSR_CAUSE_TRIGGER = 3'h2;
localparam DCSR_CAUSE_HALTREQ = 3'h3;
localparam DCSR_CAUSE_STEP    = 3'h4;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		dcsr_ebreakm <= 1'b0;
		dcsr_ebreaku <= 1'b0;
		dcsr_step <= 1'b0;
		dcsr_cause <= 3'h0;
	end else begin
		// 注意调试模式中可以使用未门控的写使能，因为对有效的
		// D 模式 CSR 地址的 CSR 写保证不会在 D 模式
		// 产生（任何形式的，包括 PMP 的）异常。
		if (debug_mode && wen_raw && addr == DCSR) begin
			{dcsr_ebreakm, dcsr_ebreaku, dcsr_step} <= {
				wdata_update[15],
				wdata_update[12] && U_MODE,
				wdata_update[2]
			} & {3{|DEBUG_SUPPORT}};
		end
		if (enter_debug_mode) begin
			dcsr_cause <= dcause_next & {3{|DEBUG_SUPPORT}};
		end
	end
end

// DPC 在译码模块中映射到真实 PC。进入调试模式时
// 我们把它设为异常返回地址; 调试模式期间
// 可以像写 CSR 一样写它。注意只有按 IALIGN 对齐的值
// 才能写入 dpc。
assign debug_dpc_wdata = (debug_mode ? wdata_update           : mepc_in) & (~X0 << 2 - EXTENSION_C);
assign debug_dpc_wen   =  debug_mode ? wen_raw && addr == DPC : enter_debug_mode;

// 调试模块的 data0 寄存器映射进内核 CSR 空间:
assign dbg_data0_wdata = wdata;
assign dbg_data0_wen = debug_mode && wen_raw && addr == DMDATA0;

reg tcontrol_mte;
reg tcontrol_mpte;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		tcontrol_mpte <= 1'b0;
		tcontrol_mte <= 1'b0;
	end else if (wen_m_mode && addr == TCONTROL) begin
		tcontrol_mpte <= wdata_update[7] && DEBUG_SUPPORT;
		tcontrol_mte <= wdata_update[3] && DEBUG_SUPPORT;
	end else if (DEBUG_SUPPORT && trapreg_update_enter) begin
		tcontrol_mte <= 1'b0;
		tcontrol_mpte <= tcontrol_mte;
	end else if (DEBUG_SUPPORT && trapreg_update_exit) begin
		tcontrol_mte <= tcontrol_mpte;
		// 与 mstatus.mie/mpie 不同，tcontrol.mpte 不因陷阱退出而改变。
	end
end

// ----------------------------------------------------------------------------
// 读端口 + 检测未映射 CSR 的寻址
// ----------------------------------------------------------------------------

reg decode_match;

// 地址匹配条件 —— 某些 CSR 在其条件
// 不满足时不可访问:
wire match_drw = DEBUG_SUPPORT && debug_mode;
wire match_mrw = m_mode || debug_mode;
wire match_mro = (m_mode || debug_mode) && !wen_soon;
wire match_urw = U_MODE && 1'b1;
wire match_uro = U_MODE && !wen_soon;

always @ (*) begin
	decode_match = 1'b0;
	rdata = {XLEN{1'b0}};
	pmp_cfg_wen = 1'b0;
	trig_cfg_wen = 1'b0;
	case (addr)

	// ------------------------------------------------------------------------
	// 强制（必备）CSR

	MISA: if (CSR_M_MANDATORY) begin
		// WARL，因此接常量是合法的
		decode_match = match_mrw;
		rdata = MISA_VAL;
	end
	MVENDORID: if (CSR_M_MANDATORY) begin
		decode_match = match_mro;
		rdata = MVENDORID_VAL;
	end
	MARCHID: if (CSR_M_MANDATORY) begin
		decode_match = match_mro;
		rdata = 32'd0;
	end
	MIMPID: if (CSR_M_MANDATORY) begin
		decode_match = match_mro;
		rdata = mimpid_rdata;
	end
	MHARTID: if (CSR_M_MANDATORY) begin
		decode_match = match_mro;
		rdata = mhartid_val;
	end

	MCONFIGPTR: if (CSR_M_MANDATORY) begin
		decode_match = match_mro;
		rdata = MCONFIGPTR_VAL;
	end

	MSTATUS: if (CSR_M_MANDATORY || CSR_M_TRAP) begin
		decode_match = match_mrw;
		rdata = {
			1'b0,             // 除 GPR 外从无脏状态
			8'd0,             // (WPRI)
			1'b0,             // TSR（Trap SRET），无 S 模式时接 0。
			mstatus_tw,       // TW（Timeout Wait）
			1'b0,             // TVM（trap virtual memory），无 S 模式时接 0。
			1'b0,             // MXR（Make eXecutable Readable），无 S 模式时接 0。
			1'b0,             // SUM，无 S 模式时接 0
			mstatus_mprv,     // MPRV（modify privilege，修改特权）
			4'd0,             // XS、FS 始终为 "off"（没有需要清除的扩展状态！）
			{2{mstatus_mpp}}, // MPP（M 模式先前特权），只支持 M 和 U
			2'd0,             // (WPRI)
			1'b0,             // SPP，不支持 S 模式时接 0
			mstatus_mpie,     // 先前中断使能
			3'd0,             // 无 S、U
			mstatus_mie,      // 中断使能
			3'd0              // 无 S、U
		};
	end

	// MSTATUSH 全为 0（字段为 MBE 与 SBE，
	// 因为我们是纯小端。）在 priv-1.12 之前，这种情况本可以不实现
	// MSTATUSH，但现在即使
	// 硬连线为 0 也必须译码。
	MSTATUSH: if (CSR_M_MANDATORY || CSR_M_TRAP) begin
		decode_match = match_mrw;
	end

	// 无 S 模式的实现不应存在 MEDELEG、MIDELEG。
	// 访问它们会触发非法指令异常。

	// 自 M 模式规范 v1.12 起，若实现了 U 模式，
	// MENVCFG 似乎是必备的。本实现中其所有字段都接 0。
	MENVCFGH: if (U_MODE) begin
		decode_match = match_mrw;
	end
	MENVCFG: if (U_MODE) begin
		decode_match = match_mrw;
	end

	// ------------------------------------------------------------------------
	// 陷阱处理 CSR

	// 这是一个带置位/清零/写/读的 32 位合成寄存器，
	// 除非迫不得已，不要启用它
	MSCRATCH: if (CSR_M_TRAP && CSR_M_MANDATORY) begin
		decode_match = match_mrw;
		rdata = mscratch;
	end

	MEPC: if (CSR_M_TRAP) begin
		decode_match = match_mrw;
		rdata = mepc;
	end

	MCAUSE: if (CSR_M_TRAP) begin
		decode_match = match_mrw;
		rdata = {
			mcause_irq,      // 符号位为 1 表示 IRQ，0 表示异常
			{27{1'b0}},      // 填充
			mcause_code[3:0] // 足以支持 16 个外部 IRQ，这也是 mip/mie 的容量上限
		};
	end

	MTVAL: if (CSR_M_TRAP) begin
		decode_match = match_mrw;
		// 硬连线为 0
	end

	MIE: if (CSR_M_TRAP) begin
		decode_match = match_mrw;
		rdata = mie;
	end

	MIP: if (CSR_M_TRAP) begin
		// 允许写入，但被忽略。
		decode_match = match_mrw;
		rdata = mip;
	end

	MTVEC: if (CSR_M_TRAP) begin
		decode_match = match_mrw;
		rdata = {
			mtvec[XLEN-1:2],  // BASE
			1'b0,             // 保留的模式位被 WARL 为 0
			irq_vector_enable // MODE 为向量模式（2'h1）或直接模式（2'h0）
		};
	end

	// ------------------------------------------------------------------------
	// 计数器 CSR

	// 先把接常量的 WARL 处理掉
	MHPMCOUNTER3:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER4:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER5:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER6:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER7:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER8:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER9:   if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER10:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER11:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER12:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER13:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER14:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER15:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER16:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER17:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER18:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER19:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER20:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER21:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER22:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER23:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER24:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER25:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER26:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER27:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER28:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER29:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER30:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER31:  if (CSR_COUNTER) begin decode_match = match_mrw; end

	MHPMCOUNTER3H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER4H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER5H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER6H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER7H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER8H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER9H:  if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER10H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER11H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER12H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER13H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER14H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER15H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER16H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER17H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER18H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER19H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER20H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER21H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER22H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER23H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER24H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER25H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER26H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER27H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER28H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER29H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER30H: if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMCOUNTER31H: if (CSR_COUNTER) begin decode_match = match_mrw; end

	MHPMEVENT3:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT4:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT5:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT6:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT7:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT8:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT9:     if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT10:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT11:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT12:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT13:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT14:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT15:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT16:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT17:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT18:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT19:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT20:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT21:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT22:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT23:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT24:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT25:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT26:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT27:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT28:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT29:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT30:    if (CSR_COUNTER) begin decode_match = match_mrw; end
	MHPMEVENT31:    if (CSR_COUNTER) begin decode_match = match_mrw; end

	MCOUNTINHIBIT:  if (CSR_COUNTER) begin
		decode_match = match_mrw;
		rdata = {
			29'd0,
			mcountinhibit_ir,
			1'b0,
			mcountinhibit_cy
		};
	end

	MCYCLE: if (CSR_COUNTER) begin
		decode_match = match_mrw;
		rdata = mcycle;
	end
	MINSTRET: if (CSR_COUNTER) begin
		decode_match = match_mrw;
		rdata = minstret;
	end

	MCYCLEH: if (CSR_COUNTER) begin
		decode_match = match_mrw;
		rdata = mcycleh;
	end
	MINSTRETH: if (CSR_COUNTER) begin
		decode_match = match_mrw;
		rdata = minstreth;
	end

	MCOUNTEREN: if (CSR_COUNTER && U_MODE) begin
		decode_match = match_mrw;
		rdata = {
			29'd0,
			mcounteren_ir,
			mcounteren_tm,
			mcounteren_cy
		};
	end

	// ------------------------------------------------------------------------
	// PMP CSR（到 PMP 配置接口的桥接）

	// 若存在 PMP，则 16 个寄存器都存在，但部分寄存器可能按实际
	// 实现的区域数被 WARL 为 0。
	PMPCFG0:   if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPCFG1:   if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPCFG2:   if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPCFG3:   if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR0:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR1:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR2:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR3:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR4:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR5:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR6:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR7:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR8:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR9:  if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR10: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR11: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR12: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR13: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR14: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end
	PMPADDR15: if (PMP_REGIONS > 0) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end

	// MSECCFG 严格可选，且我们不实现其任何
	// 特性（ePMP 等），因此不译码它。

	// ------------------------------------------------------------------------
	// U 模式 CSR

	// 只读计数器始终对 M 模式可见，若对应 mcounteren 位置起
	// 则对 U 模式也可见。
	CYCLE: if (CSR_COUNTER) begin
		decode_match = mcounteren_cy ? match_uro : match_mro;
		rdata = mcycle;
	end
	CYCLEH: if (CSR_COUNTER) begin
		decode_match = mcounteren_cy ? match_uro : match_mro;
		rdata = mcycleh;
	end

	INSTRET: if (CSR_COUNTER) begin
		decode_match = mcounteren_ir ? match_uro : match_mro;
		rdata = minstret;
	end
	INSTRETH: if (CSR_COUNTER) begin
		decode_match = mcounteren_ir ? match_uro : match_mro;
		rdata = minstreth;
	end

	// ZIGUANG: 自定义 PMU 计数器 (M-mode RO) 与 mpmuctrl (M-mode RW)
	ZPMU_MBRANCH: begin
		decode_match = match_mro;
		rdata = zg_pmu_branch_r[31:0];
	end
	ZPMU_MBRANCHH: begin
		decode_match = match_mro;
		rdata = zg_pmu_branch_r[63:32];
	end
	ZPMU_MBPMISS: begin
		decode_match = match_mro;
		rdata = zg_pmu_bpmiss_r[31:0];
	end
	ZPMU_MBPMISSH: begin
		decode_match = match_mro;
		rdata = zg_pmu_bpmiss_r[63:32];
	end
	ZPMU_MICACCESS: begin
		decode_match = match_mro;
		rdata = zg_pmu_icaccess_r[31:0];
	end
	ZPMU_MICACCESSH: begin
		decode_match = match_mro;
		rdata = zg_pmu_icaccess_r[63:32];
	end
	ZPMU_MICMISS: begin
		decode_match = match_mro;
		rdata = zg_pmu_icmiss_r[31:0];
	end
	ZPMU_MICMISS_H: begin
		decode_match = match_mro;
		rdata = zg_pmu_icmiss_r[63:32];
	end
	ZPMU_MDCACCESS: begin
		decode_match = match_mro;
		rdata = zg_pmu_dcaccess_r[31:0];
	end
	ZPMU_MDCACCESSH: begin
		decode_match = match_mro;
		rdata = zg_pmu_dcaccess_r[63:32];
	end
	ZPMU_MDCMISS: begin
		decode_match = match_mro;
		rdata = zg_pmu_dcmiss_r[31:0];
	end
	ZPMU_MDCMISS_H: begin
		decode_match = match_mro;
		rdata = zg_pmu_dcmiss_r[63:32];
	end
	ZPMU_MSTALL: begin
		decode_match = match_mro;
		rdata = zg_pmu_stall_r[31:0];
	end
	ZPMU_MSTALLH: begin
		decode_match = match_mro;
		rdata = zg_pmu_stall_r[63:32];
	end
	ZPMU_MBUSWAIT: begin
		decode_match = match_mro;
		rdata = zg_pmu_buswait_r[31:0];
	end
	ZPMU_MBUSWAITH: begin
		decode_match = match_mro;
		rdata = zg_pmu_buswait_r[63:32];
	end
	ZPMU_CTRL: begin
		decode_match = match_mrw;
		rdata = {{(XLEN-1){1'b0}}, zg_pmu_en};
	end

	// ------------------------------------------------------------------------
	// 触发模块 CSR

	// 使能调试支持时，我们始终具备基本触发支持
	// （指令计数、中断与异常）。断点为可选。
	TSELECT: if (DEBUG_SUPPORT) begin
		decode_match = match_mrw;
		trig_cfg_wen = match_mrw && wen;
		rdata = trig_cfg_rdata;
	end

	TDATA1: if (DEBUG_SUPPORT) begin
		decode_match = match_mrw;
		trig_cfg_wen = match_mrw && wen;
		rdata = trig_cfg_rdata;
	end

	TDATA2: if (DEBUG_SUPPORT) begin
		decode_match = match_mrw;
		trig_cfg_wen = match_mrw && wen;
		rdata = trig_cfg_rdata;
	end

	TDATA3: if (DEBUG_SUPPORT) begin
		// 未实现但必须可访问，因此硬连线为 0。
		decode_match = match_mrw;
		rdata = 32'h00000000;
	end

	TINFO: if (DEBUG_SUPPORT) begin
		// 注意 tinfo 是读写 CSR（写不陷入异常），尽管它
		// 完全是只读的。
		decode_match = match_mrw;
		trig_cfg_wen = match_mrw && wen;
		rdata = trig_cfg_rdata;
	end

	TCONTROL: if (DEBUG_SUPPORT) begin
		decode_match = match_mrw;
		rdata = {
			24'h0,
			tcontrol_mpte,
			3'h0,
			tcontrol_mte,
			3'h0
		};
	end

	// ------------------------------------------------------------------------
	// 调试 CSR

	DCSR: if (DEBUG_SUPPORT) begin
		decode_match = match_drw;
		rdata = {
			4'h4,         // xdebugver = 4，对应 0.13.2 调试规范
			12'd0,        // reserved
			dcsr_ebreakm,
			2'h0,         // 无模式 2/1
			dcsr_ebreaku,
			1'b0,         // stepie = 0，单步模式下无中断
			1'b1,         // stopcount = 1，调试模式下计数器不自增
			1'b1,         // stoptime = 0，调试模式下核内定时器不自增
			dcsr_cause,
			1'b0,         // reserved
			1'b0,         // mprven = 0，调试器始终以 M 模式行事
			1'b0,         // nmip = 0，我们没有 NMI
			dcsr_step,
			{2{m_mode}}   // priv = M 或 U，直接报告内核状态
		};
	end

	DPC: if (DEBUG_SUPPORT) begin
		decode_match = match_drw;
		rdata = debug_dpc_rdata;
	end

	DMDATA0: if (DEBUG_SUPPORT) begin
		decode_match = match_drw;
		rdata = dbg_data0_rdata;
	end

	// ------------------------------------------------------------------------
	// 自定义 CSR

	PMPCFGM0: if (PMP_REGIONS > 0 && EXTENSION_XH3PMPM) begin
		decode_match = match_mrw;
		pmp_cfg_wen = match_mrw && wen;
		rdata = pmp_cfg_rdata;
	end

	MEIEA: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MEIPA: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MEIFA: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MEIPRA: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MEINEXT: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MEICONTEXT: if (CSR_M_TRAP && EXTENSION_XH3IRQ) begin
		decode_match = match_mrw;
		rdata = irq_ctrl_rdata;
	end

	MSLEEP: if (EXTENSION_XH3POWER) begin
		decode_match = match_mrw;
		rdata = {
			29'h0,
			msleep_sleeponblock,
			msleep_powerdown,
			msleep_deepsleep
		};
	end

	H3MISA: if (CSR_M_MANDATORY) begin
		decode_match = match_mrw;
		rdata = h3misa_rdata;
	end

	default: begin end
	endcase
end

assign illegal = (wen_soon || ren_soon) && !decode_match;

// 某些 CSR 更新时触发顺序下一条指令的重取:
assign write_is_fetch_ordered = wen_raw && !debug_mode && (
	(PMP_REGIONS   > 0 && addr >= PMPADDR0 && addr <= PMPADDR15) ||
	(PMP_REGIONS   > 0 && addr >= PMPCFG0  && addr <= PMPCFG3  ) ||
	(DEBUG_SUPPORT > 0 && addr == TDATA1                       ) ||
	(DEBUG_SUPPORT > 0 && addr == TDATA2                       ) ||
	(DEBUG_SUPPORT > 0 && addr == TCONTROL                     )
);

// ----------------------------------------------------------------------------
// 调试运行/停机

// req_resume_prev 用于切断从请求到陷阱地址的入->出路径。
reg have_just_reset;
reg step_halt_req;
reg dbg_req_resume_prev;
reg dbg_req_halt_prev;
reg dbg_req_halt_on_reset_sticky;
reg pending_dbg_resume_prev;

wire pending_dbg_resume = (pending_dbg_resume_prev || dbg_req_resume_prev) && debug_mode;

// 注意异常进入也算一步 —— 这种情况下应先陷入陷阱，
// 然后在执行第一条陷阱指令之前打断进入调试模式。
// step 为 1 时不发生 *IRQ* 进入（因为我们把
// dcsr.stepie 硬连线为 0）
wire step_event = instr_ret || (trap_enter_vld && trap_enter_rdy);

// 停机请求输入寄存器必须始终有时钟，因为当调试器请求
// 停机一个休眠中的内核时，WFI 需要能被穿透。
always @ (posedge clk_always_on or negedge rst_n) begin
	if (!rst_n) begin
		dbg_req_halt_prev <= 1'b0;
	end else begin
		// 只是内核外部请求的延迟版本。
		// 延迟没有问题，因为 DM 在撤销请求前会等待 ack。
		dbg_req_halt_prev <= dbg_req_halt && DEBUG_SUPPORT != 0;
	end
end

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		have_just_reset <= |DEBUG_SUPPORT;
		dbg_req_halt_on_reset_sticky <= 1'b0;
		step_halt_req <= 1'b0;
		dbg_req_resume_prev <= 1'b0;
		pending_dbg_resume_prev <= 1'b0;
	end else if (DEBUG_SUPPORT) begin
		have_just_reset <= 1'b0;

		// 复位停机请求置起上的一拍延迟无害，
		// 因为第一条取指上有匹配的延迟（reset_holdoff）。
		if (have_just_reset && dbg_req_halt_on_reset) begin
			dbg_req_halt_on_reset_sticky <= 1'b1;
		end else if (enter_debug_mode) begin
			dbg_req_halt_on_reset_sticky <= 1'b0;
		end

		dbg_req_resume_prev <= dbg_req_resume;

		if (debug_mode) begin
			step_halt_req <= 1'b0;
		end else if (dcsr_step && step_event) begin
			step_halt_req <= 1'b1;
		end

		pending_dbg_resume_prev <= pending_dbg_resume;
	end
end

// 我们可以类 IRQ 方式进入停机状态（插在第 2 级与
// 第 3 级指令之间），也可以类异常方式
// （取代第 3 级的指令）。
//
// 停机请求与单步打断是类 IRQ 的。当停机请求
// 恰好对上 M 级中一条已产生异常（如 ecall）
// 或尚可能产生异常（load）的指令时须谨慎。
// 此时正确做法是:
//
// - 取消 X 级中可能的指令，并抑制 X 的 PC 自增
// - 等待异常进入完成（或该 load/store
//   成功完成）之后
// - 立即触发类 IRQ 的调试进入。
//
// 这保证若该指令产生异常，调试器看到的 mcause/mepc 设置正确，
// 且 dpc 指向处理程序入口。

wire exception_req_any;
wire halt_delayed_by_exception = exception_req_any || loadstore_dphase_pending;

wire want_halt_except = DEBUG_SUPPORT && !debug_mode && (
	dcsr_ebreakm &&  m_mode && except == EXCEPT_EBREAK ||
	dcsr_ebreaku && !m_mode && except == EXCEPT_EBREAK ||
	except_to_d_mode        && except == EXCEPT_EBREAK
);

// 注意类异常原因（触发器、ebreak）的优先级高于
// 类 IRQ 原因。
//
// 必须用 delay_irq_entry（load/store 地址相第 2 拍及以后为真）
// 屏蔽 halt_req，因为那时已无法抑制总线访问。
// 其余原因没有问题，因为它们不会出现在第 2 级指令中途。
wire want_halt_irq_if_no_exception = DEBUG_SUPPORT && !debug_mode && !want_halt_except && (
	(dbg_req_halt_prev && !delay_irq_entry) ||
	dbg_req_halt_on_reset_sticky ||
	step_halt_req
);

// M 级中的异常（或因 load/store 而可能的异常）会延迟停机
// 进入。停机意图仍会阻塞 X，因此不会被一串
// load/store 无限阻塞。这里只是为了给 M 级中的
// 异常（或可能的异常）与调试模式进入排出先后。
wire want_halt_irq = want_halt_irq_if_no_exception && !halt_delayed_by_exception;

assign dcause_next =
	except == EXCEPT_EBREAK && except_to_d_mode ? 3'h2 : // 触发器（优先级 4）
	except == EXCEPT_EBREAK                     ? 3'h1 : // ebreak（优先级 3）
	dbg_req_halt_prev || dbg_req_halt_on_reset  ? 3'h3 : // 停机或复位停机（优先级 1、2）
	                                              3'h4;  // 单步（优先级 0）

wire trap_is_debug_entry = |DEBUG_SUPPORT && !debug_mode && (want_halt_irq || want_halt_except);
assign enter_debug_mode = trap_is_debug_entry && trap_enter_rdy;

assign exit_debug_mode = debug_mode && pending_dbg_resume && trap_enter_rdy;

// 向 DM 指令注入器报告其指令序列
// 已结束（ebreak）或已异常退出
assign dbg_instr_caught_ebreak = debug_mode && except == EXCEPT_EBREAK && trap_enter_rdy;

// 注意无论 dcsr.ebreakm 为何，这里都排除 ebreak，
// 因为此时已经处于调试模式
assign dbg_instr_caught_exception = debug_mode && except != EXCEPT_NONE &&
	except != EXCEPT_EBREAK && trap_enter_rdy;

// ----------------------------------------------------------------------------
// 陷阱请求生成

reg irq_software_r;
reg irq_timer_r;

// 始终有时钟，因为它用于产生唤醒。
always @ (posedge clk_always_on or negedge rst_n) begin
	if (!rst_n) begin
		irq_software_r <= 1'b0;
		irq_timer_r <= 1'b0;
	end else begin
		irq_software_r <= irq_software;
		irq_timer_r <= irq_timer;
	end
end

assign mip = {
	20'h0,                // 保留
	external_irq_pending, // meip，外部 IRQ 的全局未决位
	3'h0,                 // 保留
	irq_timer_r,          // mtip，来自内存映射定时器外设的中断
	3'h0,                 // 保留
	irq_software_r,       // msip，来自内存映射寄存器的软件中断
	3'h0                  // 保留
};

wire irq_active = |(mip & mie) && (mstatus_mie || !m_mode) && !dcsr_step;

// WFI 清除会考虑各中断使能位，但忽略 mstatus.mie。
// 此外，单步执行与 D 模式期间 wfi 被当作 nop。
// 注意 IRQ 与调试停机请求的输入寄存器由
// clk_always_on 供时钟，以便休眠时也能产生唤醒。还要注意
// want_halt_irq_if_no_exception 在休眠时可能被屏蔽，
// 因此还要把原始停机请求作为唤醒源接入。
assign pwr_wfi_wakeup_req = |(mip & mie) || dcsr_step || debug_mode ||
	want_halt_irq_if_no_exception || dbg_req_halt_prev;

// 特权规范规定的优先级: 外部 > 软件 > 定时器
wire [3:0] standard_irq_num =
	mip[11] && mie[11] ? 4'd11 :
	mip[3]  && mie[3]  ? 4'd3  :
	mip[7]  && mie[7]  ? 4'd7  : 4'd0;

// ebreak 依据 dcsr.ebreakm 可以被处理为向调试器停机，
// 或普通 M 模式异常。
assign exception_req_any = except != EXCEPT_NONE && !(
	except == EXCEPT_EBREAK && (m_mode ? dcsr_ebreakm : dcsr_ebreaku)
);

wire [3:0] mcause_irq_num =	irq_active ? standard_irq_num : 4'd0;

wire [3:0] vector_sel =	!exception_req_any && irq_vector_enable ? mcause_irq_num : 4'd0;

// 注意重取情形下我们依赖 dpc 与 pc 的别名关系
// （意图是再次取顺序下一条指令）
assign trap_addr =
	except == EXCEPT_MRET    ? mepc            :
	except == EXCEPT_REFETCH ? debug_dpc_rdata :
	pending_dbg_resume       ? debug_dpc_rdata :
	                           mtvec | {26'h0, vector_sel, 2'h0};

// 检查类异常或类 IRQ 的陷阱进入; 任何调试模式进入
// 都优先于任何普通陷阱。
assign trap_is_irq = DEBUG_SUPPORT && (want_halt_except || want_halt_irq) ?
	!want_halt_except : !exception_req_any;

// 当有 load/store 数据相未决时，我们阻塞 IRQ 陷阱
// 请求，但仍置起 trap_enter_soon 标志。这会阻止尚未
// 锁定到地址相的后续 load/store 发出，使 IRQ
// 最终能够通过。数据相进行中进入 IRQ 不安全，
// 因为该数据相随后可能产生异常，从而采样到
// IRQ 向量 PC 而不是 load/store 指令 PC。

assign trap_enter_vld =
	CSR_M_TRAP && (exception_req_any ||
		!delay_irq_entry && !debug_mode && irq_active && !loadstore_dphase_pending) ||
	DEBUG_SUPPORT && (
		(!delay_irq_entry && want_halt_irq) || want_halt_except || pending_dbg_resume);

assign trap_enter_soon = trap_enter_vld || (
	|DEBUG_SUPPORT && want_halt_irq_if_no_exception ||
	!delay_irq_entry && !debug_mode && irq_active && loadstore_dphase_pending
);

assign mcause_irq_next = !exception_req_any;
assign mcause_code_next = exception_req_any ? except : mcause_irq_num;

// 向触发单元报告陷阱进入
assign trig_event_exception = |DEBUG_SUPPORT && trapreg_update_enter && !trap_is_irq;
assign trig_event_interrupt = |DEBUG_SUPPORT && trapreg_update_enter &&  trap_is_irq;
assign trig_event_trap_cause = {4{|DEBUG_SUPPORT}} & mcause_code_next;

// ----------------------------------------------------------------------------
// 特权状态输出

assign pmp_cfg_addr = addr;
assign pmp_cfg_wdata = wdata_update;

assign trig_cfg_addr = addr;
assign trig_cfg_wdata = wdata_update;
assign trig_m_en = tcontrol_mte;

// 执行用有效特权。用于:
// - 分支目标取指的特权级（顺序取指期间前端保持
//   取指特权不变）
// - 对照 PMP 执行权限检查 PC

assign m_mode_execution = !U_MODE || debug_mode || m_mode;

// 陷阱进入用有效特权。用于:
// - 陷阱目标取指的特权级（顺序取指期间前端保持
//   取指特权不变）

assign m_mode_trap_entry = !U_MODE || (
	except == EXCEPT_MRET ? mstatus_mpp : 1'b1
);

// load/store 用有效特权。用于:
// - 总线上 load/store 的特权级
// - 对照 PMP 读写权限检查 load/store

assign m_mode_loadstore = !U_MODE || debug_mode || (
	mstatus_mprv ? mstatus_mpp : m_mode
);

// ----------------------------------------------------------------------------
// 属性（Properties）

`ifdef OMNICORE_ASSERTIONS

// 跟踪是否处于陷阱中（仅为形式化属性用途）
reg in_trap;

always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		in_trap <= 1'b0;
	else
		in_trap <= (in_trap || (trap_enter_vld && trap_enter_rdy))
			&& !(trap_enter_vld && trap_enter_rdy && except == EXCEPT_MRET);

always @ (posedge clk) begin
`ifdef RISCV_FORMAL
	// 假定不存在嵌套异常，避免 riscv-formal 做出
	// 恼人的举动，例如通过反复
	// 注入非法指令来阻止指令退休
	if (in_trap)
		assume(except == EXCEPT_NONE || except == EXCEPT_MRET);

	// 注意实际的 IRQ 寄存器已移入 ominicore_irq_ctrl
	// （并且变为可选），因此此处为属性重复声明
	reg [NUM_IRQS-1:0] irq_r;
	always @ (posedge clk_always_on or negedge rst_n) begin
		if (!rst_n) begin
			irq_r <= {NUM_IRQS{1'b0}};
		end else begin
			irq_r <= irq;
		end
	end

	// 假定在未发生异常进入的拍上，IRQ 不会
	// 被撤销
	if (!trap_enter_rdy)
		assume(~|(irq_r & ~irq));
`endif

	// 确保 CSR 访问被冲刷
	if (trap_enter_vld && trap_enter_rdy)
		assert(!(wen || ren));

	// 陷阱进入的下一拍仍在写 CSR —— 应当不可能: CSR
	// 访问指令应已被冲刷或推进到第 3 级，
	// 且此刻不可能有新取指到达
	if ($past(trap_enter_vld && trap_enter_rdy))
		assert(!wen);

	// 进入陷阱后立即退出应当不可能:
	if (in_trap && !$past(in_trap))
		assert(except != EXCEPT_MRET);

	// 退出后这么快又到另一个 mret 应当不可能:
	if ($past(except == EXCEPT_MRET && trap_enter_vld && trap_enter_rdy))
		assert(except != EXCEPT_MRET);

	// 相邻两拍各进入一个陷阱必须不可能。最重要的是:
	//
	// - IRQ -> 异常: 到这时不可能已取到新指令，
	//   而遗留的 store 数据相等产生的异常会采样到
	//   错误的 PC。
	//
	// - 异常 -> IRQ: 需要先重新置起 mstatus.mie，不应发生
	//
	// 唯一的例外是 mret。我们可以从中断/异常返回，
	// 并在下一拍接受新的中断。

	if ($past(trap_enter_vld && trap_enter_rdy && except != EXCEPT_MRET))
		assert(!(trap_enter_vld && trap_enter_rdy));

	// 只是想强调这是唯一允许的情形:
	if (trap_enter_vld && trap_enter_rdy && $past(trap_enter_vld && trap_enter_rdy))
		assert($past(except == EXCEPT_MRET));

	if (rst_n && $past(trap_enter_vld && !trap_enter_rdy && !trap_is_irq)) begin
		// 未进入的异常不应消失
		assert(trap_enter_vld);
		// 异常不应被 IRQ 取代
		assert(!trap_is_irq);
	end

	// 这必须是断点异常（大概来自触发单元）。
	if (except_to_d_mode) begin
		assert(except == EXCEPT_EBREAK);
	end

	// 异常应在 load/store 到达数据相之前将其取消
	if (except != EXCEPT_NONE && !(except == EXCEPT_LOAD_FAULT || except == EXCEPT_STORE_FAULT)) begin
		assert(!loadstore_dphase_pending);
	end

	// 调试进入也是陷阱进入！（我们还应在 core.v 中断言
	// 陷阱进入绝不与活跃的 load/store 地址相
	// 重合，这能部分确信调试进入不会吞掉
	// load/store。）
	if (enter_debug_mode) begin
		assert(trap_enter_vld);
	end
end

`endif

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
