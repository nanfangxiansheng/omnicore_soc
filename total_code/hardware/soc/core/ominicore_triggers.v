

`default_nettype none

// OmniCore 触发单元对以下每种类型都各实现一个触发器:
//
//
// * 指令计数触发器（type=3，count=1）（可在 M 模式下单步调试 U 模式，
//   或在 M 模式异常处理程序中单步调试 M 模式前台代码）
//
// * 中断触发器（type=4）: 按 mtip/msip/meip 中断掩码触发
//
// * 异常触发器（type=5）: 按异常原因掩码触发
//
// 以下为可选支持:
//
// * 指令地址触发器（type=2 execute=1 select=0），即断点
//
// 断点始终使用精确地址匹配，时序始终为
// “提前”（early）。断点数量由 BREAKPOINT_TRIGGERS 配置，
// 可以为 0。
//
// 中断/异常触发器在内核转移到陷阱处理程序之后、
// 第一条陷阱处理指令执行之前打断。这些触发器
// 仅支持 action=1，因为当 M 是唯一特权模式时，
// 陷阱中再触发陷阱毫无用处。

module ominicore_triggers #(
`include "ominicore_config.vh"
) (
	input  wire              clk,
	input  wire              rst_n,

	// 经 CSR 块透传的配置接口
	input  wire [11:0]       cfg_addr,
	input  wire              cfg_wen,
	input  wire [W_DATA-1:0] cfg_wdata,
	output reg  [W_DATA-1:0] cfg_rdata,

	// 来自 tcontrol 的触发器到 M 模式的全局使能
	input  wire              trig_m_en,

	// 来自 F 级的取指地址查询
	input  wire [W_ADDR-1:0] fetch_addr,
	input  wire              fetch_m_mode,
	input  wire              fetch_d_mode,

	// 来自 X 级的陷阱触发事件
	input  wire              event_instr_ret,

	// 来自 M 级的陷阱触发事件
	input  wire              event_interrupt,
	input  wire              event_exception,
	input  wire [3:0]        event_trap_cause,
	input  wire              event_trap_enter,

	// 按 F 级对齐的打断请求（针对按字大小、字对齐取指的每个半字）
	output wire [1:0]        break_any,
	output wire [1:0]        break_d_mode,

	// 按 X 级对齐的单步打断请求（仅面向 M 模式）
	output wire              break_m_step,

	// X 级的调试模式标志，用于 CSR 保护（可能与查询
	// 用的调试模式标志相同，也可能不同）
	input  wire              x_d_mode,
	// X 级的 M 模式标志，用于中断/异常触发器的使能
	input  wire              x_m_mode
);

`include "ominicore_csr_addr.vh"

generate
if (DEBUG_SUPPORT == 0) begin: no_triggers

// 若无触发器，本模块的例化在 core.v 中应当已被桩掉，
// 但以 0 个触发器详细展开本模块时仍会产生警告，
// 因此这里也加一个 generate 块。

always @ (*) cfg_rdata = {W_DATA{1'b0}};
assign break_any = 1'b0;
assign break_d_mode = 1'b0;

end else begin: have_triggers

localparam TINDEX_ICOUNT     = BREAKPOINT_TRIGGERS + 0;
localparam TINDEX_INTERRUPT  = BREAKPOINT_TRIGGERS + 1;
localparam TINDEX_EXCEPTION  = BREAKPOINT_TRIGGERS + 2;
localparam N_TRIGGERS        = BREAKPOINT_TRIGGERS + 3;

// 即使没有断点，也保留一个哑寄存器（硬连线为 0），
// 用于满足 Verilog 语法处理。它对综合没有影响。
localparam N_BREAKPOINT_REGS = BREAKPOINT_TRIGGERS > 0 ? BREAKPOINT_TRIGGERS : 1;

// ----------------------------------------------------------------------------
// 配置状态

localparam W_TSELECT = $clog2(N_TRIGGERS);

reg [W_TSELECT-1:0] tselect;

// 注意 tdata1 与 mcontrol 是同一个 CSR。tdata1 指通用
// 字段（type/dmode），mcontrol 指 type=2 特有的字段
// （地址/数据匹配），这是我们实现的唯一触发器类型。

// 指令地址匹配触发器（断点）的状态。
reg              bp_tdata1_dmode  [0:N_BREAKPOINT_REGS-1];
reg              mcontrol_action  [0:N_BREAKPOINT_REGS-1];
reg              mcontrol_m       [0:N_BREAKPOINT_REGS-1];
reg              mcontrol_u       [0:N_BREAKPOINT_REGS-1];
reg              mcontrol_execute [0:N_BREAKPOINT_REGS-1];
reg [W_DATA-1:0] bp_tdata2        [0:N_BREAKPOINT_REGS-1];

// 指令计数触发器的状态
// （硬连线: count=1 dmode=0 action=0；调试模式单步
// 已经可以通过 dcsr 实现）
reg             icount_m;
reg             icount_u;

// 中断触发器的状态
// （硬连线: action=1；M 模式下陷阱中再触发陷阱没有意义，
// 因为会丢失原始陷阱状态）
reg             trigger_irq_m;
reg             trigger_irq_u;
reg             trigger_irq_dmode;
reg [15:0]      trigger_irq_cause;

localparam [15:0] IMPLEMENTED_IRQ_CAUSES = {
	4'h0, // reserved
	1'b1, // meip
	3'h0, // 保留或未实现
	1'b1, // mtip
	3'h0, // 保留或未实现
	1'b1, // msip
	3'h0  // 保留或未实现
};

// 异常触发器的状态
// （硬连线: action=1；M 模式下陷阱中再触发陷阱没有意义，
// 因为会丢失原始陷阱状态）
reg             trigger_exception_m;
reg             trigger_exception_u;
reg             trigger_exception_dmode;
reg [15:0]      trigger_exception_cause;

localparam [15:0] IMPLEMENTED_EXCEPTION_CAUSES = {
	4'h0,         // reserved
	1'b1,         // 11 -> M 模式 ecall
	2'h0,         // 保留或未实现
	|U_MODE,      // 8  -> U 模式 ecall
	1'b1,         // 7  -> store/AMO 访问错误
	1'b1,         // 6  -> store/AMO 非对齐
	1'b1,         // 5  -> load 访问错误
	1'b1,         // 4  -> load 非对齐
	1'b0,         // 3  -> 断点；看似无用且有风险，故不允许
	1'b1,         // 2  -> 非法指令
	1'b1,         // 1  -> 取指访问错误
	~|EXTENSION_C // 0  -> 取指非对齐（仅当 IALIGN 为 32 位时）
};

// ----------------------------------------------------------------------------
// 配置写端口

localparam N_TRIGGERS_PADDED = 1 << $clog2(N_TRIGGERS);
wire [N_TRIGGERS_PADDED-1:0] tselect_match = {{N_TRIGGERS_PADDED-1{1'b0}}, 1'b1} << tselect;

always @ (posedge clk or negedge rst_n) begin: cfg_update
	integer i;
	if (!rst_n) begin

		tselect <= {W_TSELECT{1'b0}};

		icount_m <= 1'b0;
		icount_u <= 1'b0;

		trigger_irq_m <= 1'b0;
		trigger_irq_u <= 1'b0;
		trigger_irq_dmode <= 1'b0;
		trigger_irq_cause <= 16'h0;

		trigger_exception_m <= 1'b0;
		trigger_exception_u <= 1'b0;
		trigger_exception_dmode <= 1'b0;
		trigger_exception_cause <= 16'h0;

		for (i = 0; i < BREAKPOINT_TRIGGERS; i = i + 1) begin
			bp_tdata1_dmode[i] <= 1'b0;
			mcontrol_action[i] <= 1'b0;
			mcontrol_m[i] <= 1'b0;
			mcontrol_u[i] <= 1'b0;
			mcontrol_execute[i] <= 1'b0;
			bp_tdata2[i] <= {W_DATA{1'b0}};
		end

	end else begin

		if (cfg_wen && cfg_addr == TSELECT) begin

			tselect <= cfg_wdata[W_TSELECT-1:0];

		end else if (cfg_wen && cfg_addr == TDATA1) begin

			if (tselect_match[TINDEX_ICOUNT]) begin
				// 该触发器不实现 dmode 位，因为调试模式下
				// 单步打断已由 dcsr.step 提供
				icount_m <= cfg_wdata[9];
				icount_u <= cfg_wdata[6] && |U_MODE;
			end
			if (tselect_match[TINDEX_INTERRUPT] && !(trigger_irq_dmode && !x_d_mode)) begin
				trigger_irq_dmode <= cfg_wdata[27];
				trigger_irq_m <= cfg_wdata[9];
				trigger_irq_u <= cfg_wdata[6] && |U_MODE;
			end
			if (tselect_match[TINDEX_EXCEPTION] && !(trigger_exception_dmode && !x_d_mode)) begin
				trigger_exception_dmode <= cfg_wdata[27];
				trigger_exception_m <= cfg_wdata[9];
				trigger_exception_u <= cfg_wdata[6] && |U_MODE;
			end
			for (i = 0; i < BREAKPOINT_TRIGGERS; i = i + 1) begin
				if (tselect_match[i] && !(bp_tdata1_dmode[i] && !x_d_mode)) begin
					if (x_d_mode) begin
						bp_tdata1_dmode[i] <= cfg_wdata[27];
					end
					mcontrol_action[i] <= cfg_wdata[12];
					mcontrol_m[i] <= cfg_wdata[6];
					mcontrol_u[i] <= cfg_wdata[3] && |U_MODE;
					mcontrol_execute[i] <= cfg_wdata[2];
				end
			end

		end else if (cfg_wen && cfg_addr == TDATA2) begin

			if (tselect_match[TINDEX_INTERRUPT] && !(trigger_irq_dmode && !x_d_mode)) begin
				trigger_irq_cause <= cfg_wdata[15:0] & IMPLEMENTED_IRQ_CAUSES;
			end
			if (tselect_match[TINDEX_EXCEPTION] && !(trigger_exception_dmode && !x_d_mode)) begin
				trigger_exception_cause <= cfg_wdata[15:0] & IMPLEMENTED_EXCEPTION_CAUSES;
			end
			for (i = 0; i < BREAKPOINT_TRIGGERS; i = i + 1) begin
				if (tselect_match[i] && !(bp_tdata1_dmode[i] && !x_d_mode)) begin
					bp_tdata2[i] <= cfg_wdata & {{W_ADDR-2{1'b1}}, |EXTENSION_C, 1'b0};
				end
			end

		end

		if ((x_d_mode || event_trap_enter) && break_m_step) begin
			// count 字段为硬连线，因此要求该触发器通过
			// 清除自身的使能位来自行关闭
			icount_m <= 1'b0;
			icount_u <= 1'b0;
		end

		// 即使没有断点，也保留一个哑表项，以避免
		// `generate` 的纠缠；工具会抱怨组合 always 块
		// 缺少敏感信号等，所以干脆同步地接 0:
		if (BREAKPOINT_TRIGGERS == 0) begin
			bp_tdata1_dmode[0] <= 1'b0;
			mcontrol_action[0] <= 1'b0;
			mcontrol_m[0] <= 1'b0;
			mcontrol_u[0] <= 1'b0;
			mcontrol_execute[0] <= 1'b0;
			bp_tdata2[0] <= {W_DATA{1'b0}};
		end

	end
end

// ----------------------------------------------------------------------------
// 配置读端口

reg [W_DATA-1:0] tdata1_rdata [0:N_TRIGGERS_PADDED-1];
reg [W_DATA-1:0] tdata2_rdata [0:N_TRIGGERS_PADDED-1];
reg [W_DATA-1:0] tinfo_rdata  [0:N_TRIGGERS_PADDED-1];

always @ (*) begin: generate_padded_rdata

	// 未实现触发器的默认值
	integer i;
	for (i = 0; i < N_TRIGGERS_PADDED; i = i + 1) begin
		tdata1_rdata[i] = {W_DATA{1'b0}};
		tdata2_rdata[i] = {W_DATA{1'b0}};
		tinfo_rdata[i]  = 32'd1 << 0; // type = 0，无触发器
	end

	// 断点是前 n 个触发器
	for (i = 0; i < BREAKPOINT_TRIGGERS; i = i + 1) begin
		tdata1_rdata[i] = {
			4'h2,                              // type = 地址/数据匹配
			bp_tdata1_dmode[i],
			6'h00,                             // maskmax = 0，仅精确匹配
			1'b0,                              // hit = 0，未实现
			1'b0,                              // select = 0，仅地址匹配
			1'b0,                              // timing = 0，执行前触发
			2'h0,                              // sizelo = 0，不限大小
			{3'h0, mcontrol_action[i]},        // action = 0/1，打断进入 M 模式/调试模式
			1'b0,                              // chain = 0，链接对精确匹配毫无用处
			4'h0,                              // match = 0，仅精确匹配
			mcontrol_m[i],
			1'b0,
			1'b0,                              // s = 0，无 S 模式
			mcontrol_u[i],
			mcontrol_execute[i],
			1'b0,                              // store = 0，这不是观察点（watchpoint）
			1'b0                               // load = 0，这不是观察点（watchpoint）
		};
		tdata2_rdata[i] = bp_tdata2[i];
		tinfo_rdata[i] = 32'd1 << 2;           // type = 2，地址/数据匹配
	end

	// 指令计数触发器
	tdata1_rdata[TINDEX_ICOUNT] = {
		4'h3,                                  // type = 指令计数
		1'b0,                                  // dmode = 0（调试模式已有 dcsr.step）
		2'h0,                                  // reserved
		1'b0,                                  // hit = 0
		14'd1,                                 // count = 1，仅单步
		icount_m,
		1'b0,                                  // reserved
		1'b0,                                  // s = 0，无 S 模式
		icount_u,
		6'h0                                   // action = 0，打断进入 M 模式
	};
	tinfo_rdata[TINDEX_ICOUNT] = 32'd1 << 3;   // type = 3，指令计数

	// 中断触发器
	tdata1_rdata[TINDEX_INTERRUPT] = {
		4'h4,                                  // type = 中断
		trigger_irq_dmode,
		1'b0,                                  // hit = 0
		16'h0,                                 // reserved
		trigger_irq_m,
		1'b0,                                  // reserved
		1'b0,                                  // s = 0，无 S 模式
		trigger_irq_u,
		6'd1                                   // action = 1，打断进入调试模式（若 dmode=1）
	};
	tdata2_rdata[TINDEX_INTERRUPT] = {
		16'h0,
		trigger_irq_cause & IMPLEMENTED_IRQ_CAUSES
	};
	tinfo_rdata[TINDEX_INTERRUPT] = 32'd1 << 4;

	// 异常触发器
	tdata1_rdata[TINDEX_EXCEPTION] = {
		4'h5,                                  // type = 异常
		trigger_exception_dmode,
		1'b0,                                  // hit = 0
		16'h0,                                 // reserved
		trigger_exception_m,
		1'b0,                                  // reserved
		1'b0,                                  // s = 0，无 S 模式
		trigger_exception_u,
		6'd1                                   // action = 1，打断进入调试模式（若 dmode=1）
	};
	tdata2_rdata[TINDEX_EXCEPTION] = {
		16'h0,
		trigger_exception_cause & IMPLEMENTED_EXCEPTION_CAUSES
	};
	tinfo_rdata[TINDEX_EXCEPTION] = 32'd1 << 5;

end

always @ (*) begin
	cfg_rdata = {W_DATA{1'b0}};
	if (cfg_addr == TSELECT) begin
		cfg_rdata = {{W_DATA-W_TSELECT{1'b0}}, tselect};
	end else if (cfg_addr == TDATA1) begin
		cfg_rdata = tdata1_rdata[tselect];
	end else if (cfg_addr == TDATA2) begin
		cfg_rdata = tdata2_rdata[tselect];
	end else if (cfg_addr == TINFO) begin
		cfg_rdata = tinfo_rdata[tselect];
	end
end

// ----------------------------------------------------------------------------
// 中断/异常触发器逻辑

// 忽略 tcontrol.mte，因为这些触发器从不面向 M 模式。
wire exception_trigger_match =
	!x_d_mode && trigger_exception_dmode &&
	(x_m_mode ? trigger_exception_m : trigger_exception_u) &&
	event_exception &&
	trigger_exception_cause[event_trap_cause] &&
	IMPLEMENTED_EXCEPTION_CAUSES[event_trap_cause];

wire interrupt_trigger_match =
	!x_d_mode && trigger_irq_dmode &&
	(x_m_mode ? trigger_irq_m : trigger_irq_u) &&
	event_interrupt &&
	trigger_irq_cause[event_trap_cause] &&
	IMPLEMENTED_IRQ_CAUSES[event_trap_cause];

// 最迟在 mtvec 处取指的地址相（aphase）结束前有效。
// 将陷阱处理程序取指的数据相（dphase）标记为
// 含有断点。
reg break_ie;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		break_ie <= 1'b0;
	end else begin
		break_ie <= !x_d_mode && (break_ie || (
			exception_trigger_match || interrupt_trigger_match
		));
	end
end

// ----------------------------------------------------------------------------
// 指令计数触发器逻辑（在 M 模式控制下的单步）

wire step_break_enabled = trig_m_en && !x_d_mode && (
	x_m_mode ? icount_m : icount_u
);

reg break_on_step;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		break_on_step <= 1'b0;
	end else begin
		// 注意 icount 触发器与 dcsr.step 的区别: 前者忽略
		// 异常，只对已退休（retired）的指令触发。
		break_on_step <= !(x_d_mode || event_trap_enter) && (break_on_step || (
			event_instr_ret && step_break_enabled
		));
	end
end

assign break_m_step = break_on_step;

// ----------------------------------------------------------------------------
// 断点触发器逻辑

// 为降低 X 级跳转和 load/store 门控的扇入，地址
// 查找放在 F 级（取指数据相）。我们检查的是*取指地址*，
// 而不是程序计数值。取指始终是字大小、字对齐的。
//
// 为确保这样做是安全的，非调试模式对 TDATA1 与
// TDATA2 CSR 的写操作会引发预取冲刷，以维持写-取指顺序。
//
// 不同的断点可能命中取指字中不同的半字。触发单元必须
// 分别上报这两个匹配，因为此时还不知道指令边界在哪里
// （我们
// 还没有指令数据）。

wire [N_BREAKPOINT_REGS-1:0] breakpoint_enabled;
wire [N_BREAKPOINT_REGS-1:0] breakpoint_match;
wire [N_BREAKPOINT_REGS-1:0] want_d_mode_break;
wire [N_BREAKPOINT_REGS-1:0] want_m_mode_break;
wire [N_BREAKPOINT_REGS-1:0] want_d_mode_break_hw0;
wire [N_BREAKPOINT_REGS-1:0] want_d_mode_break_hw1;
wire [N_BREAKPOINT_REGS-1:0] want_m_mode_break_hw0;
wire [N_BREAKPOINT_REGS-1:0] want_m_mode_break_hw1;

genvar g;
for (g = 0; g < N_BREAKPOINT_REGS; g = g + 1) begin: match_pc
	// 检测触发的断点
	assign breakpoint_enabled[g] = mcontrol_execute[g] && !fetch_d_mode && (
		fetch_m_mode ? mcontrol_m[g] : mcontrol_u[g]
	);
	assign breakpoint_match[g] = breakpoint_enabled[g] && fetch_addr == {bp_tdata2[g][W_DATA-1:2], 2'b00};
	// 判定该触发所隐含的打断类型
	assign want_d_mode_break[g] = breakpoint_match[g] &&  mcontrol_action[g] && bp_tdata1_dmode[g];
	assign want_m_mode_break[g] = breakpoint_match[g] && !mcontrol_action[g] && trig_m_en;
	// 按半字分别上报，前端可将其
	// 送入预取缓冲。当一条指令（任意大小）的
	// 首个半字被标记了断点（即精确匹配）时，
	// 进入断点异常。
	assign want_d_mode_break_hw0[g] = want_d_mode_break[g] && !bp_tdata2[g][1];
	assign want_d_mode_break_hw1[g] = want_d_mode_break[g] &&  bp_tdata2[g][1];
	assign want_m_mode_break_hw0[g] = want_m_mode_break[g] && !bp_tdata2[g][1];
	assign want_m_mode_break_hw1[g] = want_m_mode_break[g] &&  bp_tdata2[g][1];
end

// 送往前端的打断标志（将当前取指 dphase 标记为含断点）:

assign break_any    = {
	|want_m_mode_break_hw1 || |want_d_mode_break_hw1 || break_ie,
	|want_m_mode_break_hw0 || |want_d_mode_break_hw0 || break_ie
} & {2{BREAKPOINT_TRIGGERS > 0}};

assign break_d_mode = {
	|want_d_mode_break_hw1 || break_ie,
	|want_d_mode_break_hw0 || break_ie
} & {2{BREAKPOINT_TRIGGERS > 0}};

end
endgenerate

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
