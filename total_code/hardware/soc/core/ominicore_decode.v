


`default_nettype none

module ominicore_decode #(
`include "ominicore_config.vh"
,
`include "ominicore_width_const.vh"
) (
	input  wire                 clk,
	input  wire                 rst_n,

	input  wire [31:0]          fd_cir,
	input  wire [1:0]           fd_cir_err,
	input  wire [1:0]           fd_cir_predbranch,
	input  wire [1:0]           fd_cir_vld,
	input  wire                 fd_cir_is_32bit,
	input  wire                 fd_cir_invalid_16bit,
	input  wire                 fd_cir_is_uop,
	input  wire                 fd_cir_uop_nonfinal,
	input  wire                 fd_cir_uop_no_pc_update,
	input  wire                 fd_cir_uop_atomic,

	output wire [1:0]           df_cir_use,
	output wire                 df_cir_flush_behind,

	output wire                 df_uop_stall,
	output wire                 df_uop_clear,
	output wire                 df_lspair_phase_next,

	output wire [W_ADDR-1:0]    d_pc,

	input  wire                 debug_mode,
	input  wire                 m_mode,
	input  wire                 trap_wfi,

	input  wire [W_ADDR-1:0]    debug_dpc_wdata,
	input  wire                 debug_dpc_wen,
	output wire [W_ADDR-1:0]    debug_dpc_rdata,

	output wire                 d_starved,
	input  wire                 x_stall,
	input  wire                 f_jump_now,
	input  wire [W_ADDR-1:0]    f_jump_target,
	input  wire                 x_jump_not_except,
	input  wire [W_ADDR-1:0]    d_btb_target_addr,

	output reg  [W_DATA-1:0]    d_imm,
	output reg  [W_REGADDR-1:0] d_rs1,
	output reg  [W_REGADDR-1:0] d_rs2,
	output reg  [W_REGADDR-1:0] d_rd,
	output reg  [2:0]           d_funct3_32b,
	output reg  [6:0]           d_funct7_32b,
	output reg  [W_ALUSRC-1:0]  d_alusrc_a,
	output reg  [W_ALUSRC-1:0]  d_alusrc_b,
	output reg  [W_ALUOP-1:0]   d_aluop,
	output reg  [W_MEMOP-1:0]   d_memop,
	output reg  [W_MULOP-1:0]   d_mulop,
	output reg                  d_csr_ren,
	output reg                  d_csr_wen,
	output reg  [1:0]           d_csr_wtype,
	output reg                  d_csr_w_imm,
	output reg  [W_BCOND-1:0]   d_branchcond,
	output reg  [W_ADDR-1:0]    d_addr_offs,
	output reg                  d_addr_is_regoffs,
	output reg  [W_EXCEPT-1:0]  d_except,
	output reg                  d_sleep_wfi,
	output reg                  d_sleep_block,
	output reg                  d_sleep_unblock,
	output wire                 d_no_pc_increment,
	output wire                 d_uninterruptible,
	output wire [W_ADDR-1:0]    d_lspair_offset,
	output reg                  d_fence_i,
	output reg                  d_fence_d
);

`include "rv_opcodes.vh"
`include "ominicore_ops.vh"

localparam HAVE_CSR = CSR_M_MANDATORY || CSR_M_TRAP || CSR_COUNTER;

// ----------------------------------------------------------------------------

wire [31:0] d_instr = fd_cir | {
	30'd0, {2{~|EXTENSION_C}}
};

reg  d_invalid_32bit;
wire d_invalid = fd_cir_invalid_16bit || d_invalid_32bit;

assign d_uninterruptible = |EXTENSION_ZCMP && fd_cir_uop_atomic;

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	assert(!(d_invalid && fd_cir_is_uop));
	assert(!(d_invalid && fd_cir_uop_atomic));
end
`endif

wire d_lspair_nonfinal;
// 当本条指令陷入异常时，将 mepc 偏移清零的信号
// （因为 uop 序列中*可能产生异常*的 uop —— 即不含
// popret/popretz 最后的 sp 调整 —— 都与下一条 uop 具有相同的 PC，
// 而后者在这些 uop 陷入异常时位于第 2 级）
assign d_no_pc_increment = fd_cir_uop_nonfinal || d_lspair_nonfinal;

assign df_uop_stall = x_stall || d_starved;

// 注意 !df_cir_flush_behind: cm.popret/popretz 中的跳转是
// *倒数第二条*指令: 我们借取指气泡执行栈调整以节省一拍，
// 但仍需完成整个 uop 序列。
//
// 该 sp 调整不会产生异常（它是一条 `add`，其 PMP.X 与断点
// 比较结果都与先前的 uop 相同），且序列的这一部分
// 屏蔽了中断。
assign df_uop_clear = f_jump_now && !df_cir_flush_behind;

// 译码各种立即数格式
wire [31:0] d_imm_i = {{21{d_instr[31]}}, d_instr[30:20]};
wire [31:0] d_imm_s = {{21{d_instr[31]}}, d_instr[30:25], d_instr[11:7]};
wire [31:0] d_imm_b = {{20{d_instr[31]}}, d_instr[7], d_instr[30:25], d_instr[11:8], 1'b0};
wire [31:0] d_imm_u = {d_instr[31:12], {12{1'b0}}};
wire [31:0] d_imm_j = {{12{d_instr[31]}}, d_instr[19:12], d_instr[20], d_instr[30:21], 1'b0};

// ----------------------------------------------------------------------------
// PC/CIR 控制

// 对一条有效 16 位指令*之后*跟着的错误，不能标记总线错误，
// 因为取指错误是推测性的，可能被例如分支指令冲刷掉。
// 注意必须低 16 位有效，我们才能知道
// 一条指令的长度。
wire d_except_instr_bus_fault = fd_cir_vld > 2'd0 && fd_cir_err[0] ||
	fd_cir_vld > 2'd1 && fd_cir_is_32bit && fd_cir_err[1];

assign d_starved = ~|fd_cir_vld || fd_cir_vld[0] && fd_cir_is_32bit;

wire d_stall = x_stall || d_starved || fd_cir_uop_nonfinal || d_lspair_nonfinal;

assign df_cir_use =
	d_starved || d_stall ? 2'h0 :
	fd_cir_is_32bit ? 2'h2 : 2'h1;

// 若我们成功发出跳转请求但译码停顿，则需要 CIR 锁定。
// 当停顿取决于总线停顿时，无法对跳转请求进行门控
// （否则会在总线停顿与总线请求之间形成直通路径），
// 因此改为指示前端在冲刷时保留停顿中的指令，
// 并在其后方填充。
//
// 一旦停顿解除，被停顿的指令即可执行其余的
// 副作用，例如向寄存器堆写入链接值。
wire jump_caused_by_d = f_jump_now && x_jump_not_except;
wire assert_cir_lock = jump_caused_by_d && d_stall;

// 当一条完整指令（而不仅是 uop）毕业到下一级时，
// CIR 锁定自然结束:
wire finished_cir_lock = !d_stall;

// CIR 锁定也可能因陷阱进入而意外终止。一种到达该情形的
// 路径是 cm.popret 最后一次 load 在 dphase 上出错: 此时 `ret`
// 在第一个 dphase 拍停顿期间发出取指地址，
// 然后在第二拍被陷阱冲刷。
wire deassert_cir_lock = finished_cir_lock || (f_jump_now && !x_jump_not_except);

reg cir_lock_prev;
wire cir_lock = (cir_lock_prev && !deassert_cir_lock) || assert_cir_lock;
assign df_cir_flush_behind = assert_cir_lock && !cir_lock_prev;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cir_lock_prev <= 1'b0;
	end else begin
		cir_lock_prev <= cir_lock;
	end
end

reg  [W_ADDR-1:0] pc;
wire [W_ADDR-1:0] pc_seq_next = pc + (
	|EXTENSION_ZCMP && fd_cir_is_uop && fd_cir_uop_no_pc_update ? 32'd0 :
	fd_cir_is_32bit                                             ? 32'd4 : 32'd2
);

assign d_pc = pc;
assign debug_dpc_rdata = pc;

// 前端应把整条指令、且仅仅是该指令，标记为预测分支。
// 当我们以不同的 16 位对齐方式（！）两次执行包含预测分支的
// 同一地址时，这会出问题。此时需要发出一条自跳转
// （branch-to-self）以回到线性路径，
// 否则 PC 与 CIR 会失步，导致误执行。
wire partial_predicted_branch = !d_starved &&
	|BRANCH_PREDICTOR && fd_cir_is_32bit && ^fd_cir_predbranch;

wire predicted_branch = |BRANCH_PREDICTOR && fd_cir_predbranch[0];

// 通常锁定发生在停顿的跳转/分支上，它在解除停顿时
// 可能需要原始 PC 来生成链接地址。一个例外是
// 微操作序列中的跳转: 此时跳转是序列中的
// 倒数第二条指令（ret 在 addi sp 之前），
// 我们需要在 uop 序列中途捕获 pc。
wire hold_pc_on_cir_lock = assert_cir_lock && !(fd_cir_is_uop && !fd_cir_uop_no_pc_update && !x_stall);
wire update_pc_on_cir_unlock = cir_lock_prev && finished_cir_lock && !fd_cir_uop_no_pc_update;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pc <= RESET_VECTOR;
	end else begin
		if (debug_dpc_wen) begin
			pc <= debug_dpc_wdata;
		end else if (debug_mode) begin
			pc <= pc;
		end else if ((f_jump_now && !hold_pc_on_cir_lock) || update_pc_on_cir_unlock) begin
			pc <= f_jump_target;
		end else if (!f_jump_now && fd_cir_uop_nonfinal && !fd_cir_uop_no_pc_update && !x_stall) begin
			// cm.popret 与 cm.popretz 中先前停顿的 jr uop 结束:
			// 由于下一条指令（addi sp）不会陷入异常，可安全更新 PC。
			pc <= f_jump_target;
		end else if (!d_stall && !cir_lock) begin
			// 若本条指令是预测跳转的分支（且尚未产生
			// 误预测恢复跳转），则将 PC 设为
			// 预测目标，而不是顺序的下一个 PC
			pc <= predicted_branch ? d_btb_target_addr : pc_seq_next;
		end
	end
end

`ifdef OMNICORE_ASSERTIONS
always @ (posedge clk) if (rst_n) begin
	if (~|fd_cir_vld)            assert(!fd_cir_is_uop);
	if (fd_cir_uop_no_pc_update) assert(fd_cir_is_uop);
	if (fd_cir_uop_nonfinal)     assert(fd_cir_is_uop);
	if ($past(df_uop_clear))     assert(!fd_cir_is_uop);
	// 重要的是避免 cm.popret 最后一次 load 陷入异常后
	// 出现虚假的 PC 更新:
	if ($past(df_uop_clear))     assert(!fd_cir_uop_no_pc_update);
end
`endif

wire [W_ADDR-1:0] branch_offs =
	!fd_cir_is_32bit && predicted_branch ? 32'd2 :
	 fd_cir_is_32bit && predicted_branch ? 32'd4 : d_imm_b;

always @ (*) begin
	casez ({|EXTENSION_A, d_instr[6:2]})
	{1'bz, 5'b11011}: d_addr_offs = d_imm_j      ; // JAL
	{1'bz, 5'b11000}: d_addr_offs = branch_offs  ; // 分支
	{1'bz, 5'b01000}: d_addr_offs = d_imm_s      ; // store
	{1'bz, 5'b11001}: d_addr_offs = d_imm_i      ; // JALR
	{1'bz, 5'b00000}: d_addr_offs = d_imm_i      ; // load
	{1'b1, 5'b01011}: d_addr_offs = 32'h0000_0000; // 原子操作
	default:          d_addr_offs = 32'hxxxx_xxxx;
	endcase
	if (partial_predicted_branch) begin
		d_addr_offs = 32'h0000_0000;
	end
end

// ----------------------------------------------------------------------------
// 跟踪 load/store 成对指令的相位（Zilsd 与 Zclsd）

// 它本可与 uop_ctr（Zcmp 用）共享，但两者有本质区别:
// Zcmp 是展开为 32 位指令序列的 16 位指令，
// 而 Zilsd 是多相位的 32 位指令，Zclsd 是它们的
// 16 位直接别名。因此对 Zilsd/Zclsd 而言，
// 把相位管理与解压分离更干净。

wire d_lspair_phase;

// 重排访问顺序，避免在 load 前半段破坏 rs1（基址）:
wire d_lspair_reg_sel = d_lspair_phase == d_instr[15];

generate
if (EXTENSION_ZILSD) begin: have_lspair_reg_sel
	reg d_lspair_phase_r;
	assign d_lspair_phase = d_lspair_phase_r;
	reg instr_is_lspair;
	always @ (*) begin
		casez ({d_invalid || d_starved, d_instr})
			{1'b0, `RVOPC_LD}: instr_is_lspair = 1'b1;
			{1'b0, `RVOPC_SD}: instr_is_lspair = 1'b1;
			default:           instr_is_lspair = 1'b0;
		endcase
	end

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			d_lspair_phase_r <= 1'b0;
		end else begin
			d_lspair_phase_r <= df_lspair_phase_next;
		end
	end

	assign df_lspair_phase_next =
		!d_stall || f_jump_now      ? 1'b0 :
		instr_is_lspair && !x_stall ? 1'b1 : d_lspair_phase_r;

	assign d_lspair_nonfinal = instr_is_lspair && !d_lspair_phase_r;

	assign d_lspair_offset = {
		29'h0,
		d_lspair_reg_sel && instr_is_lspair,
		2'h0
	};

end else begin: no_lspair_reg_sel

	assign d_lspair_phase = 1'b0;
	assign df_lspair_phase_next = 1'b0;
	assign d_lspair_nonfinal = 1'b0;
	assign d_lspair_offset = 32'd0;

end
endgenerate

// ----------------------------------------------------------------------------
// 译码 X 级控制信号

localparam X0 = {W_REGADDR{1'b0}};

// 首先根据可用的扩展与特权状态译码指令位，
// 不与任何停顿/异常信号相与。
reg  [W_REGADDR-1:0] raw_rs1;
reg  [W_REGADDR-1:0] raw_rs2;
reg  [W_REGADDR-1:0] raw_rd;
reg  [W_DATA-1:0]    raw_imm;
reg  [W_ALUSRC-1:0]  raw_alusrc_a;
reg  [W_ALUSRC-1:0]  raw_alusrc_b;
reg  [W_ALUOP-1:0]   raw_aluop;
reg  [W_MEMOP-1:0]   raw_memop;
reg  [W_MULOP-1:0]   raw_mulop;
reg                  raw_csr_ren;
reg                  raw_csr_wen;
reg  [1:0]           raw_csr_wtype;
reg                  raw_csr_w_imm;
reg  [W_BCOND-1:0]   raw_branchcond;
reg                  raw_addr_is_regoffs;
reg  [W_EXCEPT-1:0]  raw_except;
reg                  raw_sleep_wfi;
reg                  raw_sleep_block;
reg                  raw_sleep_unblock;
reg                  raw_fence_i;
reg                  raw_fence_d;

always @ (*) begin
	// 赋一些默认值
	raw_rs1 = d_instr[19:15];
	raw_rs2 = d_instr[24:20];
	raw_rd  = d_instr[11: 7];
	raw_imm = d_imm_i;
	raw_alusrc_a = ALUSRCA_RS1;
	raw_alusrc_b = ALUSRCB_RS2;
	raw_aluop = ALUOP_ADD;
	raw_memop = MEMOP_NONE;
	raw_mulop = M_OP_MUL;
	raw_csr_ren = 1'b0;
	raw_csr_wen = 1'b0;
	raw_csr_wtype = CSR_WTYPE_W;
	raw_csr_w_imm = 1'b0;
	raw_branchcond = BCOND_NEVER;
	raw_addr_is_regoffs = 1'b0;
	raw_except = EXCEPT_NONE;
	raw_sleep_wfi = 1'b0;
	raw_sleep_block = 1'b0;
	raw_sleep_unblock = 1'b0;
	raw_fence_i = 1'b0;
	raw_fence_d = 1'b0;
	// 注意这对 funct3/funct7 仅对 32 位指令有效。
	// 它们对相关 ALU 操作簇（如 sh*add、clmul）有用。
	d_funct3_32b = fd_cir[14:12];
	d_funct7_32b = fd_cir[31:25];

	d_invalid_32bit = 1'b0;

	casez (d_instr)
	`RVOPC_BEQ:       begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_SUB; raw_branchcond = BCOND_ZERO;  end
	`RVOPC_BNE:       begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_SUB; raw_branchcond = BCOND_NZERO; end
	`RVOPC_BLT:       begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_LT;  raw_branchcond = BCOND_NZERO; end
	`RVOPC_BGE:       begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_LT;  raw_branchcond = BCOND_ZERO;  end
	`RVOPC_BLTU:      begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_LTU; raw_branchcond = BCOND_NZERO; end
	`RVOPC_BGEU:      begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_rd = X0; raw_aluop = ALUOP_LTU; raw_branchcond = BCOND_ZERO;  end
	`RVOPC_JALR:      begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_branchcond = BCOND_ALWAYS; raw_addr_is_regoffs = 1'b1;
	                        raw_rs2 = X0; raw_aluop = ALUOP_ADD; raw_alusrc_a = ALUSRCA_PC; raw_alusrc_b = ALUSRCB_IMM; raw_imm = fd_cir_is_32bit ? 32'd4 : 32'd2; end
	`RVOPC_JAL:       begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_branchcond = BCOND_ALWAYS; raw_rs1 = X0;
	                        raw_rs2 = X0; raw_aluop = ALUOP_ADD; raw_alusrc_a = ALUSRCA_PC; raw_alusrc_b = ALUSRCB_IMM; raw_imm = fd_cir_is_32bit ? 32'd4 : 32'd2; end
	`RVOPC_LUI:       begin raw_aluop = ALUOP_RS2; raw_imm = d_imm_u; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; raw_rs1 = X0; end
	`RVOPC_AUIPC:     begin d_invalid_32bit = DEBUG_SUPPORT && debug_mode; raw_aluop = ALUOP_ADD; raw_imm = d_imm_u; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; raw_alusrc_a = ALUSRCA_PC;  raw_rs1 = X0; end
	`RVOPC_ADDI:      begin raw_aluop = ALUOP_ADD; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_SLLI:      begin raw_aluop = ALUOP_SLL; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_SLTI:      begin raw_aluop = ALUOP_LT;  raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_SLTIU:     begin raw_aluop = ALUOP_LTU; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_XORI:      begin raw_aluop = ALUOP_XOR; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_SRLI:      begin raw_aluop = ALUOP_SRL; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_SRAI:      begin raw_aluop = ALUOP_SRA; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_ORI:       begin raw_aluop = ALUOP_OR;  raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_ANDI:      begin raw_aluop = ALUOP_AND; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; raw_rs2 = X0; end
	`RVOPC_ADD:       begin raw_aluop = ALUOP_ADD; end
	`RVOPC_SUB:       begin raw_aluop = ALUOP_SUB; end
	`RVOPC_SLL:       begin raw_aluop = ALUOP_SLL; end
	`RVOPC_SLTU:      begin raw_aluop = ALUOP_LTU; end
	`RVOPC_XOR:       begin raw_aluop = ALUOP_XOR; end
	`RVOPC_SRL:       begin raw_aluop = ALUOP_SRL; end
	`RVOPC_SRA:       begin raw_aluop = ALUOP_SRA; end
	`RVOPC_OR:        begin raw_aluop = ALUOP_OR;  end
	`RVOPC_AND:       begin raw_aluop = ALUOP_AND; end
	`RVOPC_LB:        begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LB;  end
	`RVOPC_LH:        begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LH;  end
	`RVOPC_LW:        begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LW;  end
	`RVOPC_LBU:       begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LBU; end
	`RVOPC_LHU:       begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LHU; end
	`RVOPC_SB:        begin raw_addr_is_regoffs = 1'b1; raw_aluop = ALUOP_RS2; raw_memop = MEMOP_SB;  raw_rd = X0; end
	`RVOPC_SH:        begin raw_addr_is_regoffs = 1'b1; raw_aluop = ALUOP_RS2; raw_memop = MEMOP_SH;  raw_rd = X0; end
	`RVOPC_SW:        begin raw_addr_is_regoffs = 1'b1; raw_aluop = ALUOP_RS2; raw_memop = MEMOP_SW;  raw_rd = X0; end

	`RVOPC_SLT:       begin
		raw_aluop = ALUOP_LT;
		if (|EXTENSION_XH3POWER && ~|raw_rd && ~|raw_rs1) begin
			if (raw_rs2 == 5'h00) begin
				// h3.block（电源管理提示）
				d_invalid_32bit = trap_wfi;
				raw_sleep_block = !trap_wfi;
			end else if (raw_rs2 == 5'h01) begin
				// h3.unblock（电源管理提示）
				d_invalid_32bit = trap_wfi;
				raw_sleep_unblock = !trap_wfi;
			end
		end
	end

	`RVOPC_MUL:       if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_MUL;    end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MULH:      if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_MULH;   end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MULHSU:    if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_MULHSU; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MULHU:     if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_MULHU;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_DIV:       if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_DIV;    end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_DIVU:      if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_DIVU;   end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_REM:       if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_REM;    end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_REMU:      if (EXTENSION_M) begin raw_aluop = ALUOP_MULDIV; raw_mulop = M_OP_REMU;   end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_LR_W:      if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_LR_W; raw_rs2 = X0;           end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SC_W:      if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_SC_W; raw_aluop = ALUOP_RS2;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOSWAP_W: if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_RS2;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOADD_W:  if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_ADD;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOXOR_W:  if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_XOR;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOAND_W:  if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_AND;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOOR_W:   if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_OR;   end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOMIN_W:  if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_MIN;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOMAX_W:  if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_MAX;  end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOMINU_W: if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_MINU; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_AMOMAXU_W: if (EXTENSION_A) begin raw_addr_is_regoffs = 1'b1; raw_memop = MEMOP_AMO;  raw_aluop = ALUOP_MAXU; end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_LD:        if (EXTENSION_ZILSD) begin raw_addr_is_regoffs = 1'b1; raw_rs2 = X0; raw_memop = MEMOP_LW; raw_rd  = {d_instr[11: 8], d_lspair_reg_sel};                        end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SD:        if (EXTENSION_ZILSD) begin raw_addr_is_regoffs = 1'b1; raw_rd  = X0; raw_memop = MEMOP_SW; raw_rs2 = {d_instr[24:21], d_lspair_reg_sel}; raw_aluop = ALUOP_RS2; end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_SH1ADD:    if (EXTENSION_ZBA)  begin raw_aluop = ALUOP_SHXADD;                                                              end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SH2ADD:    if (EXTENSION_ZBA)  begin raw_aluop = ALUOP_SHXADD;                                                              end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SH3ADD:    if (EXTENSION_ZBA)  begin raw_aluop = ALUOP_SHXADD;                                                              end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_ANDN:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ANDN;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CLZ:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_CLZ;    raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CPOP:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_CPOP;   raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CTZ:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_CTZ;    raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MAX:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_MAX;                                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MAXU:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_MAXU;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MIN:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_MIN;                                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MINU:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_MINU;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_ORC_B:     if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ORC_B;  raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_ORN:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ORN;                                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_REV8:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_REV8;   raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_ROL:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ROL;                                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_ROR:       if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ROR;                                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_RORI:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ROR;    raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SEXT_B:    if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_SEXT_B; raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_SEXT_H:    if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_SEXT_H; raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_XNOR:      if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_XNOR;                                                                end else begin d_invalid_32bit = 1'b1; end
	// 注意: ZEXT_H 是 Zbkb 中 PACK 的子集。只要本分支
	// 排在最前面就没问题，因为 OmniCore 上 Zbkb 隐含 Zbb。
	`RVOPC_ZEXT_H:    if (EXTENSION_ZBB)  begin raw_aluop = ALUOP_ZEXT_H; raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_CLMUL:     if (EXTENSION_ZBC)  begin raw_aluop = ALUOP_CLMUL;                                                               end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CLMULH:    if (EXTENSION_ZBC)  begin raw_aluop = ALUOP_CLMUL;                                                               end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CLMULR:    if (EXTENSION_ZBC)  begin raw_aluop = ALUOP_CLMUL;                                                               end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_BCLR:      if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BCLR;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BCLRI:     if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BCLR;   raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BEXT:      if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BEXT;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BEXTI:     if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BEXT;   raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BINV:      if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BINV;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BINVI:     if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BINV;   raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BSET:      if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BSET;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BSETI:     if (EXTENSION_ZBS)  begin raw_aluop = ALUOP_BSET;   raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_PACK:      if (EXTENSION_ZBKB) begin raw_aluop = ALUOP_PACK;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_PACKH:     if (EXTENSION_ZBKB) begin raw_aluop = ALUOP_PACKH;                                                               end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_BREV8:     if (EXTENSION_ZBKB) begin raw_aluop = ALUOP_BREV8;  raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_UNZIP:     if (EXTENSION_ZBKB) begin raw_aluop = ALUOP_UNZIP;  raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_ZIP:       if (EXTENSION_ZBKB) begin raw_aluop = ALUOP_ZIP;    raw_rs2 = X0;                                                end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_XPERM8:    if (EXTENSION_ZBKX) begin raw_aluop = ALUOP_XPERM;                                                               end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_XPERM4:    if (EXTENSION_ZBKX) begin raw_aluop = ALUOP_XPERM;                                                               end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_H3_BEXTM:  if (EXTENSION_XH3BEXTM) begin
	                                           raw_aluop = ALUOP_BEXTM;                                                                end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_H3_BEXTMI: if (EXTENSION_XH3BEXTM) begin
	                                           raw_aluop = ALUOP_BEXTM;   raw_rs2 = X0; raw_imm = d_imm_i; raw_alusrc_b = ALUSRCB_IMM; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRW:     if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = 1'b1  ;   raw_csr_ren = |raw_rd; raw_csr_wtype = CSR_WTYPE_W;                       end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRS:     if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = |raw_rs1; raw_csr_ren = 1'b1 ;   raw_csr_wtype = CSR_WTYPE_S;                       end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRC:     if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = |raw_rs1; raw_csr_ren = 1'b1 ;   raw_csr_wtype = CSR_WTYPE_C;                       end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRWI:    if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = 1'b1  ;   raw_csr_ren = |raw_rd; raw_csr_wtype = CSR_WTYPE_W; raw_csr_w_imm = 1'b1; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRSI:    if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = |raw_rs1; raw_csr_ren = 1'b1 ;   raw_csr_wtype = CSR_WTYPE_S; raw_csr_w_imm = 1'b1; end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_CSRRCI:    if (HAVE_CSR)              begin raw_rs2 = X0; raw_imm = d_imm_i; raw_csr_wen = |raw_rs1; raw_csr_ren = 1'b1 ;   raw_csr_wtype = CSR_WTYPE_C; raw_csr_w_imm = 1'b1; end else begin d_invalid_32bit = 1'b1; end

	`RVOPC_FENCE:     begin raw_rs2 = X0; raw_fence_d = 1'b1; end  // 注意指令中 rs1/rd 为零
	`RVOPC_FENCE_I:   if (EXTENSION_ZIFENCEI)    begin raw_except = debug_mode ? EXCEPT_NONE : EXCEPT_REFETCH; raw_fence_i = 1'b1;                                          end else begin d_invalid_32bit = 1'b1; end // 注意指令中 rs1/rs2/rd 为零
	`RVOPC_ECALL:     if (HAVE_CSR)              begin raw_except = m_mode || !U_MODE ? EXCEPT_ECALL_M : EXCEPT_ECALL_U;  raw_rs2 = X0; raw_rs1 = X0; raw_rd = X0;          end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_EBREAK:    if (HAVE_CSR)              begin raw_except = EXCEPT_EBREAK; raw_rs2 = X0; raw_rs1 = X0; raw_rd = X0;                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_MRET:      if (HAVE_CSR && m_mode)    begin raw_except = EXCEPT_MRET;   raw_rs2 = X0; raw_rs1 = X0; raw_rd = X0;                                                 end else begin d_invalid_32bit = 1'b1; end
	`RVOPC_WFI:       if (HAVE_CSR && !trap_wfi) begin raw_sleep_wfi = 1'b1;       raw_rs2 = X0; raw_rs1 = X0; raw_rd = X0;                                                 end else begin d_invalid_32bit = 1'b1; end

	default:          begin d_invalid_32bit = 1'b1; end
	endcase

	if (|EXTENSION_E && (raw_rd[4] || raw_rs1[4] || raw_rs2[4])) begin
		d_invalid_32bit = 1'b1;
	end
end

// 然后基于 CIR 满标志、取指错误等对关键信号进行门控。
// 这种拆分有助于避免事件调度的反馈环路 —— 它会让
// 仿真器又慢又不高兴，verilator 尤其如此

localparam [4:0] REGADDR_MASK = {~|EXTENSION_E, 4'hf};

always @ (*) begin
	// 默认直通
	d_rs1             = raw_rs1 & REGADDR_MASK;
	d_rs2             = raw_rs2 & REGADDR_MASK;
	d_rd              = raw_rd  & REGADDR_MASK;
	d_imm             = raw_imm;
	d_alusrc_a        = raw_alusrc_a;
	d_alusrc_b        = raw_alusrc_b;
	d_aluop           = raw_aluop;
	d_memop           = raw_memop;
	d_mulop           = raw_mulop;
	d_csr_ren         = raw_csr_ren;
	d_csr_wen         = raw_csr_wen;
	d_csr_wtype       = raw_csr_wtype;
	d_csr_w_imm       = raw_csr_w_imm;
	d_branchcond      = raw_branchcond;
	d_addr_is_regoffs = raw_addr_is_regoffs;
	d_except          = raw_except;
	d_sleep_wfi       = raw_sleep_wfi;
	d_sleep_block     = raw_sleep_block;
	d_sleep_unblock   = raw_sleep_unblock;
	d_fence_i         = raw_fence_i;
	d_fence_d         = raw_fence_d;

	if (d_invalid || d_starved || d_except_instr_bus_fault || partial_predicted_branch) begin
		d_rs1             = {W_REGADDR{1'b0}};
		d_rs2             = {W_REGADDR{1'b0}};
		d_rd              = {W_REGADDR{1'b0}};
		d_memop           = MEMOP_NONE;
		d_branchcond      = BCOND_NEVER;
		d_csr_ren         = 1'b0;
		d_csr_wen         = 1'b0;
		d_except          = EXCEPT_NONE;
		d_sleep_wfi       = 1'b0;
		d_sleep_block     = 1'b0;
		d_sleep_unblock   = 1'b0;
		d_fence_i         = 1'b0;
		d_fence_d         = 1'b0;

		if (EXTENSION_M)
			d_aluop = ALUOP_ADD;

		if (d_except_instr_bus_fault)
			d_except = EXCEPT_INSTR_FAULT;
		else if (d_invalid && !d_starved)
			d_except = EXCEPT_INSTR_ILLEGAL;
	end
	if (partial_predicted_branch) begin
		d_addr_is_regoffs = 1'b0;
		d_branchcond = BCOND_ALWAYS;
	end
	if (cir_lock_prev) begin
		d_branchcond = BCOND_NEVER;
	end
end

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
