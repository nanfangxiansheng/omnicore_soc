



`default_nettype none

// 物理内存保护单元（PMP）

module ominicore_pmp #(
`include "ominicore_config.vh"
) (
	input  wire              clk,
	input  wire              rst_n,

	// 经 CSR 块透传的配置接口
	input  wire [11:0]       cfg_addr,
	input  wire              cfg_wen,
	input  wire [W_DATA-1:0] cfg_wdata,
	output reg  [W_DATA-1:0] cfg_rdata,

	// 取指地址查询
	input  wire [W_ADDR-1:0] i_addr,
	input  wire              i_m_mode,
	output wire              i_kill,

	// load/store 地址查询
	input  wire [W_ADDR-1:0] d_addr,
	// 为进位保留（carry-save）加法单独拆出:
	input  wire [W_ADDR-1:0] d_addr_addend_rs1,
	input  wire [W_ADDR-1:0] d_addr_addend_imm,
	input  wire [W_ADDR-1:0] d_addr_addend_lspair_offs,
	input  wire              d_m_mode,
	input  wire              d_write,
	output wire              d_kill
);

localparam PMP_A_NAPOT = 2'b11;
localparam PMP_A_NA4   = 2'b10;
localparam PMP_A_TOR   = 2'b01;
localparam PMP_A_OFF   = 2'b00;

// A 字段支持哪些取值（不支持的映射为 OFF）:
localparam [3:0] PMP_A_SUPPORTED = {
	|PMP_MATCH_NAPOT,
	|PMP_MATCH_NAPOT && PMP_GRAIN == 0,
	|PMP_MATCH_TOR,
	1'b1
};

`include "ominicore_csr_addr.vh"

generate
if (PMP_REGIONS == 0) begin: no_pmp

// 这部分应当在 core.v 中已被桩掉，但这里也用一个 generate
// 以免在区域数为 0 的情况下详细展开本模块时
// 产生警告。

always @ (*) cfg_rdata = {W_DATA{1'b0}};
assign i_kill = 1'b0;
assign d_kill = 1'b0;

end else begin: have_pmp

// ----------------------------------------------------------------------------
// 配置寄存器与读写接口

// 区域配置是否可写；在支持 TOR 时这个问题并不平凡，
// 因为锁定区域 i + 1 也会连带锁定区域 i。
wire [PMP_REGIONS-1:0] region_locked;

reg  [PMP_REGIONS-1:0] pmpcfg_l;
reg  [1:0]             pmpcfg_a [0:PMP_REGIONS-1];
reg  [PMP_REGIONS-1:0] pmpcfg_x;
reg  [PMP_REGIONS-1:0] pmpcfg_w;
reg  [PMP_REGIONS-1:0] pmpcfg_r;

// 地址寄存器保存地址的 33:2 位（以支持 16 GiB
// 物理地址空间）。我们不实现第 33、32 位。
reg  [W_ADDR-3:0]      pmpaddr  [0:PMP_REGIONS-1];

// OmniCore 扩展：在不加锁的情况下将 PMP 区域应用于 M 模式。
// 与 ePMP 的 mseccfg.rlb 不同：低编号区域可以因安全原因被锁定，
// 但高编号区域仍应可用于
// 其他用途，例如栈保护、外设模拟
reg  [PMP_REGIONS-1:0] pmpcfg_m;

always @ (posedge clk or negedge rst_n) begin: cfg_update
	reg signed [31:0] i;
	if (!rst_n) begin
		for (i = 0; i < PMP_REGIONS; i = i + 1) begin
			pmpcfg_l[i] <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_CFG[8 * i + 7]      : 1'b0;
			pmpcfg_a[i] <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_CFG[8 * i + 3 +: 2] : 2'h0;
			pmpcfg_x[i] <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_CFG[8 * i + 2]      : 1'b0;
			pmpcfg_w[i] <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_CFG[8 * i + 1]      : 1'b0;
			pmpcfg_r[i] <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_CFG[8 * i + 0]      : 1'b0;

			pmpaddr[i]  <= PMP_HARDWIRED[i] ? PMP_HARDWIRED_ADDR[32 * i +: 30]  :
			               PMP_GRAIN > 1    ? ~(~30'h0 << (PMP_GRAIN - 1))      : 30'h0;
		end
		pmpcfg_m <= {PMP_REGIONS{1'b0}};
	end else if (cfg_wen) begin
		for (i = 0; i < PMP_REGIONS; i = i + 1) begin
			if (cfg_addr == PMPCFG0 + i[13:2] && !region_locked[i]) begin
				if (PMP_HARDWIRED[i]) begin
					// 保持连接到硬连线值（但仍让该“寄存器”对 clk 敏感）
					pmpcfg_l[i] <= PMP_HARDWIRED_CFG[8 * i + 7];
					pmpcfg_a[i] <= PMP_HARDWIRED_CFG[8 * i + 3 +: 2];
					pmpcfg_x[i] <= PMP_HARDWIRED_CFG[8 * i + 2];
					pmpcfg_w[i] <= PMP_HARDWIRED_CFG[8 * i + 1];
					pmpcfg_r[i] <= PMP_HARDWIRED_CFG[8 * i + 0];
					pmpaddr[i]  <= PMP_HARDWIRED_ADDR[32 * i +: 30];
				end else begin
					pmpcfg_l[i] <= cfg_wdata[i % 4 * 8 + 7];
					pmpcfg_x[i] <= cfg_wdata[i % 4 * 8 + 2];
					pmpcfg_w[i] <= cfg_wdata[i % 4 * 8 + 1];
					pmpcfg_r[i] <= cfg_wdata[i % 4 * 8 + 0];
					// 不支持的 A 值映射为 OFF（它是 WARL 字段）。
					pmpcfg_a[i] <= PMP_A_SUPPORTED[cfg_wdata[i % 4 * 8 + 3 +: 2]] ?
						cfg_wdata[i % 4 * 8 + 3 +: 2] : PMP_A_OFF;
				end
			end
			if (cfg_addr == PMPADDR0 + i[11:0] && !region_locked[i]) begin
				// 当 G > 0 且仅使能 PMP_MATCH_TOR 时，这里会多实现
				// 一位，不过该位在 rdata 与地址匹配中
				// 都被忽略，因此应当被修剪掉。
				if (PMP_GRAIN > 1) begin
					pmpaddr[i] <= cfg_wdata[W_ADDR-3:0] | ~(~30'h0 << (PMP_GRAIN - 1));
				end else begin
					pmpaddr[i] <= cfg_wdata[W_ADDR-3:0];
				end
			end
		end
		if (cfg_addr == PMPCFGM0) begin
			pmpcfg_m <= cfg_wdata[PMP_REGIONS-1:0] & ~PMP_HARDWIRED & {PMP_REGIONS{|EXTENSION_XH3PMPM}};
		end
	end
end


always @ (*) begin: cfg_read
	reg signed [31:0] i;
	cfg_rdata = {W_DATA{1'b0}};
	for (i = 0; i < PMP_REGIONS; i = i + 1) begin
		if (cfg_addr == PMPCFG0 + i[13:2]) begin
			cfg_rdata[i % 4 * 8 +: 8] = {
				pmpcfg_l[i],
				2'b00,
				pmpcfg_a[i],
				pmpcfg_x[i],
				pmpcfg_w[i],
				pmpcfg_r[i]
			};
		end else if (cfg_addr == PMPADDR0 + i[11:0]) begin
			if (PMP_GRAIN >= 2 && pmpcfg_a[i][1]) begin
				// A 为 NA4 或 NAPOT 时，G-2:0 位回读为全 1。
				cfg_rdata[W_ADDR-3:0] = pmpaddr[i] | ~({W_ADDR-2{1'b1}} << (PMP_GRAIN - 1));
			end else if (PMP_GRAIN >= 1 && !pmpcfg_a[i][1]) begin
				// A 为 OFF 或 TOR 时，G-1:0 位回读为全 0。
				cfg_rdata[W_ADDR-3:0] = pmpaddr[i] & ({W_ADDR-2{1'b1}} << PMP_GRAIN);
			end else begin
				cfg_rdata[W_ADDR-3:0] = pmpaddr[i];
			end
		end
	end
	if (cfg_addr == PMPCFGM0) begin
		cfg_rdata = {{32-PMP_REGIONS{1'b0}}, pmpcfg_m} & {32{|EXTENSION_XH3PMPM}};
	end
end

// ----------------------------------------------------------------------------
// 区域锁定规则

reg [PMP_REGIONS-1:0] pmp_region_is_tor;
always @ (*) begin: check_region_is_tor
	integer i;
	for (i = 0; i < PMP_REGIONS; i = i + 1) begin
		pmp_region_is_tor[i] = PMP_MATCH_TOR && pmpcfg_a[i] == PMP_A_TOR;
	end
end

assign region_locked = pmpcfg_l | ((pmpcfg_l & pmp_region_is_tor) >> 1);

// ----------------------------------------------------------------------------
// 将地址与各区域进行匹配

wire [PMP_REGIONS-1:0] d_match_napot;
wire [PMP_REGIONS-1:0] i_match_napot;
wire [PMP_REGIONS-1:0] d_match_tor;
wire [PMP_REGIONS-1:0] i_match_tor;

if (PMP_MATCH_NAPOT != 0) begin: have_napot

	reg [PMP_REGIONS-1:0] d_match_napot_r;
	reg [PMP_REGIONS-1:0] i_match_napot_r;

	assign d_match_napot = d_match_napot_r;
	assign i_match_napot = i_match_napot_r;

	// 将 PMPCFGx.A 与 PMPADDRx 译码为 32 位地址掩码和地址
	reg [W_ADDR-1:0] match_mask [0:PMP_REGIONS-1];
	reg [W_ADDR-1:0] match_addr [0:PMP_REGIONS-1];

	// 编码: （注意 ADDR 是 4 字节地址，而不是字地址）:
	// CFG.A |  ADDR    | 区域大小
	// ------+----------+------------
	// NA4   | y..yyyyy | 4 字节
	// NAPOT | y..yyyy0 | 8 字节
	// NAPOT | y..yyy01 | 16 字节
	// NAPOT | y..yy011 | 32 字节
	// NAPOT | y..y0111 | 64 字节
	// 以此类推。
	//
	// 因此除 NA4 外，规则是：检查比最低位的 0
	// 更高的所有位。

	always @ (*) begin: decode_match_mask_addr
		integer i, j;
		for (i = 0; i < PMP_REGIONS; i = i + 1) begin
			if (!pmpcfg_a[i][0]) begin
				match_mask[i] = {{W_ADDR-2{1'b1}}, 2'b00};
			end else begin
				// 位 1:0 恒为 0。位 2 为 0 是因为 NAPOT 至少 8 字节。
				match_mask[i] = {W_ADDR{1'b0}};
				for (j = 3; j < W_ADDR; j = j + 1) begin
					match_mask[i][j] = match_mask[i][j - 1] || !pmpaddr[i][j - 3];
				end
			end
			match_addr[i] = {pmpaddr[i], 2'b00} & match_mask[i];
		end
	end

	// 我们只检查每次访问中地址最低的那个字节。这样做的
	// 充分性论证见后文注释。

	always @ (*) begin: check_d_match
		integer i;
		for (i = PMP_REGIONS - 1; i >= 0; i = i - 1) begin
			d_match_napot_r[i] = pmpcfg_a[i][1] &&
				(d_addr & match_mask[i]) == match_addr[i];
			i_match_napot_r[i] = pmpcfg_a[i][1] &&
				(i_addr & match_mask[i]) == match_addr[i];
		end
	end

end else begin: no_napot

	assign d_match_napot = {PMP_REGIONS{1'b0}};
	assign i_match_napot = {PMP_REGIONS{1'b0}};

end

if (PMP_MATCH_TOR != 0) begin: have_tor

	reg [PMP_REGIONS-1:0] d_match_tor_r;
	reg [PMP_REGIONS-1:0] i_match_tor_r;
	reg [W_ADDR-1:0]      watermark [0:PMP_REGIONS-1];
	reg [PMP_REGIONS-1:0] d_lt;
	reg [PMP_REGIONS-1:0] i_lt;

	assign d_match_tor = d_match_tor_r;
	assign i_match_tor = i_match_tor_r;

	always @ (*) begin: compare
		integer i;
		for (i = 0; i < PMP_REGIONS; i = i + 1) begin
			watermark[i] = {
				pmpaddr[i][W_ADDR-3:0] & (~30'h0 << PMP_GRAIN),
				2'b00
			};
			// 各项分开相加，以促使综合器合并加法器
			d_lt[i] = (
				d_addr_addend_rs1 + d_addr_addend_imm + d_addr_addend_lspair_offs
			) < watermark[i];
			i_lt[i] = i_addr < watermark[i];
		end
	end

	wire [PMP_REGIONS-1:0] d_prev_ge = ~(d_lt << 1);
	wire [PMP_REGIONS-1:0] i_prev_ge = ~(i_lt << 1);

	always @ (*) begin: match
		integer i;
		for (i = 0; i < PMP_REGIONS; i = i + 1) begin
			d_match_tor_r[i] = d_lt[i] && d_prev_ge[i] && pmpcfg_a[i] == PMP_A_TOR;
			i_match_tor_r[i] = i_lt[i] && i_prev_ge[i] && pmpcfg_a[i] == PMP_A_TOR;
		end
	end

end else begin: no_tor

	assign d_match_tor = {PMP_REGIONS{1'b0}};
	assign i_match_tor = {PMP_REGIONS{1'b0}};

end

// ----------------------------------------------------------------------------
// 从匹配结果译码权限

// 对 load/store，我们假定任何非自然对齐的传输都会触发
// 非对齐 load/store/AMO 异常，因此只需为访问的
// 首字节译码 PMP 属性。注意规范允许我们
// 在两者同时发生的情况下，*任选*上报 load/store/AMO 访问错误
// （mcause = 5, 7）还是 load/store/AMO 非对齐异常
// （mcause = 4, 6），此处我们选择上报非对齐异常。

reg d_m; // OmniCore 扩展（M 模式不加锁）
reg d_l;
reg d_r;
reg d_w;

always @ (*) begin: check_d_match
	integer i;
	d_m = 1'b0;
	d_l = 1'b0;
	d_r = 1'b0;
	d_w = 1'b0;
	// 编号最低的匹配生效，因此从高端向下判断。这应当被
	// 推断为优先级多路选择器结构（级联 mux）。
	for (i = PMP_REGIONS - 1; i >= 0; i = i - 1) begin
		if (d_match_napot[i] || d_match_tor[i]) begin
			d_m = pmpcfg_m[i];
			d_l = pmpcfg_l[i];
			d_r = pmpcfg_r[i];
			d_w = pmpcfg_w[i];
		end
	end
end

// 指令的情况类似，因为我们检查的是*取指*而非指令本身。
// 取指始终是字大小、字对齐的。规范允许这样做:
//
// “在某些实现中，非对齐的 load、store 和指令取指
//  也可能被分解为多次访问，其中一部分可以在
//  访问错误异常发生之前已经完成。”
//
// OmniCore 会单独检查取一条非自然对齐指令过程中发生的
// 各次自然对齐取指。这意味着跨两个不同区域、
// 且两个区域都授予 X 权限的取指*是*被允许的，
// 与 RP2350 版本的 OmniCore 不同。

reg i_m; // OmniCore 扩展（M 模式不加锁）
reg i_l;
reg i_x;

always @ (*) begin: check_i_match
	integer i;
	i_m = 1'b0;
	i_l = 1'b0;
	i_x = 1'b0;
	for (i = PMP_REGIONS - 1; i >= 0; i = i - 1) begin
		if (i_match_napot[i] || i_match_tor[i]) begin
			i_m = pmpcfg_m[i];
			i_l = pmpcfg_l[i];
			i_x = pmpcfg_x[i];
		end
	end
end

// ----------------------------------------------------------------------------
// 访问规则

// M 模式可以忽略保护，除非锁定位或 M 模式位置起。

assign d_kill = (!d_m_mode || d_l || d_m) && (
	(!d_write && !d_r) ||
	( d_write && !d_w)
);

assign i_kill =	(!i_m_mode || i_l || i_m) && !i_x;

end
endgenerate

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
