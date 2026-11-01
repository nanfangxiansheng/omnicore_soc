



// 寄存器堆（register file）
// 单写端口、双读端口

`default_nettype none

module ominicore_regfile_1w2r #(
`include "ominicore_config.vh"
) (
	input wire clk,
	input wire rst_n,

	input wire [4:0]        raddr1,
	output reg [W_DATA-1:0] rdata1,

	input wire [4:0]        raddr2,
	output reg [W_DATA-1:0] rdata2,

	input wire [4:0]        waddr,
	input wire [W_DATA-1:0] wdata,
	input wire              wen
);

localparam       N_REGS      = EXTENSION_E == 0 ? 32 : 16;
localparam [4:0] REGNUM_MASK = {~|EXTENSION_E, 4'hf};

wire [4:0] raddr1_masked = raddr1 & REGNUM_MASK;
wire [4:0] raddr2_masked = raddr2 & REGNUM_MASK;
wire [4:0] waddr_masked  = waddr  & REGNUM_MASK;

generate
if (RESET_REGFILE) begin: real_dualport_reset
	// 这一部分通常总是用触发器（flop）实现
	reg [W_DATA-1:0] mem [0:N_REGS-1];

	integer i;
	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (i = 0; i < N_REGS; i = i + 1) begin
				mem[i] <= {W_DATA{1'b0}};
			end
			rdata1 <= {W_DATA{1'b0}};
			rdata2 <= {W_DATA{1'b0}};
		end else begin
			if (wen) begin
				mem[waddr_masked] <= wdata;
			end
			rdata1 <= mem[raddr1_masked];
			rdata2 <= mem[raddr2_masked];
		end
	end
end else begin: real_dualport_noreset
	// 在具有双端口（或 1R1W）BRAM 的 FPGA 上应当可被推断综合
	`ifdef YOSYS
	`ifdef FPGA_ICE40
	// 该 BRAM 上无需写到读旁路（write-to-read bypass）逻辑
	(* no_rw_check *)
	`endif
	`endif
	// 可选：在 Xilinx 上强制使用分布式 RAM 以获得更好的时序
	`ifdef OMNICORE_REGFILE_RAM_STYLE_DISTRIBUTED
	(* ram_style = "distributed" *)
	`endif
	reg [W_DATA-1:0] mem [0:N_REGS-1];

	always @ (posedge clk) begin
		if (wen) begin
			mem[waddr_masked] <= wdata;
		end
		rdata1 <= mem[raddr1_masked];
		rdata2 <= mem[raddr2_masked];
	end
end
endgenerate

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
