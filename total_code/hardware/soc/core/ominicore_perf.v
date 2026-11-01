

/*****************************************************************************
|  ominicore_perf.v — AHB5 性能计数器从机（零等待）                              |
|                                                                             |
|  紫光同创赛题: 性能分析支撑 (测评标准第 3 项: ILA / 流水线效率分析报告)            |
|                                                                             |
|  对 cache / 分支预测器的事件计数做快照, 供 CoreMark 通过总线读取并打印           |
|  I$ / D$ 命中率、分支预测错误率、CPI 分解。                                    |
|                                                                             |
\*****************************************************************************/

`default_nettype none

module ominicore_perf #(
	parameter W_DATA = 32
) (
	input  wire              clk,
	input  wire              rst_n,

	// AHB5 从机接口 (零等待)
	input  wire              hsel,
	input  wire              htrans_nseq,   // 仅 NONSEQ 参与地址相接受
	input  wire              hready,        // 总线 hready (地址相接受条件)
	input  wire              hwrite,
	input  wire [11:2]       haddr,         // 字地址
	input  wire [2:0]        hsize,
	input  wire [W_DATA-1:0] hwdata,
	output wire              hreadyout,
	output wire              hresp,
	output reg  [W_DATA-1:0] hrdata,

	// 来自 CPU 子系统的事件脉冲 (单拍)
	input  wire              evt_ic_req,
	input  wire              evt_ic_hit,
	input  wire              evt_ic_miss,
	input  wire              evt_ic_err,
	input  wire              evt_dc_req,
	input  wire              evt_dc_hit,
	input  wire              evt_dc_miss,
	input  wire              evt_dc_writeback,
	input  wire              evt_dc_bypass,
	input  wire              evt_br_alloc,
	input  wire              evt_br_mispredict,   // 实为 BTB 非误预测清除数, 见头注释 0xC
	input  wire              evt_cpu_stall
);

	localparam [31:0] PERF_ID = 32'h5A6C_3001;
	localparam        NCNT    = 13;

	wire        word_ok    = (hsize[1:0] == 2'b10);       // 只支持字访问
	// 窗口 4KB, 只实现 16 个寄存器。haddr 声明为 [11:2], 位号保留原始编号 ⇒
	//   寄存器号 = haddr[5:2]  (= s_haddr[7:2])
	//   窗口判定必须看它【上面】的位 haddr[11:6] (= s_haddr[11:6]) 是否为零。
	// 两个坑 (2026-10-17 自查): ① 写成 (haddr[5:2] <= 15) 恒真, 越界永远查不到;
	//   ② 写成 (haddr[11:4] == 0) 会把寄存器 4..15 全判成越界 (SNAP 写不进去,
	//   快照恒 0)。正确是 haddr[11:6] == 0。
	wire        addr_ok    = (haddr[11:6] == 6'd0);
	wire        accept     = hsel && htrans_nseq && hready;
	wire        accept_bad = accept && (!word_ok || !addr_ok);
	wire [3:0]  reg_idx    = haddr[5:2];
	wire        clr        = accept && !accept_bad && hwrite && (reg_idx == 4'h1);  // 仅 CTRL
	wire        snap_take  = accept && !accept_bad && hwrite && (reg_idx == 4'hF);  // 仅 SNAP

	reg        enabled;
	reg [31:0] cnt  [0:NCNT-1];
	reg [31:0] snap [0:NCNT-1];   // 原子快照: 读 0x2..0xE 一律取此
	reg        err_r;        // 非法访问挂起一拍 -> hresp 两周期

	wire [W_DATA-1:0] rd_mux =
		(reg_idx == 4'h0) ? PERF_ID          :
		(reg_idx == 4'h1) ? {31'd0, enabled} :
		(reg_idx == 4'h2) ? snap[0]  :   // ic_req
		(reg_idx == 4'h3) ? snap[1]  :   // ic_hit
		(reg_idx == 4'h4) ? snap[2]  :   // ic_miss
		(reg_idx == 4'h5) ? snap[3]  :   // ic_err
		(reg_idx == 4'h6) ? snap[4]  :   // dc_req
		(reg_idx == 4'h7) ? snap[5]  :   // dc_hit
		(reg_idx == 4'h8) ? snap[6]  :   // dc_miss
		(reg_idx == 4'h9) ? snap[7]  :   // dc_writeback
		(reg_idx == 4'hA) ? snap[8]  :   // dc_bypass
		(reg_idx == 4'hB) ? snap[9]  :   // br_alloc
		(reg_idx == 4'hC) ? snap[10] :   // br_mispredict
		(reg_idx == 4'hD) ? snap[11] :   // cpu_stall
		(reg_idx == 4'hE) ? snap[12] :   // cpu_cycle
		                    {W_DATA{1'b0}};

	integer i;

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			enabled <= 1'b0;
			err_r   <= 1'b0;
			hrdata  <= {W_DATA{1'b0}};
			for (i = 0; i < NCNT; i = i + 1) begin
				cnt[i]  <= 32'd0;
				snap[i] <= 32'd0;
			end
		end else begin
			err_r <= accept_bad;

			// ★ 清零必须优先于自增: 两者在同一个 always 块内, 若让自增排在
			//   后面, NBA 的后写会覆盖 "cnt<=0", 计数值清不掉 (tb_perf T5 抓到:
			//   写清零后 CPU_CYCLE 残留 0xB0 = 清零前累计值+1)。
			if (clr) begin
				// 写 CTRL: bit0 = 使能, 同时清零全部计数器与快照
				enabled <= hwdata[0];
				for (i = 0; i < NCNT; i = i + 1) begin
					cnt[i]  <= 32'd0;
					snap[i] <= 32'd0;
				end
			end else if (snap_take) begin
				// 写 SNAP: 全部计数器同一拍并行锁存 (原子), 计数不停
				for (i = 0; i < NCNT; i = i + 1)
					snap[i] <= cnt[i];
			end else if (accept && !accept_bad) begin
				hrdata <= rd_mux;   // 读: 数据相 = 接受后下一拍, 此处寄存正确
			end

			if (!clr && enabled) begin
				if (evt_ic_req)        cnt[0]  <= cnt[0]  + 32'd1;
				if (evt_ic_hit)        cnt[1]  <= cnt[1]  + 32'd1;
				if (evt_ic_miss)       cnt[2]  <= cnt[2]  + 32'd1;
				if (evt_ic_err)        cnt[3]  <= cnt[3]  + 32'd1;
				if (evt_dc_req)        cnt[4]  <= cnt[4]  + 32'd1;
				if (evt_dc_hit)        cnt[5]  <= cnt[5]  + 32'd1;
				if (evt_dc_miss)       cnt[6]  <= cnt[6]  + 32'd1;
				if (evt_dc_writeback)  cnt[7]  <= cnt[7]  + 32'd1;
				if (evt_dc_bypass)     cnt[8]  <= cnt[8]  + 32'd1;
				if (evt_br_alloc)      cnt[9]  <= cnt[9]  + 32'd1;
				if (evt_br_mispredict) cnt[10] <= cnt[10] + 32'd1;
				if (evt_cpu_stall)     cnt[11] <= cnt[11] + 32'd1;
				cnt[12] <= cnt[12] + 32'd1;
			end
		end
	end

	// 零等待: 地址相即接受, 读数据相紧跟一拍
	assign hreadyout = 1'b1;
	assign hresp     = err_r;

endmodule

`default_nettype wire
