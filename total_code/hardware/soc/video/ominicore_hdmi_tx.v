// ============================================================================
// ominicore_hdmi_tx.v — 720p@60 显示控制器 + overlay 叠框 (APP-PROFILE S3)
// ----------------------------------------------------------------------------
// 结构:
//   clk 域(50MHz, AHB): 单拍读主机从 DDR 显示缓冲取 32bit 字(2 像素 RGB565)
//             → 装双钟 FIFO(格雷指针); 读地址按"行×WPL+行内字"计数器自由
//             流转, FIFO 保序 → 生产/消费序列天然对齐(同为 WPF 循环)
//   pclk 域(74.25MHz): VESA 720p60 时序 → FIFO 出流(偶像素拍出字) →
//             RGB565→RGB888 位复制扩展 → overlay 2px 边框叠加 → 24bit
//             并行输出(至 MS7210)
//   从机口  : CTRL / BASE0 / BASE1 / STATUS / 8 组框寄存器(偶字 xy, 奇字 wh|色)
// ============================================================================
`default_nettype none

module ominicore_hdmi_tx #(
	parameter H_ACTIVE  = 1280,      // 720p60
	parameter V_ACTIVE  = 720,
	parameter H_FP = 110, H_SYNC = 40, H_BP = 220,   // 行合计 1650
	parameter V_FP = 5,   V_SYNC = 5,  V_BP = 20,    // 场合计 750
	parameter FIFO_DEPTH = 1024,     // 双钟 FIFO(32bit 字, 2 的幂)
	parameter N_BOX = 8              // overlay 框数
) (
	input  wire        clk,            // 系统钟 50MHz (AHB/FIFO 写侧)
	input  wire        rst_n,
	input  wire        pclk,           // 像素钟 74.25MHz (FIFO 读侧/时序)
	input  wire        rst_n_p,        // 像素钟域复位(已同步)
	// ---- AHB 读主机 (显示缓冲取数, clk 域) ----
	output reg  [31:0] m_haddr,
	output wire        m_hwrite,       // 恒 0
	output wire [2:0]  m_hsize,        // 恒 WORD
	output reg  [1:0]  m_htrans,
	input  wire        m_hready,
	input  wire [31:0] m_hrdata,
	input  wire        m_hresp,        // 占位监听
	// ---- AHB 从机 (配置/overlay, clk 域, 零等待) ----
	input  wire        s_hsel,
	input  wire        s_hreadyin,
	input  wire [31:0] s_haddr,
	input  wire        s_hwrite,
	input  wire [1:0]  s_htrans,
	input  wire [31:0] s_hwdata,
	output reg         s_hreadyout,
	output reg         s_hresp,
	output reg  [31:0] s_hrdata,
	// ---- 显示缓冲选择 (连 DVP 乒乓互补, clk 域) ----
	input  wire        sel_buf,        // 0=BASE0 1=BASE1
	// ---- HDMI 输出 (pclk 域, 至 MS7210) ----
	output reg  [23:0] hd_rgb,         // RGB888
	output reg         hd_de,
	output reg         hd_hs,
	output reg         hd_vs
);

	// ---------------- 参数派生 ----------------
	localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;   // 1650
	localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;   // 750
	localparam WPL = H_ACTIVE / 2;                          // 每行字数 640
	localparam WPF = WPL * V_ACTIVE;                        // 每帧字数 460800
	localparam FB_BITS = $clog2(FIFO_DEPTH);
	localparam [31:0] WPL32 = WPL[31:0];

	// ================= 从机寄存器 =================
	localparam [3:0] R_CTRL   = 4'd0;   // [0]=en [1]=overlay_en
	localparam [3:0] R_BASE0  = 4'd1;   // 显示缓冲 A 基址(字节, 字对齐)
	localparam [3:0] R_BASE1  = 4'd2;
	localparam [3:0] R_STATUS = 4'd3;   // [0]=en [1]=ovl_en [2]=vsync [3]=de
	                                    // [31:22]=h_cnt[10:1]
	localparam [3:0] R_BOX0   = 4'd4;   // 框 i = 0x10 + i*8:
	                                    //   偶字 [10:0]=x [26:16]=y (写即使能)
	                                    //   奇字 [10:0]=w [26:16]=h [31:28]=色号
	localparam [3:0] R_BOXN   = R_BOX0 + N_BOX*2;

	reg         ctrl_en, ctrl_ovl_en;
	reg  [31:0] base0_q, base1_q;
	reg  [10:0] box_x [0:N_BOX-1];
	reg  [10:0] box_y [0:N_BOX-1];
	reg  [10:0] box_w [0:N_BOX-1];
	reg  [10:0] box_h [0:N_BOX-1];
	reg  [3:0]  box_c [0:N_BOX-1];
	reg         box_en [0:N_BOX-1];

	wire        s_wr = s_hsel && s_hreadyin && s_hwrite && s_htrans[1];
	wire        s_rd = s_hsel && s_hreadyin && !s_hwrite && s_htrans[1];
	wire [3:0]  s_reg = s_haddr[5:2];
	reg  [2:0]  bidx;                  // 框号(=s_reg[3:1]-2, 自然回绕覆盖 0..7)
	integer     ri;                    // 复位循环

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ctrl_en <= 1'b0; ctrl_ovl_en <= 1'b0;
			base0_q <= 32'h0; base1_q <= 32'h0;
			for (ri = 0; ri < N_BOX; ri = ri + 1) begin
				box_x[ri[2:0]] <= 11'h0; box_y[ri[2:0]] <= 11'h0;
				box_w[ri[2:0]] <= 11'h0; box_h[ri[2:0]] <= 11'h0;
				box_c[ri[2:0]] <= 4'h0;  box_en[ri[2:0]] <= 1'b0;
			end
		end else if (s_wr) begin
			bidx = s_reg[3:1] - 2'd2;   // R_BOX0=4 起, 每 2 字一框
			if (s_reg == R_CTRL) begin
				ctrl_en     <= s_hwdata[0];
				ctrl_ovl_en <= s_hwdata[1];
			end else if (s_reg == R_BASE0) base0_q <= s_hwdata;
			else if (s_reg == R_BASE1) base1_q <= s_hwdata;
			else if (s_reg >= R_BOX0 && s_reg < R_BOXN) begin
				if (s_reg[0] == 1'b0) begin
					box_x[bidx] <= s_hwdata[10:0];
					box_y[bidx] <= s_hwdata[26:16];
					box_en[bidx] <= (|s_hwdata[26:16]) | (|s_hwdata[10:0]);
				end else begin
					box_w[bidx] <= s_hwdata[10:0];
					box_h[bidx] <= s_hwdata[26:16];
					box_c[bidx] <= s_hwdata[31:28];
				end
			end
		end
	end

	// STATUS 快照由 pclk 域同步而来(见 pclk 段)
	reg [31:0] status_c;
	always @(*) begin
		case (s_reg)
			R_CTRL:   s_hrdata = {30'h0, ctrl_ovl_en, ctrl_en};
			R_BASE0:  s_hrdata = base0_q;
			R_BASE1:  s_hrdata = base1_q;
			R_STATUS: s_hrdata = status_c;
			default:  s_hrdata = 32'h0;
		endcase
	end

	// ================= clk 域: FIFO 写侧 + 单拍读主机 =================
	reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
	reg [FB_BITS:0] wbin, rbin;
	reg [FB_BITS:0] wgray, rgray;
	reg [FB_BITS:0] rgray_w1, rgray_w2;   // 读指针 → 写域(2FF)
	reg [FB_BITS:0] wgray_r1, wgray_r2;   // 写指针 → 读域(2FF)

	wire [FB_BITS:0] wbin_n  = wbin + {{FB_BITS{1'b0}},1'b1};
	wire [FB_BITS:0] wgray_n = (wbin_n>>1) ^ wbin_n;
	wire fifo_full = (wgray_n[FB_BITS -: 2] == ~rgray_w2[FB_BITS -: 2]) &&
	                 (wgray_n[FB_BITS-2:0] == rgray_w2[FB_BITS-2:0]);

	wire fifo_empty = (rgray == wgray_r2);  // 空: 读格雷 == 同步写格雷

	// 读主机状态: 0=空闲 1=地址相(NSEQ) 2=数据相(捕获 m_hrdata)
	reg [2:0]  m_state;
	reg        cur_sel;                  // vsync 沿锁存的缓冲选择(防撕裂)
	wire [31:0] base_w = (cur_sel ? base1_q : base0_q);   // 字节地址
	reg [10:0] rd_vline;                 // 取数行号(0..V_ACTIVE-1 自由流转)
	reg [10:0] rd_hword;                 // 行内字(0..WPL-1)
	wire [31:0] rd_word_off = {{10{1'b0}}, rd_vline} * WPL32 +
	                          {{21{1'b0}}, rd_hword};
	// 突发起动: 未满过半(供水 50M 字/s ≫ 行内消费 37M 字/s 峰值)
	localparam [2:0] M_IDLE = 3'd0, M_ADDR = 3'd1;
	localparam [2:0] M_D1 = 3'd2, M_D2 = 3'd3, M_D3 = 3'd4;
	wire [FB_BITS:0] fill_lvl = wbin - rbin;
	wire       rd_req = ctrl_en && !fifo_full && (fill_lvl < FIFO_DEPTH[FB_BITS:0]/2);
	reg        after_d3;               // beat4 数据相随背靠背落在 M_ADDR 拍

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			m_state <= M_IDLE; m_haddr <= 32'h0; m_htrans <= 2'b00;
			cur_sel <= 1'b0;
			rd_vline <= 11'd0; rd_hword <= 11'd0;
		end else begin
			cur_sel <= sel_lat;
			after_d3 <= 1'b0;
			case (m_state)
				M_IDLE: begin
					m_htrans <= 2'b00;
					if (rd_req) begin
						// 字节地址 = 基址字节 + 字偏移×4 (字对齐即可)
						m_haddr  <= base_w + (rd_word_off << 2);
						m_htrans <= 2'b10;       // NONSEQ
						m_state  <= M_ADDR;
					end
				end
				M_ADDR: begin                      // 地址相 → 等接受
					if (m_hready) begin
						m_htrans <= 2'b01;       // SEQ ×3
						m_haddr  <= m_haddr + 32'h4;
						m_state  <= M_D1;
					end
				end
				M_D1, M_D2: begin                  // SEQ 数据拍
					if (m_hready) begin
						m_haddr  <= m_haddr + 32'h4;
						m_state  <= m_state + 3'd1;
					end
				end
				M_D3: begin                        // 突发末数据拍
					if (m_hready) begin
						// 背靠背: 有需求则无缝衔接下一突发(免 IDLE 空泡,
						// 供给 40M 字/s > 行内峰值 37M); 无需求回空闲
						after_d3 <= rd_req;    // beat4 数据相随背靠背落入 M_ADDR
						if (rd_req) begin
							m_haddr  <= base_w + (rd_word_off << 2);
							m_htrans <= 2'b10;   // NONSEQ
							m_state  <= M_ADDR;
						end else begin
							m_htrans <= 2'b00;
							m_state  <= M_IDLE;
						end
					end
				end
				default: m_state <= M_IDLE;
			endcase
		end
	end

	// FIFO 写: 4 个数据相拍全捕获 — D1/D2/D3 拍 + 背靠背时落入 M_ADDR 拍
	// 的 beat4(AHB 流水: 各数据相呈现其前一地址相的数据)
	wire fifo_wr = m_hready && (m_state == M_D1 || m_state == M_D2 ||
	           m_state == M_D3 || (m_state == M_ADDR && after_d3));
	reg [63:0] fw_cnt = 64'd0;
	always @(posedge clk) begin
		if (fifo_wr) begin
			fw_cnt <= fw_cnt + 64'd1;
			if (fw_cnt[63:0] >= 6 && fw_cnt < 22)
				$display("[FW %0t] #%0d data=%h wbin=%0d st=%0d after_d3=%b",
				         $time, fw_cnt, m_hrdata, wbin, m_state, after_d3);
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wbin <= {(FB_BITS+1){1'b0}}; wgray <= {(FB_BITS+1){1'b0}}; rgray_w1 <= {(FB_BITS+1){1'b0}}; rgray_w2 <= {(FB_BITS+1){1'b0}};
		end else begin
			rgray_w1 <= rgray;
			rgray_w2 <= rgray_w1;
			if (fifo_wr) begin
				fifo_mem[wbin[FB_BITS-1:0]] <= m_hrdata;
				wbin  <= wbin_n;
				wgray <= wgray_n;
				// 取数指针流转(与显示消费同序: 行内→行尾→下一行)
				if (rd_hword == WPL[10:0] - 11'd1) begin
					rd_hword <= 11'd0;
					rd_vline <= (rd_vline == V_ACTIVE-1) ? 11'd0 : rd_vline + 11'd1;
				end else begin
					rd_hword <= rd_hword + 11'd1;
				end
			end
		end
	end

	// ============ pclk 域: 选择锁存 + FIFO 读侧 + 时序 + 出流 + overlay ============
	reg        sel_p1, sel_p2, sel_lat;  // sel_buf 跨域 + vsync 沿锁存(防撕裂)
	reg [10:0] h_cnt, v_cnt;
	reg  [31:0] px_word_r;               // 偶像素拍缓存的出流字
	reg  [15:0] px_odd;                  // 奇像素(缓存字高半字)
	wire h_active = (h_cnt < H_ACTIVE);
	wire v_active = (v_cnt < V_ACTIVE);
	wire de       = h_active && v_active;
	wire hsync    = (h_cnt >= H_ACTIVE + H_FP) && (h_cnt < H_ACTIVE + H_FP + H_SYNC);
	wire vsync    = (v_cnt >= V_ACTIVE + V_FP) && (v_cnt < V_ACTIVE + V_FP + V_SYNC);

	wire [FB_BITS:0] rbin_n  = rbin + {{FB_BITS{1'b0}},1'b1};
	wire [FB_BITS:0] rgray_n = (rbin_n>>1) ^ rbin_n;
	wire fifo_rd = de && !h_cnt[0] && !fifo_empty;
	wire [31:0] fifo_rdata = fifo_mem[rbin[FB_BITS-1:0]];

	// 出流像素: 偶像素=当拍出字低半字; 奇像素=缓存字高半字
	wire [15:0] px_odd_q = px_odd;
	wire [15:0] px_cur = h_cnt[0] ? px_odd_q : fifo_rdata[15:0];

	// overlay: 像素落使能框 2px 边框带 → 框色
	reg  [3:0] hit_c;
	reg        ov_hit;
	integer    oi;
	reg [10:0] cx2, cy2;
	always @(*) begin
		ov_hit = 1'b0; hit_c = 4'h0;
		for (oi = N_BOX-1; oi >= 0; oi = oi - 1) begin
			cx2 = box_x[oi[2:0]] + box_w[oi[2:0]];
			cy2 = box_y[oi[2:0]] + box_h[oi[2:0]];
			if (box_en[oi[2:0]] && ctrl_ovl_en &&
			    (h_cnt >= box_x[oi[2:0]]) && (h_cnt < cx2) &&
			    (v_cnt >= box_y[oi[2:0]]) && (v_cnt < cy2) &&
			    !((h_cnt >= box_x[oi[2:0]] + 11'd2) && (h_cnt < cx2 - 11'd2) &&
			      (v_cnt >= box_y[oi[2:0]] + 11'd2) && (v_cnt < cy2 - 11'd2))) begin
				ov_hit = 1'b1; hit_c = box_c[oi[2:0]];
			end
		end
	end

	reg [23:0] ov_color;
	always @(*) begin
		case (hit_c)
			4'd0: ov_color = 24'hFF0000;
			4'd1: ov_color = 24'h00FF00;
			4'd2: ov_color = 24'h0000FF;
			4'd3: ov_color = 24'hFFFF00;
			4'd4: ov_color = 24'h00FFFF;
			4'd5: ov_color = 24'hFF00FF;
			4'd6: ov_color = 24'hFFFFFF;
			default: ov_color = 24'hFFA500;
		endcase
	end

	// RGB565→RGB888 位复制扩展: R8={r4,r4[4:2]} G8={g5,g5[5:4]} B8={b4,b4[4:2]}
	function [23:0] px888; input [15:0] p;
		px888 = {p[15:11], p[15:13], p[10:5], p[10:9], p[4:0], p[4:2]};
	endfunction

	always @(posedge pclk or negedge rst_n_p) begin
		if (!rst_n_p) begin
			sel_p1 <= 1'b0; sel_p2 <= 1'b0; sel_lat <= 1'b0;
			rbin <= {(FB_BITS+1){1'b0}}; rgray <= {(FB_BITS+1){1'b0}}; wgray_r1 <= {(FB_BITS+1){1'b0}}; wgray_r2 <= {(FB_BITS+1){1'b0}};
			h_cnt <= 11'd0; v_cnt <= 11'd0;
			hd_de <= 1'b0; hd_hs <= 1'b0; hd_vs <= 1'b0; hd_rgb <= 24'h0;
			px_word_r <= 32'h0; px_odd <= 16'h0;
		end else begin
			sel_p1 <= sel_buf;
			sel_p2 <= sel_p1;
			wgray_r1 <= wgray;                 // 写指针 → 读域 (2FF 同步)
			wgray_r2 <= wgray_r1;
			if ((v_cnt == V_ACTIVE + V_FP) && (h_cnt == 11'd0))
				sel_lat <= sel_p2;           // 场同步起始: 应用新缓冲选择
			// FIFO 读
			if (fifo_rd) begin
				rbin  <= rbin_n;
				rgray <= rgray_n;
			end
			// 时序计数
			if (h_cnt == H_TOTAL-1) begin
				h_cnt <= 11'd0;
				v_cnt <= (v_cnt == V_TOTAL-1) ? 11'd0 : v_cnt + 11'd1;
			end else begin
				h_cnt <= h_cnt + 11'd1;
			end
			// 出流: 偶像素拍出字并直用低半字; 奇像素拍用缓存字高半字
			if (de) begin
				if (!h_cnt[0]) begin
					px_word_r <= fifo_rdata;
					px_odd    <= fifo_rdata[31:16];
				end
				hd_rgb <= ov_hit ? ov_color : px888(px_cur);
			end
			hd_de <= de;
			hd_hs <= hsync;
			hd_vs <= vsync;
		end
	end

	// ================= STATUS 跨域快照 =================
	reg [10:0] v_cnt_c1, v_cnt_c2;
	reg [1:0]  flg_p, flg_c1, flg_c2;    // {vsync, v_active}
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			v_cnt_c1 <= 11'd0; v_cnt_c2 <= 11'd0;
			flg_p <= 2'b00; flg_c1 <= 2'b00; flg_c2 <= 2'b00;
		end else begin
			v_cnt_c1 <= v_cnt;   v_cnt_c2 <= v_cnt_c1;
			flg_p  <= {vsync, v_active};
			flg_c1 <= flg_p;     flg_c2 <= flg_c1;
		end
	end
	always @(*) begin
		status_c = {5'd0, h_cnt[10:1], 2'b00, v_cnt_c2, flg_c2, ctrl_ovl_en, ctrl_en};
	end

	assign m_hwrite = 1'b0;
	assign m_hsize  = 3'b010;
	wire unused_tie = m_hresp;

endmodule

`default_nettype wire
