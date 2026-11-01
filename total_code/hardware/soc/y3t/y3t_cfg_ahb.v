// ============================================================================
// y3t_cfg_ahb.v — Y3T 引擎配置口 AHB-Lite 从机（替代母本 engine_axil_cfg.v）
// 寄存器映射沿用 engine_axil_cfg 的 0x00~0x10 段（字节地址），剔除相机/TF/GigE 杂项。
//
// 写（字索引 = 字节地址[7:2]）：
//   0x00 CTRL    bit0 START(自清脉冲, 清粘滞位)
//   0x01 CFG0    {pad[25:24], s[21:20], k[17:16], mode[2:0]}
//   0x02 HW      {W[25:16], H[9:0]}
//   0x03 HWO     {Wo[25:16], Ho[9:0]}
//   0x04 TILE    {split[22:16], cot[14:8], cit[6:0]}
//   0x05 SH0     {shY[21:16], shM1[13:8], shM0[5:0]}
//   0x06 MULTY   cfg_multY
//   0x07 FLAG    {tiles[14:8], bits16[1], act[0]}
//   0x0B IN0     0x0C IN1     0x0D WB     0x0E OUT   （256b 字基址）
//   0x0F ROM_ADDR（RW，写 ROM_DATA 后自增）
//   0x10 ROM_DATA（W：cfg_we 单拍，waddr=ROM_ADDR，随后自增——multm 连写友好）
// 读：
//   0x00 {30'b0, err_sticky, done_sticky}（START 清）
//   0x01~0x03 CFG0/HW/HWO 读回   0x0F ROM_ADDR
//   0x13 cyc_mac(0x4C)  0x14 cyc_tot(0x50)   0x3F ID(0xFC)="Y3T1"
//
// 时序 = OmniCore boardsoc 从机统一约定（omnicore_gpio.v 同款）：
//   地址相接受 = hsel && htrans_nseq && hready && (IDLE|S_RD|S_ERR2)；
//   读零等待（接受次拍 hrdata 有效），写 1 等待拍（S_WR 采样 hwdata 提交），
//   非法地址/尺寸两周期 ERROR。
// ============================================================================
`default_nettype none

module y3t_cfg_ahb (
    input  wire        clk,
    input  wire        rst_n,
    // ---- AHB-Lite 从（boardsoc 约定口） ----
    input  wire        hsel,
    input  wire        hready,
    input  wire        htrans_nseq,
    input  wire [11:2] haddr,
    input  wire        hwrite,
    input  wire [1:0]  hsize,
    input  wire [31:0] hwdata,
    output wire        hreadyout,
    output wire [31:0] hrdata,
    output wire        hresp,
    // ---- engine_y3t 配置口 ----
    output reg         eng_start,
    input  wire        eng_done,
    input  wire        eng_err,
    output reg  [2:0]  cfg_mode,
    output reg  [1:0]  cfg_k,
    output reg  [1:0]  cfg_s,
    output reg  [1:0]  cfg_pad,
    output reg  [9:0]  cfg_H,
    output reg  [9:0]  cfg_W,
    output reg  [9:0]  cfg_Ho,
    output reg  [9:0]  cfg_Wo,
    output reg  [6:0]  cfg_cit,
    output reg  [6:0]  cfg_cot,
    output reg  [6:0]  cfg_split,
    output reg  [5:0]  cfg_shM0,
    output reg  [5:0]  cfg_shM1,
    output reg  [5:0]  cfg_shY,
    output reg  [31:0] cfg_multY,
    output reg         cfg_act,
    output reg         cfg_bits16,
    output reg  [6:0]  cfg_tiles,
    output reg  [31:0] cfg_in0,
    output reg  [31:0] cfg_in1,
    output reg  [31:0] cfg_wb,
    output reg  [31:0] cfg_out,
    output reg         cfg_we,
    output reg  [11:0] cfg_waddr,
    output reg  [31:0] cfg_wdata,
    // ---- engine 状态直通 ----
    input  wire [31:0] cyc_mac,
    input  wire [31:0] cyc_tot,
    /* 【2026-10-21 20:20】y3tram 写口观测（只读，来自 y3t_mem_ahb）
     *   0x20: ram_we 计数   0x21: 最后一次写地址 */
    input  wire [15:0] dbg_wr_cnt,
    input  wire [14:0] dbg_last_addr,
    output reg         done_sticky,
    output reg         err_sticky
);
    localparam [2:0] S_IDLE = 3'd0, S_RD = 3'd1, S_WR = 3'd2,
                     S_ERR1 = 3'd3, S_ERR2 = 3'd4;
    reg [2:0]  state;
    reg [7:0]  reg_addr;

    wire word_ok = (hsize[1:0] == 2'b10);
    wire addr_ok = (haddr[11:8] == 4'd0);          // 256B 空间
    wire accept  = hsel && htrans_nseq && hready &&
                   (state == S_IDLE || state == S_RD || state == S_ERR2);
    wire accept_bad = accept && (!word_ok || !addr_ok);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else begin
`ifdef CFG_DBG
            if (hsel || state != S_IDLE)
                $display("[CFG] t=%0t st=%0d hsel=%b htr=%b hrdy=%b ha=%h hsz=%b hw=%b acc=%b bad=%b wok=%b aok=%b",
                         $time, state, hsel, htrans_nseq, hready, haddr, hsize, hwrite,
                         accept, accept_bad, word_ok, addr_ok);
`endif
            if (accept) begin
                reg_addr <= haddr[7:2];
                state    <= accept_bad ? S_ERR1 :
                            hwrite     ? S_WR   : S_RD;
            end else begin
                case (state)
                S_WR:   state <= S_IDLE;
                S_RD:   state <= S_IDLE;
                S_ERR1: state <= S_ERR2;
                S_ERR2: state <= S_IDLE;
                default: state <= S_IDLE;
                endcase
            end
        end
    end

    assign hreadyout = (state == S_WR)   ? 1'b0 :
                       (state == S_ERR1) ? 1'b0 : 1'b1;
    assign hresp = (state == S_ERR1) || (state == S_ERR2);

    // ---------------- 寄存器堆 ----------------
    reg [11:0] rom_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            eng_start <= 1'b0; cfg_we <= 1'b0;
            cfg_waddr <= 12'd0; cfg_wdata <= 32'd0; rom_addr <= 12'd0;
            done_sticky <= 1'b0; err_sticky <= 1'b0;
            cfg_mode <= 0; cfg_k <= 0; cfg_s <= 0; cfg_pad <= 0;
            cfg_H <= 0; cfg_W <= 0; cfg_Ho <= 0; cfg_Wo <= 0;
            cfg_cit <= 0; cfg_cot <= 0; cfg_split <= 0;
            cfg_shM0 <= 0; cfg_shM1 <= 0; cfg_shY <= 0; cfg_multY <= 0;
            cfg_act <= 0; cfg_bits16 <= 0; cfg_tiles <= 0;
            cfg_in0 <= 0; cfg_in1 <= 0; cfg_wb <= 0; cfg_out <= 0;
        end else begin
            eng_start <= 1'b0;
            cfg_we    <= 1'b0;
            if (state == S_WR) begin
`ifdef CFG_DBG
                $display("[CFG-WR] t=%0t reg_addr=%h hwdata=%h", $time, reg_addr, hwdata);
`endif
                case (reg_addr)
                8'h00: if (hwdata[0]) begin
                           eng_start   <= 1'b1;
                           done_sticky <= 1'b0;
                           err_sticky  <= 1'b0;
                       end
                8'h01: begin cfg_pad   <= hwdata[25:24]; cfg_s   <= hwdata[21:20];
                             cfg_k     <= hwdata[17:16]; cfg_mode <= hwdata[2:0]; end
                8'h02: begin cfg_W     <= hwdata[25:16]; cfg_H   <= hwdata[9:0];  end
                8'h03: begin cfg_Wo    <= hwdata[25:16]; cfg_Ho  <= hwdata[9:0];  end
                8'h04: begin cfg_split <= hwdata[22:16]; cfg_cot <= hwdata[14:8];
                             cfg_cit   <= hwdata[6:0];   end
                8'h05: begin cfg_shY   <= hwdata[21:16]; cfg_shM1<= hwdata[13:8];
                             cfg_shM0  <= hwdata[5:0];   end
                8'h06: cfg_multY <= hwdata;
                8'h07: begin cfg_tiles <= hwdata[14:8]; cfg_bits16 <= hwdata[1];
                             cfg_act   <= hwdata[0];    end
                8'h0B: cfg_in0  <= hwdata;
                8'h0C: cfg_in1  <= hwdata;
                8'h0D: cfg_wb   <= hwdata;
                8'h0E: cfg_out  <= hwdata;
                8'h0F: rom_addr <= hwdata[11:0];
                8'h10: begin                       // ROM_DATA：写脉冲 + 地址自增
                           cfg_we    <= 1'b1;
                           cfg_waddr <= rom_addr;
                           cfg_wdata <= hwdata;
                           rom_addr  <= rom_addr + 12'd1;
                       end
                default: ;
                endcase
            end
            // done/err 单拍 → 粘滞（START 清除；case 已消费 START 拍，此处补锁存）
            if (eng_done && !eng_start) done_sticky <= 1'b1;
            if (eng_err  && !eng_start) err_sticky  <= 1'b1;
        end
    end

    // ---------------- 读 ----------------
    reg [31:0] rd_rdata;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_rdata <= 32'h0;
        else if (accept && !hwrite && !accept_bad) begin
            case (haddr[7:2])
            8'h00:   rd_rdata <= {30'b0, err_sticky, done_sticky};
            8'h01:   rd_rdata <= {6'b0, cfg_pad, 2'b0, cfg_s, 2'b0, cfg_k,
                                  13'b0, cfg_mode};
            8'h02:   rd_rdata <= {6'b0, cfg_W, 6'b0, cfg_H};
            8'h03:   rd_rdata <= {6'b0, cfg_Wo, 6'b0, cfg_Ho};
            8'h04:   rd_rdata <= {9'b0, cfg_split, 1'b0, cfg_cot, 1'b0, cfg_cit};
            8'h05:   rd_rdata <= {10'b0, cfg_shY, 2'b0, cfg_shM1, 2'b0, cfg_shM0};
            8'h06:   rd_rdata <= cfg_multY;
            8'h07:   rd_rdata <= {17'b0, cfg_tiles, 6'b0, cfg_bits16, cfg_act};
            8'h0F:   rd_rdata <= {20'b0, rom_addr};
            8'h13:   rd_rdata <= cyc_mac;
            8'h14:   rd_rdata <= cyc_tot;
            8'h20:   rd_rdata <= {16'd0, dbg_wr_cnt};      // y3tram 实际写入次数
            8'h21:   rd_rdata <= {17'd0, dbg_last_addr};   // 最后一次写入地址
            8'h3F:   rd_rdata <= 32'h59335431;         // "Y3T1"
            default: rd_rdata <= 32'h0;
            endcase
        end
    end
    assign hrdata = rd_rdata;

endmodule

`default_nettype wire
