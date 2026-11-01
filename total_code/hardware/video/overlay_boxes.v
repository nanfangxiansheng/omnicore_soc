// ============================================================================
// overlay_boxes.v — 检测框叠画 (ZIGUANG 融合工程新增)
//
// AHB 寄存器 (soc_clk, 字访问)
//   0x00+3n MIN:   [27:16]=ymin [11:0]=xmin
//   0x01+3n MAX:   [27:16]=ymax [11:0]=xmax     (n = 0..7)
//   0x02+3n COLOR: [15:0] RGB565 边框色
//   0x18 EN:       [7:0] 框使能
//   0x1F VERSION:  RO, 每次写 +1
//
// 跨域: 寄存器写频率极低 (每帧一次量级) → pclk 侧 2FF 同步 upd 翻转,
//   在 vsync 沿整块锁存影子寄存器 (准静态写, 安全)。
// 视频流: 2 拍流水描 2px 边框, hs/vs/de/data 同步延迟 2 拍输出。
// 坐标系: 显示活动区像素计数, de 上升沿起新行, vsync 沿清零行号。
// ============================================================================
`default_nettype none

module overlay_boxes #(
    parameter NBOX      = 8,
    parameter BORDER_PX = 3'd2    // 边框厚度 (像素)
) (
    // ---- AHB 从机口 (soc_clk) ----
    input  wire        ahb_clk,
    input  wire        ahb_rst_n,
    input  wire        hsel,
    input  wire        hready,
    input  wire        htrans_nseq,
    input  wire [7:2]  haddr,
    input  wire        hwrite,
    input  wire [2:0]  hsize,
    input  wire [31:0] hwdata,
    output wire        hreadyout,
    output wire [31:0] hrdata,
    output wire        hresp,
    // ---- 视频流 (pclk 域, RGB565) ----
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [15:0] in_data,
    input  wire [3:0]  frame_idx,   // ZIGUANG 融合: 双目帧序号 (camera 域, 内部 2FF 同步)
    output wire [6:0]  x_shift_o,   /* v52: 读侧像素延迟级数 (调试旋钮, 0..127) */
    output wire [4:0]  x_delay_o,   /* v55: 显示时序延迟级数 (0..31), 竖缝主旋钮 */
    output wire        lb_en_o,     /* v63: 1=行走缓存新路径 0=退回老路径（运行时 A/B，不用重烧）*/
    output wire [8:0]  lb_ofs_o,    /* v64: 行缓存槽位线性偏移 (0..319, 一档 16px) */
    output wire        out_hs,
    output wire        out_vs,
    output wire        out_de,
    output wire [15:0] out_data
);

    // ======================================================================
    // AHB 从机 (soc_clk)
    // ======================================================================
    reg  [31:0] box_min  [0:NBOX-1];   // {ymin[27:16], xmin[11:0]}
    reg  [31:0] box_max  [0:NBOX-1];   // {ymax[27:16], xmax[11:0]}
    reg  [15:0] box_color[0:NBOX-1];
    /* ---- v47 文字层寄存器 (0x1C..0x1F) ---- */
    reg  [15:0] txt_ch01;      /* 0x70: {ch1[7:0], ch0[7:0]} */
    reg  [15:0] txt_ch23;      /* 0x74: {ch3, ch2} */
    reg  [15:0] txt_color;     /* 0x78: RGB565 文字色 */
    reg  [6:0]  x_shift_r;     /* 0x6C (word 0x1B): 读侧像素延迟级数 */
    reg  [4:0]  x_delay_r;     /* 0x68 (word 0x1A): 显示时序延迟级数 (v55) */
    reg         lb_en_r;       /* 0x64 (word 0x19) bit0   : v63 行缓存使能, 复位默认 1 */
    reg  [8:0]  lb_ofs_r;      /* 0x64 (word 0x19) bit9:1 : v64 槽位线性偏移, 复位 0 */
    reg  [31:0] txt_pos_en;    /* 0x7C: {en[31], y[21:12], x[11:2]?} 见下方注释 */
    reg  [7:0]  box_en;
    reg  [7:0]  version;
    reg         upd_tgl;

    reg [3:0]  frame_idx_s1, frame_idx_sync;   // frame_idx 2FF → ahb_clk 读 (camera 域)

    wire word_ok = (hsize[1:0] == 2'b10);
    wire accept  = hsel && htrans_nseq && hready;

    // ZIGUANG 修复 (2026-10-09): 原 commit/读 mux 用数据相的【活】hwrite/haddr
    // —— CPU 的 AHB 主口背靠背流水时, 数据相当拍的 haddr/hwrite 已是下一笔
    // 事务的, 导致写落错寄存器、读返回下一笔的数据 (OVL_FRAME 轮询被
    // 交替的 mtime 读污染成常量, [3] 帧同步卡死; [1] VER 误读)。
    // 对齐 bram_slave 口径: 接受沿锁存, 提交/读 mux 全用锁存值。
    reg accepted;
    reg        acc_wr;
    reg [7:2]  acc_reg;
    always @(posedge ahb_clk or negedge ahb_rst_n) begin
        if (!ahb_rst_n) begin
            accepted <= 1'b0; acc_wr <= 1'b0; acc_reg <= 6'h0;
        end else if (accept) begin
            accepted <= word_ok;
            acc_wr   <= hwrite;
            acc_reg  <= haddr[7:2];
        end else begin
            accepted <= 1'b0;
        end
    end
    // ZIGUANG 修复 (2026-10-16, v8): commit 原为 accepted && acc_wr, 而 accepted 在
    // 没有新事务时会【一直保持高】=> 一次 AHB 写会连续提交多个周期。实测 VER = 13 写 x
    // ~9 拍 = 117 (且随布局变化: 4/225/26/117), 框寄存器被重复写同值虽无功能影响,
    // 但 version 计数失真, 不能再当诊断量用。修法: 用 accepted 的前一拍做边沿限定,
    // 让 commit 只在 accepted 拉高后的【第一拍】有效 (单拍脉冲)。
    reg accepted_d;
    always @(posedge ahb_clk or negedge ahb_rst_n) begin
        if (!ahb_rst_n) accepted_d <= 1'b0;
        else            accepted_d <= accepted;
    end
    wire commit = accepted && acc_wr && !accepted_d;

    integer k;
    always @(posedge ahb_clk or negedge ahb_rst_n) begin
        if (!ahb_rst_n) begin
            box_en <= 8'h0;
            txt_ch01 <= 16'd0; txt_ch23 <= 16'd0;
            txt_color <= 16'hFFFF; txt_pos_en <= 32'd0;
            x_shift_r <= 7'd0;
            x_delay_r <= 5'd0;
            lb_en_r   <= 1'b1;      /* v63: 上电默认走行缓存 */
            lb_ofs_r  <= 9'd1;      /* v65: 复位默认 +1 档(16px)。2026-10-27 板上实测:
                                     *   ofs=0 时左右两半最左各有一条 16px 竖缝,
                                     *   ofs=1 缝消失且长时间稳定 ⇒ 固化, 上电即有,
                                     *   不必每次用 set_lb.py 写 0x64。
                                     *   运行期仍可改 0x64 覆盖。 */
            version <= 8'h0;
            upd_tgl <= 1'b0;
            for (k = 0; k < NBOX; k = k + 1) begin
                box_min[k]   <= 32'h0;
                box_max[k]   <= 32'h0;
                box_color[k] <= 16'h0;
            end
        end else if (commit) begin
            case (acc_reg)
            6'h00: box_min[0] <= hwdata;
            6'h01: box_max[0] <= hwdata;
            6'h02: box_color[0] <= hwdata[15:0];
            6'h03: box_min[1] <= hwdata;
            6'h04: box_max[1] <= hwdata;
            6'h05: box_color[1] <= hwdata[15:0];
            6'h06: box_min[2] <= hwdata;
            6'h07: box_max[2] <= hwdata;
            6'h08: box_color[2] <= hwdata[15:0];
            6'h09: box_min[3] <= hwdata;
            6'h0a: box_max[3] <= hwdata;
            6'h0b: box_color[3] <= hwdata[15:0];
            6'h0c: box_min[4] <= hwdata;
            6'h0d: box_max[4] <= hwdata;
            6'h0e: box_color[4] <= hwdata[15:0];
            6'h0f: box_min[5] <= hwdata;
            6'h10: box_max[5] <= hwdata;
            6'h11: box_color[5] <= hwdata[15:0];
            6'h12: box_min[6] <= hwdata;
            6'h13: box_max[6] <= hwdata;
            6'h14: box_color[6] <= hwdata[15:0];
            6'h15: box_min[7] <= hwdata;
            6'h16: box_max[7] <= hwdata;
            6'h17: box_color[7] <= hwdata[15:0];
            6'h18: box_en <= hwdata[7:0];
            6'h19: begin lb_en_r <= hwdata[0]; lb_ofs_r <= hwdata[9:1]; end
            6'h1a: x_delay_r  <= hwdata[4:0];
            6'h1b: x_shift_r  <= hwdata[6:0];
            6'h1c: txt_ch01  <= hwdata[15:0];
            6'h1d: txt_ch23  <= hwdata[15:0];
            6'h1e: txt_color <= hwdata[15:0];
            6'h1f: txt_pos_en<= hwdata;
            default: ;
            endcase
            if (acc_reg <= 6'h18 || (acc_reg >= 6'h1a && acc_reg <= 6'h1b) || (acc_reg >= 6'h1c && acc_reg <= 6'h1f)) begin
                version <= version + 8'd1;
                upd_tgl <= ~upd_tgl;
            end
        end
    end

    // 寄存器读
    reg [31:0] rd_rdata_r;
    always @(*) begin
        case (acc_reg)
        6'h00: rd_rdata_r = box_min[0];
        6'h01: rd_rdata_r = box_max[0];
        6'h02: rd_rdata_r = {16'd0, box_color[0]};
        6'h03: rd_rdata_r = box_min[1];
        6'h04: rd_rdata_r = box_max[1];
        6'h05: rd_rdata_r = {16'd0, box_color[1]};
        6'h06: rd_rdata_r = box_min[2];
        6'h07: rd_rdata_r = box_max[2];
        6'h08: rd_rdata_r = {16'd0, box_color[2]};
        6'h09: rd_rdata_r = box_min[3];
        6'h0a: rd_rdata_r = box_max[3];
        6'h0b: rd_rdata_r = {16'd0, box_color[3]};
        6'h0c: rd_rdata_r = box_min[4];
        6'h0d: rd_rdata_r = box_max[4];
        6'h0e: rd_rdata_r = {16'd0, box_color[4]};
        6'h0f: rd_rdata_r = box_min[5];
        6'h10: rd_rdata_r = box_max[5];
        6'h11: rd_rdata_r = {16'd0, box_color[5]};
        6'h12: rd_rdata_r = box_min[6];
        6'h13: rd_rdata_r = box_max[6];
        6'h14: rd_rdata_r = {16'd0, box_color[6]};
        6'h15: rd_rdata_r = box_min[7];
        6'h16: rd_rdata_r = box_max[7];
        6'h17: rd_rdata_r = {16'd0, box_color[7]};
        6'h18: rd_rdata_r = {24'd0, box_en};
        6'h19: rd_rdata_r = {21'd0, lb_ofs_r, 1'b0, lb_en_r};  // RO: [0]=lb_en [9:1]=槽位偏移 (21+9+2=32, 原来写 22 位→33 位被截, ofs 落到 bit2..10)
        6'h1a: rd_rdata_r = {27'd0, x_delay_r};        // RO: v55 时序延迟级数
        6'h1b: rd_rdata_r = {25'd0, x_shift_r};        // RO: v52 读侧像素延迟级数
        6'h1e: rd_rdata_r = {28'd0, frame_idx_sync};   // RO: [1:0]=cam0 帧序 [3:2]=cam1 帧序
        6'h1f: rd_rdata_r = {24'd0, version};
        default: rd_rdata_r = 32'h0;
        endcase
    end
    assign hrdata = rd_rdata_r;
    assign x_shift_o = x_shift_r;
    assign x_delay_o = x_delay_r;
    assign lb_en_o   = lb_en_r;
    assign lb_ofs_o  = lb_ofs_r;
    assign hreadyout = 1'b1;   // 两拍提交口径, 无等待周期
    assign hresp = 1'b0;

    // ZIGUANG 修复: frame_idx 2FF 原在 pclk 块内, 而读 mux (ahb_clk) 直接
    // 引用 —— 跨域无同步。移到 ahb_clk 域 (camera/TB 域 → ahb_clk)。
    always @(posedge ahb_clk or negedge ahb_rst_n) begin
        if (!ahb_rst_n) begin
            frame_idx_s1   <= 4'd0;
            frame_idx_sync <= 4'd0;
        end else begin
            frame_idx_s1   <= frame_idx;
            frame_idx_sync <= frame_idx_s1;
        end
    end

    // ======================================================================
    // pclk 域: 影子锁存 + 叠画
    // ======================================================================
    reg [31:0] sh_min  [0:NBOX-1];
    reg [31:0] sh_max  [0:NBOX-1];
    reg [15:0] sh_color[0:NBOX-1];
    reg  [7:0] sh_en;
    reg        upd_tgl_s1, upd_tgl_s2, upd_tgl_prev;
    wire       upd_edge = (upd_tgl_s2 != upd_tgl_prev);
    reg        upd_pending;                    // ZIGUANG 修复 (2026-10-15)
    reg        in_vs_d1, in_vs_d2;
    wire       vs_edge = (in_vs_d2 != in_vs_d1);

    // ---- x/y 计数 ----
    reg [11:0] x_cnt, y_cnt;
    reg        in_de_d1;


    /* ============ v47 文字层（5x7 点阵，4 字符/行） ============
     * 寄存器:
     *   0x70 {ch1,ch0}  0x74 {ch3,ch2}  (ASCII)
     *   0x78 RGB565 文字色
     *   0x7C {bit31=enable, y[21:12], x[11:2]*4?}
     *       实际布局: txt_pos_en[31]=en, [21:12]=y, [11:2]=x, [1:0]=0 (字对齐)
     * 字符格 6x8 像素（5x7 字形 + 1px 间距/行距）。
     * 未使能（默认）时输出与原来完全一致。
     */
    wire        txt_en  = txt_pos_en[31];
    wire [11:0] txt_x   = {txt_pos_en[11:2], 2'b00};   /* x = [11:2]<<2 */
    wire [11:0] txt_y   = {1'b0, txt_pos_en[22:12]};   /* y = [22:12] */

    /* 5x7 字形: 每个字符 7 行, 每行 5 bit */
    function [4:0] glyph;
        input [2:0] row;
        input [7:0] ch;
        begin
            case ({ch, row})
            /* A */ {8'h41,3'd0}: glyph=5'b01110; {8'h41,3'd1}: glyph=5'b10001;
                    {8'h41,3'd2}: glyph=5'b10001; {8'h41,3'd3}: glyph=5'b11111;
                    {8'h41,3'd4}: glyph=5'b10001; {8'h41,3'd5}: glyph=5'b10001;
                    {8'h41,3'd6}: glyph=5'b10001;
            /* B */ {8'h42,3'd0}: glyph=5'b11110; {8'h42,3'd1}: glyph=5'b10001;
                    {8'h42,3'd2}: glyph=5'b11110; {8'h42,3'd3}: glyph=5'b10001;
                    {8'h42,3'd4}: glyph=5'b10001; {8'h42,3'd5}: glyph=5'b11110;
                    {8'h42,3'd6}: glyph=5'b00000;
            /* C */ {8'h43,3'd0}: glyph=5'b01110; {8'h43,3'd1}: glyph=5'b10001;
                    {8'h43,3'd2}: glyph=5'b10000; {8'h43,3'd3}: glyph=5'b10000;
                    {8'h43,3'd4}: glyph=5'b10001; {8'h43,3'd5}: glyph=5'b01110;
                    {8'h43,3'd6}: glyph=5'b00000;
            /* E */ {8'h45,3'd0}: glyph=5'b11111; {8'h45,3'd1}: glyph=5'b10000;
                    {8'h45,3'd2}: glyph=5'b11110; {8'h45,3'd3}: glyph=5'b10000;
                    {8'h45,3'd4}: glyph=5'b10000; {8'h45,3'd5}: glyph=5'b11111;
                    {8'h45,3'd6}: glyph=5'b00000;
            /* L */ {8'h4c,3'd0}: glyph=5'b10000; {8'h4c,3'd1}: glyph=5'b10000;
                    {8'h4c,3'd2}: glyph=5'b10000; {8'h4c,3'd3}: glyph=5'b10000;
                    {8'h4c,3'd4}: glyph=5'b10000; {8'h4c,3'd5}: glyph=5'b11111;
                    {8'h4c,3'd6}: glyph=5'b00000;
            /* M */ {8'h4d,3'd0}: glyph=5'b10001; {8'h4d,3'd1}: glyph=5'b11011;
                    {8'h4d,3'd2}: glyph=5'b10101; {8'h4d,3'd3}: glyph=5'b10001;
                    {8'h4d,3'd4}: glyph=5'b10001; {8'h4d,3'd5}: glyph=5'b10001;
                    {8'h4d,3'd6}: glyph=5'b00000;
            /* N */ {8'h4e,3'd0}: glyph=5'b10001; {8'h4e,3'd1}: glyph=5'b11001;
                    {8'h4e,3'd2}: glyph=5'b10101; {8'h4e,3'd3}: glyph=5'b10011;
                    {8'h4e,3'd4}: glyph=5'b10001; {8'h4e,3'd5}: glyph=5'b10001;
                    {8'h4e,3'd6}: glyph=5'b00000;
            /* O */ {8'h4f,3'd0}: glyph=5'b01110; {8'h4f,3'd1}: glyph=5'b10001;
                    {8'h4f,3'd2}: glyph=5'b10001; {8'h4f,3'd3}: glyph=5'b10001;
                    {8'h4f,3'd4}: glyph=5'b10001; {8'h4f,3'd5}: glyph=5'b01110;
                    {8'h4f,3'd6}: glyph=5'b00000;
            /* P */ {8'h50,3'd0}: glyph=5'b11110; {8'h50,3'd1}: glyph=5'b10001;
                    {8'h50,3'd2}: glyph=5'b11110; {8'h50,3'd3}: glyph=5'b10000;
                    {8'h50,3'd4}: glyph=5'b10000; {8'h50,3'd5}: glyph=5'b10000;
                    {8'h50,3'd6}: glyph=5'b00000;
            /* R */ {8'h52,3'd0}: glyph=5'b11110; {8'h52,3'd1}: glyph=5'b10001;
                    {8'h52,3'd2}: glyph=5'b11110; {8'h52,3'd3}: glyph=5'b10100;
                    {8'h52,3'd4}: glyph=5'b10010; {8'h52,3'd5}: glyph=5'b10001;
                    {8'h52,3'd6}: glyph=5'b00000;
            /* S */ {8'h53,3'd0}: glyph=5'b01111; {8'h53,3'd1}: glyph=5'b10000;
                    {8'h53,3'd2}: glyph=5'b01110; {8'h53,3'd3}: glyph=5'b00001;
                    {8'h53,3'd4}: glyph=5'b00001; {8'h53,3'd5}: glyph=5'b11110;
                    {8'h53,3'd6}: glyph=5'b00000;
            /* T */ {8'h54,3'd0}: glyph=5'b11111; {8'h54,3'd1}: glyph=5'b00100;
                    {8'h54,3'd2}: glyph=5'b00100; {8'h54,3'd3}: glyph=5'b00100;
                    {8'h54,3'd4}: glyph=5'b00100; {8'h54,3'd5}: glyph=5'b00100;
                    {8'h54,3'd6}: glyph=5'b00000;
            /* U */ {8'h55,3'd0}: glyph=5'b10001; {8'h55,3'd1}: glyph=5'b10001;
                    {8'h55,3'd2}: glyph=5'b10001; {8'h55,3'd3}: glyph=5'b10001;
                    {8'h55,3'd4}: glyph=5'b10001; {8'h55,3'd5}: glyph=5'b01110;
                    {8'h55,3'd6}: glyph=5'b00000;
            /* Y */ {8'h59,3'd0}: glyph=5'b10001; {8'h59,3'd1}: glyph=5'b10001;
                    {8'h59,3'd2}: glyph=5'b01010; {8'h59,3'd3}: glyph=5'b00100;
                    {8'h59,3'd4}: glyph=5'b00100; {8'h59,3'd5}: glyph=5'b00100;
                    {8'h59,3'd6}: glyph=5'b00000;
            /* 0 */ {8'h30,3'd0}: glyph=5'b01110; {8'h30,3'd1}: glyph=5'b10001;
                    {8'h30,3'd2}: glyph=5'b10011; {8'h30,3'd3}: glyph=5'b10101;
                    {8'h30,3'd4}: glyph=5'b11001; {8'h30,3'd5}: glyph=5'b01110;
                    {8'h30,3'd6}: glyph=5'b00000;
            /* 1 */ {8'h31,3'd0}: glyph=5'b00100; {8'h31,3'd1}: glyph=5'b01100;
                    {8'h31,3'd2}: glyph=5'b00100; {8'h31,3'd3}: glyph=5'b00100;
                    {8'h31,3'd4}: glyph=5'b00100; {8'h31,3'd5}: glyph=5'b01110;
                    {8'h31,3'd6}: glyph=5'b00000;
            /* 2 */ {8'h32,3'd0}: glyph=5'b01110; {8'h32,3'd1}: glyph=5'b10001;
                    {8'h32,3'd2}: glyph=5'b00010; {8'h32,3'd3}: glyph=5'b00100;
                    {8'h32,3'd4}: glyph=5'b01000; {8'h32,3'd5}: glyph=5'b11111;
                    {8'h32,3'd6}: glyph=5'b00000;
            /* 3 */ {8'h33,3'd0}: glyph=5'b11110; {8'h33,3'd1}: glyph=5'b00010;
                    {8'h33,3'd2}: glyph=5'b01100; {8'h33,3'd3}: glyph=5'b00010;
                    {8'h33,3'd4}: glyph=5'b00001; {8'h33,3'd5}: glyph=5'b11110;
                    {8'h33,3'd6}: glyph=5'b00000;
            /* 4 */ {8'h34,3'd0}: glyph=5'b00010; {8'h34,3'd1}: glyph=5'b00110;
                    {8'h34,3'd2}: glyph=5'b01010; {8'h34,3'd3}: glyph=5'b10010;
                    {8'h34,3'd4}: glyph=5'b11111; {8'h34,3'd5}: glyph=5'b00010;
                    {8'h34,3'd6}: glyph=5'b00000;
            /* 5 */ {8'h35,3'd0}: glyph=5'b11111; {8'h35,3'd1}: glyph=5'b10000;
                    {8'h35,3'd2}: glyph=5'b11110; {8'h35,3'd3}: glyph=5'b00001;
                    {8'h35,3'd4}: glyph=5'b00001; {8'h35,3'd5}: glyph=5'b11110;
                    {8'h35,3'd6}: glyph=5'b00000;
            /* 6 */ {8'h36,3'd0}: glyph=5'b00110; {8'h36,3'd1}: glyph=5'b01000;
                    {8'h36,3'd2}: glyph=5'b11110; {8'h36,3'd3}: glyph=5'b10001;
                    {8'h36,3'd4}: glyph=5'b10001; {8'h36,3'd5}: glyph=5'b01110;
                    {8'h36,3'd6}: glyph=5'b00000;
            /* 7 */ {8'h37,3'd0}: glyph=5'b11111; {8'h37,3'd1}: glyph=5'b00001;
                    {8'h37,3'd2}: glyph=5'b00010; {8'h37,3'd3}: glyph=5'b00100;
                    {8'h37,3'd4}: glyph=5'b01000; {8'h37,3'd5}: glyph=5'b01000;
                    {8'h37,3'd6}: glyph=5'b00000;
            /* 8 */ {8'h38,3'd0}: glyph=5'b01110; {8'h38,3'd1}: glyph=5'b10001;
                    {8'h38,3'd2}: glyph=5'b01110; {8'h38,3'd3}: glyph=5'b10001;
                    {8'h38,3'd4}: glyph=5'b10001; {8'h38,3'd5}: glyph=5'b01110;
                    {8'h38,3'd6}: glyph=5'b00000;
            /* 9 */ {8'h39,3'd0}: glyph=5'b01110; {8'h39,3'd1}: glyph=5'b10001;
                    {8'h39,3'd2}: glyph=5'b10001; {8'h39,3'd3}: glyph=5'b01111;
                    {8'h39,3'd4}: glyph=5'b00010; {8'h39,3'd5}: glyph=5'b01100;
                    {8'h39,3'd6}: glyph=5'b00000;
            /* % */ {8'h25,3'd0}: glyph=5'b11001; {8'h25,3'd1}: glyph=5'b11010;
                    {8'h25,3'd2}: glyph=5'b00010; {8'h25,3'd3}: glyph=5'b00100;
                    {8'h25,3'd4}: glyph=5'b01000; {8'h25,3'd5}: glyph=5'b01011;
                    {8'h25,3'd6}: glyph=5'b10011;
            /* . */ {8'h2e,3'd0}: glyph=5'b00000; {8'h2e,3'd1}: glyph=5'b00000;
                    {8'h2e,3'd2}: glyph=5'b00000; {8'h2e,3'd3}: glyph=5'b00000;
                    {8'h2e,3'd4}: glyph=5'b00000; {8'h2e,3'd5}: glyph=5'b01100;
                    {8'h2e,3'd6}: glyph=5'b01100;
            /* : */ {8'h3a,3'd0}: glyph=5'b00000; {8'h3a,3'd1}: glyph=5'b01100;
                    {8'h3a,3'd2}: glyph=5'b01100; {8'h3a,3'd3}: glyph=5'b00000;
                    {8'h3a,3'd4}: glyph=5'b01100; {8'h3a,3'd5}: glyph=5'b01100;
                    {8'h3a,3'd6}: glyph=5'b00000;
            default: glyph = 5'b00000;
            endcase
        end
    endfunction

    /* ===== stage1: 文字命中（与检测框同排流水） =====
     * v48 修正两处（2026-10-25 上板暴露）：
     *  ① 列的定位原来是 `txt_col = txt_dx[2:0]`——没减去该字符的起始偏移，
     *     于是第 2/3/4 个字符整体右移 2px 且丢掉最右一列 =>
     *     上板看到 "BOTT" 变 "BCT"、"SCRN" 变 "scn"。现按字符格减去基址。
     *  ② 点阵只有 1x（5x7 像素），在 640x720 上几乎看不清 => 改成 4x 放大
     *     （字形 20x28，4 字符共 96px 宽）。
     * 字符格 = 6*TSCALE 宽、8*TSCALE 高；TSCALE 必须是 2 的幂（用移位取列/行）。
     * 注意: Verilog-2001 不允许对函数调用结果做位选 (func()[i])，
     *       必须先把字形落到 wire 上再取位。
     */
    localparam integer TSCALE = 4;          /* 4x 放大 */
    localparam integer TCELL  = 6*TSCALE;   /* 24 px/字符 */

    reg  txt_hit_r, txt_hit;
    reg  [11:0] txt_dx, txt_dy;
    reg  [7:0]  txt_ch_sel;
    reg  [3:0]  txt_col_i;                  /* 0..5 列 */
    reg  [4:0]  txt_row_i;                  /* 0..7 行（v72: 原为 4 位, 装不下 txt_dy[6:2] 的 5 位） */

    wire [11:0] txt_dx_c0 = txt_dx;
    wire [11:0] txt_dx_c1 = txt_dx - 12'd24;
    wire [11:0] txt_dx_c2 = txt_dx - 12'd48;
    wire [11:0] txt_dx_c3 = txt_dx - 12'd72;
    wire [11:0] txt_dxo   = (txt_dx < 12'd24) ? txt_dx_c0 :
                            (txt_dx < 12'd48) ? txt_dx_c1 :
                            (txt_dx < 12'd72) ? txt_dx_c2 : txt_dx_c3;

    wire [4:0]  txt_glyph = glyph(txt_row_i[2:0], txt_ch_sel);
    wire [2:0]  txt_bidx  = 3'd4 - txt_col_i[2:0];
    wire        txt_pix   = txt_glyph[txt_bidx];

    always @(*) begin
        txt_dx = x_cnt - txt_x;
        txt_dy = y_cnt - txt_y;
        txt_row_i = txt_dy[6:2];            /* /4，dy<32 时得到 0..7 */
        txt_col_i = txt_dxo[5:2];           /* /4，得 0..5 */
        if      (txt_dx < 12'd24) txt_ch_sel = txt_ch01[7:0];
        else if (txt_dx < 12'd48) txt_ch_sel = txt_ch01[15:8];
        else if (txt_dx < 12'd72) txt_ch_sel = txt_ch23[7:0];
        else                      txt_ch_sel = txt_ch23[15:8];
    end

    // ---- 流水 stage1 寄存 ----
    reg [NBOX-1:0] x_hit_r;
    reg [11:0]     x_r, y_r;
    reg            de_r1, hs_r1, vs_r1;
    reg [15:0]     data_r1;

    // ---- 流水 stage2 寄存 ----
    reg        hit_r;
    reg [15:0] hit_color_r;
    reg        de_r2, hs_r2, vs_r2;
    reg [15:0] data_r2;

    integer j;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            upd_tgl_s1 <= 1'b0; upd_tgl_s2 <= 1'b0; upd_tgl_prev <= 1'b0;
            upd_pending <= 1'b0;
            in_vs_d1 <= 1'b0; in_vs_d2 <= 1'b0;
            in_de_d1 <= 1'b0;
            x_cnt <= 12'd0; y_cnt <= 12'd0;
            sh_en <= 8'h0;
            x_hit_r <= {NBOX{1'b0}};
            x_r <= 12'd0; y_r <= 12'd0;
            de_r1 <= 1'b0; hs_r1 <= 1'b0; vs_r1 <= 1'b0; data_r1 <= 16'd0;
            hit_r <= 1'b0; hit_color_r <= 16'd0;
            txt_hit <= 1'b0;
            de_r2 <= 1'b0; hs_r2 <= 1'b0; vs_r2 <= 1'b0; data_r2 <= 16'd0;
        end else begin
            // 跨域同步 + 影子锁存
            upd_tgl_s1 <= upd_tgl;
            upd_tgl_s2 <= upd_tgl_s1;
            upd_tgl_prev <= upd_tgl_s2;
            in_vs_d1 <= in_vs;
            in_vs_d2 <= in_vs_d1;

            // ---- 影子寄存器提交 ----
            // ZIGUANG 修复 (2026-10-15): 原实现为 `if (vs_edge && upd_edge)` ——
            // 要求"寄存器写脉冲(1 个 pclk 宽)"与"显示 vsync 沿(每 16.7ms 一次)"
            // 落在同一拍。固件上电时把 13 个寄存器连写完成, 撞上 vsync 沿的概率
            // 约 1e-5 ⇒ 影子寄存器**永不更新** ⇒ 检测框永远画不出来
            // (实测: 屏幕四角静态框不可见, 而 [1] VER=0x0d 证明 AHB 写全部成功)。
            // 改为"待提交标志": 写过就置位, 下一次 vsync 沿整块锁存后清零。
            // 数据在 vsync 前已稳定 ≥2 个 pclk (upd_tgl 经 2FF 同步), 准静态安全;
            // 最坏情况框晚一帧出现, 无功能影响。
            if (upd_edge)      upd_pending <= 1'b1;
            else if (vs_edge)  upd_pending <= 1'b0;

            if (vs_edge && upd_pending) begin
                for (k = 0; k < NBOX; k = k + 1) begin
                    sh_min[k]   <= box_min[k];
                    sh_max[k]   <= box_max[k];
                    sh_color[k] <= box_color[k];
                end
                sh_en <= box_en;
            end

            // x/y 计数
            in_de_d1 <= in_de;
            if (vs_edge) y_cnt <= 12'd0;
            else if (in_de && !in_de_d1) begin
                x_cnt <= 12'd0;
                y_cnt <= y_cnt + 12'd1;
            end else if (in_de) begin
                x_cnt <= x_cnt + 12'd1;
            end

            // stage1: x 命中 (data_r1 对应当前 x_cnt/y_cnt)
            de_r1 <= in_de; hs_r1 <= in_hs; vs_r1 <= in_vs; data_r1 <= in_data;
            for (j = 0; j < NBOX; j = j + 1)
                x_hit_r[j] <= sh_en[j] &&
                              (x_cnt >= sh_min[j][11:0]) &&
                              (x_cnt <= sh_max[j][11:0]);
            x_r <= x_cnt;
            y_r <= y_cnt;
            /* v47 文字命中: 在 4 字符格内且字形该位为 1 */
            /* ★ v72 修复（2026-10-29）: 板上"识别字符在屏幕上一整串"的根因。
               原来两处叠加：
                 ① txt_row_i 声明 [3:0]，而 txt_dy[6:2] 是 5 位 —— 赋值丢 bit6，
                    行号实际变成 (dy mod 64)>>2，于是【每 64 行纵向重复一整份字符】。
                    720p 一屏能重复 11 份，肉眼看就是"一整串"。横向 txt_col_i 是 4 位，
                    正好装下 txt_dxo[5:2]，所以只有纵向出问题（仿真 T2 证实横向恒定 93px）。
                 ② y_cnt < txt_y 时 txt_dy = y_cnt - txt_y 无符号下溢，低 6 位又落回 0..27，
                    在文字上方 101~128 行处多画一份残影（仿真 T3 复现）。
               直接判 txt_dy < 28（7 行 × 4 px/行）一并解决，且不依赖 row 的位宽。 */
            txt_hit_r <= txt_en && (txt_dy < 12'd28) &&
                         (txt_dx < 12'd96) && (txt_col_i < 4'd5) && txt_pix;

            // stage2: y 命中 + 边框 + 颜色
            de_r2 <= de_r1; hs_r2 <= hs_r1; vs_r2 <= vs_r1; data_r2 <= data_r1;
            txt_hit <= txt_hit_r;
            hit_r <= 1'b0;
            hit_color_r <= 16'd0;
            for (j = 0; j < NBOX; j = j + 1) begin
                if (x_hit_r[j] &&
                    (y_r >= sh_min[j][27:16]) && (y_r <= sh_max[j][27:16]) &&
                    ((x_r - sh_min[j][11:0]) < BORDER_PX ||
                     (sh_max[j][11:0] - x_r) < BORDER_PX ||
                     (y_r - sh_min[j][27:16]) < BORDER_PX ||
                     (sh_max[j][27:16] - y_r) < BORDER_PX)) begin
                    hit_r <= 1'b1;
                    hit_color_r <= sh_color[j];
                end
            end
        end
    end

    assign out_hs   = hs_r2;
    assign out_vs   = vs_r2;
    assign out_de   = de_r2;
    /* v47: 文字层优先级最高（未使能时 txt_hit 恒 0，行为与原来一致） */
    assign out_data = txt_hit ? txt_color :
                      (hit_r ? hit_color_r : data_r2);

endmodule

`default_nettype wire
