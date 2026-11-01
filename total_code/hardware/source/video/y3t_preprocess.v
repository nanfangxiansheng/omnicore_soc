// ============================================================================
// y3t_preprocess.v — 相机帧 → Y3T 引擎输入 (letterbox 96x96 + int16 量化)

// ============================================================================
`default_nettype none

module y3t_preprocess #(
    parameter SRC_W   = 1280,        // 源帧宽 (相机原始)
    parameter SRC_H   = 720,         // 源帧高
    parameter DST     = 96,          // 网络输入边长
    parameter IMG_TOP = 21,          // letterbox 上边距
    parameter IMG_H   = 54,          // 缩放后图像高度 = round(SRC_H * 96/SRC_W)
    parameter PAD_VAL = 114,         // ultralytics LetterBox 填充色
    parameter ADDR_W  = 14           // 字地址位宽 (最大 9215 < 2^14)
) (
    // ---------------- 相机像素域 ----------------
    input  wire              clk,
    input  wire              rst_n,
    input  wire              vsync,        // 帧同步, 高有效
    input  wire              de,           // 像素有效
    input  wire [15:0]       pix,          // RGB565 {R[15:11], G[10:5], B[4:0]}

    // ---------------- 控制 ----------------
    input  wire              arm,          // 1 = 允许写 IN 区 (需 clk 域内)

    // ---------------- 输出 (→ 异步 FIFO → soc 侧写 y3tram) ----------------
    // 注意: 这是标准 valid/ready 握手, valid/addr/word **必须是组合输出**。
    //   若把 valid 打一拍、却用当拍 ready 决定推进, 消费者要下一拍才看到
    //   valid&&ready, ready 一抖动就会丢字 (2026-10-19 首版仿真丢了 323/9216 字)。
    output wire [255:0]      out_word,
    output wire [ADDR_W-1:0] out_addr,
    output wire              out_valid,
    input  wire              out_ready,    // FIFO 未满
    output wire              out_last,     // 本帧最后一个字

    // ---------------- 观测 ----------------
    output reg  [7:0]        frame_cnt,    // 完成的帧数
    output reg               pad_done,     // pad 行已写好 (撤 arm 会清)
    output reg  [15:0]       sy_dbg,       // 当前源行号
    output reg  [15:0]       sx_dbg,       // 当前源列号
    output reg  [1:0]        st_dbg,       // 0=idle 1=采集 2=发射
    output wire [15:0]       yp_dbg,       // 当前目标行对应的源行 y0
    output wire [7:0]        xd_dbg        // 本行已产出的横向点数 (发射时应=96)
);

    // ========================================================================
    // 0. 常量
    // ========================================================================
    localparam [31:0] STEP_Q16 = (32'd65536 * SRC_W + DST/2) / DST;
    localparam [31:0] INIT_Q16 = (32'd65536 * (SRC_W - DST) + DST) / (2*DST);

    localparam [15:0] DST_W16 = DST;
    localparam [15:0] NPAD    = DST - IMG_H;          // 上下 pad 行数之和 (=42)

    // 精确量化常量: q(114) = 114*128 + (115>>1) = 14592 + 57 = 14649
    localparam [15:0] PAD_Q = (PAD_VAL << 7) + ((PAD_VAL + 1) >> 1);

    // ========================================================================
    // 1. RGB565 → 8bit
    // ========================================================================
    wire [4:0] r5 = pix[15:11];
    wire [5:0] g6 = pix[10:5];
    wire [4:0] b5 = pix[4:0];
    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};

    // ========================================================================
    // 2. 运算函数
    // ========================================================================

    // 双线性混合 (8bit, 四舍五入): out = a + round(((b-a) * ph) / 256)
    // 注意 >>> 优先级高于 +, 必须显式加括号
    function [7:0] blend8;
        input [7:0] a;
        input [7:0] b;
        input [7:0] ph;
        reg signed [9:0]  d;
        reg signed [18:0] p;
        reg signed [18:0] pr;
        reg signed [10:0] s;
        begin
            d  = $signed({1'b0, b}) - $signed({1'b0, a});
            p  = d * $signed({1'b0, ph});
            pr = (p + 19'sd128) >>> 8;            // 范围 ±254, 11bit 足够
            s  = $signed({3'b000, a}) + $signed(pr[10:0]);
            if (s < 0)         blend8 = 8'd0;
            else if (s > 255)  blend8 = 8'd255;
            else               blend8 = s[7:0];
        end
    endfunction

    // 相位取整: acc 低 16 位是 1/65536, 取高 8 位并四舍五入
    function [7:0] ph8_of;
        input [31:0] acc;
        reg [16:0] t;
        begin
            t      = {1'b0, acc[15:0]} + 17'd128;
            ph8_of = (t[16]) ? 8'd255 : t[15:8];
        end
    endfunction

    // 精确量化: x*128 + (x+1)/2, x=255 饱和到 32767
    function [15:0] q8to16;
        input [7:0] x;
        begin
            if (x == 8'd255)
                q8to16 = 16'd32767;
            else
                q8to16 = ({8'd0, x} << 7) + (({8'd0, x} + 16'd1) >> 1);
        end
    endfunction

    // ========================================================================
    // 3. 行缓存: 横向缩放后的 96 点 x 24bit (R[23:16] G[15:8] B[7:0])
    //    2 x 96 x 24 = 4608 bit, 纯分布式 RAM, 零 DRM
    // ========================================================================
    reg [23:0] lbuf_a [0:DST-1];
    reg [23:0] lbuf_b [0:DST-1];

    // ========================================================================
    // 4. 状态与寄存器
    // ========================================================================
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_CAP  = 2'd1;
    localparam [1:0] S_EMIT = 2'd2;

    reg [1:0]  st;
    reg        vsync_d;
    reg        arm_d;
    wire       vs_rise  = vsync & ~vsync_d;
    wire       arm_rise = arm & ~arm_d;

    reg [31:0] xacc;          // 横向坐标累加器 (当前采的源行)
    reg [31:0] yacc;          // 纵向坐标累加器 (当前目标行)
    reg [7:0]  xd;            // 当前源行已产出的横向点数 0..95
    reg [5:0]  yd;            // 当前目标行 0..IMG_H-1
    reg [15:0] xp;            // xacc 整数部分 (源列)
    reg [15:0] yp;            // yacc 整数部分 (源行)
    reg [7:0]  yph;           // 纵向相位

    reg [7:0]  a_r, a_g, a_b; // 横向前一个源像素

    reg        pad_run;       // pad 扫描进行中
    reg [5:0]  pad_row;       // 0..NPAD-1
    reg [6:0]  pad_col;       // 0..95
    reg [6:0]  emit_x;        // 0..95
    reg [1:0]  cap_sel;       // 00=不采 01=bufA(=yp) 10=bufB(=yp+1)

    // ---- 组合辅助 (统一位宽, 避免隐式截断/扩展告警) ----
    wire [31:0] xacc_nxt = xacc + STEP_Q16;
    wire [31:0] yacc_nxt = yacc + STEP_Q16;
    wire [7:0]  cur_ph8  = ph8_of(xacc);
    wire [15:0] nxt_yp    = yacc_nxt[31:16];

    wire        line_start = de & (sx_dbg == 16'd0);
    wire        line_end   = de & (sx_dbg == SRC_W - 1);
    wire        row_is_a   = (sy_dbg == yp);
    wire        row_is_b   = (sy_dbg == yp + 16'd1);
    wire        pix_hit_a  = de & (cap_sel != 2'b00) & (sx_dbg == xp)         & (xd < DST);
    wire        pix_hit_b  = de & (cap_sel != 2'b00) & (sx_dbg == xp + 16'd1) & (xd < DST);

    // 地址计算 (16bit 中间量, 最后截到 ADDR_W)
    wire [15:0] row_emit  = IMG_TOP[15:0] + {10'd0, yd};
    wire [15:0] addr_emit = row_emit * DST_W16 + {9'd0, emit_x};
    wire [15:0] row_pad   = (pad_row < IMG_TOP) ? {10'd0, pad_row}
                                                : (IMG_H[15:0] + {10'd0, pad_row});
    wire [15:0] addr_pad  = row_pad * DST_W16 + {9'd0, pad_col};

    // ========================================================================
    // 5. 输出呈现 (组合): S_EMIT 独占, 其余时间给 pad 扫描
    // ========================================================================
    wire emit_present = (st == S_EMIT);
    wire pad_present  = pad_run & ~emit_present;

    wire [255:0] emit_word = {208'd0,
                              q8to16(blend8(lbuf_a[emit_x][7:0],   lbuf_b[emit_x][7:0],   yph)),
                              q8to16(blend8(lbuf_a[emit_x][15:8],  lbuf_b[emit_x][15:8],  yph)),
                              q8to16(blend8(lbuf_a[emit_x][23:16], lbuf_b[emit_x][23:16], yph))};

    assign out_valid = emit_present | pad_present;
    assign out_word  = emit_present ? emit_word : {208'd0, PAD_Q, PAD_Q, PAD_Q};
    assign out_addr  = emit_present ? addr_emit[ADDR_W-1:0] : addr_pad[ADDR_W-1:0];
    assign out_last  = emit_present & (emit_x == DST - 1) & (yd == IMG_H - 1);

    assign yp_dbg = yp;
    assign xd_dbg = xd;

    // ========================================================================
    // 5. 主逻辑
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d    <= 1'b0;
            arm_d      <= 1'b0;
            st         <= S_IDLE;
            xacc       <= 32'd0;
            yacc       <= INIT_Q16;
            xd         <= 8'd0;
            yd         <= 6'd0;
            xp         <= 16'd0;
            yp         <= INIT_Q16[31:16];
            yph        <= 8'd0;
            a_r <= 8'd0; a_g <= 8'd0; a_b <= 8'd0;
            pad_run    <= 1'b0;
            pad_row    <= 6'd0;
            pad_col    <= 7'd0;
            emit_x     <= 7'd0;
            cap_sel    <= 2'd0;
            frame_cnt  <= 8'd0;
            pad_done   <= 1'b0;
            sy_dbg     <= 16'd0;
            sx_dbg     <= 16'd0;
            st_dbg     <= 2'd0;
        end else begin
            vsync_d   <= vsync;
            arm_d     <= arm;
            st_dbg    <= st;

            // ---------------------------------------------------------------
            // 源像素计数 (与 video_scale_down 同口径: 每行恰好 SRC_W 个有效像素)
            // sy 上界用 SRC_H 保护: 多余的行使 sy 停在 SRC_H, 永远不等于 yp/yp+1
            // ---------------------------------------------------------------
            if (vs_rise) begin
                sy_dbg <= 16'd0;
                sx_dbg <= 16'd0;
            end else if (de) begin
                if (sx_dbg == SRC_W - 1) begin
                    sx_dbg <= 16'd0;
                    if (sy_dbg < SRC_H) sy_dbg <= sy_dbg + 16'd1;
                end else begin
                    sx_dbg <= sx_dbg + 16'd1;
                end
            end

            // ---------------------------------------------------------------
            // pad 扫描: arm 上升沿触发, 与 vsync 无关 (地址与图像行不重叠)
            // ---------------------------------------------------------------
            if (arm_rise) begin
                pad_run  <= 1'b1;
                pad_row  <= 6'd0;
                pad_col  <= 7'd0;
                pad_done <= 1'b0;
            end
            if (!arm) begin
                pad_run  <= 1'b0;
                pad_done <= 1'b0;      // 撤 arm 后要求下次重新补 pad
            end

            // ---------------------------------------------------------------
            // 帧起始: 重装纵向坐标
            // ---------------------------------------------------------------
            if (vs_rise) begin
                yd   <= 6'd0;
                yacc <= INIT_Q16;
                yp   <= INIT_Q16[31:16];
                yph  <= ph8_of(INIT_Q16);
            end

            // ---------------------------------------------------------------
            // 输出口仲裁: S_EMIT 期间主路径独占, 其余时间 pad 扫描可用
            // ---------------------------------------------------------------
            if (pad_run && out_ready && (st != S_EMIT)) begin
                if (pad_col == DST - 1) begin
                    pad_col <= 7'd0;
                    if ({10'd0, pad_row} == NPAD - 16'd1) begin
                        pad_run  <= 1'b0;
                        pad_done <= 1'b1;
                    end else begin
                        pad_row <= pad_row + 6'd1;
                    end
                end else begin
                    pad_col <= pad_col + 7'd1;
                end
            end

            case (st)
            // ---------------------------------------------------------------
            // 采集: 行首决定 cap_sel, 行内做横向双线性
            // ---------------------------------------------------------------
            S_IDLE, S_CAP: begin
                if (line_start) begin
                    xacc <= INIT_Q16;
                    xp   <= INIT_Q16[31:16];
                    xd   <= 8'd0;
                    if (!arm)         cap_sel <= 2'b00;
                    else if (row_is_a) cap_sel <= 2'b01;
                    else if (row_is_b) cap_sel <= 2'b10;
                    else               cap_sel <= 2'b00;
                    st <= S_CAP;
                end

                // 命中 a 点 → 锁存前一个源像素
                if (pix_hit_a) begin
                    a_r <= r8;
                    a_g <= g8;
                    a_b <= b8;
                end

                // 命中 b 点 → 横向混合, 写入行缓存, 推进坐标
                if (pix_hit_b) begin
                    if (cap_sel == 2'b01)
                        lbuf_a[xd[6:0]] <= {blend8(a_r, r8, cur_ph8),
                                            blend8(a_g, g8, cur_ph8),
                                            blend8(a_b, b8, cur_ph8)};
                    else
                        lbuf_b[xd[6:0]] <= {blend8(a_r, r8, cur_ph8),
                                            blend8(a_g, g8, cur_ph8),
                                            blend8(a_b, b8, cur_ph8)};
                    xacc <= xacc_nxt;
                    xp   <= xacc_nxt[31:16];
                    xd   <= xd + 8'd1;
                end

                // 行尾: 若本行是 yp+1, 立刻发射该目标行
                if (line_end && cap_sel == 2'b10 && arm) begin
                    emit_x <= 7'd0;
                    st     <= S_EMIT;
                end
            end

            // ---------------------------------------------------------------
            // 发射: 纵向双线性 (a=bufA=y0, b=bufB=y0+1) + 量化打包
            // ---------------------------------------------------------------
            S_EMIT: begin
                if (out_ready) begin
                    if (emit_x == DST - 1) begin
                        cap_sel  <= 2'b00;
                        if (yd == IMG_H - 1) begin
                            frame_cnt <= frame_cnt + 8'd1;
                            st        <= S_IDLE;
                        end else begin
                            yd   <= yd + 6'd1;
                            yacc <= yacc_nxt;
                            yp   <= nxt_yp;
                            yph  <= ph8_of(yacc_nxt);
                            st   <= S_IDLE;
                        end
                    end else begin
                        emit_x <= emit_x + 7'd1;
                    end
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
