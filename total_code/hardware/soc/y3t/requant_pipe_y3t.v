// ============================================================================
// requant_pipe_y3t.v — Y3T 反量化+激活流水线
// 与母本的唯一差别：cfg_act 由 SiLU LUT 改为**直接 ReLU**（Y3T-F 训练即 ReLU，
// 训推一致；不再烧 LUT）。其余逐位一致：
//   组累加：y_g = RHU(acc_g × multM, shiftM)          （每输入通道组一次）
//   末组：  y13 = Σ_g y_g + bias_q13                   （Q6.13 域，int32 钳位）
//   激活：  v13 = act ? ReLU(y13) : y13
//   输出：  out = sat_bits( RHU(v13 × multY, shiftY) )（int16 ±32767 / int8 ±127）
// RHU：右移前加 1<<(shift-1)；shift=0 时直通。
// 流水线：S1 组乘累加 → S2 bias+钳位 → S3 ReLU → S4 输出乘饱和。4 级。
// 对拍基准：tools/y3t/golden_np.py conv_job_np（G3 bit-exact 门）。
// ============================================================================
module requant_pipe_y3t (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire        in_first,
    input  wire        in_last,
    input  wire signed [47:0] in_acc,
    input  wire signed [31:0] in_multM,    // 该组该通道 multM（正）
    input  wire [5:0]  in_shiftM,
    input  wire signed [31:0] in_bias_q13,
    input  wire signed [31:0] in_multY,
    input  wire [5:0]  in_shiftY,
    input  wire        in_bits16,          // 1=int16，0=int8
    input  wire        in_act,             // 1=ReLU
    output reg         out_valid,
    output reg  signed [15:0] out_data
);
    // ---------------- RHU 函数（int64 域） ----------------
    function signed [63:0] rhu_shift;
        input signed [63:0] a;
        input [5:0] sh;
        begin
            if (sh == 0) rhu_shift = a;
            else         rhu_shift = (a + (64'sd1 <<< (sh - 1))) >>> sh;
        end
    endfunction

    // ---------------- S1a：组乘积寄存（母本 75MHz 两段拆分原样保留） ----------------
    reg signed [63:0] s1a_prod;
    reg  [5:0]        s1a_shiftM;
    reg               s1a_first, s1a_last, s1a_valid;
    reg signed [31:0] s1a_bias;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1a_prod <= 64'sd0; s1a_shiftM <= 6'd0;
            s1a_first <= 1'b0; s1a_last <= 1'b0; s1a_valid <= 1'b0;
            s1a_bias <= 32'sd0;
        end else begin
            s1a_valid <= in_valid;
            if (in_valid) begin
                s1a_prod   <= $signed(in_acc) * $signed(in_multM);
                s1a_shiftM <= in_shiftM;
                s1a_first  <= in_first;
                s1a_last   <= in_last;
                s1a_bias   <= in_bias_q13;
            end
        end
    end

    // ---------------- S1b：RHU + 组累加 ----------------
    reg signed [63:0] y13_acc;
    reg signed [31:0] s1_bias;   // bias 随数据打拍：反压/连发时 S2 不能读端口当前值
    reg               s1_last, s1_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y13_acc  <= 64'sd0;
            s1_bias  <= 32'sd0;
            s1_last  <= 1'b0;
            s1_valid <= 1'b0;
        end else if (s1a_valid) begin
            y13_acc  <= (s1a_first ? 64'sd0 : y13_acc) +
                        rhu_shift(s1a_prod, s1a_shiftM);
            s1_bias  <= s1a_bias;
            s1_last  <= s1a_last;
            s1_valid <= 1'b1;
        end else begin
            s1_valid <= 1'b0;
        end
    end

    // ---------------- S2：+ bias_q13（Q13 域），int32 钳位 ----------------
    reg signed [31:0] y13_r;
    reg               s2_valid;
    wire signed [63:0] y13_biased = y13_acc + s1_bias;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y13_r    <= 32'sd0;
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= 1'b0;
            if (s1_valid && s1_last) begin
                if (y13_biased > 64'sd2147483647)       y13_r <= 32'sd2147483647;
                else if (y13_biased < -64'sd2147483648) y13_r <= -32'sd2147483648;
                else                                    y13_r <= y13_biased[31:0];
                s2_valid <= 1'b1;
            end
        end
    end

    // ---------------- S3：ReLU（Y3T：直接取 max(0,y13)，Q6.13 域不变） ----------------
    reg signed [31:0] v13_r;
    reg               s3_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v13_r    <= 32'sd0;
            s3_valid <= 1'b0;
        end else begin
            v13_r    <= (in_act && (y13_r[31])) ? 32'sd0 : y13_r;
            s3_valid <= s2_valid;
        end
    end

    // ---------------- S4：×multY，RHU，饱和（int16 ±32767 / int8 ±127） ----------------
    wire signed [63:0] prod  = $signed(v13_r) * $signed(in_multY);
    wire signed [63:0] o64   = rhu_shift(prod, in_shiftY);
    wire signed [63:0] qmax  = in_bits16 ? 64'sd32767 : 64'sd127;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data  <= 16'sd0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= s3_valid;
            if (o64 > qmax)       out_data <= in_bits16 ? 16'sd32767 : 16'sd127;
            else if (o64 < -qmax) out_data <= in_bits16 ? -16'sd32767 : -16'sd127;
            else                  out_data <= o64[15:0];
        end
    end
endmodule
