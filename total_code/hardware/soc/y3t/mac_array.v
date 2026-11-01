// ============================================================================
// mac_array.v — 16 OCH × 16 ICH 输出驻留乘加阵列
// 每拍：in_valid 时对每个 OCH 通道并行计算
//   psum[och] = Σ_{ich=0..ICH-1} act[ich](int16) × w[och][ich](int8)   (28bit)
//   acc[och] <= (in_first ? 0 : acc[och]) + psum[och]                  (48bit)
// in_first=1 开始新像素累加（ICH 维/tap 维分段累加时首段清零）。
// 打包格式：
//   in_act[ich*16 +: 16]            有符号 int16
//   in_w[och*ICH*8 + ich*8 +: 8]    有符号 int8（行 = 128bit = 16 ICH）
//   out_acc[och*48 +: 48]           有符号 int48（输出驻留寄存器）
// ============================================================================
module mac_array #(
    parameter OCH = 16,
    parameter ICH = 16,
    parameter NGROUP = 1        // OT 合并组数（Phase 1a：stage3_hdmi=4；线1=1 原样）
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  in_valid,
    input  wire                  in_first,
    input  wire [1:0]            in_group,   // NGROUP=1 时接 2'b0
    input  wire [ICH*16-1:0]     in_act,
    input  wire [OCH*ICH*8-1:0]  in_w,
    output reg  [NGROUP*OCH*48-1:0] out_acc,
    output reg                   out_valid
);
    localparam NPROD = OCH * ICH;   // 256
    integer o, i, g, k;

    // ---- S1：积（组合 16×16 乘）→ 寄存（引导 DSP48 MREG 吸收） ----
    reg signed [23:0] prod_c [0:NPROD-1];   // int16×int8 = 24bit（|max| = 2^22）
    reg signed [23:0] prod_r [0:NPROD-1];
    reg               vld_s1, first_s1;

    always @(*) begin
        for (o = 0; o < OCH; o = o + 1)
            for (i = 0; i < ICH; i = i + 1)
                prod_c[o*ICH+i] = $signed(in_act[i*16 +: 16]) *
                                  $signed(in_w[o*ICH*8 + i*8 +: 8]);
    end

    // ---- S2：4 积一组部分和（组合 2 级树）→ 寄存（fabric FF） ----
    reg signed [27:0] sum4_c [0:OCH*4-1];
    reg signed [27:0] sum4_r [0:OCH*4-1];
    reg               vld_s2, first_s2;

    always @(*) begin
        for (o = 0; o < OCH; o = o + 1)
            for (g = 0; g < 4; g = g + 1)
                sum4_c[o*4+g] = prod_r[o*ICH+g*4]   + prod_r[o*ICH+g*4+1] +
                                prod_r[o*ICH+g*4+2] + prod_r[o*ICH+g*4+3];
    end

    // ---- 流水 + S3 驻留累加（NGROUP 组独立 acc，grp 随流水打拍） ----
    reg [1:0] grp_s1, grp_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vld_s1 <= 1'b0; first_s1 <= 1'b0; grp_s1 <= 2'b0;
            vld_s2 <= 1'b0; first_s2 <= 1'b0; grp_s2 <= 2'b0;
            out_valid <= 1'b0;
            out_acc   <= {(NGROUP*OCH*48){1'b0}};
        end else begin
            vld_s1    <= in_valid;  first_s1 <= in_first;  grp_s1 <= in_group;
            vld_s2    <= vld_s1;    first_s2 <= first_s1;  grp_s2 <= grp_s1;
            out_valid <= vld_s2;
            if (in_valid)
                for (k = 0; k < NPROD; k = k + 1) prod_r[k] <= prod_c[k];
            if (vld_s1)
                for (k = 0; k < OCH*4; k = k + 1) sum4_r[k] <= sum4_c[k];
            if (vld_s2) begin
                for (o = 0; o < OCH; o = o + 1) begin
                    if (first_s2)
                        out_acc[grp_s2*OCH*48 + o*48 +: 48] <=
                                               sum4_r[o*4]   + sum4_r[o*4+1] +
                                               sum4_r[o*4+2] + sum4_r[o*4+3];
                    else
                        out_acc[grp_s2*OCH*48 + o*48 +: 48] <=
                                               $signed(out_acc[grp_s2*OCH*48 + o*48 +: 48]) +
                                               sum4_r[o*4]   + sum4_r[o*4+1] +
                                               sum4_r[o*4+2] + sum4_r[o*4+3];
                end
            end
        end
    end
endmodule
