`timescale 1ns/1ns

module data_init(
    input   wire            sys_clk,    // 系统时钟
    input   wire            sys_rst_n,  // 系统复位，低有效
    input   wire            cfg_end,    // 单个寄存器配置完成

    output  reg             cfg_start,  // 单个寄存器配置触发
    output  wire    [23:0]  cfg_data,   // {REG_ADDR[15:0], REG_VAL[7:0]}
    output  reg             cfg_done    // 全部配置完成
);

parameter   REG_NUM       =   8'd248;      // 0~247 共 248 条
parameter   CNT_WAIT_MAX  =   10'd1023;    // 写后等待计数最大值

// 寄存器配置数据暂存
wire [23:0] cfg_data_reg [0:REG_NUM-1];

// 计数与状态信号
reg [9:0]   cnt_wait;
reg [7:0]   reg_num;

// 同步并检测 cfg_end 上升沿
reg cfg_end_d, cfg_end_dd;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cfg_end_d  <= 1'b0;
        cfg_end_dd <= 1'b0;
    end else begin
        cfg_end_d  <= cfg_end;
        cfg_end_dd <= cfg_end_d;
    end
end
wire cfg_end_pos = cfg_end_d & ~cfg_end_dd;

// cfg_data 输出：完成后拉 0
assign cfg_data = cfg_done ? 24'd0 : cfg_data_reg[reg_num];

// cnt_wait: 写后等待
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        cnt_wait <= 0;
    else if (cnt_wait < CNT_WAIT_MAX)
        cnt_wait <= cnt_wait + 1;
end

// reg_num: 当前寄存器索引
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        reg_num <= 0;
    else if (cfg_end_pos && reg_num < REG_NUM)
        reg_num <= reg_num + 1;
end

// cfg_start: 触发写脉冲
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        cfg_start <= 1'b0;
    else if ((cnt_wait == CNT_WAIT_MAX-1) || (cfg_end_pos && reg_num < REG_NUM))
        cfg_start <= 1'b1;
    else
        cfg_start <= 1'b0;
end

// cfg_done: 全部配置完成
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        cfg_done <= 1'b0;
    else if (reg_num == REG_NUM)
        cfg_done <= 1'b1;
end

//================================================================
// 寄存器列表（来自 data_init 模块的 0～247 条）
//================================================================
assign cfg_data_reg[  0] = 24'h3008_82;  // 复位、休眠
assign cfg_data_reg[  1] = 24'h3008_02;  // 正常工作
assign cfg_data_reg[  2] = 24'h3103_02;  // PLL Clock
assign cfg_data_reg[  3] = 24'h3017_FF;  // FREX/VSYNC/HREF/PCLK/D[9:6]
assign cfg_data_reg[  4] = 24'h3018_FF;  // D[5:0]/GPIO1/GPIO0

assign cfg_data_reg[  5] = 24'h3037_13;  // PLL 分频控制
assign cfg_data_reg[  6] = 24'h3108_01;  // 系统根分频器

assign cfg_data_reg[  7] = 24'h3630_36;
assign cfg_data_reg[  8] = 24'h3631_0E;
assign cfg_data_reg[  9] = 24'h3632_E2;
assign cfg_data_reg[ 10] = 24'h3633_12;
assign cfg_data_reg[ 11] = 24'h3621_E0;
assign cfg_data_reg[ 12] = 24'h3704_A0;
assign cfg_data_reg[ 13] = 24'h3703_5A;
assign cfg_data_reg[ 14] = 24'h3715_78;
assign cfg_data_reg[ 15] = 24'h3717_01;
assign cfg_data_reg[ 16] = 24'h370B_60;
assign cfg_data_reg[ 17] = 24'h3705_1A;
assign cfg_data_reg[ 18] = 24'h3905_02;
assign cfg_data_reg[ 19] = 24'h3906_10;
assign cfg_data_reg[ 20] = 24'h3901_0A;
assign cfg_data_reg[ 21] = 24'h3731_12;
assign cfg_data_reg[ 22] = 24'h3600_08;
assign cfg_data_reg[ 23] = 24'h3601_33;
assign cfg_data_reg[ 24] = 24'h302D_60;
assign cfg_data_reg[ 25] = 24'h3620_52;
assign cfg_data_reg[ 26] = 24'h371B_20;
assign cfg_data_reg[ 27] = 24'h471C_50;
assign cfg_data_reg[ 28] = 24'h3A13_43;
assign cfg_data_reg[ 29] = 24'h3A18_00;
assign cfg_data_reg[ 30] = 24'h3A19_F8;
assign cfg_data_reg[ 31] = 24'h3635_13;
assign cfg_data_reg[ 32] = 24'h3636_03;
assign cfg_data_reg[ 33] = 24'h3634_40;
assign cfg_data_reg[ 34] = 24'h3622_01;
assign cfg_data_reg[ 35] = 24'h3C01_34;
assign cfg_data_reg[ 36] = 24'h3C04_28;
assign cfg_data_reg[ 37] = 24'h3C05_98;
assign cfg_data_reg[ 38] = 24'h3C06_00;
assign cfg_data_reg[ 39] = 24'h3C07_08;
assign cfg_data_reg[ 40] = 24'h3C08_00;
assign cfg_data_reg[ 41] = 24'h3C09_1C;
assign cfg_data_reg[ 42] = 24'h3C0A_9C;
assign cfg_data_reg[ 43] = 24'h3C0B_40;
assign cfg_data_reg[ 44] = 24'h3810_00;
assign cfg_data_reg[ 45] = 24'h3811_10;
assign cfg_data_reg[ 46] = 24'h3812_00;

assign cfg_data_reg[ 47] = 24'h3708_64;
assign cfg_data_reg[ 48] = 24'h4001_02;
assign cfg_data_reg[ 49] = 24'h4005_1A;
assign cfg_data_reg[ 50] = 24'h3000_00;
assign cfg_data_reg[ 51] = 24'h3004_FF;
assign cfg_data_reg[ 52] = 24'h4300_61;
assign cfg_data_reg[ 53] = 24'h501F_01;
assign cfg_data_reg[ 54] = 24'h440E_00;
assign cfg_data_reg[ 55] = 24'h5000_A7;
assign cfg_data_reg[ 56] = 24'h3A0F_30;
assign cfg_data_reg[ 57] = 24'h3A10_28;
assign cfg_data_reg[ 58] = 24'h3A1B_30;
assign cfg_data_reg[ 59] = 24'h3A1E_26;
assign cfg_data_reg[ 60] = 24'h3A11_60;
assign cfg_data_reg[ 61] = 24'h3A1F_14;

assign cfg_data_reg[ 62] = 24'h5800_23;
assign cfg_data_reg[ 63] = 24'h5801_14;
assign cfg_data_reg[ 64] = 24'h5802_0F;
assign cfg_data_reg[ 65] = 24'h5803_0F;
assign cfg_data_reg[ 66] = 24'h5804_12;
assign cfg_data_reg[ 67] = 24'h5805_26;
assign cfg_data_reg[ 68] = 24'h5806_0C;
assign cfg_data_reg[ 69] = 24'h5807_08;
assign cfg_data_reg[ 70] = 24'h5808_05;
assign cfg_data_reg[ 71] = 24'h5809_05;
assign cfg_data_reg[ 72] = 24'h580A_08;
assign cfg_data_reg[ 73] = 24'h580B_0D;
assign cfg_data_reg[ 74] = 24'h580C_08;
assign cfg_data_reg[ 75] = 24'h580D_03;
assign cfg_data_reg[ 76] = 24'h580E_00;
assign cfg_data_reg[ 77] = 24'h580F_00;
assign cfg_data_reg[ 78] = 24'h5810_03;
assign cfg_data_reg[ 79] = 24'h5811_09;
assign cfg_data_reg[ 80] = 24'h5812_07;
assign cfg_data_reg[ 81] = 24'h5813_03;
assign cfg_data_reg[ 82] = 24'h5814_00;
assign cfg_data_reg[ 83] = 24'h5815_01;
assign cfg_data_reg[ 84] = 24'h5816_03;
assign cfg_data_reg[ 85] = 24'h5817_08;
assign cfg_data_reg[ 86] = 24'h5818_0D;
assign cfg_data_reg[ 87] = 24'h5819_08;
assign cfg_data_reg[ 88] = 24'h581A_05;
assign cfg_data_reg[ 89] = 24'h581B_06;
assign cfg_data_reg[ 90] = 24'h581C_08;
assign cfg_data_reg[ 91] = 24'h581D_0E;
assign cfg_data_reg[ 92] = 24'h581E_29;
assign cfg_data_reg[ 93] = 24'h581F_17;
assign cfg_data_reg[ 94] = 24'h5820_11;
assign cfg_data_reg[ 95] = 24'h5821_11;
assign cfg_data_reg[ 96] = 24'h5822_15;
assign cfg_data_reg[ 97] = 24'h5823_28;
assign cfg_data_reg[ 98] = 24'h5824_46;
assign cfg_data_reg[ 99] = 24'h5825_26;
assign cfg_data_reg[100] = 24'h5826_08;
assign cfg_data_reg[101] = 24'h5827_26;
assign cfg_data_reg[102] = 24'h5828_64;
assign cfg_data_reg[103] = 24'h5829_26;
assign cfg_data_reg[104] = 24'h582A_24;
assign cfg_data_reg[105] = 24'h582B_22;
assign cfg_data_reg[106] = 24'h582C_24;
assign cfg_data_reg[107] = 24'h582D_24;
assign cfg_data_reg[108] = 24'h582E_06;
assign cfg_data_reg[109] = 24'h582F_22;
assign cfg_data_reg[110] = 24'h5830_40;
assign cfg_data_reg[111] = 24'h5831_42;
assign cfg_data_reg[112] = 24'h5832_24;
assign cfg_data_reg[113] = 24'h5833_26;
assign cfg_data_reg[114] = 24'h5834_24;
assign cfg_data_reg[115] = 24'h5835_22;
assign cfg_data_reg[116] = 24'h5836_22;
assign cfg_data_reg[117] = 24'h5837_26;
assign cfg_data_reg[118] = 24'h5838_44;
assign cfg_data_reg[119] = 24'h5839_24;
assign cfg_data_reg[120] = 24'h583A_26;
assign cfg_data_reg[121] = 24'h583B_28;
assign cfg_data_reg[122] = 24'h583C_42;
assign cfg_data_reg[123] = 24'h583D_CE;

assign cfg_data_reg[124] = 24'h5180_FF;
assign cfg_data_reg[125] = 24'h5181_F2;
assign cfg_data_reg[126] = 24'h5182_00;
assign cfg_data_reg[127] = 24'h5183_14;
assign cfg_data_reg[128] = 24'h5184_25;
assign cfg_data_reg[129] = 24'h5185_24;
assign cfg_data_reg[130] = 24'h5186_09;
assign cfg_data_reg[131] = 24'h5187_09;
assign cfg_data_reg[132] = 24'h5188_09;
assign cfg_data_reg[133] = 24'h5189_75;
assign cfg_data_reg[134] = 24'h518A_54;
assign cfg_data_reg[135] = 24'h518B_E0;
assign cfg_data_reg[136] = 24'h518C_B2;
assign cfg_data_reg[137] = 24'h518D_42;
assign cfg_data_reg[138] = 24'h518E_3D;
assign cfg_data_reg[139] = 24'h518F_56;
assign cfg_data_reg[140] = 24'h5190_46;
assign cfg_data_reg[141] = 24'h5191_F8;
assign cfg_data_reg[142] = 24'h5192_04;
assign cfg_data_reg[143] = 24'h5193_70;
assign cfg_data_reg[144] = 24'h5194_F0;
assign cfg_data_reg[145] = 24'h5195_F0;
assign cfg_data_reg[146] = 24'h5196_03;
assign cfg_data_reg[147] = 24'h5197_01;
assign cfg_data_reg[148] = 24'h5198_04;
assign cfg_data_reg[149] = 24'h5199_12;
assign cfg_data_reg[150] = 24'h519A_04;
assign cfg_data_reg[151] = 24'h519B_00;
assign cfg_data_reg[152] = 24'h519C_06;
assign cfg_data_reg[153] = 24'h519D_82;
assign cfg_data_reg[154] = 24'h519E_38;

assign cfg_data_reg[155] = 24'h5480_01;
assign cfg_data_reg[156] = 24'h5481_08;
assign cfg_data_reg[157] = 24'h5482_14;
assign cfg_data_reg[158] = 24'h5483_28;
assign cfg_data_reg[159] = 24'h5484_51;
assign cfg_data_reg[160] = 24'h5485_65;
assign cfg_data_reg[161] = 24'h5486_71;
assign cfg_data_reg[162] = 24'h5487_7D;
assign cfg_data_reg[163] = 24'h5488_87;
assign cfg_data_reg[164] = 24'h5489_91;
assign cfg_data_reg[165] = 24'h548A_9A;
assign cfg_data_reg[166] = 24'h548B_AA;
assign cfg_data_reg[167] = 24'h548C_B8;
assign cfg_data_reg[168] = 24'h548D_CD;
assign cfg_data_reg[169] = 24'h548E_DD;
assign cfg_data_reg[170] = 24'h548F_EA;
assign cfg_data_reg[171] = 24'h5490_1D;

assign cfg_data_reg[172] = 24'h5381_1E;
assign cfg_data_reg[173] = 24'h5382_5B;
assign cfg_data_reg[174] = 24'h5383_08;
assign cfg_data_reg[175] = 24'h5384_0A;
assign cfg_data_reg[176] = 24'h5385_7E;
assign cfg_data_reg[177] = 24'h5386_88;
assign cfg_data_reg[178] = 24'h5387_7C;
assign cfg_data_reg[179] = 24'h5388_6C;
assign cfg_data_reg[180] = 24'h5389_10;
assign cfg_data_reg[181] = 24'h538A_01;
assign cfg_data_reg[182] = 24'h538B_98;

assign cfg_data_reg[183] = 24'h5580_06;
assign cfg_data_reg[184] = 24'h5583_40;
assign cfg_data_reg[185] = 24'h5584_10;
assign cfg_data_reg[186] = 24'h5589_10;
assign cfg_data_reg[187] = 24'h558A_00;
assign cfg_data_reg[188] = 24'h558B_F8;
assign cfg_data_reg[189] = 24'h501D_40;

assign cfg_data_reg[190] = 24'h5300_08;
assign cfg_data_reg[191] = 24'h5301_30;
assign cfg_data_reg[192] = 24'h5302_10;
assign cfg_data_reg[193] = 24'h5303_00;
assign cfg_data_reg[194] = 24'h5304_08;
assign cfg_data_reg[195] = 24'h5305_30;
assign cfg_data_reg[196] = 24'h5306_08;
assign cfg_data_reg[197] = 24'h5307_16;
assign cfg_data_reg[198] = 24'h5309_08;
assign cfg_data_reg[199] = 24'h530A_30;
assign cfg_data_reg[200] = 24'h530B_04;
assign cfg_data_reg[201] = 24'h530C_06;

assign cfg_data_reg[202] = 24'h3035_11;
assign cfg_data_reg[203] = 24'h3036_3C;
assign cfg_data_reg[204] = 24'h3C07_08;

// 5 和 6 已在前面写过，无须重复；
// 如果您仍需保证顺序严格，请合并到 5、6 的位置并移除此条。

// 时序控制 16'h3800~16'h3821
assign cfg_data_reg[205] = 24'h3820_46;
assign cfg_data_reg[206] = 24'h3821_01;
assign cfg_data_reg[207] = 24'h3814_31;
assign cfg_data_reg[208] = 24'h3815_31;
assign cfg_data_reg[209] = 24'h3800_00;
assign cfg_data_reg[210] = 24'h3801_00;
assign cfg_data_reg[211] = 24'h3802_00;
assign cfg_data_reg[212] = 24'h3803_00;
assign cfg_data_reg[213] = 24'h3804_0A;
assign cfg_data_reg[214] = 24'h3805_3F;
assign cfg_data_reg[215] = 24'h3806_07;
assign cfg_data_reg[216] = 24'h3807_9F;

// 输出分辨率
assign cfg_data_reg[217] = 24'h3808_05;  // 1280>>8 = 0x05 24'h3808_05
assign cfg_data_reg[218] = 24'h3809_00;  // 1280[7:0]      24'h3809_00
assign cfg_data_reg[219] = 24'h380A_02;  // 720>>8  = 0x02 24'h380A_02
assign cfg_data_reg[220] = 24'h380B_D0;  // 720[7:0]       24'h380B_D0

// 行场周期
assign cfg_data_reg[221] = 24'h380C_09;  // 2496>>8 = 0x09 24'h380C_09
assign cfg_data_reg[222] = 24'h380D_C0;  // 2496[7:0]      24'h380D_C0
assign cfg_data_reg[223] = 24'h380E_04;  // 1224>>8 = 0x04 24'h380E_04
assign cfg_data_reg[224] = 24'h380F_C8;  // 1224[7:0]      24'h380F_C8

// 时序偏移
assign cfg_data_reg[225] = 24'h3813_04;  // 24'h3813_04

// 其余寄存器与前者保持一致：
assign cfg_data_reg[226] = 24'h3618_00;
assign cfg_data_reg[227] = 24'h3612_29;
assign cfg_data_reg[228] = 24'h3709_52;
assign cfg_data_reg[229] = 24'h370C_03;
assign cfg_data_reg[230] = 24'h3A02_17;
assign cfg_data_reg[231] = 24'h3A03_10;
assign cfg_data_reg[232] = 24'h3A14_17;
assign cfg_data_reg[233] = 24'h3A15_10;
assign cfg_data_reg[234] = 24'h4004_02;
assign cfg_data_reg[235] = 24'h4713_03;
assign cfg_data_reg[236] = 24'h4407_04;
assign cfg_data_reg[237] = 24'h460C_22;
assign cfg_data_reg[238] = 24'h5001_A3;
assign cfg_data_reg[239] = 24'h503D_00;//8'h00:正常模式 8'h80:彩条显示
assign cfg_data_reg[240] = 24'h3016_02;
assign cfg_data_reg[241] = 24'h301C_02;
assign cfg_data_reg[242] = 24'h3019_02;
assign cfg_data_reg[243] = 24'h3019_00;

// 索引 244-247 若有额外配置，也请一并补全。
// 若严格按前者，只到 247：
assign cfg_data_reg[244] = 24'h300A_00;
assign cfg_data_reg[245] = 24'h300A_00;
assign cfg_data_reg[246] = 24'h300A_00;
assign cfg_data_reg[247] = 24'h300A_00;

endmodule
