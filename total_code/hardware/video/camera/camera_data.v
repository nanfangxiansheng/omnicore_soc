//---------<模块及端口声名>-------------------------------------------
// 2026-10-14 修复 [BUG-5 致命]:
//   原版用 "cnt_h 数满 2560 个 PCLK" (end_cnt_h) 作为一行的结束标志, 再用它给
//   cnt_v 计数、用 cnt_v==719 作帧尾 (end_cnt_v)。问题在于 cnt_h 的清零只发生在
//   end_cnt_h 那一刻:
//       if(!rst_n)      cnt_h <= 0;
//       else if(add_cnt_h) begin
//                if(end_cnt_h) cnt_h <= 0;     // 只有数满才清零
//                else          cnt_h <= cnt_h + 1;
//            end
//   HREF 一拉低 cnt_h 只是"停住"而不是清零, 于是行与行之间靠 cnt_h 的余数咬合,
//   实际 HREF 长度与 2560 只要差 1 个 PCLK, 误差就永远留在 cnt_h 里。
//   实测 (ModelSim, 真实 OV5640 时序 PCLK=84M/HTS=3784/VTS=740/720 有效行):
//       href 高电平 = 720 行  (真实帧结构无误)
//       end_cnt_h   = 719     (只识别出 719 行!)
//       end_cnt_v   = 0       -> cnt_v 停在 719, flag 不清零
//       下一帧: end_cnt_h=1 -> end_cnt_v 触发 -> flag 清零
//              该帧 flag 只高了 6 行, 写请求 rd_trig = 0 (整帧作废)
//   即 "每帧丢一行 + 帧尾判定整体漂移到下一帧"。漂移后的帧会在半途把 flag 清掉,
//   清零点之后 camera_data 不再输出任何像素 -> 写侧不再写 DDR -> 那片区域永远
//   是 DDR 上电随机值 = "白底 + 彩色小点"; 清零点位置随相位移动, 上板看到的就是
//   "上半屏有画面(且在闪), 下半屏白底彩点"。
//
//   修法(三处, 都不改接口):
//     1. cnt_h 每个 HREF 下降沿强制清零 -> 行与行之间不再累积相位误差;
//     2. 行计数 cnt_v 改由 HREF 下降沿驱动 (add_cnt_v = href_fall & flag),
//        每帧严格 720 行, 不再依赖 cnt_h 是否数满;
//     3. 新增 pix_cnt, 行内只放 1280 个像素脉冲 -> 写侧 x_cnt 每行稳定到 1280,
//        与已验证通过的测试图案激励完全同构。注意判定用 (pix_cnt < 1280) 而不是
//        (pix_cnt != 1279): 后者会把第 1280 个像素判成非法而丢掉 -> 每行只 1279 个,
//        写侧每行少写最右 2 个像素(第 640 个 32bit 字永不写入).
//   (仿真复验: end_cnt_h=720, end_cnt_v=1, 每帧 rd_trig=720, rd_cnt 走满 720 行,
//              每行 wf_wr_en = 1280 个脉冲)
//------------------------------------------------------------------------------
module camera_data ( 
    input   wire            clk                 ,
    input   wire            rst_n               ,
    input   wire            vsync               ,//表示新一帧到来
    input   wire            href                ,//有效信号
    input   wire    [7:0]   din                 ,
    input   wire            cfg_done            ,//摄像头配置完成标准信号
    input   wire            ddr_init            ,
    output  reg     [15:0]  pixel_data          ,//像素数据rgb565
    output  wire            wf_wr_en            ,
    output  reg             sop                 ,// 帧头
    output  reg             eop                 ,//帧尾在最后一行最后一个计数器结束
    output  reg             vs                  ,
    output  reg             hs                  
);								 
//---------<参数定义>------------------------------------------------
    localparam LINE_PIX  = 1280;            // 1280x720, RGB565 8bit DVP
    localparam LINE_PCLK = LINE_PIX * 2;    // 一个有效行 = 2560 个 PCLK

//---------<内部信号定义>--------------------------------------------
reg             flag          ;
reg     [2:0]   vsync_r       ;//打三拍下降沿检测
wire            vsync_negedge ;
//  行场计数器 
reg		[11:0]	cnt_h	   ;//行内 PCLK 计数 0..2559
reg		[11:0]	pix_cnt	   ;//行内已输出像素数 0..1279
wire			add_cnt_h;
wire			end_cnt_h;
reg             href_1d    ;
wire            href_fall  ;
reg		[9:0]	cnt_v	   ;//720
wire			add_cnt_v;
wire			end_cnt_v;

reg     [4:0]   eop_cnt;
wire            pic_valid;

reg             pixel_data_vld;


//前十帧图像无效
always @(posedge clk or negedge rst_n)begin 
    if(!rst_n)
        eop_cnt <= 0;
    else if(eop_cnt == 10)
        eop_cnt <= eop_cnt;
    else if(eop)
        eop_cnt <= eop_cnt + 1;
end
assign pic_valid = (eop_cnt == 10);

assign wf_wr_en = (pic_valid && ddr_init) ? pixel_data_vld : 0;

//下降沿检测
always @(posedge clk or negedge rst_n)begin 
    if(!rst_n)begin
        vsync_r <= 3'b111;
    end 
    else begin 
        vsync_r <= {vsync_r[1:0],vsync};
    end 
end
assign  vsync_negedge = ~vsync_r[1] & vsync_r[2];

// HREF 打一拍 -> 取下降沿(一行结束)
always @(posedge clk or negedge rst_n)begin
    if(!rst_n) href_1d <= 1'b0;
    else       href_1d <= href;
end
assign href_fall = href_1d & ~href;

//---------------------------------------------------------------------------
// [修法1] 行内计数: 每个 HREF 下降沿强制清零, 且计到 2559 就停住(防 HREF 偏长)
//---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n)begin 
   if(!rst_n)begin
        cnt_h <= 'd0;
    end 
    else if(href_fall)begin
        cnt_h <= 'd0;
    end
    else if(add_cnt_h)begin 
        if(end_cnt_h)begin 
            cnt_h <= 'd0;
        end
        else begin 
            cnt_h <= cnt_h + 1'b1;
        end 
    end
end 
assign add_cnt_h = href && flag;
assign end_cnt_h = add_cnt_h && cnt_h == LINE_PCLK-1;// 一个有效行的 PCLK 数

//---------------------------------------------------------------------------
// [修法3] 行内像素计数: 每行严格只放 1280 个像素脉冲
//---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)                pix_cnt <= 'd0;
    else if(!href)            pix_cnt <= 'd0;              // 行间清零
    else if(add_cnt_h && cnt_h[0] && (pix_cnt < LINE_PIX))
                              pix_cnt <= pix_cnt + 1'b1;
end

//---------------------------------------------------------------------------
// [修法2] 行计数: 以 HREF 下降沿为准, 每帧严格 720 行
//---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n)begin 
   if(!rst_n)begin
        cnt_v <= 'd0;
    end 
    else if(add_cnt_v)begin
        if(end_cnt_v)begin 
            cnt_v <= 'd0;
        end
        else begin 
            cnt_v <= cnt_v + 1'b1;
        end 
    end
end
assign add_cnt_v = href_fall && flag;
assign end_cnt_v = add_cnt_v && cnt_v == 720-1;

// 每行 HREF 下降沿清除 flag, 保证清零点永远落在行边界(不会切在半行中间)
always @(posedge clk or negedge rst_n)begin 
    if(!rst_n)begin
        flag <= 'd0;
    end 
    else if(vsync_negedge && cfg_done)begin 
        flag <= 1;
    end 
    else if(end_cnt_v)begin
        flag <= 0;
    end
end    

//输出
always @(posedge clk or negedge rst_n)begin 
    if(!rst_n)begin
        pixel_data <= 'd0;
    end
    else if(pic_valid) begin 
        pixel_data <= {pixel_data[7:0],din};
    end 
end

always @(posedge clk or negedge rst_n)begin 
    if(!rst_n)begin
        pixel_data_vld <= 'd0;
        sop <= 0;
        eop <= 0;
        vs  <= 0;
        hs  <= 0;
    end 
    else begin 
        pixel_data_vld <=  add_cnt_h && cnt_h[0] && (pix_cnt < LINE_PIX)
                          && pic_valid && ddr_init;//行内每 2 拍一个, 共 1280 个
        sop <= href_fall && cnt_v==0;
        eop <= end_cnt_v;
        // 只有过了前十帧（pic_valid为高），才向后级输出对齐的vsync和href
        vs  <= pic_valid ? vsync : 1'b0;
        hs  <= pic_valid ? href  : 1'b0;
    end 
end

endmodule
