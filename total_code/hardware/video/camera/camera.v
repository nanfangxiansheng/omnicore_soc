module camera (
    input   wire            clk             ,
    input   wire            rst_n           ,
    output  wire            SCL             ,  
    inout   wire            SDA             ,  
    input   wire            camera_clk      ,
    input   wire    [7:0]   camera_data     ,
    input   wire            camera_herf     ,
    input   wire            camera_vsync    ,
    input   wire            ddr_init        ,
    output  wire            camera_rstn     ,
    output  wire            camera_pwnd     ,
    output  wire            init_done       ,
    output  wire    [15:0]  wf_wr_data      ,   //RGB565
    output  wire            wf_wr_en        ,
    output  wire            vs              ,
    output  wire            hs              ,
    output  wire            sop             ,
    output  wire            eop 
);

reg [18:0]cnt1;
reg [15:0]cnt2;
reg [19:0]cnt3;
reg initial_en;
reg camera_rstn_reg;
reg camera_pwnd_reg;

assign camera_rstn=camera_rstn_reg;
assign camera_pwnd=camera_pwnd_reg;

//5ms, delay from sensor power up stable to Pwdn pull down
always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
    	cnt1 <= 0;
    	camera_pwnd_reg <= 1'b1;// 1'b1 
    end
    else if(cnt1 < 19'h40000) begin
        cnt1 <= cnt1 + 1'b1;
        camera_pwnd_reg <= 1'b1;
    end
    else
        camera_pwnd_reg <= 1'b0;         
end

//1.3ms, delay from pwdn low to resetb pull up
always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
        cnt2 <= 0;
        camera_rstn_reg <= 1'b0;  
    end
    else if(camera_pwnd_reg == 1)  begin
  	    cnt2 <= 0;
        camera_rstn_reg <= 1'b0;  
    end
    else if(cnt2 < 16'hffff) begin
        cnt2 <= cnt2 + 1'b1;
        camera_rstn_reg <= 1'b0;
    end
    else
        camera_rstn_reg <= 1'b1;         
end

//21ms, delay from resetb pul high to SCCB initialization
always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
        cnt3 <= 0;
        initial_en <= 1'b0;
    end
    else if(camera_rstn_reg == 0) begin
        cnt3 <= 0;
        initial_en <= 1'b0;
    end
    else if(cnt3 < 20'hfffff) begin
        cnt3 <= cnt3 + 1'b1;
        initial_en <= 1'b0;
    end
    else
        initial_en <= 1'b1;    
end


// 官方配置引擎 (2026-10-15 移植自已验证工程 demo_ov5640):
//   reg_config 自带官方寄存器表 (含原表缺失的 0x3103=0x11 PLL 前置步) + 位注式 I2C
//   (i2c_com 的 SCL≈10.8kHz, 比原 SCCB 的 250kHz 慢 23 倍, 抗信号完整性问题强得多;
//    注意: i2c_com 虽输出 ack, 但 reg_config 从不使用它, 只看 tr_end —— 两者都不以 ACK 为准)。
//   替换掉原来的 data_init + SCCB —— 后者【不检查 ACK】, 写失败无声无息;
//   且自编 248 条表的顺序与官方不同 (官方首条 0x3103=0x11 是 OV5640 手册要求的
//   PLL 前置步骤, 原表缺失), 219/248 条内容相同但顺序不同。
reg_config reg_config_u(
    .clk_25M       (clk            ),   // 本工程 sys_clk = 27MHz (内部再分频出 ~10kHz I2C 时钟)
    .camera_rstn   (camera_rstn_reg),   // 传感器复位释放后配置引擎才工作
    .initial_en    (initial_en     ),
    .reg_conf_done (init_done      ),   // 全部配置完成
    .i2c_sclk      (SCL            ),
    .i2c_sdat      (SDA            ),
    .clock_20k     (               ),
    .reg_index     (               )
);

camera_data camera_data_u( 
    /*input   wire            */.clk           ( camera_clk      )      ,
    /*input   wire            */.rst_n         ( rst_n           )      ,
    /*input   wire            */.vsync         ( camera_vsync    )      ,//表示新一帧到来
    /*input   wire            */.href          ( camera_herf     )      ,//有效信号
    /*input   wire    [7:0]   */.din           ( camera_data     )      ,
    /*input   wire            */.cfg_done      ( init_done       )      ,//摄像头配置完成标准信号
    /*input   wire            */.ddr_init      ( ddr_init & initial_en )      ,
    /*output  reg     [15:0]  */.pixel_data    ( wf_wr_data      )      ,//像素数据rgb565
    /*output  wire            */.wf_wr_en      ( wf_wr_en        )      ,
    /*output  reg             */.vs            ( vs              )      ,
    /*output  reg             */.hs            ( hs              )      ,
    /*output  reg             */.sop           ( sop             )      ,// 帧头
    /*output  reg             */.eop           ( eop             )       //帧尾在最后一行最后一个计数器结束
);			
    
endmodule