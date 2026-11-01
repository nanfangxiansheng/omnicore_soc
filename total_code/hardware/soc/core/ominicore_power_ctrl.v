

`default_nettype none

// OmniCore 的唤醒/休眠（电源）状态机

module ominicore_power_ctrl #(
`include "ominicore_config.vh"
) (
	input  wire              clk_always_on,
	input  wire              rst_n,

	// 4 相（格雷码）req/ack 握手，用于请求和释放非处理器硬件
	// （如总线互连矩阵）的电源+时钟使能。外部控制器
	// 也可以用它来门控处理器的 clk 输入，
	// 而不是使用下方的 clk_en 信号。
	output reg               pwrup_req,
	input  wire              pwrup_ack,

	// 顶层时钟使能，用于处理器 clk 输入上可选的时钟门控单元
	// （但不含 clk_always_on——它为本模块及 IRQ 输入
	// 触发器供时钟）。这使得处理器休眠时可进行时钟门控。
	// 门控单元在 clk_en 变化时有一拍的延迟是
	// 可以接受的。
	output reg               clk_en,

	// 来自 CSR 的电源状态控制信号
	input  wire              allow_clkgate,
	input  wire              allow_power_down,
	input  wire              allow_sleep_on_block,

	// 来自前端的信号：前端已停在 WFI 流水线停顿上，
	// 现在可以放心进入深度休眠状态
	input  wire              frontend_pwrdown_ok,

	input  wire              sleeping_on_wfi,
	input  wire              wfi_wakeup_req,
	input  wire              sleeping_on_block,
	input  wire              block_wakeup_req_pulse,
	output reg               stall_release
);

// ----------------------------------------------------------------------------
// 唤醒/休眠状态机

localparam W_STATE              = 2;
localparam S_AWAKE              = 2'h0;
localparam S_ENTER_ASLEEP       = 2'h1;
localparam S_ASLEEP             = 2'h2;
localparam S_ENTER_AWAKE        = 2'h3;

reg [W_STATE-1:0] state;
reg               block_wakeup_req;

wire active_wake_req =
	(sleeping_on_block && (block_wakeup_req || wfi_wakeup_req)) ||
	(sleeping_on_wfi && wfi_wakeup_req);

// 注意：复位期间我们就发出上电请求，并*假定*复位时
// 上电应答同样为高。如果这有问题，请延长
// 内核复位时间。

always @ (posedge clk_always_on or negedge rst_n) begin
	if (!rst_n) begin
		state <= S_AWAKE;
		pwrup_req <= 1'b1;
		clk_en <= 1'b1;
		stall_release <= 1'b0;
	end else begin
		stall_release <= 1'b0;
		case (state)
		S_AWAKE: if (sleeping_on_wfi || sleeping_on_block) begin
			if (stall_release) begin
				// 这是刚被我们释放的那个持续中停顿的最后一拍。稍安
				// 勿躁，本条指令将在本拍结束时向流水线下游移动。
				// （有断言保证这种情况
				// 不会发生两次。）
				state <= S_AWAKE;
			end else if (active_wake_req) begin
				// 若深度休眠会立即被穿透（fall through），则跳过休眠。
				stall_release <= 1'b1;
			end else if ((allow_power_down || allow_clkgate) && (sleeping_on_wfi || allow_sleep_on_block)) begin
				if (frontend_pwrdown_ok) begin
					pwrup_req <= !allow_power_down;
					clk_en <= !allow_clkgate;
					state <= allow_power_down ? S_ENTER_ASLEEP : S_ASLEEP;
				end else begin
					// 保持清醒，直到断电安全为止（即等到
					// 取指操作归于沉寂）。
					state <= S_AWAKE;
				end					
			end else begin
				// 电源状态不变。就带着流水线停顿原地等待。
				state <= S_AWAKE;
			end
		end
		S_ENTER_ASLEEP: if (!pwrup_ack) begin
			state <= S_ASLEEP;
		end
		S_ASLEEP: if (active_wake_req) begin
			pwrup_req <= 1'b1;
			clk_en <= 1'b1;
			// 非断电唤醒仍要经过 enter 状态，
			// 以防时钟门控单元有 1 拍延迟。
			state <= S_ENTER_AWAKE;
		end
		S_ENTER_AWAKE: if (pwrup_ack || !allow_power_down) begin
			state <= S_AWAKE;
			stall_release <= 1'b1;
		end
		default: begin
			state <= S_AWAKE;
		end
		endcase
	end
end

`ifdef OMNICORE_ASSERTIONS
// 这些寄存器是为了规避 yosys-smtbmc 中 $past()
// 复位值非常量的问题而设的变通方案。
reg past_sleeping;
reg past_stall_release;
always @ (posedge clk_always_on) begin
	if (!rst_n) begin
		past_sleeping <= 1'b0;
		past_stall_release <= 1'b0;
	end else begin
		past_sleeping <= sleeping_on_wfi || sleeping_on_block;
		past_stall_release <= stall_release;
		// 这两者必须始终互斥。
		assert(!(sleeping_on_wfi && sleeping_on_block));
		if (stall_release) begin
			// 应该是存在一个我们刚刚释放的停顿
			assert(past_sleeping);
			// 应该是仍处于该停顿之中
			assert(sleeping_on_wfi|| sleeping_on_block);
			// 完成一次释放并进入新休眠状态需要一拍，因此
			// 不可能出现双重释放。
			assert(!past_stall_release);
		end
		if (state == S_ASLEEP) begin
			assert(allow_power_down || allow_clkgate);
		end
	end
end
`endif

// ----------------------------------------------------------------------------
// 块唤醒的脉冲转电平

// 解除阻塞信号是粘滞的：此前一次解除阻塞之后若无再阻塞，
// 下一次阻塞将立即被穿透（fall through）。

always @ (posedge clk_always_on or negedge rst_n) begin
	if (!rst_n) begin
		block_wakeup_req <= 1'b0;
	end else begin
		// 注意 OR 优先级高于 AND，因此不会漏掉在唤醒
		// 那一瞬间到达的第二个解除阻塞。
		block_wakeup_req <= (block_wakeup_req && !(
			sleeping_on_block && stall_release
		)) || block_wakeup_req_pulse;
	end
end

endmodule

`ifndef YOSYS
`default_nettype wire
`endif
