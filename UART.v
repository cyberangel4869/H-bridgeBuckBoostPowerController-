// ============================================================
// USART全双工通信模块（优化版）
// 主时钟频率：27MHz
// 波特率：9600
// 数据位：8位
// 停止位：1位
// 无校验位
// 
// 优化点：
// 1. 加入TX数据锁存机制，防止发送过程中数据被修改
// 2. 优化状态机逻辑，提高稳定性
// 3. 增加发送数据寄存器，确保数据一致性
// ============================================================

module uart (
    // 系统信号
    input wire clk_27m,        // 27MHz主时钟
    input wire rst_n,          // 异步低电平复位
    
    // 发送接口
    input wire [7:0] tx_data,  // 发送数据
    input wire tx_start,       // 发送启动信号（上升沿触发）
    output reg tx_busy,        // 发送忙标志
    output reg tx_done,        // 发送完成标志（单脉冲）
    output wire txd,           // 发送数据线
    
    // 接收接口
    input wire rxd,            // 接收数据线
    output reg [7:0] rx_data,  // 接收到的数据
    output reg rx_valid,       // 接收数据有效标志（单脉冲）
    output reg rx_error,       // 接收错误标志（帧错误）
    output reg rx_busy,        // 接收忙标志
    
    // 调试接口（可选）
    output wire [3:0] debug_state_tx,  // 发送状态机状态
    output wire [3:0] debug_state_rx   // 接收状态机状态
);

// ============================================================
// 参数定义
// ============================================================

// 27MHz时钟下，9600波特率的时钟分频系数
// 每个波特周期 = 27,000,000 / 9600 = 2812.5 个时钟周期
// 为了更精确，我们使用16倍过采样，所以分频系数为：
// 27,000,000 / (9600 * 16) = 175.78125 ≈ 176
parameter BAUD_DIV = 2812;           // 波特率分频系数
parameter BAUD_DIV_HALF = 1406;       // 半位周期分频系数（用于过采样）

// 发送状态机状态定义（新增TX_LATCH状态）
localparam [3:0]
    TX_IDLE      = 4'd0,
    TX_LATCH     = 4'd1,  // 新增：数据锁存状态
    TX_START     = 4'd2,
    TX_BIT0      = 4'd3,
    TX_BIT1      = 4'd4,
    TX_BIT2      = 4'd5,
    TX_BIT3      = 4'd6,
    TX_BIT4      = 4'd7,
    TX_BIT5      = 4'd8,
    TX_BIT6      = 4'd9,
    TX_BIT7      = 4'd10,
    TX_STOP      = 4'd11;

localparam [3:0]
    RX_IDLE      = 4'd0,
    RX_START     = 4'd1,
    RX_BIT0      = 4'd2,
    RX_BIT1      = 4'd3,
    RX_BIT2      = 4'd4,
    RX_BIT3      = 4'd5,
    RX_BIT4      = 4'd6,
    RX_BIT5      = 4'd7,
    RX_BIT6      = 4'd8,
    RX_BIT7      = 4'd9,
    RX_STOP      = 4'd10;

// ============================================================
// 内部信号定义
// ============================================================

// 发送部分信号（新增锁存寄存器）
reg [7:0] tx_data_latched;     // 发送数据锁存寄存器（关键优化）
reg [7:0] tx_shift_reg;        // 发送移位寄存器
reg [3:0] tx_state;            // 发送状态机状态
reg [8:0] tx_bit_counter;      // 发送位计数器（0-8）
reg [15:0] tx_baud_counter;    // 发送波特率计数器
reg tx_start_reg;              // 发送启动信号寄存器（用于边沿检测）
reg tx_start_rise;             // 发送启动上升沿检测信号

// 接收部分信号
reg [7:0] rx_shift_reg;        // 接收移位寄存器
reg [3:0] rx_state;            // 接收状态机状态
reg [8:0] rx_bit_counter;      // 接收位计数器
reg [15:0] rx_baud_counter;    // 接收波特率计数器
reg [3:0] rx_sample_counter;   // 接收采样计数器
reg rx_d0, rx_d1, rx_d2;       // RXD同步器（三级同步防亚稳态）
wire rx_synced;                 // 同步后的RXD信号
reg rx_start_detected;         // 起始位检测标志

// ============================================================
// 接收数据同步器（防亚稳态）
// ============================================================

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        rx_d0 <= 1'b1;
        rx_d1 <= 1'b1;
        rx_d2 <= 1'b1;
    end else begin
        rx_d0 <= rxd;           // 第一级同步
        rx_d1 <= rx_d0;         // 第二级同步
        rx_d2 <= rx_d1;         // 第三级同步
    end
end

assign rx_synced = rx_d2;       // 使用第三级同步信号

// ============================================================
// 发送状态机（优化版：加入数据锁存）
// ============================================================

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_IDLE;
        tx_data_latched <= 8'h00;    // 复位锁存数据
        tx_shift_reg <= 8'h00;
        tx_bit_counter <= 9'd0;
        tx_baud_counter <= 16'd0;
        tx_busy <= 1'b0;
        tx_done <= 1'b0;
        tx_start_reg <= 1'b0;
        tx_start_rise <= 1'b0;
    end else begin
        // 发送启动信号边沿检测
        tx_start_reg <= tx_start;
        tx_start_rise <= tx_start && !tx_start_reg;  // 检测上升沿
        
        case (tx_state)
            TX_IDLE: begin
                tx_busy <= 1'b0;
                tx_baud_counter <= 16'd0;
                tx_bit_counter <= 9'd0;
                
                // 检测tx_start的上升沿
                if (tx_start_rise) begin
                    tx_state <= TX_LATCH;      // 进入锁存状态
                    tx_busy <= 1'b1;           // 立即置忙标志
                    tx_done<=0;
                end
            end
            
            TX_LATCH: begin
                // 关键优化：在此状态稳定锁存发送数据
                // 确保在开始发送前，数据已经被稳定锁存
                tx_data_latched <= tx_data;    // 锁存输入数据
                tx_shift_reg <= tx_data;       // 同时加载到移位寄存器
                tx_baud_counter <= 16'd0;
                tx_state <= TX_START;          // 转到起始位状态
            end
            
            TX_START: begin
                // 发送起始位（0）
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_state <= TX_BIT0;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT0: begin
                // 发送第0位数据
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd1;
                    tx_shift_reg <= tx_shift_reg >> 1;  // 右移一位
                    tx_state <= TX_BIT1;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT1: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd2;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT2;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT2: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd3;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT3;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT3: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd4;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT4;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT4: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd5;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT5;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT5: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd6;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT6;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT6: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd7;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_BIT7;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_BIT7: begin
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_bit_counter <= 9'd8;
                    tx_shift_reg <= tx_shift_reg >> 1;
                    tx_state <= TX_STOP;
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            TX_STOP: begin
                // 发送停止位（1）
                if (tx_baud_counter >= BAUD_DIV - 1) begin
                    tx_baud_counter <= 16'd0;
                    tx_state <= TX_IDLE;
                    tx_busy <= 1'b0;
                    tx_done <= 1'b1;  // 产生发送完成信号
                end else begin
                    tx_baud_counter <= tx_baud_counter + 16'd1;
                end
            end
            
            default: begin
                tx_state <= TX_IDLE;
            end
        endcase
    end
end

// 发送数据输出
assign txd = (tx_state == TX_IDLE) ? 1'b1 :      // 空闲时为高电平
             (tx_state == TX_LATCH) ? 1'b1 :     // 锁存状态保持高电平
             (tx_state == TX_START) ? 1'b0 :     // 起始位为低电平
             (tx_state == TX_STOP) ? 1'b1 :      // 停止位为高电平
             tx_shift_reg[0];                    // 数据位

// ============================================================
// 接收状态机（保持不变）
// ============================================================

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= RX_IDLE;
        rx_shift_reg <= 8'h00;
        rx_bit_counter <= 9'd0;
        rx_baud_counter <= 16'd0;
        rx_sample_counter <= 4'd0;
        rx_valid <= 1'b0;
        rx_error <= 1'b0;
        rx_busy <= 1'b0;
        rx_data <= 8'h00;
        rx_start_detected <= 1'b0;
    end else begin
        case (rx_state)
            RX_IDLE: begin
                rx_busy <= 1'b0;
                rx_baud_counter <= 16'd0;
                rx_sample_counter <= 4'd0;
                rx_start_detected <= 1'b0;
                rx_shift_reg<=8'd0;
                rx_bit_counter<=0;
                // 检测起始位（下降沿）
                if (rx_synced == 1'b0) begin
                    rx_state <= RX_START;
                    rx_busy <= 1'b1;
                    rx_valid<=0;
                    rx_error<=0;
                    rx_baud_counter <= 16'd0;
                end
            end
            
            RX_START: begin
                // 在起始位中间采样（第8个采样点）
                if (rx_baud_counter >=  BAUD_DIV_HALF- 1) begin
                    rx_baud_counter <= 16'd0;
                    
                    // 检查起始位是否仍然为低电平
                    if (rx_synced == 1'b0) begin
                        rx_start_detected <= 1'b1;
                        rx_state <= RX_BIT0;
                        rx_sample_counter <= 4'd0;
                    end else begin
                        // 起始位错误，回到空闲状态
                        rx_state <= RX_IDLE;
                        rx_error <= 1'b1;
                    end
                end else begin
                    rx_baud_counter <= rx_baud_counter + 16'd1;
                end
            end
            RX_BIT0, RX_BIT1, RX_BIT2, RX_BIT3,
            RX_BIT4, RX_BIT5, RX_BIT6, RX_BIT7: begin
                if (rx_baud_counter >= BAUD_DIV - 1) begin
                    rx_baud_counter <= 16'd0;
                    rx_shift_reg<= {rx_synced,rx_shift_reg[7:1]};
                    rx_bit_counter <= rx_bit_counter + 9'd1;
                        
                        // 根据bit_counter转到下一个状态
                        case (rx_bit_counter)
                            9'd0: rx_state <= RX_BIT1;
                            9'd1: rx_state <= RX_BIT2;
                            9'd2: rx_state <= RX_BIT3;
                            9'd3: rx_state <= RX_BIT4;
                            9'd4: rx_state <= RX_BIT5;
                            9'd5: rx_state <= RX_BIT6;
                            9'd6: rx_state <= RX_BIT7;
                            9'd7: rx_state <= RX_STOP;
                            default: rx_state <= RX_STOP;
                        endcase
                end else begin
                    rx_baud_counter <= rx_baud_counter + 16'd1;
                end
            end
            
            RX_STOP: begin
                // 采样停止位
                if (rx_baud_counter >= BAUD_DIV - 1) begin
                    rx_baud_counter <= 16'd0;
                    // 在停止位中间采样
                    if (rx_synced == 1'b1) begin
                        // 接收成功
                        rx_data <= rx_shift_reg;  // 输出接收到的数据
                        rx_valid <= 1'b1;         // 产生数据有效
                    end else begin
                        // 帧错误
                        rx_error <= 1'b1;
                    end
                    // 回到空闲状态
                    rx_state <= RX_IDLE;
                    rx_busy <= 1'b0;
                end else begin
                    rx_baud_counter <= rx_baud_counter + 16'd1;
                end
            end
            
            default: begin
                rx_state <= RX_IDLE;
            end
        endcase
    end
end

// ============================================================
// 调试信号输出
// ============================================================

assign debug_state_tx = tx_state;
assign debug_state_rx = rx_state;

endmodule