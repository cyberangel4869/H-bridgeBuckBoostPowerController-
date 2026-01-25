module adc0809_driver (
    input wire clk,           // 系统时钟
    input wire rst_n,         // 异步复位，低电平有效
    input wire adc_driver_start,
    input wire [7:0] data_in, // ADC数据输入
    output reg adc_clk,       //ADC时钟
    output reg start,         // 启动转换信号
    output reg oe,            // 输出使能信号
    output wire addr_a,
    output wire addr_b,
    output wire addr_c,
    output reg [7:0] data_out,// 转换结果输出
    output reg data_valid,    // 数据有效信号
    output reg [1:0] state    // 状态指示（调试用）
);


// 参数定义 - 可配置部分
parameter CLK_FREQ = 27_000_000;    // 系统时钟频率(Hz)
parameter ADC_CLK_FREQ = 1_000_000;   // ADC时钟频率(Hz)，典型值500kHz


// 内部参数计算
localparam ADC_CLK_DIV = CLK_FREQ / ADC_CLK_FREQ;  // ADC时钟分频系数
localparam START_PULSE_CYCLES = 137; // START脉冲时钟周期数
localparam ADC_TRANSFROM_CYCLES = 270;//ADC转换所需的系统clk周期数

// 状态机定义
localparam [1:0]
    IDLE      = 2'b00,    // 空闲状态
    START_CONV = 2'b01,   // 启动转换
    WAIT_EOC  = 2'b11,    // 等待转换结束
    READ_DATA = 2'b10;    // 读取数据

// 内部信号
reg [1:0] current_state, next_state;
reg [15:0] clk_counter;           // 时钟分频计数器
reg [15:0] start_counter;          // START脉冲宽度计数器
reg [15:0] transfrom_counter;      //ADC转换时间计数器

reg [2:0] channel_sel;            // 通道选择寄存器

reg [3:0] adc_driver_start_reg;
wire adc_start;
always @(posedge clk or negedge rst_n) begin  //开始信号上升沿监测，开始信号同步
    if(!rst_n)adc_driver_start_reg<=0;
    else adc_driver_start_reg<={adc_driver_start_reg[2:0],adc_driver_start};
end
assign adc_start=(!adc_driver_start_reg[3])&&adc_driver_start_reg[0];

// ADC时钟生成
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_counter <= 16'd0;
        adc_clk <= 1'b0;
    end else begin
        if (clk_counter >= ADC_CLK_DIV/2 - 1) begin
            clk_counter <= 16'd0;
            adc_clk <= ~adc_clk;
        end else begin
            clk_counter <= clk_counter + 16'd1;
        end
    end
end

//ADC转换时间延时
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        transfrom_counter <= 8'd0;
    end else if (current_state == WAIT_EOC) begin
        if (transfrom_counter <= ADC_TRANSFROM_CYCLES-1) begin
            transfrom_counter <= transfrom_counter + 8'd1;
        end else begin
            transfrom_counter <= 8'd0;
        end
    end else begin
        transfrom_counter <= 8'd0;
    end
end


// START脉冲宽度计数器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_counter <= 8'd0;
    end else if (current_state == START_CONV) begin
        if (start_counter <= START_PULSE_CYCLES - 1) begin
            start_counter <= start_counter + 8'd1;
        end else begin
            start_counter <= 8'd0;
        end
    end else begin
        start_counter <= 8'd0;
    end
end

// 状态寄存器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

// 下一状态逻辑
always @(*) begin
    case (current_state)
        IDLE: begin
            next_state<=START_CONV;
        end
        
        START_CONV: begin
            if (start_counter == START_PULSE_CYCLES - 1) begin
                next_state = WAIT_EOC;
            end else begin
                next_state = START_CONV;
            end
        end
        
        WAIT_EOC: begin
            if(transfrom_counter==ADC_TRANSFROM_CYCLES-1)begin
                next_state<=READ_DATA;
            end else begin
                next_state<=WAIT_EOC;
            end
        end
        
        READ_DATA: begin
            if(adc_start)next_state<=IDLE;
            else next_state<=READ_DATA;
        end
        
        default: begin
            next_state = IDLE;
        end
    endcase
end

assign addr_c=channel_sel[2];
assign addr_b=channel_sel[1];
assign addr_a=channel_sel[0];
// 输出逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start <= 1'b0;
        oe <= 1'b0;
        data_out <= 8'h00;
        data_valid <= 1'b0;
        state <= IDLE;
        channel_sel <= 3'd3;
    end else begin
        state <= current_state;  // 状态输出，用于调试
        
        case (current_state)
            IDLE: begin
                start <= 1'b0;
                oe <= 1'b0;
                data_valid <= 1'b0;
                // 可以选择下一个通道
                //channel_sel <= channel_sel + 3'b001;
            end
            
            START_CONV: begin
                start <= 1'b1;         // 启动转换
                oe <= 1'b0;
                data_valid <= 1'b0;
            end
            
            WAIT_EOC: begin
                start <= 1'b0;  // START恢复低电平
                oe <= 1'b1;     // 开启输出允许信号
                data_valid <= 1'b0;
            end
            
            READ_DATA: begin
                oe <= 1'b1;           // 等待输出稳定后读取
                data_out <= data_in;  // 读取转换数据
                data_valid <= 1'b1;   // 数据有效信号
            end
            
            default: begin
                start <= 1'b0;
                oe <= 1'b0;
                data_valid <= 1'b0;
            end
        endcase
    end
end

endmodule