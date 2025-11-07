module boost_pid (
    input wire clk,                    // 时钟信号 (27MHz)
    input wire rst_n,                  // 复位信号（低电平有效）
    input wire ready,                  // 电压测量完成信号（高有效）
    input wire [15:0] setpoint,        // 目标电压值（16位无符号）
    input wire [15:0] voltage_actual,  // 实际电压值（16位无符号）
    output reg start,                  // 开始测量信号（高有效）
    output reg [7:0] new_duty          // PID控制后占空比（8位无符号）
);

// PID参数（Q8.8定点数格式）
parameter signed [15:0] uint16_Kp = 16'hffff;
//parameter signed [15:0] uint16_Ki = 16'sh0010; // 积分增益 = 0.0625
parameter signed [15:0] uint16_Kd = 16'sh0000; // 微分增益 = 0.1875

reg signed [16:0] int16_error,int16_error_prev;

// 10Hz控制周期计数器 (27MHz / 100Hz )
parameter CTR_MAX = 27'd270;       // 100Hz周期计数器最大值

// 状态机定义
localparam IDLE        = 2'b00;        // 空闲状态
localparam WAIT_READY  = 2'b01;        // 等待测量完成
localparam CALCULATE   = 2'b10;        // 计算PID
localparam UPDATE      = 2'b11;        // 更新输出

// 寄存器定义
reg [1:0] state, next_state;           // 状态机寄存器
reg [26:0] period_counter;             // 10Hz周期计数器
reg signed [8:0] int8_internal_duty;               // 内部占空比寄存器（从0开始）
reg signed [8:0] int8_new_duty;
reg start_measure;                     // 内部开始测量信号

// 输入同步寄存器
reg [15:0] uint16_setpoint_reg, uint16_voltage_actual_reg;


reg signed [32:0] int32_pid_raw;
reg signed [8:0] int8_delt_duty;


// 10Hz周期计数器
always @(negedge clk or negedge rst_n) begin
    if (!rst_n) begin
        period_counter <= 27'b0;
        start_measure <= 1'b0;
    end else begin
        if (period_counter >= CTR_MAX/2 - 1) begin
            period_counter <= 27'b0;
            start_measure <= 1'b1;     // 产生开始测量脉冲
        end else begin
            period_counter <= period_counter + 1'b1;
            start_measure=1'b0;
        end
    end
end

// 输入同步：避免亚稳态
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uint16_setpoint_reg <= 16'b0;
        uint16_voltage_actual_reg <= 16'b0;
    end else begin
        uint16_setpoint_reg <= setpoint;
        uint16_voltage_actual_reg <= voltage_actual;
    end
end

// 状态转移逻辑
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        next_state=IDLE;
    end
    else begin
    case (state)
        IDLE: begin
            if (start_measure) begin
                next_state = WAIT_READY;
            end else begin
                next_state = IDLE;
            end
        end
        
        WAIT_READY: begin
            if (ready) begin  // 检测ready上升沿
                next_state = CALCULATE;
            end else begin
                next_state = WAIT_READY;
            end
        end
        
        CALCULATE: begin
            next_state = UPDATE;       // 计算完成，进入更新状态
        end
        
        UPDATE: begin
            next_state = IDLE;         // 更新完成，返回空闲
        end
        
        default: next_state = IDLE;
    endcase
    state<=next_state;
    end
end

// 输出逻辑和数据处理
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start <= 1'b0;
        new_duty <= 8'b0;
        int8_internal_duty <= 9'b0;
        int16_error<=0;
        int16_error_prev<=0;
    end else begin
        case (state)
            IDLE: begin
                start <= 1'b0;
            end
            
            WAIT_READY: begin
                start <= 1'b1;         // 输出开始测量信号
            end
            
            CALCULATE: begin
                start <= 1'b0;
                int16_error_prev=int16_error;
                int16_error=$signed({1'b0,uint16_setpoint_reg})-$signed({1'b0,uint16_voltage_actual_reg});
                int32_pid_raw=int16_error*$signed({1'b0,uint16_Kp})-(int16_error-int16_error_prev)*$signed({1'b0,uint16_Kd});
                int8_delt_duty=int32_pid_raw[28:20];
            end
            
            UPDATE: begin
                int8_new_duty=int8_internal_duty+int8_delt_duty;
                if(int8_new_duty>9'h0ff)begin
                    new_duty=8'hff;
                end else if(int8_new_duty<9'h000)begin
                    new_duty=8'h00;
                end else begin
                    new_duty=int8_new_duty[7:0];
                end
                int8_internal_duty=$signed({1'b0,new_duty});
            end
            
            default: begin
                start <= 1'b0;
            end
        endcase
    end
end



endmodule