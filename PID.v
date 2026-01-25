//2025.12.27

module pid (
    input wire clk,                    // 时钟信号 (27MHz)
    input wire rst_n,                  // 复位信号（低电平有效）
    input wire ready,                  // 电压测量完成信号（高有效）
    input wire [7:0] setpoint,        // 目标电压值（16位无符号）
    input wire [7:0] voltage_actual,  // 实际电压值（16位无符号）
    output reg [7:0] new_duty,          // PID控制后占空比（8位无符号）

    output wire [1:0] test_state
);

// PID参数（Q0.8定点数格式）
parameter signed [7:0] uint8_Kp = 8'd35;
parameter signed [7:0] uint8_Ki = 8'd0; // 积分增益 = 0
parameter signed [7:0] uint8_Kd = 8'd10; // 微分增益 = 0.1875

reg signed [8:0] int9_error,int9_error_prev,int9_error_prev1;





// 状态机定义

localparam WAIT_READY  = 2'b00;        // 等待测量完成
localparam CALCULATE   = 2'b01;        // 计算PID
localparam UPDATE      = 2'b11;        // 更新输出

// 寄存器定义
reg [1:0] state, next_state;           // 状态机寄存器
reg signed [8:0] int9_internal_duty;               // 内部占空比寄存器（从0开始）
reg signed [8:0] int9_new_duty;
reg [3:0] ready_reg;                  //用于检测ready信号的上升沿

assign test_state=state;

// 输入同步寄存器
reg [7:0] uint8_setpoint_reg, uint8_voltage_actual_reg;


reg signed [17:0] int18_pid_raw;
reg signed [8:0] int9_delt_duty;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        ready_reg<=4'd0;
    end else begin
        ready_reg<={ready_reg[2:0],ready};
    end
end


// 输入同步：避免亚稳态
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uint8_setpoint_reg <= 8'b0;
        uint8_voltage_actual_reg <= 8'b0;
    end else begin
        uint8_setpoint_reg <= setpoint;
        uint8_voltage_actual_reg <= voltage_actual;
    end
end

// 状态转移逻辑
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        next_state=WAIT_READY;
    end
    else begin
    case (state)
        WAIT_READY: begin
            if ((!ready_reg[3])&&ready_reg[0]) begin  // 检测ready上升沿
                next_state = CALCULATE;
            end else begin
                next_state = WAIT_READY;
            end
        end
        
        CALCULATE: begin
            next_state = UPDATE;       // 计算完成，进入更新状态
        end
        
        UPDATE: begin
            next_state = WAIT_READY;         // 更新完成，返回空闲
        end
        
        default: next_state = WAIT_READY;
    endcase
    state<=next_state;
    end
end

// 输出逻辑和数据处理
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        new_duty <= 8'b0;
        int9_internal_duty <= 9'b0;
        int9_error<=0;
        int9_error_prev<=0;
        int9_error_prev1<=0;
    end else begin
        case (state)
            CALCULATE: begin
                int9_error_prev1<=int9_error_prev;
                int9_error_prev<=int9_error;
                int9_error=$signed({1'b0,uint8_setpoint_reg})-$signed({1'b0,uint8_voltage_actual_reg});
                int18_pid_raw=int9_error*$signed({1'b0,uint8_Kp})-(int9_error-int9_error_prev)*$signed({1'b0,uint8_Kd})
                +(int9_error_prev1+int9_error_prev)*$signed({1'b0,uint8_Ki});
                int9_delt_duty=int18_pid_raw[16:8];
            end
            
            UPDATE: begin
                int9_new_duty=int9_internal_duty+int9_delt_duty;
                if(int9_new_duty[8])begin//防止最高位溢出
                    new_duty=8'h00;
                end else begin
                    new_duty=int9_new_duty[7:0];
                end
                int9_internal_duty=$signed({1'b0,new_duty});
            end
            
            default: begin

            end
        endcase
    end
end



endmodule