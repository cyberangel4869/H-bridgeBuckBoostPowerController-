// PWM信号发生器模块
// 功能：根据工作模式生成不同拓扑结构所需的PWM信号
module PWM_gen(
    input sys_clk,        // 系统时钟，上升沿有效
    input rst_n,          // 异步复位信号，低电平有效
    input [7:0] Duty,     // 占空比设置，范围0-255 (对应0%-100%)
    input [1:0] Mode,     // 工作模式选择
    output reg PWM_LOW_1,     // 通道1低侧驱动信号
    output reg PWM_HIGH_1,    // 通道1高侧驱动信号  
    output reg PWM_LOW_2,     // 通道2低侧驱动信号
    output reg PWM_HIGH_2,     // 通道2高侧驱动信号
    output PWM_clk              //测试用
);

// 工作模式定义
localparam OFF_MODE = 2'b00;   // 关闭模式：所有PWM输出无效
localparam BUCK_MODE = 2'b01;  // BUCK模式：仅通道1工作，用于输入大于目标电压的两倍
localparam BOOST_MODE = 2'b10; // BOOST模式：仅通道2工作，用于输入小于目标电压
localparam BUCK_BOOST = 2'B11; //BUCK-BOOST模式，通道1，2均工作，用于输入电压和目标电压相近时

reg PWM;                    // 内部PWM信号
reg [1:0] Mode_reg;
reg [7:0] counter;         // 8位计数器，用于PWM生成
assign PWM_clk=counter[7];

// 主控制逻辑：时钟上升沿或复位下降沿触发
always @(negedge rst_n or posedge sys_clk) begin
    if(!rst_n) begin
        // 异步复位：初始化所有寄存器
        counter <= 0;
        PWM <= 0;
        Mode_reg<=2'b00;
    end
    else begin
        // 计数器达到最大值时更新通道使能信号
        if(counter == 8'hff) begin
            Mode_reg <= Mode;  // 根据模式设置通道使能
        end
        
        // 计数器递增，达到255后自动归零
        counter <= counter + 1'b1;
        
        // PWM生成逻辑：计数器值小于占空比设置时输出高电平
        if(counter < Duty) begin
            PWM <= 1;      // 高电平阶段
        end
        else begin
            PWM <= 0;      // 低电平阶段
        end
    end
end

always @(*) begin
    case (Mode_reg)
        OFF_MODE: begin
            PWM_LOW_1=0;
            PWM_HIGH_1=0;
            PWM_LOW_2=0;
            PWM_HIGH_2=0;
        end
        BUCK_MODE:begin
            PWM_HIGH_1=PWM;
            PWM_LOW_1=~PWM;
            PWM_LOW_2=0;
            PWM_HIGH_2=1;
        end
        BOOST_MODE:begin
            PWM_HIGH_1=1;
            PWM_LOW_1=0;
            PWM_LOW_2=PWM;
            PWM_HIGH_2=~PWM;
        end
        BUCK_BOOST:begin
            if(PWM)begin
                PWM_HIGH_1=1;
                PWM_LOW_1=0;
                PWM_HIGH_2=0;
                PWM_LOW_2=1;
            end
            else begin
                PWM_HIGH_1=0;
                PWM_LOW_1=1;
                PWM_HIGH_2=1;
                PWM_LOW_2=0;
            end
        end
        default: begin
            PWM_LOW_1=0;
            PWM_HIGH_1=0;
            PWM_LOW_2=0;
            PWM_HIGH_2=0;
        end
    endcase
end

endmodule