// INA226驱动模块 - I2C通讯控制器
// 功能：实现与INA226功率监测芯片的I2C通讯，支持读写操作


module INA226_Driver (
    input rst_n,           // 异步复位信号，低电平有效
    input start,           // 通讯启动信号，高电平有效，下降沿返回等待状态
    input rw,              // 读写控制：1=读取数据，0=写入数据
    inout SDA,             // I2C数据线（双向）
    output SCL,            // I2C时钟线
    input sys_clk,         // 系统时钟
    input [6:0] addr,      // INA226设备地址（7位）
    input [7:0] reg_addr,  // 寄存器地址（8位）
    input [15:0] tx_data,  // 发送数据（16位，写入操作时使用）
    output reg [15:0] rx_data, // 接收数据（16位，读取操作时使用）
    output reg busy,       // 忙信号，高电平表示正在通讯
    output reg done        // 完成信号，高电平表示通讯完成且数据稳定
    //output [7:0] TEST    // 测试端口（已注释）
);

// 内部信号定义
reg scl_oe;               // SCL输出使能
reg sda_oe;               // SDA输出使能  
reg scl_reg;              // SCL寄存器值
reg sda_reg;              // SDA寄存器值
reg iic_clk;              // I2C时钟（分频后）
reg [7:0] PC = 0;         // 程序计数器，控制状态机流程
reg [7:0] DPTR;           // 子程序返回地址指针
reg [7:0] TX_SHIFT_REG;   // 发送移位寄存器
reg [7:0] RX_SHIFT_REG;   // 接收移位寄存器
reg [3:0] BIT_CONT;       // 位计数器（8位数据传输）

// 三态输出控制
assign SCL = scl_oe ? scl_reg : 1'b1;  // SCL：使能时输出scl_reg，否则上拉
assign SDA = sda_oe ? sda_reg : 1'bz;  // SDA：使能时输出sda_reg，否则高阻

// I2C时钟频率配置
localparam SYS_FREQ  = 27_000_000;   // 系统时钟频率：27MHz
localparam IIC_FREQ  = 100_000;      // 目标I2C时钟频率：100kHz（标准模式）

// 计算分频系数
localparam DIV_RATIO = SYS_FREQ / (IIC_FREQ * 2);  // 分频比 = 135
localparam CNT_WIDTH = 16;            // 计数器位宽

reg [CNT_WIDTH-1:0] counter;          // 分频计数器

// I2C时钟分频器 - 将系统时钟分频为I2C时钟
always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 0;
        iic_clk <= 1'b0;
    end else begin
        if (counter == DIV_RATIO - 1) begin
            counter <= 0;
            iic_clk <= ~iic_clk;      // 翻转生成I2C时钟
        end else begin
            counter <= counter + 1'b1;
        end
    end
end

// 子程序地址定义
localparam SEND = 8'h80;  // 发送子程序入口地址
localparam READ = 8'h90;  // 接收子程序入口地址

// 主状态机 - 在I2C时钟边沿触发
always @(posedge iic_clk or negedge rst_n) begin
    if(!rst_n) begin
        // 复位初始化
        PC <= 0; 
        busy <= 0; 
        done <= 0;
    end
    else begin
        // 检测start信号下降沿，返回等待状态
        if (!start && PC != 8'h00) begin
            PC <= 8'h00;
            busy <= 0;
            done <= 0;
        end
        else begin
            case (PC)
                // 状态0：初始化和空闲状态 - 等待start信号
                8'h00: begin
                    scl_oe <= 0; scl_reg <= 1;     // SCL释放（高电平）
                    sda_oe <= 0; sda_reg <= 1;     // SDA释放（高电平）
                    RX_SHIFT_REG <= 0; TX_SHIFT_REG <= 0; BIT_CONT <= 8;
                    busy <= 0; done <= 0;          // 空闲状态
                    if (start) begin               // 检测到start信号开始通讯
                        PC <= 8'h01;
                        busy <= 1;                 // 进入忙状态
                    end
                end
                
                // 状态1-2：生成START条件（SDA在SCL高电平时由高变低）
                8'h01: begin
                    sda_oe <= 1; sda_reg <= 0;     // SDA拉低
                    PC <= PC + 1;
                end
                8'h02: begin
                    scl_oe <= 1; scl_reg <= 0;     // SCL拉低，完成START
                    PC <= PC + 1;
                end
                
                // 状态3：发送设备地址（写操作）
                8'h03: begin
                    TX_SHIFT_REG <= {addr, 1'b0};  // 地址+写标志(0)
                    DPTR <= 8'h04;                 // 设置返回地址
                    PC <= SEND;                    // 跳转到发送子程序
                end
                
                // 状态4：发送寄存器地址
                8'h04: begin
                    TX_SHIFT_REG <= reg_addr;      // 要访问的寄存器地址
                    PC <= SEND;                    // 跳转到发送子程序
                    if(rw) begin                   // 读操作
                        DPTR <= 8'h10;             // 完成后跳转到读流程
                    end
                    else begin                     // 写操作  
                        DPTR <= 8'h05;             // 完成后跳转到写数据流程
                    end
                end
                
                // 状态5-6：写操作 - 发送16位数据（先高8位，后低8位）
                8'h05: begin
                    TX_SHIFT_REG <= tx_data[15:8]; // 高8位数据
                    DPTR <= 8'h06;
                    PC <= SEND;
                end
                8'h06: begin
                    TX_SHIFT_REG <= tx_data[7:0];  // 低8位数据
                    DPTR <= 8'h07;
                    PC <= SEND;
                end
                
                // 状态7-8：写操作 - 生成STOP条件（SDA在SCL高电平时由低变高）
                8'h07: begin
                    scl_reg <= 1;                  // SCL先拉高
                    PC <= PC + 1;
                end
                8'h08: begin
                    sda_reg <= 1;                  // SDA再拉高，完成STOP
                    busy <= 0; 
                    done <= 1;                     // 写操作完成，数据稳定
                    if (!start) begin              // 如果start变低，返回等待状态
                        PC <= 8'h00;
                    end
                end
                
                // 状态10-14：读操作 - 在发送寄存器地址后重新开始
                8'h10: begin
                    scl_reg <= 1;                  // SCL拉高
                    PC <= PC + 1;
                end
                8'h11: begin
                    sda_reg <= 1;                  // SDA拉高
                    PC <= PC + 1;
                end
                8'h12: begin
                    PC <= PC + 1;                  // 等待一个周期
                end
                8'h13: begin
                    sda_reg <= 0;                  // SDA拉低，生成重复START
                    PC <= PC + 1;
                end
                8'h14: begin
                    scl_reg <= 0;                  // SCL拉低，完成重复START
                    PC <= PC + 1;
                end
                
                // 状态15：发送设备地址（读操作）
                8'h15: begin
                    TX_SHIFT_REG <= {addr, 1'b1};  // 地址+读标志(1)
                    DPTR <= 8'h16;
                    PC <= SEND;
                end
                
                // 状态16-18：读操作 - 接收16位数据（先高8位，后低8位）
                8'h16: begin                       // 读高8位数据
                    PC <= READ;
                    DPTR <= 8'h17;
                end
                8'h17: begin                       // 存储高8位，读低8位
                    rx_data[15:8] <= RX_SHIFT_REG; // 保存高8位数据
                    PC <= READ;
                    DPTR <= 8'h18;
                end
                8'h18: begin                       // 存储低8位
                    rx_data[7:0] <= RX_SHIFT_REG;  // 保存低8位数据
                    PC <= PC + 1;
                end
                
                // 状态19-1a：读操作 - 生成STOP条件
                8'h19: begin
                    scl_reg <= 1;                  // SCL拉高
                    PC <= PC + 1;
                end
                8'h1a: begin
                    sda_reg <= 1;                  // SDA拉高，完成STOP
                    busy <= 0; 
                    done <= 1;                     // 读操作完成，数据稳定
                    if (!start) begin              // 如果start变低，返回等待状态
                        PC <= 8'h00;
                    end
                end

                // === 发送子程序 (SEND: 0x80-0x86) ===
                // 功能：发送一个字节数据并检测应答
                SEND: begin
                    scl_reg <= 0; sda_reg <= 0;
                    scl_oe <= 1; sda_oe <= 1;      // 使能SCL和SDA输出
                    BIT_CONT <= 8;                 // 初始化位计数器
                    PC <= PC + 1;
                end
                8'h81: begin                       // 准备数据位
                    sda_reg <= TX_SHIFT_REG[7];    // 发送最高位（MSB first）
                    BIT_CONT <= BIT_CONT - 1'b1;   // 位计数减1
                    PC <= PC + 1;
                end
                8'h82: begin
                    scl_reg <= 1;                  // SCL拉高，数据采样
                    PC <= PC + 1;
                end
                8'h83: begin
                    scl_reg <= 0;                  // SCL拉低
                    TX_SHIFT_REG <= {TX_SHIFT_REG[6:0], 1'b0}; // 数据左移
                    if(BIT_CONT != 0) PC <= 8'h81; // 继续发送剩余位
                    else PC <= PC + 1;             // 字节发送完成
                end
                8'h84: begin                       // 释放SDA，准备接收应答
                    sda_oe <= 0; sda_reg <= 0;     // SDA设为高阻，等待从机应答
                    PC <= PC + 1;
                end
                8'h85: begin
                    scl_reg <= 1;                  // SCL拉高，检测应答位
                    PC <= PC + 1;
                end
                8'h86: begin                       // 恢复SDA控制，结束发送
                    scl_reg <= 0; scl_oe <= 1;
                    sda_reg <= 0; sda_oe <= 1;
                    PC <= DPTR;                    // 返回主程序
                end

                // === 接收子程序 (READ: 0x90-0x99) ===
                // 功能：接收一个字节数据并发送应答
                8'h90: begin
                    scl_reg <= 0; scl_oe <= 1;     // SCL输出使能
                    sda_oe <= 0;                   // SDA输入模式
                    BIT_CONT <= 8; RX_SHIFT_REG <= 0; // 初始化
                    PC <= PC + 1;
                end
                8'h91: begin
                    scl_reg <= 1; PC <= PC + 1;    // SCL拉高，准备采样
                end
                8'h92: begin                       // 采样数据位
                    RX_SHIFT_REG[0] <= SDA;        // 接收数据（LSB first）
                    BIT_CONT <= BIT_CONT - 1'b1;   // 位计数减1
                    PC <= PC + 1;
                end
                8'h93: begin
                    scl_reg <= 0;                  // SCL拉低
                    PC <= PC + 1;
                end
                8'h94: begin
                    RX_SHIFT_REG <= {RX_SHIFT_REG[6:0], TX_SHIFT_REG[7]}; // 数据移位
                    if(BIT_CONT != 0) PC <= 8'h91; // 继续接收剩余位
                    else PC <= PC + 1;             // 字节接收完成
                end
                8'h95: begin                       // 发送应答信号（低电平）
                    sda_reg <= 0; sda_oe <= 1;     // SDA拉低表示应答
                    PC <= PC + 1;
                end
                8'h96: begin
                    scl_reg <= 1;                  // SCL拉高
                    PC <= PC + 1;
                end
                8'h97: begin
                    PC <= PC + 1;                  // 等待
                end
                8'h98: begin
                    scl_reg <= 0;                  // SCL拉低
                    PC <= PC + 1;
                end
                8'h99: begin                       // 恢复SDA控制，结束接收
                    scl_oe <= 1; sda_oe <= 1;
                    scl_reg <= 0; sda_reg <= 0;
                    PC <= DPTR;                    // 返回主程序
                end
                
                default: PC <= PC + 1;             // 默认状态转移
            endcase
        end
    end
end
    
endmodule