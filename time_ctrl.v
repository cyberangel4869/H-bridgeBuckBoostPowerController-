module time_ctrl (
    input sys_clk,
    input rst_n,
    output reg adc_start,
    input adc_done,
    input uart_busy,
    output reg uart_send,
    input uart_received,
    input[7:0] uart_rx_data,
    output reg pid_start,
    output reg [7:0] pid_setpoint
);
reg [15:0] counter_10k;
//ADC测量与PID调控周期10KHz
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n)begin
        counter_10k<=0;adc_start<=0;
    end else begin
        if(counter_10k<16'd1349)begin
            counter_10k<=counter_10k+1'b1;
            adc_start<=0;
        end else begin
            adc_start<=1;
            if(counter_10k<16'd2700)counter_10k<=counter_10k+1'b1;
            else counter_10k<=0;
        end
    end
end

//运行逻辑控制
reg [7:0] setpoint;
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n)begin
        pid_start<=0;
        pid_setpoint<=8'd0;
        setpoint<=8'd0;
    end else begin
        if(adc_done)begin
            pid_setpoint<=setpoint;
            pid_start<=1;
        end else begin
            pid_start<=0;
        end
        if(uart_received)begin
            setpoint<=uart_rx_data;
        end else begin
            pid_setpoint<=setpoint;
        end
    end
end

reg [1:0] adc_done_edge_reg;
reg [7:0] adc_done_edge_counter;
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n)begin
        adc_done_edge_counter<=0;
        adc_done_edge_reg<=0;
        uart_send<=0;
    end else begin
        adc_done_edge_reg<={adc_done_edge_reg[0],adc_done};
        if(adc_done_edge_reg[0]&&(!adc_done_edge_reg[1]))begin
            if(adc_done_edge_counter>50)begin
                uart_send<=1;
                if(adc_done_edge_counter>100)adc_done_edge_counter<=0;
                else adc_done_edge_counter<=adc_done_edge_counter+1'b1;
            end else begin
                uart_send<=0;
                adc_done_edge_counter<=adc_done_edge_counter+1'b1;
            end
        end
    end
end
endmodule