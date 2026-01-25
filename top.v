
module top (
    input sys_clk,
    input rst_n,
    output PWM_HIGH_1,
    output PWM_LOW_1,
    output PWM_HIGH_2,
    output PWM_LOW_2,

    input [7:0] adc_data_in,
    output adc_clk,
    output adc_start,
    output oe,
    output addr_a,
    output addr_b,
    output addr_c,

    input uart_rx,
    output wire uart_tx,
    output wire [3:0] test

);
wire adc_driver_start,adc_done,
    pid_start,
    uart_send,uart_received,uart_busy;
wire [7:0] voltage;
wire [7:0] uart_rx_data;
wire [7:0] pid_setpoint;
wire [7:0] PWM_duty;

time_ctrl CTRL(
    .sys_clk(sys_clk),
    .rst_n(rst_n),

    .adc_start(adc_driver_start),
    .adc_done(adc_done),

    .uart_send(uart_send),
    .uart_busy(uart_busy),
    .uart_received(uart_received),
    .uart_rx_data(uart_rx_data),

    .pid_start(pid_start),
    .pid_setpoint(pid_setpoint)
);

adc0809_driver ADC(
    .clk(sys_clk),
    .rst_n(rst_n),
    .adc_driver_start(adc_driver_start),
    .data_in(adc_data_in),
    .adc_clk(adc_clk),
    .start(adc_start),
    .oe(oe),
    .addr_a(addr_a),
    .addr_b(addr_b),
    .addr_c(addr_c),
    .data_out(voltage),
    .data_valid(adc_done)
);

uart UART(
    .clk_27m(sys_clk),
    .rst_n(rst_n),
    .tx_data(voltage),
    .tx_start(uart_send),
    .txd(uart_tx),
    .rx_valid(uart_received),
    .rx_data(uart_rx_data),
    .rxd(uart_rx),
    .debug_state_tx(test)
);

pid PID(
    .clk(sys_clk),
    .rst_n(rst_n),
    .ready(pid_start),
    .setpoint(pid_setpoint),
    .voltage_actual(voltage),
    .new_duty(PWM_duty)
);

PWM_gen H_bridge(
    .sys_clk(sys_clk),
    .rst_n(rst_n),
    .Duty(PWM_duty),
    .Mode(2'b01),
    .PWM_HIGH_1(PWM_HIGH_1),
    .PWM_LOW_1(PWM_LOW_1),
    .PWM_HIGH_2(PWM_HIGH_2),
    .PWM_LOW_2(PWM_LOW_2)
);
endmodule