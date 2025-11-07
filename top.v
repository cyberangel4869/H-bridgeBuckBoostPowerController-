
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

    output [7:0] test_voltage
);
wire pwm_clk,adc_finish;
//assign test_adc_finish=adc_finish;
//assign test_pwm_clk=pwm_clk;
wire [7:0] voltage;
assign test_voltage=PWM_duty;
wire [7:0] PWM_duty;
reg [15:0] counter;
reg adc_driver_start;

always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n)begin
        counter<=0;adc_driver_start<=0;
    end else begin
        if(counter<16'd2700)begin
            counter<=counter+1'b1;
            adc_driver_start<=0;
        end else begin
            counter<=0;
            adc_driver_start<=1;
        end
    end
end

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
    .data_valid(adc_finish)
);

boost_pid PID(
    .clk(sys_clk),
    .rst_n(rst_n),
    .ready(adc_finish),
    .setpoint(8'd214),
    .voltage_actual(voltage),
    .new_duty(PWM_duty),
    .test_state(test_pid_state)
);

PWM_gen H_bridge(
    .sys_clk(sys_clk),
    .rst_n(rst_n),
    .Duty(PWM_duty),
    .Mode(2'b10),
    .PWM_HIGH_1(PWM_HIGH_1),
    .PWM_LOW_1(PWM_LOW_1),
    .PWM_HIGH_2(PWM_HIGH_2),
    .PWM_LOW_2(PWM_LOW_2),
    .PWM_clk(pwm_clk)
);
endmodule