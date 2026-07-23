`timescale 1ns/1ps

module uart_hello_fpga_top #(
    parameter integer CLK_FREQ_HZ = 24000000,
    parameter integer BAUD_RATE   = 115200,
    parameter integer RESET_CYCLES = 100000
) (
    input  wire clk,
    input  wire reset_n,
    output wire uart_tx,
    output wire led_done,
    output wire flash_sck_out,
    output wire flash_cs_out,
    output wire flash_mosi_out,
    input  wire flash_miso_in
);
    // Integer divider: for 24 MHz and 115200 baud this resolves to 208.
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    reg [31:0] reset_count = 32'd0;
    reg        rst_i = 1'b1;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            reset_count <= 32'd0;
            rst_i <= 1'b1;
        end else begin
            if (reset_count < RESET_CYCLES) begin
                reset_count <= reset_count + 32'd1;
                rst_i <= 1'b1;
            end else begin
                rst_i <= 1'b0;
            end
        end
    end

    uart_hello #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_hello (
        .clk(clk),
        .rst(rst_i),
        .uart_tx_o(uart_tx),
        .flash_sck_out(flash_sck_out),
        .flash_cs_out(flash_cs_out),
        .flash_mosi_out(flash_mosi_out),
        .flash_miso_in(flash_miso_in),
        .done(led_done)
    );
endmodule
