`timescale 1ns/1ps

module spi_byte_engine #(
    parameter integer CLKS_PER_HALF_BIT = 12
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] tx_byte,
    input  wire       miso,
    output reg        busy,
    output reg        done,
    output reg  [7:0] rx_byte,
    output reg        sck,
    output reg        mosi
);
    localparam [1:0] ST_IDLE   = 2'd0;
    localparam [1:0] ST_CLK_HI = 2'd1;
    localparam [1:0] ST_CLK_LO = 2'd2;

    reg [1:0] state;
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [2:0] bit_index;
    reg [15:0] clk_count;

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            tx_shift <= 8'd0;
            rx_shift <= 8'd0;
            rx_byte <= 8'd0;
            bit_index <= 3'd7;
            clk_count <= 16'd0;
            busy <= 1'b0;
            done <= 1'b0;
            sck <= 1'b0;
            mosi <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    sck <= 1'b0;

                    if (start) begin
                        busy <= 1'b1;
                        tx_shift <= tx_byte;
                        rx_shift <= 8'd0;
                        bit_index <= 3'd7;
                        clk_count <= 16'd0;
                        mosi <= tx_byte[7];
                        state <= ST_CLK_HI;
                    end
                end

                ST_CLK_HI: begin
                    if (clk_count < CLKS_PER_HALF_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        sck <= 1'b1;
                        rx_shift[bit_index] <= miso;
                        state <= ST_CLK_LO;
                    end
                end

                ST_CLK_LO: begin
                    if (clk_count < CLKS_PER_HALF_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        sck <= 1'b0;

                        if (bit_index == 3'd0) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            rx_byte <= rx_shift;
                            state <= ST_IDLE;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            mosi <= tx_shift[bit_index - 3'd1];
                            state <= ST_CLK_HI;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end
endmodule
