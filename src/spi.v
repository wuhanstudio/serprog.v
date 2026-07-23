`timescale 1ns/1ps

module spi_jedec_reader #(
    parameter integer CLKS_PER_HALF_BIT = 12
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    output reg        busy,
    output reg        done,
    output reg        flash_sck_out,
    output reg        flash_cs_out,
    output reg        flash_mosi_out,
    input  wire       flash_miso_in,
    output reg [7:0]  jedec0,
    output reg [7:0]  jedec1,
    output reg [7:0]  jedec2
);
    function [7:0] spi_tx_byte;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: spi_tx_byte = 8'h9F;
                default: spi_tx_byte = 8'h00;
            endcase
        end
    endfunction

    localparam [2:0] ST_IDLE     = 3'd0;
    localparam [2:0] ST_ASSERT   = 3'd1;
    localparam [2:0] ST_SETUP    = 3'd2;
    localparam [2:0] ST_CLK_HI   = 3'd3;
    localparam [2:0] ST_CLK_LO   = 3'd4;
    localparam [2:0] ST_DEASSERT = 3'd5;

    reg [2:0] state;
    reg [1:0] spi_byte_index;
    reg [2:0] spi_bit_index;
    reg [7:0] spi_tx_shift;
    reg [7:0] spi_rx_shift;
    reg [7:0] spi_rx0;
    reg [7:0] spi_rx1;
    reg [7:0] spi_rx2;
    reg [7:0] spi_rx3;
    reg [7:0] spi_clk_count;

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;

            flash_sck_out <= 1'b0;
            flash_cs_out <= 1'b1;
            flash_mosi_out <= 1'b0;

            spi_byte_index <= 2'd0;
            spi_bit_index <= 3'd7;
            spi_tx_shift <= 8'h9F;
            spi_rx_shift <= 8'd0;
            spi_rx0 <= 8'd0;
            spi_rx1 <= 8'd0;
            spi_rx2 <= 8'd0;
            spi_rx3 <= 8'd0;
            spi_clk_count <= 8'd0;

            jedec0 <= 8'd0;
            jedec1 <= 8'd0;
            jedec2 <= 8'd0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    flash_cs_out <= 1'b1;
                    flash_sck_out <= 1'b0;
                    flash_mosi_out <= 1'b0;

                    if (start) begin
                        busy <= 1'b1;
                        state <= ST_ASSERT;
                    end
                end

                ST_ASSERT: begin
                    flash_cs_out <= 1'b0;
                    flash_sck_out <= 1'b0;
                    spi_byte_index <= 2'd0;
                    spi_bit_index <= 3'd7;
                    spi_tx_shift <= spi_tx_byte(2'd0);
                    spi_rx_shift <= 8'd0;
                    spi_clk_count <= 8'd0;
                    state <= ST_SETUP;
                end

                ST_SETUP: begin
                    flash_sck_out <= 1'b0;
                    flash_mosi_out <= spi_tx_shift[spi_bit_index];
                    spi_clk_count <= 8'd0;
                    state <= ST_CLK_HI;
                end

                ST_CLK_HI: begin
                    if (spi_clk_count < CLKS_PER_HALF_BIT - 1) begin
                        spi_clk_count <= spi_clk_count + 8'd1;
                    end else begin
                        spi_clk_count <= 8'd0;
                        flash_sck_out <= 1'b1;
                        spi_rx_shift[spi_bit_index] <= flash_miso_in;
                        state <= ST_CLK_LO;
                    end
                end

                ST_CLK_LO: begin
                    if (spi_clk_count < CLKS_PER_HALF_BIT - 1) begin
                        spi_clk_count <= spi_clk_count + 8'd1;
                    end else begin
                        spi_clk_count <= 8'd0;
                        flash_sck_out <= 1'b0;

                        if (spi_bit_index == 3'd0) begin
                            case (spi_byte_index)
                                2'd0: spi_rx0 <= spi_rx_shift;
                                2'd1: spi_rx1 <= spi_rx_shift;
                                2'd2: spi_rx2 <= spi_rx_shift;
                                default: spi_rx3 <= spi_rx_shift;
                            endcase

                            spi_rx_shift <= 8'd0;

                            if (spi_byte_index == 2'd3) begin
                                state <= ST_DEASSERT;
                            end else begin
                                spi_byte_index <= spi_byte_index + 2'd1;
                                spi_bit_index <= 3'd7;
                                spi_tx_shift <= spi_tx_byte(spi_byte_index + 2'd1);
                                state <= ST_SETUP;
                            end
                        end else begin
                            spi_bit_index <= spi_bit_index - 3'd1;
                            state <= ST_SETUP;
                        end
                    end
                end

                ST_DEASSERT: begin
                    flash_cs_out <= 1'b1;
                    busy <= 1'b0;
                    jedec0 <= spi_rx1;
                    jedec1 <= spi_rx2;
                    jedec2 <= spi_rx3;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end
endmodule
