`timescale 1ns/1ps

module app #(
    parameter integer CLKS_PER_BIT = 208,
    parameter integer MSG_LEN = 13,
    parameter integer SPI_CLKS_PER_HALF_BIT = 12
) (
    input  wire clk,
    input  wire rst,
    output wire uart_tx_o,
    output wire flash_sck_out,
    output wire flash_cs_out,
    output wire flash_mosi_out,
    input  wire flash_miso_in,
    output wire done
);
    function [7:0] msg_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  msg_byte = 8'h48; // H
                5'd1:  msg_byte = 8'h65; // e
                5'd2:  msg_byte = 8'h6C; // l
                5'd3:  msg_byte = 8'h6C; // l
                5'd4:  msg_byte = 8'h6F; // o
                5'd5:  msg_byte = 8'h20; // space
                5'd6:  msg_byte = 8'h57; // W
                5'd7:  msg_byte = 8'h6F; // o
                5'd8:  msg_byte = 8'h72; // r
                5'd9:  msg_byte = 8'h6C; // l
                5'd10: msg_byte = 8'h64; // d
                5'd11: msg_byte = 8'h0D; // CR
                5'd12: msg_byte = 8'h0A; // LF
                default: msg_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] hex_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10) begin
                hex_ascii = 8'h30 + {4'd0, nibble};
            end else begin
                hex_ascii = 8'h41 + {4'd0, (nibble - 4'd10)};
            end
        end
    endfunction

    function [7:0] jedec_ascii_char;
        input [2:0] idx;
        input [7:0] r1;
        input [7:0] r2;
        input [7:0] r3;
        begin
            case (idx)
                3'd0: jedec_ascii_char = hex_ascii(r1[7:4]);
                3'd1: jedec_ascii_char = hex_ascii(r1[3:0]);
                3'd2: jedec_ascii_char = hex_ascii(r2[7:4]);
                3'd3: jedec_ascii_char = hex_ascii(r2[3:0]);
                3'd4: jedec_ascii_char = hex_ascii(r3[7:4]);
                3'd5: jedec_ascii_char = hex_ascii(r3[3:0]);
                3'd6: jedec_ascii_char = 8'h0D;
                default: jedec_ascii_char = 8'h0A;
            endcase
        end
    endfunction

    localparam [3:0] ST_HELLO_SEND   = 4'd0;
    localparam [3:0] ST_HELLO_WAIT   = 4'd1;
    localparam [3:0] ST_SPI_START    = 4'd2;
    localparam [3:0] ST_SPI_WAIT     = 4'd3;
    localparam [3:0] ST_JEDEC_SEND   = 4'd4;
    localparam [3:0] ST_JEDEC_WAIT   = 4'd5;
    localparam [3:0] ST_DONE         = 4'd6;

    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;

    reg [4:0] msg_index;
    reg [2:0] jedec_index;
    reg [3:0] state;
    reg       all_done;
    reg       spi_start;

    wire      spi_busy;
    wire      spi_done;
    wire [7:0] spi_jedec0;
    wire [7:0] spi_jedec1;
    wire [7:0] spi_jedec2;

    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_tx (
        .clk(clk),
        .rst(rst),
        .tx_data(tx_data),
        .tx_start(tx_start),
        .tx(uart_tx_o),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    spi_jedec_reader #(
        .CLKS_PER_HALF_BIT(SPI_CLKS_PER_HALF_BIT)
    ) u_spi_jedec (
        .clk(clk),
        .rst(rst),
        .start(spi_start),
        .busy(spi_busy),
        .done(spi_done),
        .flash_sck_out(flash_sck_out),
        .flash_cs_out(flash_cs_out),
        .flash_mosi_out(flash_mosi_out),
        .flash_miso_in(flash_miso_in),
        .jedec0(spi_jedec0),
        .jedec1(spi_jedec1),
        .jedec2(spi_jedec2)
    );

    assign done = all_done;

    always @(posedge clk) begin
        if (rst) begin
            tx_data   <= 8'd0;
            tx_start  <= 1'b0;
            msg_index <= 5'd0;
            jedec_index <= 3'd0;
            state <= ST_HELLO_SEND;
            all_done  <= 1'b0;
            spi_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            spi_start <= 1'b0;

            case (state)
                ST_HELLO_SEND: begin
                    if (!tx_busy) begin
                        tx_data  <= msg_byte(msg_index);
                        tx_start <= 1'b1;
                        state <= ST_HELLO_WAIT;
                    end
                end

                ST_HELLO_WAIT: begin
                    if (tx_done) begin
                        if (msg_index == MSG_LEN - 1) begin
                            state <= ST_SPI_START;
                        end else begin
                            msg_index <= msg_index + 5'd1;
                            state <= ST_HELLO_SEND;
                        end
                    end
                end

                ST_SPI_START: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        state <= ST_SPI_WAIT;
                    end
                end

                ST_SPI_WAIT: begin
                    if (spi_done) begin
                        jedec_index <= 3'd0;
                        state <= ST_JEDEC_SEND;
                    end
                end

                ST_JEDEC_SEND: begin
                    if (!tx_busy) begin
                        tx_data <= jedec_ascii_char(jedec_index, spi_jedec0, spi_jedec1, spi_jedec2);
                        tx_start <= 1'b1;
                        state <= ST_JEDEC_WAIT;
                    end
                end

                ST_JEDEC_WAIT: begin
                    if (tx_done) begin
                        if (jedec_index == 3'd7) begin
                            all_done <= 1'b1;
                            state <= ST_DONE;
                        end else begin
                            jedec_index <= jedec_index + 3'd1;
                            state <= ST_JEDEC_SEND;
                        end
                    end
                end

                ST_DONE: begin
                    all_done <= 1'b1;
                end

                default: begin
                    state <= ST_HELLO_SEND;
                end
            endcase
        end
    end
endmodule
