`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLKS_PER_BIT = 208
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx,
    output reg        tx_busy,
    output reg        tx_done
);
    localparam [1:0] IDLE  = 2'd0;
    localparam [1:0] START = 2'd1;
    localparam [1:0] DATA  = 2'd2;
    localparam [1:0] STOP  = 2'd3;

    reg [1:0] state;
    reg [13:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] data_reg;

    always @(posedge clk) begin
        if (rst) begin
            state      <= IDLE;
            clk_count  <= 14'd0;
            bit_index  <= 3'd0;
            data_reg   <= 8'd0;
            tx         <= 1'b1;
            tx_busy    <= 1'b0;
            tx_done    <= 1'b0;
        end else begin
            tx_done <= 1'b0;

            case (state)
                IDLE: begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                    clk_count <= 14'd0;
                    bit_index <= 3'd0;

                    if (tx_start) begin
                        tx_busy  <= 1'b1;
                        data_reg <= tx_data;
                        state    <= START;
                    end
                end

                START: begin
                    tx <= 1'b0;
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 14'd1;
                    end else begin
                        clk_count <= 14'd0;
                        state <= DATA;
                    end
                end

                DATA: begin
                    tx <= data_reg[bit_index];
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 14'd1;
                    end else begin
                        clk_count <= 14'd0;
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 3'd1;
                        end else begin
                            bit_index <= 3'd0;
                            state <= STOP;
                        end
                    end
                end

                STOP: begin
                    tx <= 1'b1;
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 14'd1;
                    end else begin
                        clk_count <= 14'd0;
                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;
                        state <= IDLE;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule

module uart_hello #(
    parameter integer CLKS_PER_BIT = 208,
    parameter integer MSG_LEN = 13,
    parameter integer SPI_CLKS_PER_HALF_BIT = 12
) (
    input  wire clk,
    input  wire rst,
    output wire uart_tx_o,
    output reg  flash_sck_out,
    output reg  flash_cs_out,
    output reg  flash_mosi_out,
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

    function [7:0] spi_tx_byte;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: spi_tx_byte = 8'h9F;
                default: spi_tx_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] jedec_byte;
        input [1:0] idx;
        input [7:0] r1;
        input [7:0] r2;
        input [7:0] r3;
        begin
            case (idx)
                2'd0: jedec_byte = r1;
                2'd1: jedec_byte = r2;
                default: jedec_byte = r3;
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
    localparam [3:0] ST_SPI_ASSERT   = 4'd2;
    localparam [3:0] ST_SPI_SETUP    = 4'd3;
    localparam [3:0] ST_SPI_CLK_HI   = 4'd4;
    localparam [3:0] ST_SPI_CLK_LO   = 4'd5;
    localparam [3:0] ST_SPI_DEASSERT = 4'd6;
    localparam [3:0] ST_JEDEC_SEND   = 4'd7;
    localparam [3:0] ST_JEDEC_WAIT   = 4'd8;
    localparam [3:0] ST_DONE         = 4'd9;

    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;

    reg [4:0] msg_index;
    reg [2:0] jedec_index;
    reg [3:0] state;
    reg       all_done;

    reg [1:0] spi_byte_index;
    reg [2:0] spi_bit_index;
    reg [7:0] spi_tx_shift;
    reg [7:0] spi_rx_shift;
    reg [7:0] spi_rx0;
    reg [7:0] spi_rx1;
    reg [7:0] spi_rx2;
    reg [7:0] spi_rx3;
    reg [7:0] spi_clk_count;

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

    assign done = all_done;

    always @(posedge clk) begin
        if (rst) begin
            tx_data   <= 8'd0;
            tx_start  <= 1'b0;
            msg_index <= 5'd0;
            jedec_index <= 3'd0;
            state <= ST_HELLO_SEND;
            all_done  <= 1'b0;

            flash_sck_out  <= 1'b0;
            flash_cs_out   <= 1'b1;
            flash_mosi_out <= 1'b0;

            spi_byte_index <= 2'd0;
            spi_bit_index  <= 3'd7;
            spi_tx_shift   <= 8'h9F;
            spi_rx_shift   <= 8'd0;
            spi_rx0 <= 8'd0;
            spi_rx1 <= 8'd0;
            spi_rx2 <= 8'd0;
            spi_rx3 <= 8'd0;
            spi_clk_count <= 8'd0;
        end else begin
            tx_start <= 1'b0;

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
                            state <= ST_SPI_ASSERT;
                        end else begin
                            msg_index <= msg_index + 5'd1;
                            state <= ST_HELLO_SEND;
                        end
                    end
                end

                ST_SPI_ASSERT: begin
                    flash_cs_out <= 1'b0;
                    flash_sck_out <= 1'b0;
                    spi_byte_index <= 2'd0;
                    spi_bit_index <= 3'd7;
                    spi_tx_shift <= spi_tx_byte(2'd0);
                    spi_rx_shift <= 8'd0;
                    spi_clk_count <= 8'd0;
                    state <= ST_SPI_SETUP;
                end

                ST_SPI_SETUP: begin
                    flash_sck_out <= 1'b0;
                    flash_mosi_out <= spi_tx_shift[spi_bit_index];
                    spi_clk_count <= 8'd0;
                    state <= ST_SPI_CLK_HI;
                end

                ST_SPI_CLK_HI: begin
                    if (spi_clk_count < SPI_CLKS_PER_HALF_BIT - 1) begin
                        spi_clk_count <= spi_clk_count + 8'd1;
                    end else begin
                        spi_clk_count <= 8'd0;
                        flash_sck_out <= 1'b1;
                        spi_rx_shift[spi_bit_index] <= flash_miso_in;
                        state <= ST_SPI_CLK_LO;
                    end
                end

                ST_SPI_CLK_LO: begin
                    if (spi_clk_count < SPI_CLKS_PER_HALF_BIT - 1) begin
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
                                state <= ST_SPI_DEASSERT;
                            end else begin
                                spi_byte_index <= spi_byte_index + 2'd1;
                                spi_bit_index <= 3'd7;
                                spi_tx_shift <= spi_tx_byte(spi_byte_index + 2'd1);
                                state <= ST_SPI_SETUP;
                            end
                        end else begin
                            spi_bit_index <= spi_bit_index - 3'd1;
                            state <= ST_SPI_SETUP;
                        end
                    end
                end

                ST_SPI_DEASSERT: begin
                    flash_cs_out <= 1'b1;
                    jedec_index <= 3'd0;
                    state <= ST_JEDEC_SEND;
                end

                ST_JEDEC_SEND: begin
                    if (!tx_busy) begin
                        tx_data <= jedec_ascii_char(jedec_index, spi_rx1, spi_rx2, spi_rx3);
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
