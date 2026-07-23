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
    parameter integer MSG_LEN = 13
) (
    input  wire clk,
    input  wire rst,
    output wire uart_tx_o,
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

    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;

    reg [4:0] msg_index;
    reg       all_done;

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
            all_done  <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            if (!all_done) begin
                if (!tx_busy && !tx_start) begin
                    tx_data  <= msg_byte(msg_index);
                    tx_start <= 1'b1;
                end

                if (tx_done) begin
                    if (msg_index == MSG_LEN - 1) begin
                        all_done <= 1'b1;
                    end else begin
                        msg_index <= msg_index + 5'd1;
                    end
                end
            end
        end
    end
endmodule
