`timescale 1ns/1ps

module app #(
    parameter integer CLKS_PER_BIT = 208,
    parameter integer SPI_CLKS_PER_HALF_BIT = 12
) (
    input  wire clk,
    input  wire rst,
    input  wire uart_rx_i,
    output wire uart_tx_o,
    output wire flash_sck_out,
    output wire flash_cs_out,
    output wire flash_mosi_out,
    input  wire flash_miso_in,
    output wire done
);
    localparam [7:0] S_NOP          = 8'h00;
    localparam [7:0] S_Q_IFACE      = 8'h01;
    localparam [7:0] S_Q_CMDMAP     = 8'h02;
    localparam [7:0] S_Q_PGMNAME    = 8'h03;
    localparam [7:0] S_Q_SERBUF     = 8'h04;
    localparam [7:0] S_Q_BUSTYPE    = 8'h05;
    localparam [7:0] S_Q_WRNMAXLEN  = 8'h08;
    localparam [7:0] S_O_INIT       = 8'h0B;
    localparam [7:0] S_O_DELAY      = 8'h0E;
    localparam [7:0] S_O_EXEC       = 8'h0F;
    localparam [7:0] S_SYNCNOP      = 8'h10;
    localparam [7:0] S_Q_RDNMAXLEN  = 8'h11;
    localparam [7:0] S_S_BUSTYPE    = 8'h12;
    localparam [7:0] S_CMD_S_SPI_OP = 8'h13;
    localparam [7:0] S_S_SPI_FREQ   = 8'h14;
    localparam [7:0] S_S_PIN_STATE  = 8'h15;

    localparam [7:0] S_ACK = 8'h06;
    localparam [7:0] S_NAK = 8'h15;
    localparam [7:0] BUS_SPI = 8'h08;

    localparam [7:0] SERPROG_MAX_WRITE = 8'd256;
    localparam [7:0] SERPROG_MAX_READ  = 8'd256;

    localparam [2:0] ST_IDLE        = 3'd0;
    localparam [2:0] ST_WAIT_EXTRA  = 3'd1;
    localparam [2:0] ST_TX_RESP     = 3'd2;
    localparam [2:0] ST_SPI_RX_DATA = 3'd3;

    reg [2:0] state;
    wire [7:0] rx_data;
    wire       rx_valid;
    reg [7:0] current_cmd;
    reg [7:0] extra_count;
    reg [7:0] extra_index;
    reg [7:0] cmd_buffer [0:15];
    reg [7:0] tx_buffer [0:63];
    reg [7:0] response_len;
    reg [7:0] response_index;
    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;
    reg [7:0] delay_count;
    reg [7:0] spi_write_count;
    reg [7:0] spi_read_count;
    reg [23:0] spi_write_len;
    reg [23:0] spi_read_len;
    reg [7:0] bus_type_reg;
    reg [7:0] last_status;
    reg       activity;

    function [7:0] command_map_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  command_map_byte = 8'h3F;
                5'd1:  command_map_byte = 8'hC9;
                5'd2:  command_map_byte = 8'h3F;
                default: command_map_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] ascii_char;
        input [7:0] value;
        begin
            if (value < 8'd10)
                ascii_char = 8'h30 + value;
            else
                ascii_char = 8'h41 + (value - 8'd10);
        end
    endfunction

    task automatic send_response;
        input [7:0] len;
        begin
            response_len  <= len;
            response_index <= 8'd0;
            state <= ST_TX_RESP;
        end
    endtask

    task automatic build_ack_only;
        begin
            tx_buffer[0] <= S_ACK;
            send_response(8'd1);
        end
    endtask

    task automatic build_command_map;
        integer i;
        begin
            tx_buffer[0] <= S_ACK;
            for (i = 0; i < 32; i = i + 1) begin
                tx_buffer[i + 1] <= command_map_byte(i[4:0]);
            end
            send_response(8'd33);
        end
    endtask

    task automatic build_pgm_name;
        integer i;
        begin
            tx_buffer[0] <= S_ACK;
            tx_buffer[1] <= "s";
            tx_buffer[2] <= "e";
            tx_buffer[3] <= "r";
            tx_buffer[4] <= "p";
            tx_buffer[5] <= "r";
            tx_buffer[6] <= "o";
            tx_buffer[7] <= "g";
            tx_buffer[8] <= 8'h00;
            send_response(8'd9);
        end
    endtask

    task automatic build_numeric_response;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        input [7:0] len;
        begin
            tx_buffer[0] <= S_ACK;
            tx_buffer[1] <= a;
            tx_buffer[2] <= b;
            tx_buffer[3] <= c;
            send_response(len);
        end
    endtask

    uart_rx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_rx (
        .clk(clk),
        .rst(rst),
        .rx(uart_rx_i),
        .rx_data(rx_data),
        .rx_valid(rx_valid)
    );

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

    assign flash_cs_out = 1'b1;
    assign flash_sck_out = 1'b0;
    assign flash_mosi_out = 1'b0;
    assign done = activity;

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            current_cmd <= 8'd0;
            extra_count <= 8'd0;
            extra_index <= 8'd0;
            tx_data <= 8'd0;
            tx_start <= 1'b0;
            response_len <= 8'd0;
            response_index <= 8'd0;
            delay_count <= 8'd0;
            spi_write_count <= 8'd0;
            spi_read_count <= 8'd0;
            spi_write_len <= 24'd0;
            spi_read_len <= 24'd0;
            bus_type_reg <= 8'd0;
            last_status <= 8'd0;
            activity <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            if (rx_valid) begin
                activity <= 1'b1;
                case (state)
                    ST_IDLE: begin
                        current_cmd <= rx_data;

                        case (rx_data)
                            S_NOP: begin
                                tx_buffer[0] <= S_ACK;
                                send_response(8'd1);
                            end
                            S_Q_IFACE: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= 8'd1;
                                tx_buffer[2] <= 8'd0;
                                send_response(8'd3);
                            end
                            S_Q_CMDMAP: begin
                                build_command_map();
                            end
                            S_Q_PGMNAME: begin
                                build_pgm_name();
                            end
                            S_Q_SERBUF: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= 8'd0;
                                tx_buffer[2] <= 8'd0;
                                send_response(8'd3);
                            end
                            S_Q_BUSTYPE: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= BUS_SPI;
                                send_response(8'd2);
                            end
                            S_Q_WRNMAXLEN: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= 8'd0;
                                tx_buffer[2] <= 8'd1;
                                tx_buffer[3] <= 8'd0;
                                send_response(8'd4);
                            end
                            S_O_INIT: begin
                                build_ack_only();
                            end
                            S_O_DELAY: begin
                                extra_count <= 8'd4;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            S_O_EXEC: begin
                                tx_buffer[0] <= S_ACK;
                                send_response(8'd1);
                            end
                            S_SYNCNOP: begin
                                tx_buffer[0] <= S_NAK;
                                tx_buffer[1] <= S_ACK;
                                send_response(8'd2);
                            end
                            S_Q_RDNMAXLEN: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= 8'd0;
                                tx_buffer[2] <= 8'd1;
                                tx_buffer[3] <= 8'd0;
                                send_response(8'd4);
                            end
                            S_S_BUSTYPE: begin
                                extra_count <= 8'd1;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            S_CMD_S_SPI_OP: begin
                                extra_count <= 8'd6;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            S_S_SPI_FREQ: begin
                                extra_count <= 8'd4;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            S_S_PIN_STATE: begin
                                extra_count <= 8'd1;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            default: begin
                                tx_buffer[0] <= S_NAK;
                                send_response(8'd1);
                            end
                        endcase
                    end

                    ST_WAIT_EXTRA: begin
                        cmd_buffer[extra_index] <= rx_data;
                        if (extra_index == extra_count - 8'd1) begin
                            case (current_cmd)
                                S_O_DELAY: begin
                                    delay_count <= cmd_buffer[0] + cmd_buffer[1] + cmd_buffer[2] + cmd_buffer[3];
                                    tx_buffer[0] <= S_ACK;
                                    send_response(8'd1);
                                end
                                S_S_BUSTYPE: begin
                                    bus_type_reg <= rx_data;
                                    tx_buffer[0] <= S_ACK;
                                    send_response(8'd1);
                                end
                                S_CMD_S_SPI_OP: begin
                                    spi_write_len <= {cmd_buffer[0], cmd_buffer[1], cmd_buffer[2]};
                                    spi_read_len  <= {cmd_buffer[3], cmd_buffer[4], cmd_buffer[5]};
                                    state <= ST_SPI_RX_DATA;
                                    spi_write_count <= 8'd0;
                                    spi_read_count <= 8'd0;
                                end
                                S_S_SPI_FREQ: begin
                                    tx_buffer[0] <= S_ACK;
                                    tx_buffer[1] <= cmd_buffer[0];
                                    tx_buffer[2] <= cmd_buffer[1];
                                    tx_buffer[3] <= cmd_buffer[2];
                                    tx_buffer[4] <= cmd_buffer[3];
                                    send_response(8'd5);
                                end
                                S_S_PIN_STATE: begin
                                    tx_buffer[0] <= S_ACK;
                                    send_response(8'd1);
                                end
                                default: begin
                                    tx_buffer[0] <= S_NAK;
                                    send_response(8'd1);
                                end
                            endcase
                        end else begin
                            extra_index <= extra_index + 8'd1;
                        end
                    end

                    ST_SPI_RX_DATA: begin
                        if (spi_write_len > 24'd0) begin
                            spi_write_count <= spi_write_count + 8'd1;
                            if (spi_write_count >= spi_write_len[7:0]) begin
                                spi_write_len <= 24'd0;
                            end
                        end

                        if (spi_read_len > 24'd0) begin
                            spi_read_count <= spi_read_count + 8'd1;
                        end

                        if ((spi_write_len == 24'd0) && (spi_read_len == 24'd0)) begin
                            tx_buffer[0] <= S_ACK;
                            send_response(8'd1);
                        end else begin
                            if (rx_valid) begin
                                if (spi_write_len > 24'd0) begin
                                    spi_write_len <= spi_write_len - 24'd1;
                                end
                                if (spi_read_len > 24'd0) begin
                                    spi_read_len <= spi_read_len - 24'd1;
                                end
                            end
                        end
                    end

                    ST_TX_RESP: begin
                        if (!tx_busy) begin
                            tx_data <= tx_buffer[response_index];
                            tx_start <= 1'b1;
                            if (response_index == response_len - 8'd1) begin
                                state <= ST_IDLE;
                            end else begin
                                response_index <= response_index + 8'd1;
                            end
                        end
                    end

                    default: begin
                        state <= ST_IDLE;
                    end
                endcase
            end else begin
                if (state == ST_TX_RESP && !tx_busy) begin
                    tx_data <= tx_buffer[response_index];
                    tx_start <= 1'b1;
                    if (response_index == response_len - 8'd1) begin
                        state <= ST_IDLE;
                    end else begin
                        response_index <= response_index + 8'd1;
                    end
                end
            end
        end
    end
endmodule
