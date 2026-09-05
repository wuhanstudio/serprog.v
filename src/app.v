`timescale 1ns/1ps

module app #(
    parameter integer CLKS_PER_BIT = 208,
    parameter integer SPI_CLKS_PER_HALF_BIT = 12,
    parameter integer SPI_TIMEOUT_CYCLES = 240000
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
    // Bank 0
    localparam [7:0] S_NOP          = 8'h00;
    localparam [7:0] S_Q_IFACE      = 8'h01;
    localparam [7:0] S_Q_CMDMAP     = 8'h02;
    localparam [7:0] S_Q_PGMNAME    = 8'h03;
    localparam [7:0] S_Q_SERBUF     = 8'h04;
    localparam [7:0] S_Q_BUSTYPE    = 8'h05;

    // localparam [7:0] S_Q_CHIPSIZE  = 8'h06;
    // localparam [7:0] S_Q_OPBUF     = 8'h07

    // Bank 1
    // localparam [7:0] S_Q_WRNMAXLEN   = 8'h08;
    // localparam [7:0] S_R_BYTE        = 8'h09;
    // localparam [7:0] S_R_NBYTES      = 8'h0A;
    // localparam [7:0] S_O_INIT        = 8'h0B;
    // localparam [7:0] S_O_WRITEB      = 8'h0C;
    // localparam [7:0] S_O_WRITEN      = 8'h0D;
    // localparam [7:0] S_O_DELAY       = 8'h0E;
    // localparam [7:0] S_O_EXEC        = 8'h0F;

    // Bank 2
    localparam [7:0] S_SYNCNOP      = 8'h10;
    // localparam [7:0] S_Q_RDNMAXLEN  = 8'h11;
    localparam [7:0] S_S_BUSTYPE    = 8'h12;
    localparam [7:0] S_S_SPI_OP = 8'h13;
    localparam [7:0] S_S_SPI_FREQ   = 8'h14;
    localparam [7:0] S_S_PIN_STATE  = 8'h15;

    localparam [7:0] S_ACK = 8'h06;
    localparam [7:0] S_NAK = 8'h15;
    localparam [7:0] BUS_SPI = 8'h08;

    localparam integer TX_BUFFER_SIZE = 257;

    localparam [4:0] ST_IDLE           = 5'd0;
    localparam [4:0] ST_WAIT_EXTRA     = 5'd1;
    localparam [4:0] ST_WAIT_SPI_WRITE = 5'd2;
    localparam [4:0] ST_SPI_START      = 5'd3;
    localparam [4:0] ST_SPI_WAIT       = 5'd4;
    localparam [4:0] ST_TX_RESP        = 5'd5;
    localparam [4:0] ST_TX_SPI_ACK     = 5'd6;
    localparam [4:0] ST_TX_SPI_READ    = 5'd7;

    reg [4:0] state;
    wire [7:0] rx_data;
    wire       rx_valid;
    reg [7:0] current_cmd;
    reg [7:0] extra_count;
    reg [7:0] extra_index;
    reg [7:0] cmd_buffer [0:15];
    reg [7:0] tx_buffer [0:TX_BUFFER_SIZE - 1];
    reg [8:0] response_len;
    reg [8:0] response_index;
    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;
    reg [23:0] spi_write_remaining;
    reg [23:0] spi_read_remaining;
    reg [7:0] spi_tx_byte;
    reg       spi_start;
    wire      spi_busy;
    wire      spi_done;
    wire [7:0] spi_rx_byte;
    reg       spi_read_phase;
    reg [31:0] spi_timeout_count;
    reg       flash_cs_reg;
    wire      spi_sck;
    wire      spi_mosi;
    reg [7:0] bus_type_reg;
    reg       activity;

    function [7:0] command_map_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  command_map_byte = 8'h3F;
                5'd1:  command_map_byte = 8'h00;
                5'd2:  command_map_byte = 8'h3D;
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
        input [8:0] len;
        begin
            response_len  <= len;
            response_index <= 9'd0;
            state <= ST_TX_RESP;
        end
    endtask

    task automatic build_command_map;
        integer i;
        begin
            tx_buffer[0] <= S_ACK;
            for (i = 0; i < 32; i = i + 1) begin
                tx_buffer[i + 1] <= command_map_byte(i[4:0]);
            end
            send_response(9'd33);
        end
    endtask

    task automatic build_pgm_name;
        begin
            tx_buffer[0] <= S_ACK;
            tx_buffer[1] <= "s";
            tx_buffer[2] <= "e";
            tx_buffer[3] <= "r";
            tx_buffer[4] <= "p";
            tx_buffer[5] <= "r";
            tx_buffer[6] <= "o";
            tx_buffer[7] <= "g";
            tx_buffer[8] <= "-";
            tx_buffer[9] <= "v";
            tx_buffer[10] <= "e";
            tx_buffer[11] <= "r";
            tx_buffer[12] <= "i";
            tx_buffer[13] <= "l";
            tx_buffer[14] <= "o";
            tx_buffer[15] <= "g";
            tx_buffer[16] <= 8'h00;
            send_response(9'd17);
        end
    endtask

    task automatic build_numeric_response;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        input [8:0] len;
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

    spi_byte_engine #(
        .CLKS_PER_HALF_BIT(SPI_CLKS_PER_HALF_BIT)
    ) u_spi_byte_engine (
        .clk(clk),
        .rst(rst),
        .start(spi_start),
        .tx_byte(spi_tx_byte),
        .miso(flash_miso_in),
        .busy(spi_busy),
        .done(spi_done),
        .rx_byte(spi_rx_byte),
        .sck(spi_sck),
        .mosi(spi_mosi)
    );

    assign flash_cs_out = flash_cs_reg;
    assign flash_sck_out = spi_sck;
    assign flash_mosi_out = spi_mosi;
    assign done = activity;

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            current_cmd <= 8'd0;
            extra_count <= 8'd0;
            extra_index <= 8'd0;
            tx_data <= 8'd0;
            tx_start <= 1'b0;
            response_len <= 9'd0;
            response_index <= 9'd0;
            spi_write_remaining <= 24'd0;
            spi_read_remaining <= 24'd0;
            spi_tx_byte <= 8'd0;
            spi_start <= 1'b0;
            spi_read_phase <= 1'b0;
            spi_timeout_count <= 32'd0;
            flash_cs_reg <= 1'b1;
            bus_type_reg <= 8'd0;
            activity <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            spi_start <= 1'b0;

            if (state == ST_TX_RESP) begin
                if (tx_done) begin
                    if (response_index + 9'd1 < response_len) begin
                        response_index <= response_index + 9'd1;
                    end else begin
                        response_index <= 9'd0;
                        state <= ST_IDLE;
                    end
                end else if (!tx_busy && response_index < response_len) begin
                    tx_data <= tx_buffer[response_index];
                    tx_start <= 1'b1;
                end
            end else if (state == ST_TX_SPI_ACK) begin
                if (tx_done) begin
                    spi_tx_byte <= 8'hFF;
                    spi_read_phase <= 1'b1;
                    state <= ST_SPI_START;
                end else if (!tx_busy) begin
                    tx_data <= S_ACK;
                    tx_start <= 1'b1;
                end
            end else if (state == ST_TX_SPI_READ) begin
                if (tx_done) begin
                    if (spi_read_remaining != 24'd0) begin
                        spi_tx_byte <= 8'hFF;
                        state <= ST_SPI_START;
                    end else begin
                        flash_cs_reg <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
            end else if (state == ST_WAIT_SPI_WRITE) begin
                if (rx_valid) begin
                    spi_tx_byte <= rx_data;
                    spi_read_phase <= 1'b0;
                    state <= ST_SPI_START;
                end
            end else if (state == ST_SPI_START) begin
                if (!spi_busy) begin
                    spi_start <= 1'b1;
                    spi_timeout_count <= 32'd0;
                    state <= ST_SPI_WAIT;
                end
            end else if (state == ST_SPI_WAIT) begin
                if (spi_done) begin
                    spi_timeout_count <= 32'd0;

                    if (spi_read_phase) begin
                        tx_data <= spi_rx_byte;
                        tx_start <= 1'b1;

                        if (spi_read_remaining > 24'd1) begin
                            spi_read_remaining <= spi_read_remaining - 24'd1;
                        end else begin
                            spi_read_remaining <= 24'd0;
                            spi_read_phase <= 1'b0;
                        end
                        state <= ST_TX_SPI_READ;
                    end else begin
                        if (spi_write_remaining > 24'd1) begin
                            spi_write_remaining <= spi_write_remaining - 24'd1;
                            state <= ST_WAIT_SPI_WRITE;
                        end else begin
                            spi_write_remaining <= 24'd0;

                            if (spi_read_remaining != 24'd0) begin
                                state <= ST_TX_SPI_ACK;
                            end else begin
                                flash_cs_reg <= 1'b1;
                                tx_buffer[0] <= S_ACK;
                                send_response(9'd1);
                            end
                        end
                    end
                end else if (spi_timeout_count < SPI_TIMEOUT_CYCLES - 1) begin
                    spi_timeout_count <= spi_timeout_count + 32'd1;
                end else begin
                    flash_cs_reg <= 1'b1;
                    spi_write_remaining <= 24'd0;
                    spi_read_remaining <= 24'd0;
                    spi_read_phase <= 1'b0;

                    tx_buffer[0] <= S_NAK;
                    send_response(9'd1);
                end
            end else if (rx_valid) begin
                activity <= 1'b1;
                case (state)
                    ST_IDLE: begin
                        current_cmd <= rx_data;

                        case (rx_data)
                            // Bank 0
                            S_NOP: begin
                                tx_buffer[0] <= S_ACK;
                                send_response(9'd1);
                            end
                            S_Q_IFACE: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= 8'd1;
                                tx_buffer[2] <= 8'd0;
                                send_response(9'd3);
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
                                tx_buffer[2] <= 8'd1;
                                send_response(9'd3);
                            end
                            S_Q_BUSTYPE: begin
                                tx_buffer[0] <= S_ACK;
                                tx_buffer[1] <= BUS_SPI;
                                send_response(9'd2);
                            end
                            // Bank 2
                            S_SYNCNOP: begin
                                tx_buffer[0] <= S_NAK;
                                tx_buffer[1] <= S_ACK;
                                send_response(9'd2);
                            end
                            S_S_BUSTYPE: begin
                                extra_count <= 8'd1;
                                extra_index <= 8'd0;
                                state <= ST_WAIT_EXTRA;
                            end
                            S_S_SPI_OP: begin
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
                                send_response(9'd1);
                            end
                        endcase
                    end

                    ST_WAIT_EXTRA: begin
                        cmd_buffer[extra_index] <= rx_data;
                        if (extra_index == extra_count - 8'd1) begin
                            case (current_cmd)
                                S_S_BUSTYPE: begin
                                    bus_type_reg <= rx_data;
                                    tx_buffer[0] <= S_ACK;
                                    send_response(9'd1);
                                end
                                S_S_SPI_OP: begin
                                    begin
                                        spi_write_remaining <= {cmd_buffer[2], cmd_buffer[1], cmd_buffer[0]};
                                        spi_read_remaining <= {rx_data, cmd_buffer[4], cmd_buffer[3]};
                                        spi_read_phase <= 1'b0;
                                        flash_cs_reg <= 1'b0;

                                        if ({cmd_buffer[2], cmd_buffer[1], cmd_buffer[0]} != 24'd0) begin
                                            state <= ST_WAIT_SPI_WRITE;
                                        end else if ({rx_data, cmd_buffer[4], cmd_buffer[3]} != 24'd0) begin
                                            state <= ST_TX_SPI_ACK;
                                        end else begin
                                            flash_cs_reg <= 1'b1;
                                            tx_buffer[0] <= S_ACK;
                                            send_response(9'd1);
                                        end
                                    end
                                end
                                S_S_SPI_FREQ: begin
                                    tx_buffer[0] <= S_ACK;
                                    tx_buffer[1] <= cmd_buffer[0];
                                    tx_buffer[2] <= cmd_buffer[1];
                                    tx_buffer[3] <= cmd_buffer[2];
                                    tx_buffer[4] <= rx_data;
                                    send_response(9'd5);
                                end
                                S_S_PIN_STATE: begin
                                    tx_buffer[0] <= S_ACK;
                                    send_response(9'd1);
                                end
                                default: begin
                                    tx_buffer[0] <= S_NAK;
                                    send_response(9'd1);
                                end
                            endcase
                        end else begin
                            extra_index <= extra_index + 8'd1;
                        end
                    end

                    default: begin
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end
endmodule
