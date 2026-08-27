/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  spis_phy.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - Physical layer (mode 0: CPOL = 0, CPHA = 0)
======================================================================

RX: MOSI is sampled on the rising edge of SCK, MSB first.

TX: MISO changes only on the falling edge of SCK, with one exception -- the MSB
    is presented as soon as CS falls. It has to be: the master samples the first
    bit on the first rising edge, and in CPHA=0 there is no falling edge before
    it.

Master-side timing requirement: spis_synchro delays every edge by 2-3 system
clocks, so the master must leave at least ~4 system clocks (400 ns at 10 MHz)
between CS falling and the first rising SCK edge, or the MSB will not be on the
pad in time. This is the usual SPI CS-setup time, it is just larger here than on
a natively-clocked slave.
*/

`default_nettype none

module spis_phy (
    input  wire     clk,
    input  wire     rst_n,

    // input from synchronyzer
    input wire      spi_mosi_sync,
    input wire      spi_cs_sync,
    
    // Synchronous edge
    input wire      spi_clk_rising,     // high for 1 cc on CLK low to high, CPHA0 sample MOSI on rising edge
    input wire      spi_clk_falling,    // high for 1 cc on CLK high to low, CPHA0 output MIS0 on rising edge
    input wire      spi_cs_falling,     // high for 1 cc on CS high to low
    
    // Module output for MOSI (receiving)
    output reg      [7:0] rx_data,
    output reg      rx_byte_valid,        // high for 1 cc as soon as rx_data is filled

    // MISO (transmitting) & port
    input wire      [7:0] tx_data,
    output wire     tx_load,              // 1 cc: tx data was sent, can load new ones
    output wire     spi_miso,             // drive to the pad
    output wire     spi_miso_oe           // 1 = drive, 0 = Hi-Z (uio_oe is a per-bit enable)

);

    reg [2:0] bit_cnt; // 0 to 7 (3 bits needed)

    // ------------------------------------------------------------------
    // RX path
    // ------------------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt    <= 3'd0;
            rx_data    <= 8'h00;
            rx_byte_valid <= 1'b0;
        end else begin
            // Default: clear every cycle
            rx_byte_valid <= 1'b0;

            if (spi_cs_falling || spi_cs_sync) begin
                // Reset frame when CS is high or falling
                bit_cnt <= 3'd0;
            end else if (spi_clk_rising) begin
                // Shift in incoming bit
                rx_data <= {rx_data[6:0], spi_mosi_sync};

                if (bit_cnt == 3'd7) begin
                    bit_cnt    <= 3'd0;   // Reset counter for next byte
                    rx_byte_valid <= 1'b1;   // Pulse HIGH on next cycle
                end else begin
                    bit_cnt    <= bit_cnt + 3'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // TX path
    // Shares bit_cnt with the RX path above: 
    // ------------------------------------------------------------------
    reg [7:0] tx_shift;
    reg       miso_r;
    reg       miso_oe_r;
    reg       tx_load_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift  <= 8'h00;
            miso_r    <= 1'b0;
            miso_oe_r <= 1'b0;
            tx_load_r <= 1'b0;
        end else begin
            // Default: clear every cycle
            tx_load_r <= 1'b0;

            if (spi_cs_falling) begin
                // Frame start: the MSB has to be on the pad before the first
                // rising edge, so it is presented here and not on a falling edge.
                tx_shift  <= tx_data;
                miso_r    <= tx_data[7];
                miso_oe_r <= 1'b1;
                tx_load_r <= 1'b1;
            end else if (spi_cs_sync) begin
                // Deselected: release the bus and park MISO low.
                miso_r    <= 1'b0;
                miso_oe_r <= 1'b0;
            end else if (spi_clk_rising) begin
                if (bit_cnt == 3'd7) begin
                    // Byte boundary. MISO is left alone on purpose: the master is
                    // sampling the last bit on this very edge.
                    tx_shift  <= tx_data;
                    tx_load_r <= 1'b1;
                end
            end else if (spi_clk_falling) begin
                if (bit_cnt == 3'd0) begin
                    // Freshly loaded byte: present the MSB, do not shift yet.
                    miso_r <= tx_shift[7];
                end else begin
                    miso_r   <= tx_shift[6];
                    tx_shift <= {tx_shift[6:0], 1'b0};
                end
            end
        end
    end

    assign spi_miso    = miso_r;
    assign spi_miso_oe = miso_oe_r;
    assign tx_load     = tx_load_r;

endmodule
