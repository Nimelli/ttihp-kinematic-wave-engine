/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  spis_synchro.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - Physical layer
======================================================================
*/

`default_nettype none

module spis_phy (
    input  wire     clk,
    input  wire     rst_n,

    // input from synchronyzer
    input wire      spi_mosi_sync,
    input wire      spi_cs_sync,
    
    // Synchronous edge
    input wire      spi_clk_rising,     // high for 1 cc on CLK low to high
    input wire      spi_clk_falling,    // high for 1 cc on CLK high to low
    input wire      spi_cs_falling,     // high for 1 cc on CS high to low

    // Output
    output reg      [7:0] rx_data,
    output reg     byte_valid         // high for 1 cc as soon as rx_data is filled
);

    reg [2:0] bit_cnt; // 0 to 7 (3 bits needed)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt    <= 3'd0;
            rx_data    <= 8'h00;
            byte_valid <= 1'b0;
        end else begin
            // Default: clear every cycle
            byte_valid <= 1'b0;

            if (spi_cs_falling || spi_cs_sync) begin
                // Reset frame when CS is high or falling
                bit_cnt <= 3'd0;
            end else if (spi_clk_rising) begin
                // Shift in incoming bit
                rx_data <= {rx_data[6:0], spi_mosi_sync};

                if (bit_cnt == 3'd7) begin
                    bit_cnt    <= 3'd0;   // Reset counter for next byte
                    byte_valid <= 1'b1;   // Pulse HIGH on next cycle
                end else begin
                    bit_cnt    <= bit_cnt + 3'd1;
                end
            end
        end
    end


endmodule