/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  spis_synchro.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - Clock Domain Crossing of asynchronous SPI clock
======================================================================

Pass each pin through a two-stage D flip-flop (double-flop) chain followed by a 1-bit delay stage for edge detection

Maximum SPI Clock Constraint: 
Because the project system clock is 10 MHz, the external SPI SCK frequency must not exceed 2.5 MHz 
Oversampling requires SCK to stay high and low for at least 1–2 system clock cycles to guarantee reliable edge detection without missing pulses.

*/

`default_nettype none

module spis_synchro (
    input  wire         clk,
    input  wire         rst_n,


    // Asynchronous signals
    input wire          spi_clk_async,
    input wire          spi_mosi_async,
    input wire          spi_cs_async,

    // Synchronous output
    output wire         spi_clk_sync,
    output wire         spi_mosi_sync,
    output wire         spi_cs_sync,
    
    // Synchronous edge trigger
    output wire         spi_clk_rising,     // high for 1 cc on CLK low to high
    output wire         spi_clk_falling,    // high for 1 cc on CLK high to low
    output wire         spi_cs_falling     // high for 1 cc on CS high to low

);

    // 3-bit shift registers: 
    // [0] = raw async, [1] = stage1, [2] = stage2
    reg [2:0] clk_r;
    reg [2:0] cs_r;
    reg [2:0] mosi_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_r   <= 3'b000; // CLK default low (SPI mode 0, CPOL 0)
            cs_r    <= 3'b111; // CS default high
            mosi_r  <= 3'b000; 
        end else begin
            // shift in the async signal
            clk_r  <= {clk_r[1:0],  spi_clk_async};
            cs_r   <= {cs_r[1:0],   spi_cs_async};
            mosi_r <= {mosi_r[1:0], spi_mosi_async};
        end
    end

    // Stable synchronized level outputs (using stage 2 flip-flop)
    assign spi_clk_sync     = clk_r[1];
    assign spi_cs_sync      = cs_r[1];
    assign spi_mosi_sync    = mosi_r[1];

    // Edge detectors: compare stage 2 with stage 3 (previous cycle)
    assign spi_clk_rising  = (clk_r[2:1] == 2'b01); // was 0, now 1 (rising)
    assign spi_clk_falling = (clk_r[2:1] == 2'b10); // was 1, now 0 (falling)
    assign spi_cs_falling  = (cs_r[2:1] == 2'b10);  // was 1, now 0 (falling)

endmodule