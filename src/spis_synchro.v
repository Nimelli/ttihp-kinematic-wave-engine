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

Two separate limits apply, and the tighter one wins.

1. EDGE DETECTION (this module). Oversampling requires SCK to stay high and low
   for at least 1-2 system clock cycles to guarantee reliable edge detection
   without missing pulses. At a 10 MHz system clock that is SCK <= 2.5 MHz.

2. MISO TURNAROUND (spis_phy, through this module). This one is tighter, and it
   is the one that actually caps the bus. A physical SCK falling edge at time T
   reaches miso_r like this:

       T+1  clk_r[0] captures the new level
       T+2  clk_r[1] follows -- spi_clk_falling asserts for this cycle
       T+3  spis_phy has updated miso_r; MISO is valid on the pad

   The master samples MISO at the NEXT RISING edge, half an SCK period after T,
   so it needs half >= 3 system clocks: SCK <= clk/6 = 1.67 MHz.

   Add one more clock because SCK is asynchronous. Whether the first flop sees
   an edge in cycle k or k+1 depends on where it lands relative to the system
   clock, and on real hardware that alignment drifts continuously -- so the
   worst case is one clock later than the nominal path above. half >= 4:

       SCK <= clk/8 = 1.25 MHz at a 10 MHz system clock

Measured against the RTL, a read returns correct data at clk/5 and fails at
clk/3, which brackets the analysis. Simulation drives SCK synchronously to clk
and therefore only ever exercises one phase alignment, so the derived clk/8 is
the number to design to, not the measured clk/5.

RECOMMENDED: 1 MHz (clk/10). It clears the worst case with margin and there is
nothing to gain from going faster -- the whole register map is 2 bytes.

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