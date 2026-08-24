/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_speed_rom.v
Author:     Jeremie W (willab.ch)
Brief:      speed table
======================================================================

**SPEED** — `ui[4:1]`, 16-entry ROM of 13-bit phase increments. Wave period
`T = 65536 / (inc x 50.08)`. Trough velocity assumes the 161 mm array span from §4.

| Sel | inc | Period | Trough velocity |
|---|---|---|---|
| 0 | 65 | 20.1 s | 8 mm/s | 
| 1 | 87 | 15.0 s | 11 mm/s |
| 2 | 119 | 11.0 s | 15 mm/s 
| 3 | 156 | 8.4 s | 19 mm/s |
| 4 | 208 | 6.3 s | 26 mm/s |
| 5 | 278 | 4.7 s | 34 mm/s |
| 6 | 374 | 3.5 s | 46 mm/s |
| 7 | 503 | 2.6 s | 62 mm/s |
| 8 | 654 | 2.00 s | 81 mm/s |
| 9 | 872 | 1.50 s | 107 mm/s |
| 10 | 1190 | 1.10 s | 146 mm/s |
| 11 | 1577 | 0.83 s | 194 mm/s |
| 12 | 2111 | 0.62 s | 260 mm/s |
| 13 | 2784 | 0.47 s | 343 mm/s |
| 14 | 3739 | 0.35 s | 460 mm/s |
| 15 | 5033 | 0.26 s | 619 mm/s |

Spacing is ~1.35x per step, covering 0.26 s to 20 s. Settings **12–15 exceed the SG90's
slew rate at 100% amplitude** (peak demand `377/T` °/s vs. ~600 °/s capability, so the
limit is around T = 0.62 s) — the waveform will be lag-clipped by the servo. They remain
usable at reduced amplitude, and they are cheap to keep. Reset default should be **8**
(2.0 s), the middle of the plausible band.

*/

`default_nettype none

module kwe_speed_rom (
    input  wire  [3:0] sel,
    output reg  [12:0] inc
);

    always @(*) begin
        case (sel)
            4'd0: inc = 13'd65; 
            4'd1: inc = 13'd87; 
            4'd2: inc = 13'd119; 
            4'd3: inc = 13'd156; 
            4'd4: inc = 13'd208; 
            4'd5: inc = 13'd278; 
            4'd6: inc = 13'd374; 
            4'd7: inc = 13'd503;
            4'd8:  inc = 13'd654;
            4'd9:  inc = 13'd872;
            4'd10: inc = 13'd1190;
            4'd11: inc = 13'd1577;
            4'd12: inc = 13'd2111;
            4'd13: inc = 13'd2784;
            4'd14: inc = 13'd3739;
            4'd15: inc = 13'd5033;
            default: inc = 13'd654; 
        endcase
    end

endmodule

