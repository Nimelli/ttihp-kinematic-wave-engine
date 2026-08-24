/* 
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_amp_scale.v
Author:     Jeremie W (willab.ch)
Brief:      Scale the input sine & clamp it
======================================================================

**AMP** — `ui[6:5]`, implemented as shifts and one subtract. No multiplier.

| Sel | Scale | Expression | Servo swing |
|---|---|---|---|
| 00 | 25% | `s >>> 2` | ±15° |
| 01 | 50% | `s >>> 1` | ±30° |
| 10 | 75% | `s - (s >>> 2)` | ±45° |
| 11 | 100% | `s` | ±60° |

Reset default should be **01** (50%) — a conservative amplitude that will not slam the
linkages on first power-up with an untested mechanism.
*/

`default_nettype none

module kwe_amp_scale (
    input  wire signed [8:0] sine,      // -127 .. +127
    input  wire        [1:0] amp,       // 00=25%, 01=50%, 10=75%, 11=100%
    output wire        [7:0] pos        // 128 + scaled, clamped to 1..255
);

    reg signed [8:0] scaled_sine;
    wire signed [9:0] raw_pos;

    // Amplitude Scaling via bitshifting
    always @(*) begin
        case (amp)
            2'b00:   scaled_sine = sine >>> 2;                  // 25%
            2'b01:   scaled_sine = sine >>> 1;                  // 50%
            2'b10:   scaled_sine = sine - (sine >>> 2);         // 75%  (1 - 1/4)
            2'b11:   scaled_sine = sine;                        // 100%
            default: scaled_sine = sine >>> 1;                  // 50%
        endcase
    end

    // Add DC Offset (+128)
    assign raw_pos = 10'sd128 + scaled_sine;

    // Clamp output strictly to 1..255 range
    assign pos = (raw_pos < 10'sd1)   ? 8'd1   :
                (raw_pos > 10'sd255) ? 8'd255 : raw_pos[7:0];

endmodule