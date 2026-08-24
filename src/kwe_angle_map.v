/* 
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_angle_map.v
Author:     Jeremie W
Brief:      Compute the angle for the rod (
            Where along the sine should this rod be looking
======================================================================


angle = phase + (rod_nb * DELTA)
    phase:  TIME, advances every frame, same for all rods
    rod_nb: SPACE, fixed per rod, constant

The space term is what makes it a wave rather than a flap. If every rod got the same angle,
all eight would move up and down together — the array would flap. Offsetting each rod by DELTA
staggers them along the sine, so at any instant you see a wave shape frozen across the array.

mirror should make the last rod be similar as the first, like a mirror in the middle

*/

`default_nettype none

module kwe_angle_map (
    input  wire [6:0] phase,
    input  wire [2:0] slot,
    input  wire       spread,   // 0 => delta 16 (128/8), 1 => delta 8
    input  wire       mirror,   // 1 => two-ball mode
    output wire [6:0] angle
);

    wire [6:0] delta;
    wire [2:0] slot_eff; 

    assign delta = spread ? 7'd8 : 7'd16;


    /* 
    this is logically correct, but there is an optimized way using XOR gate
    always @(*) begin
        if (mirror && slot > 3) begin
            slot_eff = 7 - slot;;
        end else begin
            slot_eff = slot;
        end
    end
    */
    assign slot_eff = slot ^ {3{mirror & slot[2]}};


    assign angle = phase + (slot_eff * delta);

endmodule