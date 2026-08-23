/* clk 10.000 MHz
  └─ ÷39 prescaler          → tick   = 256.41 kHz  (3.90 us)
       └─ tick counter 0..639 → slot   = 2.496 ms
            └─ slot counter 0..7 → frame  = 19.968 ms  (50.08 Hz)
 */

module kwe_timebase(
    input  wire         clk,
    input  wire         rst_n,
    output wire         tick_en,      // 1-cycle pulse @ 256.41 kHz (every 39 clk)
    output reg [9:0]    tick_cnt,     // 0..639, position within the current slot, reg because driven inside 'always'
    output reg [2:0]    slot,         // 0..7, channel index owning this slot, reg because driven inside 'always'
    output wire [2:0]   slot_nxt,     // slot + 1; drives the wave datapath (see Trap 1)
    output wire         slot_start,   // tick_en && tick_cnt == 639  (slot changes this edge)
    output wire         phase_tick    // tick_en && slot == 7 && tick_cnt == 600  (see Trap 2)


);

    // Prescaler Counter (0 to 38 = 39 cycles total)
    reg [5:0] prescale_cnt;

    // Prescaler logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prescale_cnt <= 6'd0;
        end else begin
            if (prescale_cnt == 6'd38) begin
                prescale_cnt <= 6'd0;
            end else begin
                prescale_cnt <= prescale_cnt + 1'b1;
            end
        end
    end

    // High for exactly 1 clock cycle every 39 clock cycles
    assign tick_en = (prescale_cnt == 6'd38);


    // Slot position counter (0..639)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 10'd0;
        end else if (tick_en) begin
            if (tick_cnt == 10'd639) begin
                tick_cnt <= 10'd0;
            end else begin
                tick_cnt <= tick_cnt + 10'd1;
            end
        end
    end

    // Slot index counter (0..7)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot <= 3'd0;
        end else if (tick_en && tick_cnt == 10'd639) begin
            slot <= slot + 3'd1; // Wraps automatically from 7 to 0
        end
    end

    // Combinational Flag Decoding
    assign slot_nxt   = slot + 3'd1;
    assign slot_start = tick_en && (tick_cnt == 10'd639);
    assign phase_tick = tick_en && (slot == 3'd7) && (tick_cnt == 10'd600);



endmodule