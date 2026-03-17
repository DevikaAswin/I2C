//=============================================================================
// image_pattern_gen.v
//
// ARINC 818 Image Pattern Generator
// Generates an 8 vertical color-bar test image at HACTIVE x VACTIVE
// resolution with 3 bytes/pixel (RGB24). Packs pixels into 32-bit words
// and writes them into the async FIFO write port.
//
// Color Bars (left to right):
//   Bar 0 = White   (FF,FF,FF)    Bar 4 = Magenta (FF,00,FF)
//   Bar 1 = Yellow  (FF,FF,00)    Bar 5 = Red     (FF,00,00)
//   Bar 2 = Cyan    (00,FF,FF)    Bar 6 = Blue    (00,00,FF)
//   Bar 3 = Green   (00,FF,00)    Bar 7 = Black   (00,00,00)
//
// Pixel Packing (RGB24 -> 32-bit words):
//   Every 4 pixels (12 bytes) pack into 3 words:
//     Word 0: {R0, G0, B0, R1}
//     Word 1: {G1, B1, R2, G2}
//     Word 2: {B2, R3, G3, B3}
//
//   1024 pixels/line -> 768 words/line (256 groups of 3 words)
//
// FIXES applied:
//   1. Removed unused registers (pixel_x, cur_r/g/b, active)
//   2. bar_index declared as [3:0] to match x_pos[10:7] width
//   3. word_count tracking moved to use dedicated group_count for clarity
//   4. Added proper `timescale
//=============================================================================

`timescale 1ns / 1ps

module image_pattern_gen #(
  parameter HACTIVE       = 1024,
  parameter VACTIVE       = 768,
  parameter BytesPerPixel = 3
)(
  input             clk,         // write-side clock
  input             rst_n,       // active-low reset (active-low to match asyncfifo)
  input             wfull,       // FIFO full flag (backpressure)
  output reg [31:0] wdata,       // 32-bit data output to FIFO
  output reg        winc,        // write enable to FIFO
  output reg        frame_sync   // pulse high for 1 cycle at start of each frame
);

  //=========================================================================
  // Local Parameters
  //=========================================================================
  localparam BYTES_PER_LINE  = HACTIVE * BytesPerPixel;     // 3072
  localparam WORDS_PER_LINE  = BYTES_PER_LINE / 4;          // 768
  localparam PIXELS_PER_GROUP = 4;                          // 4 pixels per packing group
  localparam WORDS_PER_GROUP  = 3;                          // 3 words per packing group
  localparam GROUPS_PER_LINE  = HACTIVE / PIXELS_PER_GROUP; // 256 groups per line
  localparam BAR_WIDTH        = HACTIVE / 8;                // 128 pixels per bar

  //=========================================================================
  // State Machine States
  //=========================================================================
  localparam S_IDLE      = 3'd0;
  localparam S_LOAD      = 3'd1;  // latch pixel colors for a group of 4
  localparam S_PACK0     = 3'd2;  // output word 0: {R0, G0, B0, R1}
  localparam S_PACK1     = 3'd3;  // output word 1: {G1, B1, R2, G2}
  localparam S_PACK2     = 3'd4;  // output word 2: {B2, R3, G3, B3}
  localparam S_LINE_DONE = 3'd5;  // line complete, advance

  //=========================================================================
  // Internal Registers
  //=========================================================================
  reg [2:0]  gen_state;
  reg [10:0] group_base;        // base pixel X for current group (0, 4, 8, ...)
  reg [9:0]  line_y;            // current video line (0 to VACTIVE-1)
  reg [8:0]  group_count;       // number of groups completed this line (0 to 255)

  // Latched pixel colors for 4 pixels in current group
  reg [7:0] p0_r, p0_g, p0_b;
  reg [7:0] p1_r, p1_g, p1_b;
  reg [7:0] p2_r, p2_g, p2_b;
  reg [7:0] p3_r, p3_g, p3_b;

  //=========================================================================
  // Color Bar Lookup Function
  // Given a pixel's X position, returns the 24-bit RGB color
  //=========================================================================
  function [23:0] get_bar_color;
    input [10:0] x_pos;
    reg [3:0] bar_index;   // FIX: 4 bits to match x_pos[10:7] width
    begin
      bar_index = x_pos[10:7]; // x_pos / 128 (BAR_WIDTH = 128 = 2^7)
      // For HACTIVE=1024, x goes 0..1023, so bar_index goes 0..7
      // (bit 10 is always 0, so effectively 3 bits, but declared 4 for safety)
      case (bar_index[2:0])    // use lower 3 bits since bar_index <= 7
        3'd0: get_bar_color = 24'hFFFFFF; // White
        3'd1: get_bar_color = 24'hFFFF00; // Yellow
        3'd2: get_bar_color = 24'h00FFFF; // Cyan
        3'd3: get_bar_color = 24'h00FF00; // Green
        3'd4: get_bar_color = 24'hFF00FF; // Magenta
        3'd5: get_bar_color = 24'hFF0000; // Red
        3'd6: get_bar_color = 24'h0000FF; // Blue
        3'd7: get_bar_color = 24'h000000; // Black
        default: get_bar_color = 24'h000000;
      endcase
    end
  endfunction

  //=========================================================================
  // Combinational color lookups for the current group of 4 pixels
  //=========================================================================
  wire [23:0] color_p0 = get_bar_color(group_base);
  wire [23:0] color_p1 = get_bar_color(group_base + 11'd1);
  wire [23:0] color_p2 = get_bar_color(group_base + 11'd2);
  wire [23:0] color_p3 = get_bar_color(group_base + 11'd3);

  //=========================================================================
  // Main State Machine (synchronous reset to match advbtx style)
  //=========================================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gen_state   <= S_IDLE;
      group_base  <= 11'd0;
      line_y      <= 10'd0;
      group_count <= 9'd0;
      wdata       <= 32'd0;
      winc        <= 1'b0;
      frame_sync  <= 1'b0;
      p0_r <= 8'd0; p0_g <= 8'd0; p0_b <= 8'd0;
      p1_r <= 8'd0; p1_g <= 8'd0; p1_b <= 8'd0;
      p2_r <= 8'd0; p2_g <= 8'd0; p2_b <= 8'd0;
      p3_r <= 8'd0; p3_g <= 8'd0; p3_b <= 8'd0;
    end
    else begin
      // Defaults: de-assert single-cycle pulses
      winc       <= 1'b0;
      frame_sync <= 1'b0;

      case (gen_state)
        //-------------------------------------------------------------------
        // S_IDLE: Reset per-line state, emit frame_sync for line 0
        //-------------------------------------------------------------------
        S_IDLE: begin
          group_base  <= 11'd0;
          group_count <= 9'd0;

          if (line_y == 10'd0)
            frame_sync <= 1'b1;   // pulse at start of new video frame

          gen_state <= S_LOAD;
        end

        //-------------------------------------------------------------------
        // S_LOAD: Latch RGB values for 4 consecutive pixels
        //-------------------------------------------------------------------
        S_LOAD: begin
          p0_r <= color_p0[23:16]; p0_g <= color_p0[15:8]; p0_b <= color_p0[7:0];
          p1_r <= color_p1[23:16]; p1_g <= color_p1[15:8]; p1_b <= color_p1[7:0];
          p2_r <= color_p2[23:16]; p2_g <= color_p2[15:8]; p2_b <= color_p2[7:0];
          p3_r <= color_p3[23:16]; p3_g <= color_p3[15:8]; p3_b <= color_p3[7:0];
          gen_state <= S_PACK0;
        end

        //-------------------------------------------------------------------
        // S_PACK0: Output Word 0 = {R0, G0, B0, R1}
        //-------------------------------------------------------------------
        S_PACK0: begin
          if (wfull) begin
            winc <= 1'b0;          // stall — FIFO full
          end
          else begin
            wdata <= {p0_r, p0_g, p0_b, p1_r};
            winc  <= 1'b1;
            gen_state <= S_PACK1;
          end
        end

        //-------------------------------------------------------------------
        // S_PACK1: Output Word 1 = {G1, B1, R2, G2}
        //-------------------------------------------------------------------
        S_PACK1: begin
          if (wfull) begin
            winc <= 1'b0;          // stall — FIFO full
          end
          else begin
            wdata <= {p1_g, p1_b, p2_r, p2_g};
            winc  <= 1'b1;
            gen_state <= S_PACK2;
          end
        end

        //-------------------------------------------------------------------
        // S_PACK2: Output Word 2 = {B2, R3, G3, B3}
        //         Then advance to next group or finish line
        //-------------------------------------------------------------------
        S_PACK2: begin
          if (wfull) begin
            winc <= 1'b0;          // stall — FIFO full
          end
          else begin
            wdata       <= {p2_b, p3_r, p3_g, p3_b};
            winc        <= 1'b1;
            group_count <= group_count + 9'd1;

            if ((group_count + 9'd1) >= GROUPS_PER_LINE) begin
              // All 256 groups done → 768 words output → line complete
              gen_state <= S_LINE_DONE;
            end
            else begin
              // More groups remain — advance to next 4 pixels
              group_base <= group_base + 11'd4;
              gen_state  <= S_LOAD;
            end
          end
        end

        //-------------------------------------------------------------------
        // S_LINE_DONE: Advance to next line or wrap to next frame
        //-------------------------------------------------------------------
        S_LINE_DONE: begin
          group_base  <= 11'd0;
          group_count <= 9'd0;

          if (line_y == (VACTIVE - 1))
            line_y <= 10'd0;       // end of frame → wrap to line 0
          else
            line_y <= line_y + 10'd1;

          gen_state <= S_IDLE;
        end

        default: begin
          gen_state <= S_IDLE;
        end
      endcase
    end
  end

endmodule
