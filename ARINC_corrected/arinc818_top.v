//=============================================================================
// arinc818_top.v
//
// Top-level module integrating the complete ARINC 818 video transmission chain:
//
//   image_pattern_gen  --[wdata/winc]--> asyncfifo --[rdata/rinc]--> advbtx
//        (generates                    (crosses              (packetizes into
//         color bars)                   clock                ARINC 818 ADVB
//                                       domains)              frames)
//
// Write clock (wclk) drives the image generator.
// Read clock  (rclk) drives the ARINC 818 transmitter.
//=============================================================================

`timescale 1ns / 1ps

module arinc818_top #(
  parameter HACTIVE           = 1024,
  parameter VACTIVE           = 768,
  parameter BytesPerPixel     = 3,
  parameter FRAMERATE         = 60,
  parameter VERTICALBLANKPRE  = 0,
  parameter VERTICALBLANKPOST = 0,
  parameter FIFO_ADDR_SIZE    = 9     // 2^9 = 512 entries deep
)(
  input         wclk,         // write-side clock (image generator domain)
  input         rclk,         // read-side clock  (transmitter domain)
  input         rst_n,        // active-low reset (shared by all modules)

  // Transmitter outputs
  output [31:0] advb_txdata,
  output [3:0]  txcharisk,

  // Debug / status
  output        frame_sync    // pulse at start of each video frame
);

  //=========================================================================
  // Internal Wires
  //=========================================================================

  // Image Generator -> FIFO (write side)
  wire [31:0] fifo_wdata;
  wire        fifo_winc;
  wire        fifo_wfull;

  // FIFO -> Transmitter (read side)
  wire [31:0] fifo_rdata;
  wire        fifo_rempty;
  wire        fifo_rinc;

  //=========================================================================
  // 1. Image Pattern Generator
  //=========================================================================
  image_pattern_gen #(
    .HACTIVE      (HACTIVE),
    .VACTIVE      (VACTIVE),
    .BytesPerPixel(BytesPerPixel)
  ) u_img_gen (
    .clk        (wclk),
    .rst_n      (rst_n),
    .wfull      (fifo_wfull),
    .wdata      (fifo_wdata),
    .winc       (fifo_winc),
    .frame_sync (frame_sync)
  );

  //=========================================================================
  // 2. Asynchronous FIFO (clock domain crossing)
  //=========================================================================
  asyncfifo #(
    .DATASIZE(32),
    .ADDRSIZE(FIFO_ADDR_SIZE)
  ) u_fifo (
    .wclk  (wclk),
    .wrstn (rst_n),
    .winc  (fifo_winc),
    .wdata (fifo_wdata),
    .rclk  (rclk),
    .rrstn (rst_n),
    .rinc  (fifo_rinc),
    .rdata (fifo_rdata),
    .wfull (fifo_wfull),
    .rempty(fifo_rempty)
  );

  //=========================================================================
  // 3. ARINC 818 Transmitter
  //=========================================================================
  advbtx #(
    .HACTIVE          (HACTIVE),
    .VACTIVE          (VACTIVE),
    .BytesPerPixel    (BytesPerPixel),
    .FRAMERATE        (FRAMERATE),
    .VERTICALBLANKPRE (VERTICALBLANKPRE),
    .VERTICALBLANKPOST(VERTICALBLANKPOST)
  ) u_advbtx (
    .clk          (rclk),
    .rst          (rst_n),
    .txfifo_empty (fifo_rempty),
    .advb_txdata  (advb_txdata),
    .txcharisk    (txcharisk),
    .txfifo_rd    (fifo_rinc),
    .txfifo_rdata (fifo_rdata)
  );

endmodule
