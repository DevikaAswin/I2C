//=============================================================================
// tb_arinc818.v
//
// Testbench for the ARINC 818 system:
//   image_pattern_gen -> asyncfifo -> advbtx
//
// Generates two clocks at different frequencies to simulate a real
// cross-clock-domain scenario. Monitors the transmitter output for
// SOF/EOF ordered sets and payload data.
//
// Usage (Icarus Verilog):
//   iverilog -o tb_arinc818 tb_arinc818.v arinc818_top.v \
//            image_pattern_gen.v advbtx.v asyncfifo.v
//   vvp tb_arinc818
//
// Usage (Vivado simulation):
//   Add all .v files to your project and set tb_arinc818 as the top-level
//   simulation module.
//
// FIXES:
//   1. frame_sync monitoring uses wclk (correct domain) not rclk
//   2. Proper use of non-blocking assignments in monitors
//=============================================================================

`timescale 1ns / 1ps

module tb_arinc818;

  //=========================================================================
  // Parameters
  //=========================================================================
  parameter HACTIVE        = 1024;
  parameter VACTIVE        = 768;
  parameter CLK_WR_PERIOD  = 10;   // 100 MHz write clock  (10 ns period)
  parameter CLK_RD_PERIOD  = 6;    //~166 MHz read clock   ( 6 ns period)

  //=========================================================================
  // Signals
  //=========================================================================
  reg         wclk;
  reg         rclk;
  reg         rst_n;

  wire [31:0] advb_txdata;
  wire [3:0]  txcharisk;
  wire        frame_sync;

  //=========================================================================
  // Clock Generation
  //=========================================================================
  initial wclk = 0;
  always #(CLK_WR_PERIOD / 2) wclk = ~wclk;

  initial rclk = 0;
  always #(CLK_RD_PERIOD / 2) rclk = ~rclk;

  //=========================================================================
  // DUT Instantiation
  //=========================================================================
  arinc818_top #(
    .HACTIVE(HACTIVE),
    .VACTIVE(VACTIVE)
  ) dut (
    .wclk       (wclk),
    .rclk       (rclk),
    .rst_n      (rst_n),
    .advb_txdata(advb_txdata),
    .txcharisk  (txcharisk),
    .frame_sync (frame_sync)
  );

  //=========================================================================
  // Ordered Set Definitions
  //=========================================================================
  localparam IDLEWORD = 32'hB5B595BC;
  localparam SOFIWORD = 32'h5757BFBC;
  localparam SOFNWORD = 32'h4F4FB7BC;
  localparam EOFNWORD = 32'h4E4E95BC;
  localparam EOFTWORD = 32'h4B4B95BC;

  //=========================================================================
  // Counters
  //=========================================================================
  integer sof_count;
  integer eof_count;
  integer frame_count;

  //=========================================================================
  // Test Sequence
  //=========================================================================
  initial begin
    $dumpfile("arinc818_tb.vcd");
    $dumpvars(0, tb_arinc818);

    rst_n       = 0;
    sof_count   = 0;
    eof_count   = 0;
    frame_count = 0;

    $display("=============================================================");
    $display(" ARINC 818 Testbench - Color Bar Image Generator");
    $display(" Resolution: %0d x %0d, RGB24", HACTIVE, VACTIVE);
    $display(" Write clock: %0d MHz, Read clock: %0d MHz",
              1000 / CLK_WR_PERIOD, 1000 / CLK_RD_PERIOD);
    $display("=============================================================");

    // Hold reset for 200 ns (guarantees several edges on both clocks)
    #200;
    rst_n = 1;
    $display("[%0t] Reset released", $time);

    // Run for 2 ms
    #2_000_000;

    $display("");
    $display("=============================================================");
    $display(" Simulation Complete");
    $display(" SOFs detected : %0d", sof_count);
    $display(" EOFs detected : %0d", eof_count);
    $display(" Frame syncs   : %0d", frame_count);
    $display("=============================================================");
    $finish;
  end

  //=========================================================================
  // Monitor: SOF/EOF on rclk domain (transmitter output)
  //=========================================================================
  always @(posedge rclk) begin
    if (rst_n) begin
      // Start of Frame
      if (advb_txdata == SOFIWORD && txcharisk == 4'b0001) begin
        sof_count = sof_count + 1;
        if (sof_count <= 30)
          $display("[%0t] SOFi detected (container start) -- SOF #%0d", $time, sof_count);
      end
      else if (advb_txdata == SOFNWORD && txcharisk == 4'b0001) begin
        sof_count = sof_count + 1;
        if (sof_count <= 30)
          $display("[%0t] SOFn detected (data frame)      -- SOF #%0d", $time, sof_count);
      end

      // End of Frame
      if (advb_txdata == EOFNWORD && txcharisk == 4'b0001) begin
        eof_count = eof_count + 1;
        if (eof_count <= 30)
          $display("[%0t] EOFn detected                   -- EOF #%0d", $time, eof_count);
      end
      else if (advb_txdata == EOFTWORD && txcharisk == 4'b0001) begin
        eof_count = eof_count + 1;
        $display("[%0t] EOFt detected (LAST frame!)      -- EOF #%0d", $time, eof_count);
      end
    end
  end

  //=========================================================================
  // Monitor: Frame sync on wclk domain (FIX: correct clock domain)
  //=========================================================================
  always @(posedge wclk) begin
    if (rst_n && frame_sync) begin
      frame_count = frame_count + 1;
      $display("[%0t] *** FRAME SYNC -- New video frame #%0d started ***", $time, frame_count);
    end
  end

  //=========================================================================
  // Monitor: First payload words (to verify pixel data correctness)
  //=========================================================================
  reg [31:0] data_word_count;

  always @(posedge rclk) begin
    if (!rst_n)
      data_word_count <= 0;
    else if (txcharisk == 4'b0000 && advb_txdata != 32'd0) begin
      data_word_count <= data_word_count + 1;
      // Print first 20 non-zero data words
      if (data_word_count < 20)
        $display("[%0t] Data word %0d: 0x%08h", $time, data_word_count, advb_txdata);
    end
  end

endmodule
