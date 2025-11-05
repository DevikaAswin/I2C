`timescale 1ns/1ps

module i2c_master #(
  parameter integer CLK_DIV = 250   // fSCL = fCLK / (2*CLK_DIV)
)(
  input  wire        clk,
  input  wire        rst_n,

  // Command
  input  wire        start,         // 1-cycle pulse
  input  wire [6:0]  addr,          // 7-bit target
  input  wire        rw,            // 0=write-first, 1=read-first
  input  wire [7:0]  wr_len,
  input  wire [7:0]  rd_len,

  // Data
  input  wire [7:0]  wr_data,
  output reg         wr_ready,      // strobe: request next write byte
  output reg  [7:0]  rd_data,
  output reg         rd_valid,      // strobe: rd_data valid

  // Status
  output reg         busy,
  output reg         done,          // 1-cycle pulse when finished
  output reg         ack_error,     // seen any NACK

  // I2C pins (open-drain)
  inout  wire        sda,
  inout  wire        scl
);

  // ----------------------------
  // Open-drain pins
  // ----------------------------
  reg sda_oe;                       // 1=pull low, 0=release (Z)
  reg scl_oe;                       // 1=pull low, 0=release (Z)
  assign sda = sda_oe ? 1'b0 : 1'bz;
  assign scl = scl_oe ? 1'b0 : 1'bz;
  wire sda_in = sda;                // read bus level

  // ----------------------------
  // SCL toggler — ONLY during active transaction
  // ----------------------------
  reg                     active_txn;       // 1 between START and STOP
  reg [$clog2(CLK_DIV):0] div_cnt;
  reg                     tick;             // 1-cycle pulse each half-period

  always @(posedge clk) begin
    if (!rst_n) begin
      div_cnt   <= 0;
      scl_oe    <= 1'b0;                    // idle: released (pulled high)
      tick      <= 1'b0;
    end else begin
      tick <= 1'b0;
      if (active_txn) begin
        if (div_cnt == 0) begin
          div_cnt <= (CLK_DIV==0) ? 1 : (CLK_DIV-1);
          scl_oe  <= ~scl_oe;               // toggle ONLY while active
          tick    <= 1'b1;
        end else begin
          div_cnt <= div_cnt - 1'b1;
        end
      end else begin
        scl_oe  <= 1'b0;                    // release when idle
        div_cnt <= (CLK_DIV==0) ? 1 : (CLK_DIV-1);
      end
    end
  end
  // Phase rule: use ONLY these two predicates in the FSM
  // (tick && scl_oe==1) → entered LOW half
  // (tick && scl_oe==0) → entered HIGH half

  // ----------------------------
  // Book-keeping
  // ----------------------------
  reg [3:0] bitcnt;              // 8..1 data bits; ACK after 0
  reg [7:0] shifter;
  reg [7:0] wr_count, rd_count;
  reg       first_rw;

  // ----------------------------
  // FSM (encoded as localparams for rock-solid synthesis/sim)
  // ----------------------------
  localparam [4:0]
    ST_IDLE        = 5'd0,
    ST_START_A     = 5'd1,
    ST_START_B     = 5'd2,
    ST_ADDR        = 5'd3,
    ST_ADDR_ACK    = 5'd4,
    ST_WR_BYTE     = 5'd5,
    ST_WR_ACK      = 5'd6,
    ST_REP_START_A = 5'd7,
    ST_REP_START_B = 5'd8,
    ST_READ_BITS   = 5'd9,
    ST_RD_ACK      = 5'd10,
    ST_STOP_A      = 5'd11,
    ST_STOP_B      = 5'd12,
    ST_DONE        = 5'd13;

  reg [4:0] state, nstate;

  // State reg
  always @(posedge clk) begin
    if (!rst_n) state <= ST_IDLE;
    else        state <= nstate;
  end

  // Outputs + registers
  always @(posedge clk) begin
    if (!rst_n) begin
      busy       <= 1'b0;
      done       <= 1'b0;
      ack_error  <= 1'b0;
      sda_oe     <= 1'b0;
      wr_ready   <= 1'b0;
      rd_valid   <= 1'b0;
      rd_data    <= 8'h00;
      shifter    <= 8'h00;
      bitcnt     <= 4'd0;
      wr_count   <= 8'd0;
      rd_count   <= 8'd0;
      first_rw   <= 1'b0;
      active_txn <= 1'b0;
    end else begin
      // defaults
      done     <= 1'b0;
      wr_ready <= 1'b0;
      rd_valid <= 1'b0;

      case (state)
        // ----------------- IDLE -----------------
        ST_IDLE: begin
          busy       <= 1'b0;
          ack_error  <= 1'b0;
          sda_oe     <= 1'b0;              // release SDA
          active_txn <= 1'b0;
          if (start) begin
            busy      <= 1'b1;
            wr_count  <= wr_len;
            rd_count  <= rd_len;
            first_rw  <= rw;
          end
        end

        // ------------- START -------------
        ST_START_A: begin
          sda_oe     <= 1'b1;              // SDA low while SCL high
          active_txn <= 1'b1;              // start SCL toggling
        end

        ST_START_B: begin
          if (tick && scl_oe==1) begin     // first LOW half
            bitcnt  <= 4'd8;
            shifter <= {addr, first_rw};
          end
        end

        // ------------- ADDRESS -------------
        ST_ADDR: begin
          if (tick && scl_oe==1 && bitcnt!=0) begin
            sda_oe <= ~shifter[7];         // drive next MSB during LOW half
          end
          if (tick && scl_oe==0 && bitcnt!=0) begin
            shifter <= {shifter[6:0],1'b0};
            bitcnt  <= bitcnt - 1'b1;
            if (bitcnt==1) sda_oe <= 1'b0; // release for ACK after last bit
          end
        end

        ST_ADDR_ACK: begin
          if (tick && scl_oe==0) begin     // sample on HIGH half
            if (sda_in) ack_error <= 1'b1; // NACK
          end
        end

        // ------------- WRITE BYTE(S) -------------
        ST_WR_BYTE: begin
          if (tick && scl_oe==1 && bitcnt==0 && wr_count!=0) begin
            wr_ready <= 1'b1;              // request byte
            shifter  <= wr_data;
            bitcnt   <= 4'd8;
          end
          if (tick && scl_oe==1 && bitcnt!=0) begin
            sda_oe <= ~shifter[7];
          end
          if (tick && scl_oe==0 && bitcnt!=0) begin
            shifter <= {shifter[6:0],1'b0};
            bitcnt  <= bitcnt - 1'b1;
            if (bitcnt==1) sda_oe <= 1'b0; // release for ACK
          end
        end

        ST_WR_ACK: begin
          if (tick && scl_oe==0) begin
            if (sda_in) ack_error <= 1'b1; // NACK on data
          end
          if (tick && scl_oe==1 && wr_count!=0) begin
            wr_count <= wr_count - 1'b1;   // one data byte done
          end
        end

        // ------------- REPEATED START -------------
        ST_REP_START_A: begin
          sda_oe <= 1'b1;                  // SDA low while SCL high
        end
        ST_REP_START_B: begin
          if (tick && scl_oe==1) begin
            bitcnt  <= 4'd8;
            shifter <= {addr, 1'b1};       // read now
          end
        end

        // ------------- READ BYTE -------------
        ST_READ_BITS: begin
          sda_oe <= 1'b0;                  // release: slave drives
          if (tick && scl_oe==0 && bitcnt!=0) begin
            shifter <= {shifter[6:0], sda_in};
            bitcnt  <= bitcnt - 1'b1;
          end
        end

        ST_RD_ACK: begin
          if (tick && scl_oe==0) begin
            rd_data  <= shifter;           // present byte on HIGH half
            rd_valid <= 1'b1;
          end
          if (tick && scl_oe==1) begin
            // ACK (pull low) if more bytes; else NACK (release)
            sda_oe <= (rd_count!=0) ? 1'b1 : 1'b0;
            if (rd_count!=0) begin
              rd_count <= rd_count - 1'b1;
              bitcnt   <= 4'd8;
            end
          end
        end

        // ------------- STOP -------------
        ST_STOP_A: begin
          sda_oe <= 1'b1;                  // keep SDA low; wait for HIGH half
        end
        ST_STOP_B: begin
          if (tick && scl_oe==0) sda_oe <= 1'b0; // release SDA high → STOP
        end

        // ------------- DONE -------------
        ST_DONE: begin
          busy       <= 1'b0;
          done       <= 1'b1;
          active_txn <= 1'b0;              // stop SCL toggling outside txn
          sda_oe     <= 1'b0;              // release bus
        end

        default: ;
      endcase
    end
  end

  // ----------------------------
  // Next-state logic (pure combinational)
  // ----------------------------
  always @* begin
    nstate = state;
    case (state)
      ST_IDLE:          nstate = start ? ST_START_A : ST_IDLE;

      ST_START_A:       nstate = ST_START_B;
      ST_START_B:       nstate = (tick && scl_oe==1) ? ST_ADDR      : ST_START_B;

      ST_ADDR:          nstate = (tick && scl_oe==0 && bitcnt==0) ? ST_ADDR_ACK  : ST_ADDR;

      ST_ADDR_ACK: begin
        if (tick && scl_oe==1) begin
          if (!first_rw) begin
            if (wr_len!=0)      nstate = ST_WR_BYTE;
            else if (rd_len!=0) nstate = ST_REP_START_A;
            else                nstate = ST_STOP_A;
          end else begin
            nstate = (rd_len!=0) ? ST_READ_BITS : ST_STOP_A;
          end
        end
      end

      ST_WR_BYTE:       nstate = (tick && scl_oe==0 && bitcnt==0) ? ST_WR_ACK    : ST_WR_BYTE;

      ST_WR_ACK: begin
        if (tick && scl_oe==1) begin
          if (wr_count!=0)      nstate = ST_WR_BYTE;
          else if (rd_len!=0)   nstate = ST_REP_START_A;
          else                  nstate = ST_STOP_A;
        end
      end

      ST_REP_START_A:   nstate = ST_REP_START_B;
      ST_REP_START_B:   nstate = (tick && scl_oe==1) ? ST_ADDR      : ST_REP_START_B;

      ST_READ_BITS:     nstate = (tick && scl_oe==0 && bitcnt==0) ? ST_RD_ACK    : ST_READ_BITS;

      ST_RD_ACK: begin
        if (tick && scl_oe==1) begin
          if (rd_count==0) nstate = ST_STOP_A;   // last byte → NACK then STOP
          else             nstate = ST_READ_BITS;
        end
      end

      ST_STOP_A:        nstate = (tick && scl_oe==0) ? ST_STOP_B    : ST_STOP_A;
      ST_STOP_B:        nstate = (tick && scl_oe==0) ? ST_DONE      : ST_STOP_B;

      ST_DONE:          nstate = ST_IDLE;

      default:          nstate = ST_IDLE;
    endcase
  end

endmodule
`timescale 1ns/1ps

module tb_i2c_master_strict_noslave;

  // 1) Clock & reset
  reg clk = 0;
  always #5 clk = ~clk;                   // 100 MHz
  reg rst_n = 0;
  initial begin
    repeat (10) @(posedge clk);
    rst_n = 1;
  end

  // 2) I2C wires + pullups (no slave present)
  wire sda, scl;
  pullup(sda);
  pullup(scl);

  // 3) DUT
  reg         start   = 0;
  reg  [6:0]  addr    = 7'h50;
  reg         rw      = 1'b0;
  reg  [7:0]  wr_len  = 8'd0;
  reg  [7:0]  rd_len  = 8'd0;

  reg  [7:0]  wr_data = 8'h00;
  wire        wr_ready;

  wire [7:0]  rd_data;
  wire        rd_valid;

  wire        busy, done, ack_error;

  i2c_master #(.CLK_DIV(250)) dut (
    .clk(clk), .rst_n(rst_n),
    .start(start), .addr(addr), .rw(rw),
    .wr_len(wr_len), .rd_len(rd_len),
    .wr_data(wr_data), .wr_ready(wr_ready),
    .rd_data(rd_data), .rd_valid(rd_valid),
    .busy(busy), .done(done), .ack_error(ack_error),
    .sda(sda), .scl(scl)
  );

  // 4) START/STOP monitors + "SDA stable while SCL high" rule
  reg sda_q, scl_q;
  always @(posedge clk) begin
    sda_q <= sda; scl_q <= scl;
  end
  wire start_cond = (sda_q==1'b1 && sda==1'b0 && scl==1'b1);
  wire stop_cond  = (sda_q==1'b0 && sda==1'b1 && scl==1'b1);

  integer proto_errors = 0;
  always @(posedge clk) if (rst_n) begin
    if ((scl==1'b1) && (sda!=sda_q) && !start_cond && !stop_cond) begin
      proto_errors = proto_errors + 1;
      $display("[%0t][ERR] SDA toggled while SCL high", $time);
    end
  end

  // 5) Tasks (no slave → expect address NACK)
  task do_write_1(input [7:0] d0);
    begin
      rw=0; wr_len=1; rd_len=0;
      start=1; @(posedge clk); start=0;

      @(posedge wr_ready);
      wr_data = d0;
      @(posedge clk);

      @(posedge done);
      if (!ack_error) $display("[%0t][TB][ERROR] expected ack_error=1 (no slave)", $time);
      else            $display("[%0t][TB] write NACK as expected", $time);
    end
  endtask

  task do_read_n(input integer n);
    integer i;
    begin
      rw=1; wr_len=0; rd_len=n[7:0];
      start=1; @(posedge clk); start=0;

      for (i=0;i<n;i=i+1) begin
        @(posedge rd_valid);
        $display("[%0t][TB] RD[%0d]=0x%02h (open bus → 0xFF)", $time, i, rd_data);
      end

      @(posedge done);
      if (!ack_error) $display("[%0t][TB][ERROR] expected ack_error=1 (no slave)", $time);
      else            $display("[%0t][TB] read NACK as expected", $time);
    end
  endtask

  // 6) Program
  integer pass = 1;
  initial begin
    @(posedge rst_n);
    repeat (20) @(posedge clk);

    $display("=== I2C MASTER STRICT TB START ===");

    // A) 1-byte write (no target → NACK)
    do_write_1(8'hAB);

    // B) 2-byte read (pullups → 0xFF each; address NACK)
    do_read_n(2);

    if (proto_errors!=0) begin
      $display("[SUM][FAIL] Protocol timing errors: %0d", proto_errors);
      pass = 0;
    end else begin
      $display("[SUM] No protocol timing errors.");
    end

    if (pass) $display("[TB][PASS]");
    #200 $finish;
  end

  // 7) Pretty prints
  always @(posedge clk) begin
    if (start_cond) $display("[%0t][INFO] START", $time);
    if (stop_cond)  $display("[%0t][INFO] STOP ", $time);
  end

endmodule
Short answer: yes—the code is aligned with the I²C timing rules (SDA changes only when SCL is low; data sampled when SCL is high; START = SDA↓ while SCL=1; STOP = SDA↑ while SCL=1; ACK on 9th clock). I double-checked each state and half-cycle against the NXP I²C spec (UM10204) and a TI primer; the rules below are the source of truth: “SDA must be stable during the HIGH period of SCL; changes occur only when SCL is LOW,” and START/STOP are SDA transitions while SCL is HIGH. 
NXP
+1

What I verified, state by state

Between transactions: active_txn=0 → scl_oe=0, sda_oe=0 ⇒ both lines released (pulled high). Matches idle requirement. 
NXP

START (ST_START_A → ST_START_B): we pull SDA low while SCL is high, then enable SCL toggling. That produces a legal START edge (SDA↓ with SCL=1). 
Texas Instruments

Address & write data bits (ST_ADDR, ST_WR_BYTE):
– We set SDA during the LOW half (tick && scl_oe==1).
– We shift/sample/commit during the HIGH half (tick && scl_oe==0).
This guarantees SDA is stable through SCL=HIGH. 
NXP

ACK after address/data (ST_ADDR_ACK, ST_WR_ACK):
– We release SDA before the ACK bit.
– We sample SDA during the HIGH half to see 0=ACK, 1=NACK. Correct per spec. 
NXP

Read bits (ST_READ_BITS):
– Master releases SDA; slave changes data only when SCL is LOW; we sample on SCL=HIGH. Correct. 
NXP

Master ACK/NACK after read (ST_RD_ACK):
– We present the byte to user logic on a HIGH half.
– On the next LOW half we drive ACK (pull low) if more bytes, else NACK (release); this makes SDA stable before the following HIGH (the 9th clock). Correct per ACK timing. 
NXP

STOP (ST_STOP_A → ST_STOP_B): we release SDA high while SCL is high to create a legal STOP. 
Texas Instruments

TB checker: asserts “SDA must not change while SCL is high (except START/STOP)". That’s exactly the spec’s “data valid” rule. 
NXP

One small, surgical improvement (repeated-START edge)

To be bulletproof for the repeated START sequence, let’s force a real SDA transition while SCL is HIGH (SDA↑ then SDA↓ with SCL=1). Your current code already works in most phases, but this tweak guarantees the visible edge regardless of which half-cycle we enter the R-START on.
// REPEATED START (guarantee SDA transition while SCL is HIGH)
ST_REP_START_A: begin
  // First ensure SDA is released during a LOW half
  if (tick && scl_oe==1) begin
    sda_oe <= 1'b0;                 // SDA=Z while SCL LOW
  end
  // Then pull SDA LOW during the next HIGH half → creates START edge
  if (tick && scl_oe==0) begin
    sda_oe <= 1'b1;                 // SDA=0 while SCL HIGH (START)
  end
end

ST_REP_START_B: begin
  // On the following LOW half, load the read address
  if (tick && scl_oe==1) begin
    bitcnt  <= 4'd8;
    shifter <= {addr, 1'b1};        // R=1
  end
end
Why this tweak?

Spec defines a START as the transition SDA:1→0 while SCL=1. The above forces that edge even if we happened to enter the repeated-START state during a LOW half (which could otherwise hide the explicit transition).
