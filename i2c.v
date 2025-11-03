// -----------------------------------------------------------------------------
// i2c_master.v  —  Simple I²C Master (NO CLOCK STRETCHING), 7-bit address
//   • Open-drain SCL/SDA (drive 0 or release 'Z')
//   • START / repeated-START / STOP generation
//   • Multi-byte write and read
//   • Parameterizable SCL via CLK_DIV:  fSCL = fCLK / (2*CLK_DIV)
//   • No clock stretching (we never wait for scl_in to rise)
//   • Handshake: wr_ready (request next write byte), rd_valid (read byte ready)
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module i2c_master #(
  parameter integer CLK_DIV = 250          // Half-period divider: fSCL = fCLK/(2*CLK_DIV)
)(
  input  wire        clk,                  // System clock (e.g., 100 MHz)
  input  wire        rst_n,                // Active-low synchronous reset

  // Command interface
  input  wire        start,                // 1-cycle pulse to begin a transaction
  input  wire [6:0]  addr,                 // 7-bit I²C address
  input  wire        rw,                   // 0 = write first, 1 = read first
  input  wire [7:0]  wr_len,               // number of data bytes to write
  input  wire [7:0]  rd_len,               // number of data bytes to read

  // Write data (provided by user logic when wr_ready=1)
  input  wire [7:0]  wr_data,              // next data byte to send
  output wire        wr_ready,             // strobe: "provide wr_data now" (1 cycle)

  // Read data (emitted by master)
  output reg  [7:0]  rd_data,              // data byte we just read
  output reg         rd_valid,             // strobe: "rd_data is valid" (1 cycle)

  // Status
  output reg         busy,                 // high during a transaction
  output reg         done,                 // 1-cycle pulse at end of transaction
  output reg         ack_error,            // goes high if any address/data NACK observed

  // I²C pins (open-drain)
  inout  wire        sda,                  // serial data
  inout  wire        scl                   // serial clock
);

  // -----------------------------
  // Open-drain outputs:
  //   - We can only pull the line low (drive 0)
  //   - Or release it (high-Z) so the pull-up makes it '1'
  // -----------------------------
  reg sda_oe;                              // 1=pull SDA low, 0=release
  reg scl_oe;                              // 1=pull SCL low, 0=release
  assign sda = sda_oe ? 1'b0 : 1'bz;       // drive 0 or Z
  assign scl = scl_oe ? 1'b0 : 1'bz;       // drive 0 or Z

  // Readback (not used for stretching, but handy if you want to probe)
  wire sda_in = sda;                       // sampled SDA
  wire scl_in = scl;                       // sampled SCL

  // -----------------------------
  // SCL generator (NO stretching)
  // We produce a "desired phase" for SCL:
  //   scl_high = 1 => we release SCL (bus sees '1' via pullup)
  //   scl_high = 0 => we drive SCL low
  // scl_tick pulses every half SCL to clock our FSM activities.
  // -----------------------------
  reg [$clog2(CLK_DIV):0] div_cnt;         // divider counter
  reg                     scl_high;        // desired SCL phase
  reg                     scl_tick;        // 1-cycle pulse each half-period

  always @(posedge clk) begin
    if (!rst_n) begin
      div_cnt  <= 0;
      scl_high <= 1'b1;                    // idle = high
      scl_tick <= 1'b0;
    end else begin
      if (div_cnt == 0) begin
        div_cnt  <= CLK_DIV - 1;
        scl_high <= ~scl_high;             // toggle phase
        scl_tick <= 1'b1;                  // pulse on each half period
      end else begin
        div_cnt  <= div_cnt - 1;
        scl_tick <= 1'b0;
      end
    end
  end

  // Convert desired phase to open-drain drive for SCL
  always @(posedge clk) begin
    if (!rst_n) scl_oe <= 1'b0;            // released (bus high)
    else         scl_oe <= ~scl_high;       // high phase => release; low phase => pull low
  end

  // Handy one-shot "edges" in our timebase (not the physical line edges)
  wire scl_rise =  scl_tick &&  scl_high;   // just entered high phase
  wire scl_fall =  scl_tick && !scl_high;   // just entered low  phase

  // -----------------------------
  // Byte/bit book-keeping
  // -----------------------------
  reg [3:0] bitcnt;                        // counts 8 bits then an ACK phase
  reg [7:0] shifter;                       // holds current byte to send/receive
  reg [7:0] wr_count;                      // remaining write bytes
  reg [7:0] rd_count;                      // remaining read  bytes
  reg       first_rw;                      // latched initial R/W bit

  // Ask upstream for the next write byte exactly when we start a new byte
  reg wr_req;                              // internal "request byte now"
  assign wr_ready = wr_req;                // expose as handshake

  // -----------------------------
  // State machine declaration
  // -----------------------------
  typedef enum reg [4:0] {
    ST_IDLE = 0,           // waiting for start pulse

    ST_START_A,            // SDA low while SCL high  (create START)
    ST_START_B,            // pull SCL low to lock bus

    ST_ADDR,               // shift out A6..A0,R/W (MSB first)
    ST_ADDR_ACK,           // release SDA and sample slave ACK/NACK

    ST_WR_BYTE,            // shift out a data byte
    ST_WR_ACK,             // sample slave ACK/NACK after data byte

    ST_REP_START_A,        // repeated START (before read part)
    ST_REP_START_B,

    ST_READ_BIT,           // sample data bits from slave (SCL high)
    ST_RD_ACK,             // drive ACK (if more) or NACK (if last)

    ST_STOP_A,             // ensure SCL high while SDA low
    ST_STOP_B,             // release SDA high while SCL high  (STOP)

    ST_DONE                // pulse done then go idle
  } state_t;

  state_t state, nstate;

  // -----------------------------
  // Sequential state register
  // -----------------------------
  always @(posedge clk) begin
    if (!rst_n) state <= ST_IDLE;
    else        state <= nstate;
  end

  // -----------------------------
  // Outputs and regs defaulting
  // -----------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      busy      <= 1'b0;
      done      <= 1'b0;
      ack_error <= 1'b0;
      sda_oe    <= 1'b0;                   // release bus
      bitcnt    <= 4'd0;
      shifter   <= 8'h00;
      wr_count  <= 8'd0;
      rd_count  <= 8'd0;
      first_rw  <= 1'b0;
      wr_req    <= 1'b0;
      rd_data   <= 8'h00;
      rd_valid  <= 1'b0;
    end else begin
      // sensible defaults every cycle
      done     <= 1'b0;
      wr_req   <= 1'b0;
      rd_valid <= 1'b0;

      case (state)
        // ----------------- IDLE -----------------
        ST_IDLE: begin
          busy      <= 1'b0;               // not busy
          sda_oe    <= 1'b0;               // release SDA (bus idle high)
          ack_error <= 1'b0;               // clear previous error
          if (start) begin                  // on a start pulse...
            busy     <= 1'b1;              // we are busy now
            wr_count <= wr_len;            // latch write length
            rd_count <= rd_len;            // latch read  length
            first_rw <= rw;                // latch initial R/W
          end
        end

        // ------------- START condition -------------
        ST_START_A: begin
          // While SCL high, pull SDA low to create START edge
          if (scl_high) sda_oe <= 1'b1;    // drive SDA=0
        end
        ST_START_B: begin
          // After START edge formed, SCL goes low (phase next)
          // no special action; we move to shift address on low phase
        end

        // ------------- SEND ADDRESS+R/W -------------
        ST_ADDR: begin
          // Load and shift the address+R/W MSB->LSB on SCL falling edges
          if (scl_fall) begin
            if (bitcnt == 4'd8) begin
              shifter <= {addr, first_rw}; // load A6..A0,R/W
            end else begin
              shifter <= {shifter[6:0], 1'b0}; // shift left
            end
          end
          // Drive SDA during low phase so it is stable during next high
          if (!scl_high) begin
            if (bitcnt != 4'd0) begin
              sda_oe <= ~shifter[7];       // MSB 0 -> drive, 1 -> release
            end else begin
              sda_oe <= 1'b0;              // ACK phase: release for slave
            end
          end
        end

        // ------------- ADDRESS ACK -------------
        ST_ADDR_ACK: begin
          // Sample ACK at SCL rising (ACK=0 means OK; 1=NACK)
          if (scl_high && scl_rise) begin
            if (sda_in) ack_error <= 1'b1; // NACK seen
          end
        end

        // ------------- WRITE BYTE -------------
        ST_WR_BYTE: begin
          // Request new byte right at the start of a byte time
          if (bitcnt == 4'd8 && scl_fall && wr_count != 0) begin
            wr_req  <= 1'b1;               // ask upstream for wr_data now
            shifter <= wr_data;            // capture it
          end else if (scl_fall && bitcnt != 4'd8 && bitcnt != 4'd0) begin
            shifter <= {shifter[6:0], 1'b0}; // shift next bit
          end
          // Drive SDA during low phase, release on ACK bit
          if (!scl_high) begin
            if (bitcnt != 4'd0) sda_oe <= ~shifter[7];
            else                sda_oe <= 1'b0;    // release for slave ACK
          end
        end

        // ------------- WRITE ACK -------------
        ST_WR_ACK: begin
          if (scl_high && scl_rise) begin
            if (sda_in) ack_error <= 1'b1; // NACK on data byte
          end
        end

        // ------------- REPEATED START -------------
        ST_REP_START_A: begin
          if (scl_high) sda_oe <= 1'b1;    // SDA low while SCL high
        end
        ST_REP_START_B: begin
          // nothing special; next we send address again (this time R=1)
        end

        // ------------- READ ONE BYTE -------------
        ST_READ_BIT: begin
          sda_oe <= 1'b0;                  // release: slave drives SDA
          if (scl_high && scl_rise) begin  // sample on SCL high
            shifter <= {shifter[6:0], sda_in};
          end
        end

        // ------------- DRIVE ACK/NACK AFTER READ -------------
        ST_RD_ACK: begin
          // While SCL low, master drives ACK=0 if more bytes, else NACK=1 (release)
          if (!scl_high) begin
            sda_oe <= (rd_count == 0) ? 1'b0 : 1'b1;
          end
          // On SCL rising, the just-read byte is complete: present it
          if (scl_high && scl_rise) begin
            rd_data  <= shifter;
            rd_valid <= 1'b1;              // 1-cycle strobe
          end
        end

        // ------------- STOP -------------
        ST_STOP_A: begin
          sda_oe <= 1'b1;                  // keep SDA low until SCL is high
        end
        ST_STOP_B: begin
          if (scl_high) sda_oe <= 1'b0;    // release SDA high while SCL high => STOP
        end

        // ------------- DONE -------------
        ST_DONE: begin
          busy   <= 1'b0;                  // not busy anymore
          done   <= 1'b1;                  // pulse 'done'
          sda_oe <= 1'b0;                  // release bus
        end

        default: ;
      endcase
    end
  end

  // -----------------------------
  // Next-state logic (pure combinational)
  // -----------------------------
  always @(*) begin
    nstate = state;
    case (state)
      ST_IDLE:         nstate = start ? ST_START_A : ST_IDLE;

      ST_START_A:      nstate = (scl_rise && scl_high) ? ST_START_B : ST_START_A;
      ST_START_B:      nstate = (scl_fall) ? ST_ADDR : ST_START_B;

      // Address byte: after 8 bits (bitcnt->0) and one high sample, go to ACK
      ST_ADDR:         nstate = (scl_rise && scl_high && bitcnt==4'd0) ? ST_ADDR_ACK : ST_ADDR;

      ST_ADDR_ACK: begin
        if (scl_fall) begin
          if (first_rw==1'b0) begin
            // Write first
            if (wr_len!=0) nstate = ST_WR_BYTE;
            else if (rd_len!=0) nstate = ST_REP_START_A;
            else                nstate = ST_STOP_A;
          end else begin
            // Read first
            if (rd_len!=0) nstate = ST_READ_BIT;
            else           nstate = ST_STOP_A;
          end
        end
      end

      ST_WR_BYTE:      nstate = (scl_rise && scl_high && bitcnt==4'd0) ? ST_WR_ACK : ST_WR_BYTE;

      ST_WR_ACK: begin
        if (scl_fall) begin
          if (wr_count!=0)      nstate = ST_WR_BYTE;     // more write bytes
          else if (rd_len!=0)   nstate = ST_REP_START_A; // then go read part
          else                  nstate = ST_STOP_A;      // finish with stop
        end
      end

      ST_REP_START_A:  nstate = (scl_rise && scl_high) ? ST_REP_START_B : ST_REP_START_A;
      ST_REP_START_B:  nstate = (scl_fall) ? ST_ADDR : ST_REP_START_B;

      ST_READ_BIT:     nstate = (scl_rise && scl_high && bitcnt==4'd1) ? ST_RD_ACK : ST_READ_BIT;

      ST_RD_ACK: begin
        if (scl_fall) begin
          if (rd_count!=0) nstate = ST_READ_BIT; // ACKed (we said more) => read next
          else             nstate = ST_STOP_A;   // NACKed last => stop
        end
      end

      ST_STOP_A:       nstate = (scl_rise && scl_high) ? ST_STOP_B : ST_STOP_A;
      ST_STOP_B:       nstate = (scl_rise && scl_high) ? ST_DONE   : ST_STOP_B;

      ST_DONE:         nstate = ST_IDLE;

      default:         nstate = ST_IDLE;
    endcase
  end

  // -----------------------------
  // Counters (bits/bytes) update
  // -----------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      bitcnt   <= 4'd0;
      wr_count <= 8'd0;
      rd_count <= 8'd0;
    end else begin
      case (state)
        ST_START_B: begin
          if (scl_fall) bitcnt <= 4'd8;    // prepare to shift 8 address bits
        end

        ST_ADDR: begin
          if (scl_rise && scl_high && bitcnt!=0) bitcnt <= bitcnt - 1'b1;
          // when bitcnt==0 we’re entering ACK phase next
        end

        ST_ADDR_ACK: begin
          if (scl_fall) begin
            if (first_rw==1'b0 && wr_len!=0) bitcnt <= 4'd8; // next: a data byte
            else if (rd_len!=0)              bitcnt <= 4'd8; // or prepare for read byte
          end
        end

        ST_WR_BYTE: begin
          if (scl_rise && scl_high && bitcnt!=0) bitcnt <= bitcnt - 1'b1;
          // when bitcnt==0 -> ACK phase next
        end

        ST_WR_ACK: begin
          if (scl_fall) begin
            if (wr_count!=0) begin
              wr_count <= wr_count - 1'b1; // one write byte finished
              bitcnt   <= 4'd8;            // next data byte
            end
          end
        end

        ST_REP_START_B: begin
          if (scl_fall) bitcnt <= 4'd8;    // address again (this time R=1)
        end

        ST_READ_BIT: begin
          if (scl_rise && scl_high && bitcnt!=0) bitcnt <= bitcnt - 1'b1;
          if (scl_rise && scl_high && bitcnt==1) bitcnt <= 4'd0; // next is ACK/NACK time
        end

        ST_RD_ACK: begin
          if (scl_fall) begin
            if (rd_count!=0) begin
              rd_count <= rd_count - 1'b1; // one read byte done (we just ACKed it)
              bitcnt   <= 4'd8;            // next read byte
            end
          end
        end

        default: ;
      endcase
    end
  end

endmodule

`timescale 1ns/1ps                          // Simulation time unit/precision: 1ns steps, 1ps precision

module tb_i2c_master_strict_noslave;        // Testbench module (no ports)

// -----------------------------------------------------------------------------
// 1) CLOCK AND RESET
// -----------------------------------------------------------------------------
  reg clk = 0;                               // System clock signal we’ll drive in simulation
  always #5 clk = ~clk;                      // Toggle every 5ns → 10ns period → 100 MHz clock

  reg rst_n = 0;                             // Active-LOW reset (0 = in reset)
  initial begin                              // This block runs once at time 0
    repeat (10) @(posedge clk);              // Wait 10 rising clock edges (let things settle)
    rst_n = 1;                               // Deassert reset (now DUT can run)
  end

// -----------------------------------------------------------------------------
// 2) RAW I2C WIRES + PULLUPS (NO SLAVE PRESENT)
// -----------------------------------------------------------------------------
  wire sda;                                  // I²C data line on the board/bus
  wire scl;                                  // I²C clock line on the board/bus
  pullup(sda);                               // Model the external pull-up resistor on SDA
  pullup(scl);                               // Model the external pull-up resistor on SCL
                                             // With no slave, these pull-ups make idle = logic '1'.

// -----------------------------------------------------------------------------
// 3) DUT (YOUR I2C MASTER) PORTS + INSTANTIATION
// -----------------------------------------------------------------------------
  reg         start   = 0;                   // Pulse to begin a transaction
  reg  [6:0]  addr    = 7'h50;               // Target address (no device will answer — that’s OK)
  reg         rw      = 1'b0;                // 0 = write first, 1 = read first
  reg  [7:0]  wr_len  = 8'd0;                // Number of bytes to write
  reg  [7:0]  rd_len  = 8'd0;                // Number of bytes to read

  reg  [7:0]  wr_data = 8'h00;               // Byte presented to master when it asks for it
  wire        wr_ready;                      // Master asks for a write byte (1-cycle strobe)

  wire [7:0]  rd_data;                       // Byte produced by master during a read
  wire        rd_valid;                      // 1-cycle strobe when rd_data is fresh

  wire        busy, done, ack_error;         // Master status: active, finished, and any NACK seen

  // Instantiate your previously provided i2c_master (no clock stretching, open-drain pins)
  // CLK_DIV sets SCL ≈ fclk / (2*CLK_DIV). With 100 MHz and 250, SCL ≈ 200 kHz.
  i2c_master #(.CLK_DIV(250)) dut (
    .clk(clk), .rst_n(rst_n),
    .start(start), .addr(addr), .rw(rw),
    .wr_len(wr_len), .rd_len(rd_len),
    .wr_data(wr_data), .wr_ready(wr_ready),
    .rd_data(rd_data), .rd_valid(rd_valid),
    .busy(busy), .done(done), .ack_error(ack_error),
    .sda(sda), .scl(scl)
  );

// -----------------------------------------------------------------------------
// 4) BUS MONITORS (START/STOP + “SDA stable while SCL high” RULE)
// -----------------------------------------------------------------------------
  reg sda_q, scl_q;                          // Previous-cycle samples of SDA/SCL
  always @(posedge clk) begin                 // On each system clock edge…
    sda_q <= sda;                            // …remember last SDA
    scl_q <= scl;                            // …remember last SCL
  end

  wire sda_fall   = (sda_q==1'b1 && sda==1'b0); // Detect SDA falling edge
  wire sda_rise   = (sda_q==1'b0 && sda==1'b1); // Detect SDA rising edge
  wire start_cond = sda_fall && (scl==1'b1);    // START: SDA falls while SCL is high
  wire stop_cond  = sda_rise && (scl==1'b1);    // STOP : SDA rises while SCL is high

  integer proto_errors = 0;                   // Count protocol timing violations
  always @(posedge clk) begin
    if (rst_n) begin                          // Only check after reset is released
      if ((scl==1'b1) && (sda!=sda_q)) begin  // If SDA changed while SCL was high…
        if (!start_cond && !stop_cond) begin  // …and it was NOT a START or STOP…
          proto_errors = proto_errors + 1;    // …then this violates I²C timing → log error
          $display("[%0t][CHK][ERR] SDA toggled while SCL high (not START/STOP)", $time);
        end
      end
    end
  end

// -----------------------------------------------------------------------------
// 5) TASKS: SIMPLE, REUSABLE TEST STEPS (NO SLAVE PRESENT)
// -----------------------------------------------------------------------------

  // One-byte write with no slave: expect address NACK → ack_error should be 1
  task do_write_1byte(input [7:0] data0);
    begin
      rw     = 1'b0;                         // Write transaction
      wr_len = 8'd1;                         // Exactly one data byte
      rd_len = 8'd0;                         // No read phase

      start = 1; @(posedge clk); start = 0;  // 1-cycle start pulse

      wait (wr_ready);                       // Master asks us to supply the byte
      wr_data = data0;                       // Provide byte
      @(posedge clk);                        // Hold for a clean cycle

      wait (done);                           // Wait until master finishes the transaction

      // With no target on the bus, address ACK will be missing → expect ack_error = 1
      if (!ack_error)
        $display("[%0t][TB][ERROR] Write expected NACK (no slave), but ack_error=0", $time);
      else
        $display("[%0t][TB] Write saw NACK as expected (no slave).", $time);
    end
  endtask

  // N-byte read with no slave: expect each read byte to be 0xFF (pulled up), and ack_error = 1
  task do_read_nbytes(input integer n);
    integer i;
    begin
      rw     = 1'b1;                         // Read transaction
      wr_len = 8'd0;                         // No write bytes
      rd_len = n[7:0];                       // Number of bytes to read

      start = 1; @(posedge clk); start = 0;  // Kick it off

      for (i=0; i<n; i=i+1) begin
        @(posedge rd_valid);                 // Wait for each read byte
        $display("[%0t][TB] Read[%0d] = 0x%02h (expected 0xFF with no slave)",
                 $time, i, rd_data);
        if (rd_data !== 8'hFF)               // On open bus, pull-ups make '1's → 0xFF
          $display("[%0t][TB][ERROR] Expected 0xFF, got 0x%02h", $time, rd_data);
      end

      wait (done);                           // Transaction complete

      if (!ack_error)
        $display("[%0t][TB][ERROR] Read expected NACK (no slave), but ack_error=0", $time);
      else
        $display("[%0t][TB] Read saw NACK as expected (no slave).", $time);
    end
  endtask

// -----------------------------------------------------------------------------
// 6) MAIN TEST PROGRAM (SEQUENCE)
// -----------------------------------------------------------------------------
  integer pass = 1;                          // Overall pass/fail flag
  initial begin
    @(posedge rst_n);                        // Wait until reset deasserted
    repeat (20) @(posedge clk);              // A few idle cycles for safety

    $display("=== I2C MASTER TB (NO-SLAVE) START ===");

    // A) Try a 1-byte write → should NACK (no target present)
    do_write_1byte(8'hAB);                   // Send 0xAB as the data byte

    // B) Try a 2-byte read → bus floats high → 0xFF 0xFF; also NACK at address
    do_read_nbytes(2);

    // Protocol timing summary
    if (proto_errors != 0) begin
      $display("[SUM][FAIL] Protocol timing errors: %0d", proto_errors);
      pass = 0;
    end else begin
      $display("[SUM] No protocol timing errors (SDA stable while SCL high).");
    end

    if (pass) $display("[TB][PASS] No-slave tests passed.");
    else      $display("[TB][FAIL] See errors above.");

    #200 $finish;                            // End simulation after a short wait
  end

// -----------------------------------------------------------------------------
// 7) OPTIONAL INFO PRINTS (SEE START/STOP MOMENTS IN LOG)
// -----------------------------------------------------------------------------
  always @(posedge clk) begin
    if (start_cond) $display("[%0t][INFO] START detected", $time);  // SDA↓ while SCL=1
    if (stop_cond)  $display("[%0t][INFO] STOP  detected", $time);  // SDA↑ while SCL=1
  end

endmodule

// THE END

