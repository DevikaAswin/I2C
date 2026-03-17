# ARINC 818 Complete System — Line-by-Line Explanation

This document provides a comprehensive, detailed explanation of every module in the ARINC 818 video transmission system. It covers:

1. **Asynchronous FIFO** (`asyncfifo`) — clock domain crossing
2. **ARINC 818 Transmitter** (`advbtx`) — ADVB frame construction
3. **Image Pattern Generator** (`image_pattern_gen`) — color bar generation + pixel packing
4. **Top-Level Wrapper** (`arinc818_top`) — system integration
5. **Testbench** (`tb_arinc818`) — simulation

---

## Table of Contents

- [1. Background: What is ARINC 818?](#1-background-what-is-arinc-818)
- [2. System Architecture](#2-system-architecture)
- [3. Module 1: asyncfifo (Asynchronous FIFO)](#3-module-1-asyncfifo)
- [4. Module 2: advbtx (ARINC 818 Transmitter)](#4-module-2-advbtx)
- [5. Module 3: image_pattern_gen (Image Generator)](#5-module-3-image_pattern_gen)
- [6. Module 4: arinc818_top (Top-Level Wrapper)](#6-module-4-arinc818_top)
- [7. Module 5: tb_arinc818 (Testbench)](#7-module-5-tb_arinc818)

---

## 1. Background: What is ARINC 818?

**ARINC 818** (Avionics Digital Video Bus — ADVB) is a Fibre Channel–based protocol designed for transmitting uncompressed digital video in avionics systems (aircraft cockpit displays, cameras, HUDs, etc.). Key concepts:

- **ADVB Frame**: The fundamental transport unit. NOT the same as a "video frame." One line of video may require multiple ADVB frames.
- **Container**: A group of ADVB frames that together carry one complete video image.
- **Object 0**: The first ADVB frame in a container. Contains the container header + ancillary data (metadata about the video).
- **Object 2**: Subsequent ADVB frames that carry the actual pixel payload data.
- **Ordered Sets**: Special 32-bit patterns (SOF, EOF, IDLE) used to delimit frames and fill gaps. They use K-characters (special 8b/10b symbols).
- **CRC-32**: Each ADVB frame is protected by a CRC computed over the header, container/ancillary data, and payload.

### ADVB Frame Structure

```
┌─────────┬──────────┬──────────────────────────────┬───────┬─────────┐
│ SOF     │ Header   │ Payload                      │ CRC   │ EOF     │
│(1 word) │(6 words) │(Container+Anc OR Pixel data) │(1 wd) │(1 word) │
└─────────┴──────────┴──────────────────────────────┴───────┴─────────┘
```

---

## 2. System Architecture

```
┌──────────────────┐     ┌─────────────┐     ┌──────────────────┐
│ image_pattern_gen│────>│  asyncfifo  │────>│     advbtx       │──> ADVB Link
│  (color bars)    │wclk │ (CDC bridge)│     │  (ARINC 818 TX)  │rclk
│  writes pixels   │     │             │     │  reads pixels,    │
│  into FIFO       │     │  512 x 32   │     │  builds frames    │
└──────────────────┘     └─────────────┘     └──────────────────┘
```

The **write clock** (wclk) drives the image generator. The **read clock** (rclk) drives the transmitter. The async FIFO safely crosses between these two clock domains.

---

## 3. Module 1: asyncfifo (Asynchronous FIFO)

### Purpose
An asynchronous FIFO (First-In-First-Out) buffer that safely transfers data between two independent clock domains. This is essential because the image generator and ARINC transmitter may run at different clock frequencies.

### Module Declaration

```verilog
module asyncfifo #(
    parameter DATASIZE=32,    // Width of each FIFO entry in bits
    parameter ADDRSIZE=9      // Address width → depth = 2^9 = 512 entries
)
```

- `DATASIZE=32`: Each FIFO word is 32 bits wide, matching one packed pixel-group word.
- `ADDRSIZE=9`: The binary address is 9 bits, giving a depth of 2^9 = **512 entries**. This means the FIFO can buffer up to 512 × 4 = 2048 bytes of pixel data.

### Ports

```verilog
input wclk,              // Write-side clock
input wrstn,             // Write-side reset (active-low)
input winc,              // Write increment (write enable) — "I want to write this cycle"
input [DATASIZE-1:0] wdata,  // Write data (32 bits)
input rclk,              // Read-side clock
input rrstn,             // Read-side reset (active-low)
input rinc,              // Read increment (read enable) — "I want to read this cycle"
output [DATASIZE-1:0] rdata, // Read data (32 bits)
output reg wfull,        // Full flag (tells writer: STOP, FIFO is full)
output reg rempty        // Empty flag (tells reader: STOP, no data available)
```

### Internal Storage

```verilog
localparam DEPTH = 1<<ADDRSIZE;             // 512
reg [DATASIZE-1:0] mem[0:DEPTH-1];          // The actual memory array: 512 × 32 bits
```

This is a simple dual-port RAM. The write side writes to `mem[waddr]`, the read side reads from `mem[raddr]`.

### Gray-Code Pointer Crossing — The Heart of the Design

The central challenge of an async FIFO is: **How does the write side know when the FIFO is full, when the read pointer is in a different clock domain?** (And vice versa for empty.)

The answer: **Gray code**. In gray code, consecutive values differ by exactly one bit. This means if a pointer is being sampled across clock domains during a transition, only one bit is changing, so you'll either see the old value or the new value — never a corrupted intermediate value.

#### Write-side pointers

```verilog
reg [ADDRSIZE:0] wbin, wptr;                // Binary and Gray-code write pointers (10 bits)
wire [ADDRSIZE:0] wbinnext, wgraynext;       // Next values
```

- `wbin`: The binary write pointer (10 bits: 9 for address + 1 MSB for wrap detection)
- `wptr`: The Gray-code version of wbin, synchronized to the read domain
- The extra MSB is crucial for distinguishing "full" from "empty" — both look like equal pointers, but the MSB tells them apart

```verilog
assign wbinnext = wbin + (winc & ~wfull);    // Increment binary pointer if writing and not full
assign wgraynext = (wbinnext>>1) ^ wbinnext; // Convert binary → Gray code
assign waddr = wbin[ADDRSIZE-1:0];           // Lower 9 bits = actual memory address
```

**Binary to Gray conversion**: `gray = (binary >> 1) XOR binary`. This is a standard formula. For example:
- Binary 5 = `0101` → `(0010) XOR (0101)` = `0111` = Gray 7

#### Read-side pointers (symmetric)

```verilog
reg [ADDRSIZE:0] rbin, rptr;
wire [ADDRSIZE:0] rbinnext, rgraynext;

assign rbinnext  = rbin + (rinc & ~rempty);
assign rgraynext = (rbinnext>>1) ^ rbinnext;
assign raddr     = rbin[ADDRSIZE-1:0];
```

Same structure as write side. `rinc & ~rempty` means: advance the read pointer only if a read is requested AND the FIFO isn't empty.

#### Synchronizing pointers across clock domains

```verilog
// Write side: synchronize read pointer (from rclk → wclk)
reg [ADDRSIZE:0] wq1rptr, wq2rptr;          // Two-flip-flop synchronizer

// Read side: synchronize write pointer (from wclk → rclk)
reg [ADDRSIZE:0] rq1wptr, rq2wptr;          // Two-flip-flop synchronizer
```

A **two-flip-flop synchronizer** is the industry-standard technique for safely crossing clock domains. The pointer passes through two sequential flip-flops clocked by the destination domain, which gives metastability time to resolve.

#### Full and Empty detection

```verilog
// Empty: read gray-next equals synchronized write pointer
assign remptyval = (rgraynext == rq2wptr);

// Full: write gray-next matches inverted-MSB synchronized read pointer
assign wfullval = (wgraynext == {~wq2rptr[ADDRSIZE:ADDRSIZE-1], wq2rptr[ADDRSIZE-2:0]});
```

- **Empty**: The FIFO is empty when the read pointer equals the write pointer (in Gray code). Since we're comparing the read's next value to the synchronized write pointer, this gives us the correct empty status.

- **Full**: The FIFO is full when the write pointer has "lapped" the read pointer — they differ only in the two MSBs (which are inverted). The expression `{~wq2rptr[ADDRSIZE:ADDRSIZE-1], wq2rptr[ADDRSIZE-2:0]}` inverts the top 2 bits of the synchronized read pointer. If `wgraynext` matches this, the FIFO is full.

> **Why invert TWO bits for full?** In Gray code, when the binary MSB flips (indicating a wrap-around), TWO gray-code bits change (the MSB and the bit below it). So to detect that the write pointer is exactly one full cycle ahead of the read pointer, we invert the top two Gray-code bits.

#### Memory read

```verilog
assign rdata = mem[raddr];    // Asynchronous read — data is always available at raddr
```

This is a combinational read. Whenever `raddr` changes, `rdata` immediately reflects the stored value. No read clock edge needed for the memory itself.

#### Read-side sequential logic

```verilog
always @(posedge rclk or negedge rrstn)
begin
    if (!rrstn) begin
        {rbin, rptr}       <= 0;           // Clear binary and Gray pointers
        rempty             <= 1'b1;         // FIFO starts empty
        {rq2wptr, rq1wptr} <= 0;           // Clear synchronizer
    end
    else begin
        {rbin, rptr}       <= {rbinnext, rgraynext};  // Update pointers
        rempty             <= remptyval;               // Update empty flag  
        {rq2wptr, rq1wptr} <= {rq1wptr, wptr};        // 2-FF synchronizer shift
    end
end
```

On each `rclk` edge:
1. Advance read pointers (binary + Gray) if a read occurred
2. Update the empty flag
3. Shift the write pointer through the 2-stage synchronizer: `wptr → rq1wptr → rq2wptr`

#### Write-side sequential logic

```verilog
always @(posedge wclk or negedge wrstn)
begin
    if (!wrstn) begin
        {wbin, wptr}       <= 0;
        wfull              <= 1'b0;         // FIFO starts not-full
        {wq2rptr, wq1rptr} <= 0;
    end
    else begin
        {wbin, wptr}       <= {wbinnext, wgraynext};
        wfull              <= wfullval;
        {wq2rptr, wq1rptr} <= {wq1rptr, rptr};        // 2-FF synchronizer
        if (winc && !wfull)
            mem[waddr] <= wdata;                        // Write data to memory
    end
end
```

On each `wclk` edge:
1. Advance write pointers
2. Update the full flag
3. Synchronize the read pointer into this domain
4. **Write data to memory** if `winc` is asserted and FIFO isn't full

### Key Takeaway
The async FIFO uses **Gray-code** pointers + **two-flip-flop synchronizers** to safely cross clock domains. Full/empty flags are **conservatively** generated — they may briefly indicate full/empty when the FIFO isn't actually full/empty (due to synchronization latency), but they will never falsely indicate "not full" or "not empty." This is the safe behavior.

---

## 4. Module 2: advbtx (ARINC 818 Transmitter)

### Purpose
Reads pixel data from the FIFO and constructs ARINC 818 ADVB frames, including:
- Start-of-Frame (SOF) ordered sets
- Frame headers (6 words)
- Container headers + ancillary data (for Object 0)
- Pixel payload (for Object 2)
- CRC-32
- End-of-Frame (EOF) ordered sets

### Module Parameters

```verilog
parameter HACTIVE=1024,            // Active pixels per line (horizontal resolution)
parameter VACTIVE=768,             // Active lines per frame (vertical resolution)
parameter BytesPerPixel=3,         // RGB24 → 3 bytes per pixel
parameter FRAMERATE=60,            // Target frame rate (informational, not used in logic)
parameter VERTICALBLANKPRE=0,      // # of blank lines BEFORE active video (set to 0)
parameter VERTICALBLANKPOST=0,     // # of blank lines AFTER active video (set to 0)
parameter HORIZONTALIDLECYCLES=6   // # of idle cycles between frames (not directly used)
```

### Derived Parameters (localparams)

```verilog
localparam PAYLOADBYTESPERFRAME = 2112;   // Max payload per ADVB frame (ARINC 818 spec)
localparam BYTESPERLINE = (HACTIVE * BytesPerPixel);  // = 1024 × 3 = 3072 bytes/line
localparam WORDSPERLINE = (BYTESPERLINE / 4);         // = 3072 / 4 = 768 words/line
localparam HEADERWORDS = 6;               // Frame header is 6 words (Fibre Channel standard)
localparam CONTAINERWORDS = 22;           // Container header is 22 words
localparam ANCILLARYWORDS = 4;            // Ancillary data is 4 words
localparam FRAMESPERLINE = (BYTESPERLINE + PAYLOADBYTESPERFRAME) / PAYLOADBYTESPERFRAME;
    // = (3072 + 2112) / 2112 = 5184 / 2112 = 2 (integer division)
    // So each video line needs 2 ADVB frames to carry all its pixel data
localparam PAYLOADWORDS = (WORDSPERLINE / FRAMESPERLINE);
    // = 768 / 2 = 384 words per ADVB payload frame
localparam TOTALOBJ2FRAMES = VACTIVE * FRAMESPERLINE;
    // = 768 × 2 = 1536 total Object-2 ADVB frames per video frame
localparam VTOTAL = VACTIVE + VERTICALBLANKPRE + VERTICALBLANKPOST;
    // = 768 + 0 + 0 = 768 total lines
```

### State Machine States

```verilog
localparam IDLE        = 4'd0;   // Send idle patterns, wait for data
localparam SENDSOF     = 4'd1;   // Send Start-of-Frame ordered set
localparam SENDHDR     = 4'd2;   // Send 6-word frame header
localparam SENDCONT    = 4'd3;   // Send 22-word container header (Object 0 only)
localparam SENDANC     = 4'd4;   // Send 4-word ancillary data (Object 0 only)
localparam SENDPAYLOAD = 4'd5;   // Send pixel payload (Object 2 frames)
localparam SENDCRC     = 4'd6;   // Send CRC-32 word
localparam SENDEOF     = 4'd7;   // Send End-of-Frame ordered set
```

### Ordered Set Constants

```verilog
localparam IDLEWORD = 32'hB5B595BC;  // Sent between frames (idle fill)
localparam SOFIWORD = 32'h5757BFBC;  // Start-of-Frame Initiate (first frame of container)
localparam SOFNWORD = 32'h4F4FB7BC;  // Start-of-Frame Normal (subsequent frames)
localparam EOFNWORD = 32'h4E4E95BC;  // End-of-Frame Normal 
localparam EOFTWORD = 32'h4B4B95BC;  // End-of-Frame Terminate (last frame of last line)
```

The byte `0xBC` in the LSB is the **K28.5** comma character used in 8b/10b encoding. It's the standard alignment character in Fibre Channel. The `txcharisk` signal indicates which bytes are K-characters (`4'b0001` means only byte 0 is a K-char).

### Internal Registers

```verilog
wire [13:0] VACT = VACTIVE;     // 14-bit representation of vertical active lines
wire [13:0] HACT = HACTIVE;     // 14-bit representation of horizontal active pixels
wire [7:0]  sequenceid;         // Lower 8 bits of container count

reg [4:0]  state;               // Current state of the FSM
reg [4:0]  nextstate;           // Next state (combinational)
reg [31:0] containercount;      // Increments each time a full container (image) is sent
reg [15:0] seq_count;           // Sequence counter within a container (per-line sub-frame)
reg [15:0] lineindex;           // Current line being transmitted (0 to VTOTAL-1)
reg [1:0]  sublineindex;        // Which ADVB frame within the current line (0 or 1)
reg [2:0]  idlecounter;         // Counts idle cycles between frames
reg [31:0] crcaccum;            // Running CRC-32 accumulator
reg [9:0]  wordcounter;         // Word counter within the current phase (header, payload, etc.)
reg [10:0] count;               // General-purpose counter (used during vertical blank)
reg        lastframe;           // Flag: set to 1 when the very last frame of the image is sent
reg        object0;             // Flag: 0 = currently sending Object 0, 1 = sending Object 2
```

### CRC-32 Function

```verilog
function [31:0] nextcrc32;
  input [31:0] crc;       // Current CRC accumulator
  input [31:0] data;      // 32 bits of new data to incorporate
  integer p;
  reg [31:0] c;
  begin
    c = crc;
    for (p = 0; p < 32; p = p + 1) begin
      // Process each bit, MSB first
      // If the feedback bit (c[31] XOR data bit) is 1, XOR with polynomial
      c = (c[31] ^ data[31-p]) ? ((c << 1) ^ 32'h04C11DB7) : (c << 1);
      nextcrc32 = c;
    end
  end
endfunction
```

This implements the standard **CRC-32** (polynomial `0x04C11DB7`, same as Ethernet/Fibre Channel). It processes 32 bits of data at once by iterating bit-by-bit from MSB to LSB. The accumulator is initialized to `0xFFFFFFFF` at the start of each frame.

### Ancillary Header Function

```verilog
function [31:0] ancillaryheader;
  input [2:0] index;     // Which word (0-3) of the ancillary data
  begin
    case(index)
      3'd0: ancillaryheader = {VACT, HACT, 4'b0000};
          // Word 0: {14-bit VACTIVE, 14-bit HACTIVE, 4 zero bits}
          // = {768, 1024, 0} packed into 32 bits
          // This tells the receiver the video resolution
      3'd1: ancillaryheader = 32'h10008888;
          // Word 1: Video format descriptor
          // Contains pixel format information (RGB 8:8:8)
      3'd2: ancillaryheader = 32'b0;    // Reserved
      3'd3: ancillaryheader = 32'b0;    // Reserved
    endcase
  end
endfunction
```

The ancillary data tells the receiver about the video format: resolution and pixel encoding.

### Container Header Function

```verilog
function [31:0] containerheader;
  input [4:0] index;     // Which word (0-21) of the 22-word container header
  begin
    case(index)
      5'd0:  containerheader = containercount;
          // Word 0: Container sequence number — increments with each new image
      5'd1:  containerheader = 32'h0;        // Reserved
      5'd2:  containerheader = 32'h0;        // Reserved
      5'd3:  containerheader = 32'h0;        // Reserved
      5'd4:  containerheader = 32'h10000;    // Container type/flags
      5'd5:  containerheader = 32'h00040000; // Container data type
      5'd6:  containerheader = {8'h50, 8'b0, 16'hd000};
          // Object pointers/sizes
      5'd7:  containerheader = 32'h10;       // Object size (16 bytes)
      5'd8:  containerheader = 32'h58;       // Offset to data (88 = 22 words × 4)
      5'd9:  containerheader = 32'b0;        // Reserved
      // Words 10-21: Object 1 & 2 ancillary data spec in container header
      5'd10: containerheader = 32'b0;
      5'd11: containerheader = 32'b0;
      // ... (all zeros for words 10-21)
      5'd21: containerheader = 32'b0;
    endcase
  end
endfunction
```

The container header describes the structure of the container: how many objects, their types, sizes, and offsets.

### Frame Header Function

```verilog
function [31:0] frameheader;
  input [2:0] index;     // Which word (0-5) of the 6-word Fibre Channel header
  begin
    case(index)
      3'd0: frameheader = {8'h44, 24'b0};
          // Word 0: R_CTL field (0x44 = unsolicited data) + D_ID (destination = 0)
      3'd1: frameheader = 32'b0;
          // Word 1: CS_CTL/S_ID — source identifier (0)
      3'd2: if (lastframe == 1'b1)
              frameheader = {8'h61, 24'h380000};
              // TYPE (0x61 = ADVB) + F_CTL (0x38 = last frame of sequence)
            else
              frameheader = {8'h61, 24'h300000};
              // TYPE (0x61 = ADVB) + F_CTL (0x30 = not last frame)
      3'd3: frameheader = {containercount[7:0], 8'b0, seq_count};
          // Word 3: SEQ_ID (container count) + SEQ_CNT (frame sequence counter)
      3'd4: frameheader = {16'hFFFF, 16'hFFFF};
          // Word 4: OX_ID/RX_ID — exchange IDs (0xFFFF = broadcast/unassigned)
      3'd5: frameheader = 32'b0;
          // Word 5: Parameter — relative offset (0 for simplicity)
    endcase
  end
endfunction
```

This is a standard **Fibre Channel frame header**. The TYPE byte `0x61` identifies these as ARINC 818/ADVB frames. The `lastframe` flag controls whether `F_CTL` indicates "last frame of sequence" (`0x38`) or "not last" (`0x30`).

### State Machine — Next State Logic (Combinational)

```verilog
always @(state, lineindex, object0, idlecounter, wordcounter, txfifo_empty)
begin
  nextstate = state;    // Default: stay in current state

  case(state)
    IDLE:
    begin
      // During vertical blank (before active), wait
      if (lineindex < VERTICALBLANKPRE)
        nextstate = IDLE;
      // After last Object-2 frame, wait for vertical blank post lines
      else if (object0 == 1'b1 && lineindex < (VERTICALBLANKPOST + VERTICALBLANKPRE + 1))
        nextstate = IDLE;
      else begin
        // Wait for 6 idle cycles AND FIFO has data
        if (idlecounter == 3'b101 && txfifo_empty == 1'b0)
          nextstate = SENDSOF;    // Start a new ADVB frame
        else
          nextstate = IDLE;
      end
    end

    SENDSOF:
      nextstate = SENDHDR;        // After SOF, always send header

    SENDHDR:
    begin
      if (wordcounter == (HEADERWORDS - 1)) begin
        if (object0 == 1'b0)
          nextstate = SENDCONT;   // Object 0: send container header next
        else
          nextstate = SENDPAYLOAD; // Object 2: send pixel payload next
      end
      else
        nextstate = SENDHDR;      // Continue sending header words
    end

    SENDPAYLOAD:
    begin
      if (wordcounter == (PAYLOADWORDS - 1))
        nextstate = SENDCRC;      // Done with payload, send CRC
      else
        nextstate = SENDPAYLOAD;  // Continue sending payload
    end

    SENDCONT:
    begin
      if (wordcounter == (CONTAINERWORDS - 1))
        nextstate = SENDANC;      // Done with container, send ancillary
      else
        nextstate = SENDCONT;
    end

    SENDANC:
    begin
      if (wordcounter == (ANCILLARYWORDS - 1))
        nextstate = SENDCRC;      // Done with ancillary, send CRC
      else
        nextstate = SENDANC;
    end

    SENDCRC:
      nextstate = SENDEOF;        // After CRC, always send EOF

    SENDEOF:
      nextstate = IDLE;           // After EOF, return to idle

    default:
      nextstate = IDLE;
  endcase
end
```

### Frame Sequence per Video Line

For each video line, the transmitter sends this sequence:

```
Line N:
  ┌─ Object 0 (container header frame) ──────────────────────┐
  │ IDLE × 6 → SOFi → HDR(6) → CONT(22) → ANC(4) → CRC → EOF │
  └───────────────────────────────────────────────────────────┘
  ┌─ Object 2, subframe 0 (pixel data, first half) ──────────┐
  │ IDLE × 6 → SOFn → HDR(6) → PAYLOAD(384) → CRC → EOF      │
  └───────────────────────────────────────────────────────────┘
  ┌─ Object 2, subframe 1 (pixel data, second half) ─────────┐
  │ IDLE × 6 → SOFn → HDR(6) → PAYLOAD(384) → CRC → EOF      │
  └───────────────────────────────────────────────────────────┘
```

Wait — let me clarify. Looking at the code more carefully:

1. **First frame (object0=0)**: SOFi → Header → Container(22) → Ancillary(4) → CRC → EOFn. This sets `object0 = 1`.
2. **Subsequent frames (object0=1)**: SOFn → Header → Payload(384) → CRC → EOFn/EOFt. The `sublineindex` tracks which sub-frame of the current line we're on.

For `FRAMESPERLINE = 2`, each line has 2 payload frames (sublineindex 0 and 1). After both sub-frames are sent, `lineindex` advances.

### State Machine — Sequential Logic (Clocked)

This is the main registered logic. Let me explain each state:

#### IDLE State
```verilog
IDLE:
begin
  advb_txdata <= IDLEWORD;    // Output idle pattern
  txcharisk   <= 4'b0001;     // Byte 0 is K-character
  txfifo_rd   <= 1'b0;        // Don't read from FIFO

  if (lineindex < VERTICALBLANKPRE) begin
    // Count through vertical blank pre period
    if (count == 11'd1024) begin
      lineindex <= lineindex + 1;
      count     <= 0;
    end else begin
      count <= count + 1;
    end
  end else begin
    // Count idle cycles (need 6 before starting a new frame)
    if (idlecounter == 3'b110)
      idlecounter <= 0;
    else
      idlecounter <= idlecounter + 1;
  end
end
```

During idle, the transmitter outputs the idle word (`0xB5B595BC`). If in vertical blank, it counts up to simulate the blank period. Otherwise, it counts idle cycles until it reaches 6 (at which point the next-state logic triggers `SENDSOF`).

#### SENDSOF State
```verilog
SENDSOF:
begin
  txcharisk <= 4'b0001;       // K-char in byte 0

  if (object0 == 1'b0)
    seq_count <= 0;           // Reset sequence counter for Object 0
  else
    seq_count <= seq_count + 1; // Increment for Object 2 frames

  if (object0 == 1'b0) begin
    advb_txdata <= SOFIWORD;  // SOFi — first frame of container
    object0     <= 1'b1;      // Mark that Object 0 has been sent
  end else
    advb_txdata <= SOFNWORD;  // SOFn — subsequent frames
end
```

The SOF type depends on whether this is the first frame of the container (SOFi) or a subsequent data frame (SOFn).

#### SENDHDR State
```verilog
SENDHDR:
begin
  advb_txdata <= frameheader(wordcounter);  // Output header word
  txcharisk   <= 4'b0000;                   // All data bytes (no K-chars)

  if (wordcounter == (HEADERWORDS - 1)) begin
    wordcounter <= 0;                       // Reset for next phase
    if (object0 == 1'b1)
      txfifo_rd <= 1'b1;                    // Pre-fetch first FIFO word for payload
  end else
    wordcounter <= wordcounter + 1;
end
```

Sends 6 header words. At the end, if this is an Object-2 frame, it starts reading from the FIFO one cycle early (pipeline pre-fetch).

#### SENDPAYLOAD State
```verilog
SENDPAYLOAD:
begin
  advb_txdata <= txfifo_rdata;              // Output FIFO data directly
  txcharisk   <= 4'b0000;                   // All data bytes
  crcaccum    <= nextcrc32(crcaccum, txfifo_rdata); // Update CRC with payload data
  wordcounter <= wordcounter + 1;

  if (wordcounter >= (PAYLOADWORDS - 1))
    txfifo_rd <= 1'b0;                      // Stop reading — last word
  else
    txfifo_rd <= 1'b1;                      // Keep reading
end
```

This is where the actual pixel data flows through. The data comes directly from `txfifo_rdata` (the FIFO's read port). The CRC accumulator is updated with each word. FIFO reads continue until we've output all 384 payload words.

#### SENDCONT State
```verilog
SENDCONT:
begin
  advb_txdata <= containerheader(wordcounter);
  txcharisk   <= 4'b0000;
  crcaccum    <= nextcrc32(crcaccum, containerheader(wordcounter));
  // CRC covers the container header too!

  if (wordcounter == (CONTAINERWORDS - 1))
    wordcounter <= 0;
  else
    wordcounter <= wordcounter + 1;
end
```

Sends all 22 container header words, updating CRC with each.

#### SENDANC State
```verilog
SENDANC:
begin
  advb_txdata <= ancillaryheader(wordcounter);
  txcharisk   <= 4'b0000;
  crcaccum    <= nextcrc32(crcaccum, ancillaryheader(wordcounter));

  if (wordcounter == (ANCILLARYWORDS - 1))
    wordcounter <= 0;
  else
    wordcounter <= wordcounter + 1;
end
```

Sends 4 ancillary words (resolution + format info), updating CRC.

#### SENDCRC State
```verilog
SENDCRC:
begin
  advb_txdata <= crcaccum;    // Output the final CRC-32 value
  txcharisk   <= 4'b0000;     // Data bytes
end
```

Simply outputs the accumulated CRC word. The CRC is NOT inverted here (some protocols invert; this one doesn't based on the code).

#### SENDEOF State
```verilog
SENDEOF:
begin
  txcharisk <= 4'b0001;       // K-char in byte 0

  if ((lineindex == VACTIVE + VERTICALBLANKPRE + VERTICALBLANKPOST - 1)
      && (sublineindex == FRAMESPERLINE - 1))
  begin
    // LAST frame of the ENTIRE image!
    advb_txdata    <= EOFTWORD;        // EOFt = End of Frame Terminate
    containercount <= containercount + 1; // Next image gets a new container number
    sublineindex   <= 0;
    lineindex      <= 0;               // Reset to line 0
    object0        <= 1'b0;            // Next frame will be Object 0 again
    lastframe      <= 1'b1;            // Flag for frame header
  end
  else if (sublineindex < (FRAMESPERLINE - 1))
  begin
    // More sub-frames remain for this line
    advb_txdata  <= EOFNWORD;          // EOFn = normal end
    sublineindex <= sublineindex + 1;   // Move to next sub-frame
  end
  else
  begin
    // All sub-frames for this line done, move to next line
    advb_txdata  <= EOFNWORD;
    sublineindex <= 0;
    lineindex    <= lineindex + 1;
  end
end
```

The EOF logic handles three cases:
1. **Last frame of the entire video**: Send EOFt, reset everything for next image
2. **More sub-frames for this line**: Send EOFn, increment sub-frame counter
3. **Line complete**: Send EOFn, advance to next line

---

## 5. Module 3: image_pattern_gen (Image Generator)

### Purpose
Generates a 1024×768 image with 8 vertical color bars, packs the RGB24 pixel data into 32-bit words, and writes them into the FIFO.

### The Pixel Packing Problem

Each pixel is 3 bytes (R, G, B) but the FIFO is 32 bits (4 bytes) wide. This means pixels don't align to word boundaries:

```
Byte stream: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3 R4 G4 B4 ...
             |←─ Word 0 ──→| |←─ Word 1 ──→| |←─ Word 2 ──→|

Word 0: {R0, G0, B0, R1}    ← pixel 0 + start of pixel 1
Word 1: {G1, B1, R2, G2}    ← end of pixel 1 + start of pixel 2
Word 2: {B2, R3, G3, B3}    ← end of pixel 2 + pixel 3
```

Every **4 pixels** (12 bytes) pack perfectly into **3 words** (12 bytes). Since 1024 pixels = 256 groups of 4, we get exactly 256 × 3 = **768 words per line**. No leftover bytes!

### Color Bar Generation

```verilog
function [23:0] get_bar_color;
  input [10:0] x_pos;
  reg [2:0] bar_index;
  begin
    bar_index = x_pos[10:7];    // = x_pos / 128 (since each bar is 128 pixels wide)
    case (bar_index)
      3'd0: get_bar_color = 24'hFFFFFF;  // White
      3'd1: get_bar_color = 24'hFFFF00;  // Yellow
      3'd2: get_bar_color = 24'h00FFFF;  // Cyan
      3'd3: get_bar_color = 24'h00FF00;  // Green
      3'd4: get_bar_color = 24'hFF00FF;  // Magenta
      3'd5: get_bar_color = 24'hFF0000;  // Red
      3'd6: get_bar_color = 24'h0000FF;  // Blue
      3'd7: get_bar_color = 24'h000000;  // Black
    endcase
  end
endfunction
```

The trick here is `x_pos[10:7]` — this extracts bits 10:7, which is equivalent to dividing by 128 (since 128 = 2^7). For x = 0..127 → bar 0 (White), x = 128..255 → bar 1 (Yellow), etc. No division hardware needed!

These 8 colors are the standard **SMPTE/EBU color bars** used universally in broadcast video testing. They cover all combinations of full-on/full-off for R, G, B.

### State Machine

The generator uses a simple 4-state FSM:

```
S_IDLE → S_LOAD → S_OUTPUT → (back to S_LOAD or S_LINE_DONE)
                       ↑ stall if wfull
                       
S_LINE_DONE → S_IDLE (advance line)
```

#### S_IDLE
```verilog
S_IDLE: begin
  group_base <= 11'd0;        // Start at pixel 0
  pack_phase <= 2'd0;         // Start at word 0 of the 3-word cycle
  word_count <= 10'd0;        // Reset word counter for this line

  if (line_y == 10'd0)
    frame_sync <= 1'b1;       // Pulse at start of new frame

  gen_state <= S_LOAD;        // Move to load state
end
```

Resets per-line counters and emits a `frame_sync` pulse at the start of each video frame.

#### S_LOAD
```verilog
S_LOAD: begin
  // Latch RGB values for 4 pixels using the color lookup function
  p0_r <= color_p0[23:16]; p0_g <= color_p0[15:8]; p0_b <= color_p0[7:0];
  p1_r <= color_p1[23:16]; p1_g <= color_p1[15:8]; p1_b <= color_p1[7:0];
  p2_r <= color_p2[23:16]; p2_g <= color_p2[15:8]; p2_b <= color_p2[7:0];
  p3_r <= color_p3[23:16]; p3_g <= color_p3[15:8]; p3_b <= color_p3[7:0];
  pack_phase <= 2'd0;
  gen_state  <= S_OUTPUT;
end
```

Looks up the color for 4 consecutive pixels and latches their R, G, B bytes. The colors are determined by `group_base` through `group_base + 3`.

#### S_OUTPUT
```verilog
S_OUTPUT: begin
  if (wfull) begin
    winc <= 1'b0;             // FIFO full — stall, don't write
  end
  else begin
    winc <= 1'b1;             // Write to FIFO

    case (pack_phase)
      2'd0: begin
        wdata <= {p0_r, p0_g, p0_b, p1_r};   // Word 0 of 3
        pack_phase <= 2'd1;
      end
      2'd1: begin
        wdata <= {p1_g, p1_b, p2_r, p2_g};   // Word 1 of 3
        pack_phase <= 2'd2;
      end
      2'd2: begin
        wdata <= {p2_b, p3_r, p3_g, p3_b};   // Word 2 of 3
        word_count <= word_count + 10'd3;

        if ((word_count + 10'd3) >= WORDS_PER_LINE)
          gen_state <= S_LINE_DONE;           // Line complete!
        else begin
          group_base <= group_base + 11'd4;   // Next 4 pixels
          gen_state  <= S_LOAD;               // Load their colors
        end
      end
    endcase
  end
end
```

This is the core packing logic. It outputs 3 words per group of 4 pixels, handling FIFO backpressure by stalling when `wfull` is asserted.

#### S_LINE_DONE
```verilog
S_LINE_DONE: begin
  group_base <= 11'd0;        // Reset to pixel 0 for next line
  word_count <= 10'd0;

  if (line_y == (VACTIVE - 1))
    line_y <= 10'd0;          // Wrap to start of next frame
  else
    line_y <= line_y + 10'd1; // Advance to next line

  gen_state <= S_IDLE;
end
```

After completing a full line (768 words), advance to the next line or wrap around for the next frame.

### Why This Pattern Was Chosen

1. **No memory required**: Color bars are generated procedurally from the X coordinate. No frame buffer or ROM needed.
2. **Perfect alignment**: 1024 pixels / 8 bars = 128 pixels/bar = 2^7, so bar selection is just bit extraction.
3. **Standard test pattern**: The 8-bar pattern is universally recognized in video engineering and immediately tells you if the channel is working correctly.
4. **Easy to verify**: Each bar has a unique, easily recognizable RGB value. If you see the wrong color in the wrong position, you know exactly what went wrong.

---

## 6. Module 4: arinc818_top (Top-Level Wrapper)

### Purpose
Wires together the three main modules:

```verilog
image_pattern_gen  ──[wdata, winc, wfull]──>  asyncfifo  ──[rdata, rinc, rempty]──>  advbtx
```

### Key Connections

| Signal | From | To | Purpose |
|--------|------|----|---------|
| `fifo_wdata` | image_pattern_gen.wdata | asyncfifo.wdata | 32-bit packed pixel data |
| `fifo_winc` | image_pattern_gen.winc | asyncfifo.winc | Write enable |
| `fifo_wfull` | asyncfifo.wfull | image_pattern_gen.wfull | Backpressure (stall) |
| `fifo_rdata` | asyncfifo.rdata | advbtx.txfifo_rdata | Pixel data to transmitter |
| `fifo_rinc` | advbtx.txfifo_rd | asyncfifo.rinc | Read enable |
| `fifo_rempty` | asyncfifo.rempty | advbtx.txfifo_empty | No data available |

### Clock Domains

- `wclk` → image_pattern_gen, asyncfifo write side
- `rclk` → advbtx, asyncfifo read side
- `rst_n` → shared active-low reset for all modules

---

## 7. Module 5: tb_arinc818 (Testbench)

### Purpose
Provides a simulation environment with two clocks and monitors the transmitter output.

### Clock Configuration
```verilog
parameter CLK_WR_PERIOD = 10;   // 100 MHz write clock
parameter CLK_RD_PERIOD = 6;    // ~166 MHz read clock
```

The read clock is faster than the write clock. This ensures the transmitter can drain the FIFO faster than the image generator fills it, which is the typical operating condition (you don't want the FIFO to overflow).

### Monitors

The testbench monitors the transmitter output (`advb_txdata`) for:

1. **SOFi/SOFn ordered sets**: Indicates the start of each ADVB frame. SOFi marks the first frame of a container (Object 0). SOFn marks subsequent frames (Object 2).

2. **EOFn/EOFt ordered sets**: EOFn is a normal end-of-frame. EOFt indicates the very last frame of the entire video image.

3. **Payload data**: The first 12 words of data output are printed so you can verify the pixel data is correct (should match the color bar RGB values).

4. **Frame sync**: A pulse from the image generator at the start of each new video frame.

### Expected Output

When you run the simulation, you should see:
```
[time] *** FRAME SYNC — New image frame started ***
[time] SOFi detected (container start) — SOF #1
[time] Payload word 0: 0xXXXXXXXX    ← container header data
...
[time] EOFn detected — EOF #1
[time] SOFn detected (data frame) — SOF #2
[time] Payload word N: 0xFFFFFFFF    ← White bar pixel data (R=FF, G=FF, B=FF)
...
```

### VCD Waveform Dump
```verilog
$dumpfile("arinc818_tb.vcd");
$dumpvars(0, tb_arinc818);
```

This creates a Value Change Dump file that you can open in **GTKWave** or any waveform viewer to visually inspect all signals.

---

## Summary: Data Flow Through the System

```
1. image_pattern_gen determines the color for pixels 0,1,2,3 from the X coordinate
2. It packs them: Word0={R0,G0,B0,R1}, Word1={G1,B1,R2,G2}, Word2={B2,R3,G3,B3}
3. Each word is written to the async FIFO (if not full)
4. Steps 1-3 repeat for all 256 groups, producing 768 words per line
5. After 768 lines, a complete frame is done; repeat for next frame

6. Meanwhile, advbtx reads from the FIFO:
   a. First, it sends Object 0: SOFi → Header → Container(22) → Ancillary(4) → CRC → EOF
   b. Then, for each sub-frame: SOFn → Header → Payload(384 words from FIFO) → CRC → EOF
   c. 2 sub-frames per line × 768 lines = 1536 ADVB payload frames per video frame
   d. After the last frame of the last line, it sends EOFt instead of EOFn
   e. Container count increments, and the cycle repeats
```
