//=============================================================================
// advbtx.v
//
// ARINC 818 (ADVB) Transmitter
// Reads pixel data from an async FIFO and constructs ADVB frames:
//   SOF -> Header(6) -> Container/Ancillary OR Payload -> CRC -> EOF
//
// Object 0 = Container header + Ancillary data (first frame per line)
// Object 2 = Pixel payload data (subsequent frames per line)
//
// FIXES applied:
//   1. CRC accumulator is reset to 0xFFFFFFFF at start of each new frame
//      (in SENDSOF state), not just on global reset.
//   2. SENDHDR begin/end braces clarified for correct nesting.
//   3. lastframe cleared at start of new container (was sticky).
//=============================================================================

module advbtx #(
  parameter HACTIVE             = 1024,  
  parameter VACTIVE             = 768,
  parameter BytesPerPixel       = 3,
  parameter FRAMERATE           = 60,
  parameter VERTICALBLANKPRE    = 0,
  parameter VERTICALBLANKPOST   = 0,
  parameter HORIZONTALIDLECYCLES = 6
)(
  input             clk,
  input             rst,
  input             txfifo_empty,
  output reg [31:0] advb_txdata,
  output reg [3:0]  txcharisk,
  output reg        txfifo_rd,
  input      [31:0] txfifo_rdata
);

//=========================================================================
// Derived Parameters
//=========================================================================
localparam PAYLOADBYTESPERFRAME = 2112;	
localparam BYTESPERLINE   = (HACTIVE * BytesPerPixel);               // 3072
localparam WORDSPERLINE   = (BYTESPERLINE / 4);                      // 768
localparam HEADERWORDS    = 6;
localparam CONTAINERWORDS = 22;
localparam ANCILLARYWORDS = 4;
// Ceiling division: how many ADVB frames needed per video line
localparam FRAMESPERLINE  = (BYTESPERLINE + PAYLOADBYTESPERFRAME - 1) / PAYLOADBYTESPERFRAME;
                            // = (3072 + 2112 - 1) / 2112 = 5183/2112 = 2
localparam PAYLOADWORDS   = (WORDSPERLINE / FRAMESPERLINE);          // 384
localparam TOTALOBJ2FRAMES = VACTIVE * FRAMESPERLINE;
localparam VTOTAL         = VACTIVE + VERTICALBLANKPRE + VERTICALBLANKPOST;

//=========================================================================
// State Encoding
//=========================================================================
localparam IDLE        = 4'd0;
localparam SENDSOF     = 4'd1;
localparam SENDHDR     = 4'd2;
localparam SENDCONT    = 4'd3;
localparam SENDANC     = 4'd4;
localparam SENDPAYLOAD = 4'd5;
localparam SENDCRC     = 4'd6;
localparam SENDEOF     = 4'd7;

//=========================================================================
// Ordered Set Constants (8b/10b K-character patterns)
//=========================================================================
localparam IDLEWORD = 32'hB5B595BC;
localparam SOFIWORD = 32'h5757BFBC;
localparam SOFNWORD = 32'h4F4FB7BC;
localparam EOFNWORD = 32'h4E4E95BC;
localparam EOFTWORD = 32'h4B4B95BC;

//=========================================================================
// Internal signals
//=========================================================================
wire [13:0] VACT = VACTIVE;
wire [13:0] HACT = HACTIVE; 	
wire [7:0]  sequenceid;	

reg [4:0]  state;
reg [4:0]  nextstate;
reg [31:0] containercount;
reg [15:0] seq_count;
reg [15:0] lineindex;
reg [1:0]  sublineindex;
reg [2:0]  idlecounter;
reg [31:0] crcaccum;
reg [9:0]  wordcounter;
reg [10:0] count;
reg        lastframe;
reg        object0;
	
assign sequenceid = containercount[7:0];

//=========================================================================
// CRC-32 Function (polynomial 0x04C11DB7, same as Fibre Channel)
//=========================================================================
function [31:0] nextcrc32;
  input [31:0] crc;
  input [31:0] data;
  integer p;
  reg [31:0] c;
  begin 
    c = crc;
    for (p = 0; p < 32; p = p + 1) begin
      c = (c[31] ^ data[31-p]) ? ((c << 1) ^ 32'h04C11DB7) : (c << 1);
      nextcrc32 = c;
    end
  end 
endfunction

//=========================================================================
// Ancillary Header (4 words: resolution + format info)
//=========================================================================
function [31:0] ancillaryheader;
  input [2:0] index;
  begin
    case(index)
      3'd0    : ancillaryheader = {VACT, HACT, 4'b0000};
      3'd1    : ancillaryheader = 32'h10008888;
      3'd2    : ancillaryheader = 32'b0;
      3'd3    : ancillaryheader = 32'b0;
      default : ancillaryheader = 32'b0;
    endcase
  end 
endfunction

//=========================================================================
// Container Header (22 words: container metadata)
//=========================================================================
function [31:0] containerheader;
  input [4:0] index;
  begin
    case(index)
      5'd0  : containerheader = containercount;
      5'd1  : containerheader = 32'h0;
      5'd2  : containerheader = 32'h0;
      5'd3  : containerheader = 32'h0;
      5'd4  : containerheader = 32'h10000;
      5'd5  : containerheader = 32'h00040000;
      5'd6  : containerheader = {8'h50, 8'b0, 16'hd000};
      5'd7  : containerheader = 32'h10;
      5'd8  : containerheader = 32'h58;
      5'd9  : containerheader = 32'b0;
      5'd10 : containerheader = 32'b0;  // Object 1&2 ancillary data spec
      5'd11 : containerheader = 32'b0;
      5'd12 : containerheader = 32'b0;
      5'd13 : containerheader = 32'b0;
      5'd14 : containerheader = 32'b0;
      5'd15 : containerheader = 32'b0;
      5'd16 : containerheader = 32'b0;
      5'd17 : containerheader = 32'b0;
      5'd18 : containerheader = 32'b0;
      5'd19 : containerheader = 32'b0;
      5'd20 : containerheader = 32'b0;
      5'd21 : containerheader = 32'b0;
      default: containerheader = 32'b0;
    endcase
  end
endfunction            

//=========================================================================
// Frame Header (6 words: Fibre Channel header fields)
//=========================================================================
function [31:0] frameheader;
  input [2:0] index;
  begin
    case(index)
      3'd0 : frameheader = {8'h44, 24'b0};
      3'd1 : frameheader = 32'b0;
      3'd2 : if (lastframe == 1'b1)
               frameheader = {8'h61, 24'h380000};
             else
               frameheader = {8'h61, 24'h300000};
      3'd3 : frameheader = {containercount[7:0], 8'b0, seq_count};
      3'd4 : frameheader = {16'hFFFF, 16'hFFFF};
      3'd5 : frameheader = 32'b0;
      default : frameheader = 32'b0;
    endcase
  end
endfunction

//=========================================================================
// State Register (asynchronous active-low reset)
//=========================================================================
always @(posedge clk or negedge rst) begin
  if (!rst)
    state <= IDLE;
  else
    state <= nextstate;
end 

//=========================================================================
// Next-State Logic (combinational)
//=========================================================================
always @(state, lineindex, object0, idlecounter, wordcounter, txfifo_empty) begin
  nextstate = state;
  case(state)
    IDLE: begin
      if (lineindex < VERTICALBLANKPRE)
        nextstate = IDLE;
      else if (object0 == 1'b1 && lineindex < (VERTICALBLANKPOST + VERTICALBLANKPRE + 1)) 
        nextstate = IDLE;
      else begin
        if (idlecounter == 3'b101 && txfifo_empty == 1'b0)
          nextstate = SENDSOF;
        else
          nextstate = IDLE;
      end
    end

    SENDSOF: begin 	
      nextstate = SENDHDR;
    end

    SENDHDR: begin
      if (wordcounter == (HEADERWORDS - 1)) begin
        if (object0 == 1'b0)
          nextstate = SENDCONT;
        else 
          nextstate = SENDPAYLOAD;
      end
      else 
        nextstate = SENDHDR;
    end

    SENDPAYLOAD: begin
      if (wordcounter == (PAYLOADWORDS - 1))
        nextstate = SENDCRC;
      else
        nextstate = SENDPAYLOAD;
    end

    SENDCONT: begin
      if (wordcounter == (CONTAINERWORDS - 1))
        nextstate = SENDANC;
      else 
        nextstate = SENDCONT;
    end

    SENDANC: begin
      if (wordcounter == (ANCILLARYWORDS - 1))
        nextstate = SENDCRC;
      else 
        nextstate = SENDANC;
    end

    SENDCRC:
      nextstate = SENDEOF;

    SENDEOF:
      nextstate = IDLE;

    default: 
      nextstate = IDLE;
  endcase
end

//=========================================================================
// Output & Datapath Logic (clocked)
//=========================================================================
always @(posedge clk or negedge rst) begin
  if (!rst) begin
    advb_txdata    <= IDLEWORD;
    txcharisk      <= 4'b0001;		
    wordcounter    <= 0;
    crcaccum       <= 32'hFFFFFFFF;
    containercount <= 0;
    idlecounter    <= 0;
    lineindex      <= 0;
    sublineindex   <= 0;
    count          <= 0;
    object0        <= 1'b0;
    lastframe      <= 1'b0;
    txfifo_rd      <= 1'b0;
    seq_count      <= 16'h0;
  end             
  else begin
    case(state)
      //-------------------------------------------------------------------
      // IDLE: Send idle words, count through vertical blank or idle gaps
      //-------------------------------------------------------------------
      IDLE: begin
        advb_txdata <= IDLEWORD;
        txcharisk   <= 4'b0001;
        txfifo_rd   <= 1'b0;
        if (lineindex < VERTICALBLANKPRE) begin
          if (count == 11'd1024) begin
            lineindex <= lineindex + 1;
            count     <= 0;
          end
          else begin
            lineindex <= lineindex;	
            count     <= count + 1;
          end
        end			
        else begin
          if (idlecounter == 3'b110)
            idlecounter <= 0;
          else
            idlecounter <= idlecounter + 1;				
        end
      end	

      //-------------------------------------------------------------------
      // SENDSOF: Send Start-of-Frame ordered set
      //         FIX: Reset CRC at start of every new ADVB frame
      //         FIX: Clear lastframe when starting a new container
      //-------------------------------------------------------------------
      SENDSOF: begin	
        txcharisk <= 4'b0001;
        crcaccum  <= 32'hFFFFFFFF;   // FIX: Reset CRC for this frame

        if (object0 == 1'b0) begin
          seq_count   <= 0;
          advb_txdata <= SOFIWORD;
          object0     <= 1'b1;
          lastframe   <= 1'b0;       // FIX: Clear lastframe for new container
        end
        else begin
          seq_count   <= seq_count + 1;
          advb_txdata <= SOFNWORD;
        end
      end

      //-------------------------------------------------------------------
      // SENDHDR: Send 6-word Fibre Channel frame header
      //         FIX: Corrected begin/end brace nesting
      //-------------------------------------------------------------------
      SENDHDR: begin 
        advb_txdata <= frameheader(wordcounter);
        txcharisk   <= 4'b0000;
        if (wordcounter == (HEADERWORDS - 1)) begin
          wordcounter <= 0;
          if (object0 == 1'b1)
            txfifo_rd <= 1'b1;       // Pre-fetch first FIFO word
        end
        else begin
          wordcounter <= wordcounter + 1;
        end
      end

      //-------------------------------------------------------------------
      // SENDPAYLOAD: Send pixel data from FIFO, update CRC
      //-------------------------------------------------------------------
      SENDPAYLOAD: begin  
        advb_txdata <= txfifo_rdata;
        txcharisk   <= 4'b0000;
        crcaccum    <= nextcrc32(crcaccum, txfifo_rdata);
        wordcounter <= wordcounter + 1;
        if (wordcounter >= (PAYLOADWORDS - 1))
          txfifo_rd <= 1'b0;
        else
          txfifo_rd <= 1'b1;
      end     

      //-------------------------------------------------------------------
      // SENDCONT: Send 22-word container header, update CRC
      //-------------------------------------------------------------------
      SENDCONT: begin 
        advb_txdata <= containerheader(wordcounter);
        txcharisk   <= 4'b0000;
        crcaccum    <= nextcrc32(crcaccum, containerheader(wordcounter));
        if (wordcounter == (CONTAINERWORDS - 1))
          wordcounter <= 0;
        else
          wordcounter <= wordcounter + 1;				
      end

      //-------------------------------------------------------------------
      // SENDANC: Send 4-word ancillary data, update CRC
      //-------------------------------------------------------------------
      SENDANC: begin 
        advb_txdata <= ancillaryheader(wordcounter);
        txcharisk   <= 4'b0000;				
        crcaccum    <= nextcrc32(crcaccum, ancillaryheader(wordcounter));
        if (wordcounter == (ANCILLARYWORDS - 1))
          wordcounter <= 0;
        else
          wordcounter <= wordcounter + 1;
      end

      //-------------------------------------------------------------------
      // SENDCRC: Output accumulated CRC-32
      //-------------------------------------------------------------------
      SENDCRC: begin
        advb_txdata <= crcaccum;
        txcharisk   <= 4'b0000;
      end

      //-------------------------------------------------------------------
      // SENDEOF: Send End-of-Frame, advance line/subline counters
      //-------------------------------------------------------------------
      SENDEOF: begin
        txcharisk <= 4'b0001;
        if ((lineindex == VACTIVE + VERTICALBLANKPRE + VERTICALBLANKPOST - 1) 
            && (sublineindex == FRAMESPERLINE - 1))
        begin 
          // Last frame of entire video image
          advb_txdata    <= EOFTWORD;
          containercount <= containercount + 1;
          sublineindex   <= 0;
          lineindex      <= 0;
          object0        <= 1'b0;
          lastframe      <= 1'b1;
        end
        else if (sublineindex < (FRAMESPERLINE - 1))
        begin
          // More sub-frames for this line
          advb_txdata    <= EOFNWORD;
          sublineindex   <= sublineindex + 1;
          lineindex      <= lineindex;
          containercount <= containercount;
          lastframe      <= lastframe;
        end
        else begin
          // Line complete, advance to next line
          advb_txdata    <= EOFNWORD;
          sublineindex   <= 0;
          lineindex      <= lineindex + 1;
          containercount <= containercount;
          lastframe      <= lastframe;
        end
      end

      default: begin
        advb_txdata <= IDLEWORD;
        txcharisk   <= 4'b0001;
      end	
    endcase
  end
end

endmodule
