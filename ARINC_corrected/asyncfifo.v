//=============================================================================
// asyncfifo.v
//
// Asynchronous FIFO with Gray-code pointer crossing for safe CDC.
// 
// Parameters:
//   DATASIZE - width of each entry (default 32 bits)
//   ADDRSIZE - address bits (depth = 2^ADDRSIZE, default 512)
//
// Write side: wclk, wrstn, winc, wdata, wfull
// Read side:  rclk, rrstn, rinc, rdata, rempty
//=============================================================================

module asyncfifo #(
	parameter DATASIZE = 32,
	parameter ADDRSIZE = 9
) (
	input                   wclk,
	input                   wrstn,
	input                   winc,	
	input  [DATASIZE-1:0]   wdata,		
	input                   rclk,
	input                   rrstn,
	input                   rinc,	
	output [DATASIZE-1:0]   rdata,	
	output reg              wfull,
	output reg              rempty
);

localparam DEPTH = 1 << ADDRSIZE;

//--- Pointer registers and synchronizers ---
reg  [ADDRSIZE:0] wq1rptr, wq2rptr;    // Read ptr synced into write domain
reg  [ADDRSIZE:0] rq1wptr, rq2wptr;    // Write ptr synced into read domain

wire [ADDRSIZE-1:0] waddr, raddr;

//--- Read-side pointers ---
reg  [ADDRSIZE:0] rbin, rptr;
wire [ADDRSIZE:0] rbinnext, rgraynext;
wire              remptyval;

//--- Write-side pointers ---
reg  [ADDRSIZE:0] wbin, wptr;
wire [ADDRSIZE:0] wbinnext, wgraynext;
wire              wfullval;             // FIX: explicitly declared as wire

//--- Memory ---
reg  [DATASIZE-1:0] mem [0:DEPTH-1];

//=========================================================================
// Read-side logic
//=========================================================================
assign rbinnext  = rbin + (rinc & ~rempty);
assign rgraynext = (rbinnext >> 1) ^ rbinnext;
assign raddr     = rbin[ADDRSIZE-1:0];
assign remptyval = (rgraynext == rq2wptr);

//=========================================================================
// Write-side logic
//=========================================================================
assign wbinnext  = wbin + (winc & ~wfull);
assign wgraynext = (wbinnext >> 1) ^ wbinnext;
assign waddr     = wbin[ADDRSIZE-1:0];
assign wfullval  = (wgraynext == {~wq2rptr[ADDRSIZE:ADDRSIZE-1], wq2rptr[ADDRSIZE-2:0]});

//=========================================================================
// Asynchronous read from memory
//=========================================================================
assign rdata = mem[raddr];

//=========================================================================
// Read clock domain
//=========================================================================
always @(posedge rclk or negedge rrstn) begin
	if (!rrstn) begin
		{rbin, rptr}       <= 0;
		rempty             <= 1'b1;
		{rq2wptr, rq1wptr} <= 0;
	end
	else begin
		{rbin, rptr}       <= {rbinnext, rgraynext};
		rempty             <= remptyval;
		{rq2wptr, rq1wptr} <= {rq1wptr, wptr};	
	end
end

//=========================================================================
// Write clock domain
//=========================================================================
always @(posedge wclk or negedge wrstn) begin
	if (!wrstn) begin
		{wbin, wptr}       <= 0;
		wfull              <= 1'b0;
		{wq2rptr, wq1rptr} <= 0;	
	end
	else begin
		{wbin, wptr}       <= {wbinnext, wgraynext};
		wfull              <= wfullval;
		{wq2rptr, wq1rptr} <= {wq1rptr, rptr};
		if (winc && !wfull) 
			mem[waddr] <= wdata;		
	end
end  

endmodule
