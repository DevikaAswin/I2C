`timescale 1ns/1ps

module i2c_master_final #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer I2C_HZ = 100_000
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    input  wire        op_read,       // 0=WRITE, 1=READ
    input  wire [1:0]  reg_addr,
    input  wire [15:0] wr_word,

    output reg  [15:0] rd_word,
    output reg         busy,
    output reg         nack_abort,

    inout  wire        sda,
    inout  wire        scl
);

    // ------------------------------------------------------------
    // Open-drain drivers
    // ------------------------------------------------------------
    reg sda_oe, scl_oe;
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    wire sda_in = sda;

    // ------------------------------------------------------------
    // Clock divider: one tick per half SCL
    // ------------------------------------------------------------
    localparam integer DIV_HALF = CLK_HZ / (I2C_HZ * 2);
    localparam integer DIVW = (DIV_HALF <= 1) ? 1 : $clog2(DIV_HALF);
    reg [DIVW-1:0] divcnt;
    reg tick;

    always @(posedge clk) begin
        if (!rst_n) begin
            divcnt <= 0;
            tick   <= 1'b0;
        end else if (divcnt == DIV_HALF-1) begin
            divcnt <= 0;
            tick   <= 1'b1;
        end else begin
            divcnt <= divcnt + 1;
            tick   <= 1'b0;
        end
    end

    // ------------------------------------------------------------
    // SCL phase tracking
    // ------------------------------------------------------------
    reg scl_high; // 1 = released high, 0 = driven low

    // ------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------
    localparam [6:0] DEV_ADDR = 7'b1001000;

    // ------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------
    typedef enum logic [5:0] {
        IDLE,

        START_0, START_1, START_2,

        SEND_BIT,
        ACK_PREP,
        ACK_SAMPLE,

        READ_BIT,
        MACK_PREP,
        MACK_SAMPLE,

        RPT_0, RPT_1, RPT_2,

        STOP_0, STOP_1, STOP_2,

        DONE,
        ABORT
    } state_t;

    state_t state;

    reg [7:0] tx;
    reg [7:0] rx;
    reg [2:0] bitn;
    reg [2:0] seq;
    reg       do_read;
    reg       reading_msb;

    // ------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------
    task scl_release; begin scl_oe <= 0; scl_high <= 1; end endtask
    task scl_pull;    begin scl_oe <= 1; scl_high <= 0; end endtask

    // ------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= IDLE;
            sda_oe     <= 0;
            scl_oe     <= 0;
            scl_high   <= 1;
            busy       <= 0;
            nack_abort <= 0;
            rd_word    <= 0;
            bitn       <= 7;
            seq        <= 0;
            do_read    <= 0;
            reading_msb<= 1;
        end else if (tick) begin
            case (state)

            // ------------------------------------------------
            IDLE: begin
                sda_oe <= 0; scl_release();
                busy   <= 0; nack_abort <= 0;
                if (start) begin
                    busy <= 1;
                    do_read <= op_read;
                    seq <= 0;
                    state <= START_0;
                end
            end

            // ------------------------------------------------
            // START
            START_0: begin scl_release(); sda_oe <= 0; state <= START_1; end
            START_1: begin scl_release(); sda_oe <= 1; state <= START_2; end
            START_2: begin
                scl_pull();
                tx <= {DEV_ADDR, 1'b0};
                bitn <= 7;
                state <= SEND_BIT;
            end

            // ------------------------------------------------
            // SEND BYTE
            SEND_BIT: begin
                if (!scl_high) begin
                    scl_release();
                end else begin
                    scl_pull();
                    if (tx[bitn] == 0) sda_oe <= 1; else sda_oe <= 0;
                    if (bitn == 0) state <= ACK_PREP;
                    else bitn <= bitn - 1;
                end
            end

            // ------------------------------------------------
            // ACK (two states, safe)
            ACK_PREP: begin scl_pull(); sda_oe <= 0; state <= ACK_SAMPLE; end
            ACK_SAMPLE: begin
                if (!scl_high) begin
                    scl_release();
                    if (sda_in) begin
                        nack_abort <= 1;
                        state <= ABORT;
                    end else begin
                        if (!do_read) begin
                            if (seq == 0) begin tx <= {6'b0, reg_addr}; seq <= 1; end
                            else if (seq == 1) begin tx <= wr_word[15:8]; seq <= 2; end
                            else if (seq == 2) begin tx <= wr_word[7:0]; seq <= 3; end
                            else state <= STOP_0;
                            bitn <= 7;
                            if (state != STOP_0) state <= SEND_BIT;
                        end else begin
                            if (seq == 0) begin tx <= {6'b0, reg_addr}; seq <= 1; bitn<=7; state<=SEND_BIT; end
                            else if (seq == 1) state <= RPT_0;
                            else if (seq == 2) begin rx<=0; bitn<=7; reading_msb<=1; state<=READ_BIT; end
                        end
                    end
                end else scl_pull();
            end

            // ------------------------------------------------
            // REPEATED PULSE (EXACT)
            RPT_0: begin scl_release(); sda_oe<=0; state<=RPT_1; end
            RPT_1: begin scl_release(); sda_oe<=1; state<=RPT_2; end
            RPT_2: begin
                scl_release(); sda_oe<=0;
                tx <= {DEV_ADDR,1'b1};
                bitn<=7; seq<=2;
                state<=SEND_BIT;
            end

            // ------------------------------------------------
            // READ
            READ_BIT: begin
                if (!scl_high) begin scl_release(); rx[bitn]<=sda_in; end
                else begin
                    scl_pull();
                    if (bitn==0) begin
                        if (reading_msb) rd_word[15:8]<=rx;
                        else rd_word[7:0]<=rx;
                        state<=MACK_PREP;
                    end else bitn<=bitn-1;
                end
            end

            MACK_PREP: begin scl_pull(); sda_oe<=1; state<=MACK_SAMPLE; end
            MACK_SAMPLE: begin
                if (!scl_high) begin
                    scl_release();
                    if (reading_msb) begin
                        reading_msb<=0; rx<=0; bitn<=7; state<=READ_BIT;
                    end else state<=STOP_0;
                end else scl_pull();
            end

            // ------------------------------------------------
            // STOP
            STOP_0: begin scl_release(); sda_oe<=0; state<=STOP_1; end
            STOP_1: begin scl_release(); sda_oe<=1; state<=STOP_2; end
            STOP_2: begin scl_release(); sda_oe<=0; state<=DONE;  end

            DONE: begin sda_oe<=0; scl_release(); busy<=0; if(!start) state<=IDLE; end
            ABORT: begin sda_oe<=0; scl_release(); busy<=0; if(!start) state<=IDLE; end

            endcase
        end
    end

endmodule
`timescale 1ns/1ps

module tb_i2c_master_final;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0;

    wire sda, scl;
    pullup(sda); pullup(scl);

    reg start=0, op_read=0;
    reg [1:0] reg_addr=0;
    reg [15:0] wr_word=16'hABCD;

    wire [15:0] rd_word;
    wire busy, nack_abort;

    i2c_master_final dut(
        .clk(clk), .rst_n(rst_n),
        .start(start), .op_read(op_read),
        .reg_addr(reg_addr), .wr_word(wr_word),
        .rd_word(rd_word),
        .busy(busy), .nack_abort(nack_abort),
        .sda(sda), .scl(scl)
    );

    // ------------------------------------------------------------
    // Dummy slave
    // ------------------------------------------------------------
    reg sda_oe;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    reg [15:0] mem[0:3];
    initial begin
        mem[0]=16'h1111;
        mem[1]=16'h2222;
        mem[2]=16'h3333;
        mem[3]=16'h4444;
    end

    reg [7:0] shift;
    reg [2:0] bitn;
    reg [1:0] cur_reg;
    reg rw, in_xfer;

    always @(negedge sda) if (scl) begin in_xfer<=1; bitn<=7; end
    always @(posedge sda) if (scl) begin in_xfer<=0; sda_oe<=0; end

    always @(posedge scl) if (in_xfer) begin
        shift[bitn]<=sda;
        if (bitn==0) bitn<=7; else bitn<=bitn-1;
    end

    always @(negedge scl) if (in_xfer) begin
        if (bitn==7) begin
            sda_oe<=0; // ACK=0
            if (shift[7:1]==7'b1001000) rw<=shift[0];
            else rw<=0;
        end else sda_oe<=0;
    end

    initial begin
        $dumpfile("i2c_final.vcd");
        $dumpvars(0,tb_i2c_master_final);

        #200 rst_n=1;

        // WRITE
        reg_addr=2; wr_word=16'hBEEF; op_read=0;
        #100 start=1; #10 start=0;
        wait(!busy);

        // READ
        op_read=1;
        #100 start=1; #10 start=0;
        wait(!busy);

        $finish;
    end

endmodule

