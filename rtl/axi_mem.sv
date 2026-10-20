// -----------------------------------------------------------------------------
// axi_mem.sv
// Shared memory model with an AXI4 slave port. One burst at a time, fixed
// read latency (DRAM-like), byte strobes, optional pseudo-random stalls on
// every ready/valid it drives (stress mode for backpressure testing).
// -----------------------------------------------------------------------------
module axi_mem #(
  parameter int ADDR_W    = 32,
  parameter int DATA_W    = 32,
  parameter int ID_W      = 6,
  parameter int MEM_WORDS = 16384,
  parameter int READ_LAT  = 6
) (
  input  logic                clk,
  input  logic                rst_n,
  input  logic                stall_en,

  input  logic                s_arvalid,
  output logic                s_arready,
  input  logic [ADDR_W-1:0]   s_araddr,
  input  logic [7:0]          s_arlen,
  input  logic [ID_W-1:0]     s_arid,

  output logic                s_rvalid,
  input  logic                s_rready,
  output logic [DATA_W-1:0]   s_rdata,
  output logic [ID_W-1:0]     s_rid,
  output logic [1:0]          s_rresp,
  output logic                s_rlast,

  input  logic                s_awvalid,
  output logic                s_awready,
  input  logic [ADDR_W-1:0]   s_awaddr,
  input  logic [7:0]          s_awlen,
  input  logic [ID_W-1:0]     s_awid,

  input  logic                s_wvalid,
  output logic                s_wready,
  input  logic [DATA_W-1:0]   s_wdata,
  input  logic [DATA_W/8-1:0] s_wstrb,
  input  logic                s_wlast,

  output logic                s_bvalid,
  input  logic                s_bready,
  output logic [ID_W-1:0]     s_bid,
  output logic [1:0]          s_bresp
);
  localparam int WA = $clog2(MEM_WORDS);
  localparam int SB = DATA_W / 8;

  logic [DATA_W-1:0] mem [0:MEM_WORDS-1];

  // 16-bit Galois LFSR drives stall injection (~37% of cycles)
  logic [15:0] lfsr;
  wire         stall = stall_en && (lfsr[0] | lfsr[5]) && lfsr[9];
  always_ff @(posedge clk) begin
    if (!rst_n) lfsr <= 16'hACE1;
    else        lfsr <= {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0);
  end

  // ---------------- read engine ----------------
  logic          r_busy, rvalid_q;
  logic [WA-1:0] r_addr;
  logic [7:0]    r_left;
  logic [ID_W-1:0] r_id;
  logic [7:0]    r_lat;

  // ---------------- write engine ----------------
  logic          w_busy, b_pend;
  logic [WA-1:0] w_addr;
  logic [ID_W-1:0] w_id;

  wire busy = r_busy || w_busy;

  assign s_arready = !busy && !stall;
  assign s_awready = !busy && !stall && !s_arvalid;

  assign s_rvalid  = rvalid_q;
  assign s_rdata   = mem[r_addr];
  assign s_rid     = r_id;
  assign s_rresp   = 2'b00;
  assign s_rlast   = (r_left == 8'd0);

  assign s_wready  = w_busy && !b_pend && !stall;
  assign s_bvalid  = b_pend;
  assign s_bid     = w_id;
  assign s_bresp   = 2'b00;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      r_busy   <= 1'b0;
      rvalid_q <= 1'b0;
      r_addr   <= '0;
      r_left   <= '0;
      r_id     <= '0;
      r_lat    <= '0;
    end else if (!r_busy) begin
      if (s_arvalid && s_arready) begin
        r_busy <= 1'b1;
        r_addr <= s_araddr[WA+1:2];
        r_left <= s_arlen;
        r_id   <= s_arid;
        r_lat  <= 8'(READ_LAT);
      end
    end else if (!rvalid_q) begin
      if (r_lat != 0) r_lat    <= r_lat - 1'b1;
      else if (!stall) rvalid_q <= 1'b1;
    end else if (s_rready) begin
      if (r_left == 0) begin
        r_busy   <= 1'b0;
        rvalid_q <= 1'b0;
      end else begin
        r_left   <= r_left - 1'b1;
        r_addr   <= r_addr + 1'b1;
        rvalid_q <= !stall;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      w_busy <= 1'b0;
      b_pend <= 1'b0;
      w_addr <= '0;
      w_id   <= '0;
    end else if (!w_busy) begin
      if (s_awvalid && s_awready) begin
        w_busy <= 1'b1;
        w_addr <= s_awaddr[WA+1:2];
        w_id   <= s_awid;
      end
    end else if (!b_pend) begin
      if (s_wvalid && s_wready) begin
        w_addr <= w_addr + 1'b1;
        if (s_wlast) b_pend <= 1'b1;
      end
    end else if (s_bready) begin
      b_pend <= 1'b0;
      w_busy <= 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (w_busy && !b_pend && s_wvalid && s_wready)
      for (int b = 0; b < SB; b++)
        if (s_wstrb[b]) mem[w_addr][8*b +: 8] <= s_wdata[8*b +: 8];
  end

endmodule
