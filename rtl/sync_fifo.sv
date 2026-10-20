// -----------------------------------------------------------------------------
// sync_fifo.sv
// Parameterized first-word-fall-through FIFO used for per-client request queues.
//
// Formal (SymbiYosys, `define FORMAL):
//   * occupancy never exceeds DEPTH, flags consistent with count
//   * FIFO ordering / data integrity: an arbitrarily chosen pushed element is
//     popped exactly after all elements ahead of it, with its data unchanged
// -----------------------------------------------------------------------------
module sync_fifo #(
  parameter int WIDTH = 32,
  parameter int DEPTH = 4            // power of two
) (
  input  logic                   clk,
  input  logic                   rst_n,

  input  logic                   push,
  input  logic [WIDTH-1:0]       din,
  output logic                   full,

  input  logic                   pop,
  output logic [WIDTH-1:0]       dout,
  output logic                   empty,

  output logic [$clog2(DEPTH):0] count
);
  localparam int          AW       = $clog2(DEPTH);
  localparam logic [AW:0] FULL_CNT = (AW+1)'(DEPTH);

  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [AW-1:0]    wptr, rptr;

`ifdef BUG_FIFO_OVERFLOW
  wire do_push = push;               // BUG: ignores full -> overwrites oldest entry
`else
  wire do_push = push && !full;
`endif
  wire do_pop  = pop && !empty;

  assign full  = (count == FULL_CNT);
  assign empty = (count == '0);
  assign dout  = mem[rptr];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wptr  <= '0;
      rptr  <= '0;
      count <= '0;
    end else begin
      if (do_push) begin
        mem[wptr] <= din;
        wptr      <= wptr + 1'b1;
      end
      if (do_pop)
        rptr <= rptr + 1'b1;
      case ({do_push, do_pop})
        2'b10:   count <= count + 1'b1;
        2'b01:   count <= count - 1'b1;
        default: count <= count;
      endcase
    end
  end

`ifdef FORMAL
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume(!rst_n);

  // ---- occupancy bounds & flag consistency -------------------------------
  always @(*) if (f_past_valid) begin
    assert(count <= DEPTH);
    assert(full  == (count == DEPTH));
    assert(empty == (count == 0));
    assert(wptr  == rptr + count[AW-1:0]);
  end

  // ---- ordering: track one arbitrary element through the queue -----------
  (* anyseq *) wire f_tag;
  reg             f_tracking;
  reg [WIDTH-1:0] f_data;
  reg [AW:0]      f_pos;          // number of entries ahead of tracked one

  always @(posedge clk) begin
    if (!rst_n) begin
      f_tracking <= 1'b0;
    end else if (!f_tracking && do_push && f_tag) begin
      f_tracking <= 1'b1;
      f_data     <= din;
      f_pos      <= count - {{AW{1'b0}}, do_pop};
    end else if (f_tracking && do_pop) begin
      if (f_pos == 0) f_tracking <= 1'b0;
      else            f_pos      <= f_pos - 1'b1;
    end
  end

  always @(*) if (f_past_valid && rst_n && f_tracking) begin
    assert(f_pos < count);
    assert(mem[rptr + f_pos[AW-1:0]] == f_data);
    if (f_pos == 0) assert(dout == f_data);
  end

  // ---- reachability ------------------------------------------------------
  always @(*) if (f_past_valid && rst_n) cover(full);
  always @(*) if (f_past_valid && rst_n) cover(f_tracking && f_pos == DEPTH-1 && do_pop);
`endif

endmodule
