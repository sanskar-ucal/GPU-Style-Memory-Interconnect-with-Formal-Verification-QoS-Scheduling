// -----------------------------------------------------------------------------
// qos_arbiter.sv
// Runtime-selectable arbitration policy for N requesters.
//
//   policy = 0 : ROUND-ROBIN       rotating priority, starvation-free
//   policy = 1 : WEIGHTED RR (QoS) each requester gets `weight[i]` grants per
//                                  round (credit based), starvation-free
//   policy = 2 : FIXED PRIORITY    index 0 highest; can starve low priority
//
// `grant` is combinational from `req`; arbiter state advances on `ack`.
//
// Formal (SymbiYosys, `define FORMAL):
//   * grant is one-hot or zero (no duplicate grants)
//   * grant only to an active requester; work conserving
//   * round-robin bounded wait: a continuously requesting client is passed
//     over at most N-1 times
// -----------------------------------------------------------------------------
module qos_arbiter #(
  parameter int N  = 4,
  parameter int WW = 4                    // weight / credit width
) (
  input  logic                   clk,
  input  logic                   rst_n,
  input  logic [1:0]             policy,
  input  logic [N-1:0][WW-1:0]   weight,
  input  logic [N-1:0]           req,
  input  logic                   ack,
  output logic [N-1:0]           grant,
  output logic [$clog2(N)-1:0]   grant_idx
);
  localparam int        IW        = $clog2(N);
  localparam logic [1:0] POL_RR    = 2'd0;
  localparam logic [1:0] POL_WRR   = 2'd1;
  localparam logic [1:0] POL_FIXED = 2'd2;

  logic [IW-1:0]          ptr;            // next index with highest RR priority
  logic [N-1:0][WW-1:0]   credit;
  logic [N-1:0][WW-1:0]   w_eff;
  logic [N-1:0]           has_credit, eligible, cand;

  always_comb begin
    for (int i = 0; i < N; i++) begin
      w_eff[i]      = (weight[i] == '0) ? {{(WW-1){1'b0}}, 1'b1} : weight[i];
      has_credit[i] = (credit[i] != '0);
    end
  end

  assign eligible = req & has_credit;
  assign cand     = (policy == POL_WRR && |eligible) ? eligible : req;

  // ---- rotating search starting at ptr (RR and WRR) -----------------------
  logic          rr_found;
  logic [IW-1:0] rr_idx;
  always_comb begin
    rr_found = 1'b0;
    rr_idx   = '0;
    for (int k = 0; k < N; k++) begin
      if (!rr_found && cand[(int'(ptr) + k) % N]) begin
        rr_found = 1'b1;
        rr_idx   = IW'((int'(ptr) + k) % N);
      end
    end
  end

  // ---- fixed priority: lowest index wins ----------------------------------
  logic          fx_found;
  logic [IW-1:0] fx_idx;
  always_comb begin
    fx_found = 1'b0;
    fx_idx   = '0;
    for (int i = N-1; i >= 0; i--) begin
      if (req[i]) begin
        fx_found = 1'b1;
        fx_idx   = IW'(i);
      end
    end
  end

  logic [N-1:0] rr_grant, fx_grant;
  assign rr_grant = rr_found ? (N'(1) << rr_idx) : '0;
`ifdef BUG_ARB_DOUBLE_GRANT
  assign fx_grant = req & ~(req << 1);    // BUG: "isolate lowest bit" done wrong
`else
  assign fx_grant = fx_found ? (N'(1) << fx_idx) : '0;
`endif

  assign grant     = (policy == POL_FIXED) ? fx_grant : rr_grant;
  assign grant_idx = (policy == POL_FIXED) ? fx_idx   : rr_idx;

  // ---- state update -------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ptr    <= '0;
      credit <= w_eff;
    end else if (ack && |req) begin
`ifndef BUG_RR_NO_ROTATE
      ptr <= (grant_idx == IW'(N-1)) ? '0 : grant_idx + 1'b1;
`endif
      if (policy == POL_WRR) begin
        if (|eligible) begin
          credit[grant_idx] <= credit[grant_idx] - 1'b1;
        end else begin
          credit            <= w_eff;               // new round
          credit[grant_idx] <= w_eff[grant_idx] - 1'b1;
        end
      end
    end
  end

`ifdef FORMAL
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume(!rst_n);
  always @(posedge clk) if (f_past_valid) assume(policy == $past(policy));

  always @(*) if (f_past_valid) begin
    assert($onehot0(grant));                 // no duplicate grants
    assert((grant & ~req) == '0);            // never grant an idle client
    if (|req) assert(|grant);                // work conserving
    assert(grant_idx < N);
    if (|grant) assert(grant[grant_idx]);
    assert(ptr < N);
  end

  // bounded wait under round-robin
  genvar gi;
  generate for (gi = 0; gi < N; gi++) begin : g_fair
    reg  [IW:0] f_wait;
    wire [IW:0] f_dist = (gi >= ptr) ? (gi - ptr) : (gi + N - ptr);
    always @(posedge clk) begin
      if (!rst_n || !req[gi] || (ack && grant[gi]))
        f_wait <= '0;
      else if (ack && policy != POL_WRR && policy != POL_FIXED)
        f_wait <= f_wait + 1'b1;
    end
    always @(*) if (f_past_valid && policy != POL_WRR && policy != POL_FIXED) begin
      assert(f_wait + f_dist <= N-1);
      assert(f_wait < N);
    end
    always @(*) if (f_past_valid && rst_n) cover(policy == POL_RR && f_wait == N-1 && ack && grant[gi]);
  end endgenerate
`endif

endmodule
