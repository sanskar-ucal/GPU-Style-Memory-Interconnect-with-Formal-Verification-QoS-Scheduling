// -----------------------------------------------------------------------------
// axi_interconnect.sv
// N-client -> 1-memory AXI4 fabric (INCR bursts, ready/valid on all channels).
//
//   * per-client AR and AW request queues (sync_fifo)
//   * QoS arbiter selects the next client; read/write alternate per client
//   * the winning burst owns the memory port until RLAST / BRESP
//   * client index is prepended to the AXI ID on the memory side
//
// Formal (SymbiYosys, `define FORMAL): AXI handshake rules on every outbound
// channel, single-owner routing (no duplicate grants on R/W/B), reset recovery.
// -----------------------------------------------------------------------------
module axi_interconnect #(
  parameter int N          = 4,
  parameter int ADDR_W     = 32,
  parameter int DATA_W     = 32,
  parameter int ID_W       = 4,
  parameter int FIFO_DEPTH = 4,
  parameter int WW         = 4
) (
  input  logic                          clk,
  input  logic                          rst_n,

  input  logic [1:0]                    cfg_policy,
  input  logic [N-1:0][WW-1:0]          cfg_weight,

  // ---------------- client-facing slave ports ----------------
  input  logic [N-1:0]                  s_arvalid,
  output logic [N-1:0]                  s_arready,
  input  logic [N-1:0][ADDR_W-1:0]      s_araddr,
  input  logic [N-1:0][7:0]             s_arlen,
  input  logic [N-1:0][ID_W-1:0]        s_arid,

  output logic [N-1:0]                  s_rvalid,
  input  logic [N-1:0]                  s_rready,
  output logic [N-1:0][DATA_W-1:0]      s_rdata,
  output logic [N-1:0][ID_W-1:0]        s_rid,
  output logic [N-1:0][1:0]             s_rresp,
  output logic [N-1:0]                  s_rlast,

  input  logic [N-1:0]                  s_awvalid,
  output logic [N-1:0]                  s_awready,
  input  logic [N-1:0][ADDR_W-1:0]      s_awaddr,
  input  logic [N-1:0][7:0]             s_awlen,
  input  logic [N-1:0][ID_W-1:0]        s_awid,

  input  logic [N-1:0]                  s_wvalid,
  output logic [N-1:0]                  s_wready,
  input  logic [N-1:0][DATA_W-1:0]      s_wdata,
  input  logic [N-1:0][DATA_W/8-1:0]    s_wstrb,
  input  logic [N-1:0]                  s_wlast,

  output logic [N-1:0]                  s_bvalid,
  input  logic [N-1:0]                  s_bready,
  output logic [N-1:0][ID_W-1:0]        s_bid,
  output logic [N-1:0][1:0]             s_bresp,

  // ---------------- memory-facing master port ----------------
  output logic                          m_arvalid,
  input  logic                          m_arready,
  output logic [ADDR_W-1:0]             m_araddr,
  output logic [7:0]                    m_arlen,
  output logic [ID_W+$clog2(N)-1:0]     m_arid,

  input  logic                          m_rvalid,
  output logic                          m_rready,
  input  logic [DATA_W-1:0]             m_rdata,
  input  logic [ID_W+$clog2(N)-1:0]     m_rid,
  input  logic [1:0]                    m_rresp,
  input  logic                          m_rlast,

  output logic                          m_awvalid,
  input  logic                          m_awready,
  output logic [ADDR_W-1:0]             m_awaddr,
  output logic [7:0]                    m_awlen,
  output logic [ID_W+$clog2(N)-1:0]     m_awid,

  output logic                          m_wvalid,
  input  logic                          m_wready,
  output logic [DATA_W-1:0]             m_wdata,
  output logic [DATA_W/8-1:0]           m_wstrb,
  output logic                          m_wlast,

  input  logic                          m_bvalid,
  output logic                          m_bready,
  input  logic [ID_W+$clog2(N)-1:0]     m_bid,
  input  logic [1:0]                    m_bresp
);
  localparam int MW    = $clog2(N);
  localparam int CMD_W = ADDR_W + 8 + ID_W;

  localparam logic [2:0] S_IDLE = 3'd0;
  localparam logic [2:0] S_AR   = 3'd1;
  localparam logic [2:0] S_R    = 3'd2;
  localparam logic [2:0] S_AW   = 3'd3;
  localparam logic [2:0] S_W    = 3'd4;
  localparam logic [2:0] S_B    = 3'd5;

  logic [2:0]        state;
  logic [MW-1:0]     owner;
  logic [ADDR_W-1:0] cmd_addr;
  logic [7:0]        cmd_len;
  logic [ID_W-1:0]   cmd_id;

  // ---------------- request queues ----------------
  logic [N-1:0]            ar_empty, ar_full, aw_empty, aw_full;
  logic [N-1:0]            ar_pop, aw_pop;
  logic [N-1:0][CMD_W-1:0] ar_head, aw_head;

  genvar m;
  generate for (m = 0; m < N; m++) begin : g_q
    sync_fifo #(.WIDTH(CMD_W), .DEPTH(FIFO_DEPTH)) u_arq (
      .clk   (clk),
      .rst_n (rst_n),
      .push  (s_arvalid[m]),
      .din   ({s_araddr[m], s_arlen[m], s_arid[m]}),
      .full  (ar_full[m]),
      .pop   (ar_pop[m]),
      .dout  (ar_head[m]),
      .empty (ar_empty[m]),
      .count ()
    );
    sync_fifo #(.WIDTH(CMD_W), .DEPTH(FIFO_DEPTH)) u_awq (
      .clk   (clk),
      .rst_n (rst_n),
      .push  (s_awvalid[m]),
      .din   ({s_awaddr[m], s_awlen[m], s_awid[m]}),
      .full  (aw_full[m]),
      .pop   (aw_pop[m]),
      .dout  (aw_head[m]),
      .empty (aw_empty[m]),
      .count ()
    );
    assign s_arready[m] = !ar_full[m];
    assign s_awready[m] = !aw_full[m];
  end endgenerate

  // ---------------- arbitration ----------------
  logic [N-1:0]  req, grant, pick_w, rw_toggle;
  logic [MW-1:0] grant_idx;
  logic          arb_ack;

  assign req     = ~ar_empty | ~aw_empty;
  assign arb_ack = (state == S_IDLE) && |req;
  assign pick_w  = ~aw_empty & (ar_empty | rw_toggle);

  qos_arbiter #(.N(N), .WW(WW)) u_arb (
    .clk       (clk),
    .rst_n     (rst_n),
    .policy    (cfg_policy),
    .weight    (cfg_weight),
    .req       (req),
    .ack       (arb_ack),
    .grant     (grant),
    .grant_idx (grant_idx)
  );

  assign ar_pop = arb_ack ? (grant & ~pick_w) : '0;
  assign aw_pop = arb_ack ? (grant &  pick_w) : '0;

  // ---------------- transaction FSM ----------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
`ifndef BUG_RESET_FSM
      state <= S_IDLE;                     // BUG variant: FSM survives reset
`endif
      owner     <= '0;
      rw_toggle <= '0;
      cmd_addr  <= '0;
      cmd_len   <= '0;
      cmd_id    <= '0;
    end else begin
      case (state)
        S_IDLE: if (arb_ack) begin
          owner <= grant_idx;
          if (pick_w[grant_idx]) begin
            {cmd_addr, cmd_len, cmd_id} <= aw_head[grant_idx];
            state <= S_AW;
          end else begin
            {cmd_addr, cmd_len, cmd_id} <= ar_head[grant_idx];
            state <= S_AR;
          end
          if (!ar_empty[grant_idx] && !aw_empty[grant_idx])
            rw_toggle[grant_idx] <= ~rw_toggle[grant_idx];
        end
`ifdef BUG_HANDSHAKE_DROP
        S_AR:    state <= S_R;             // BUG: does not wait for ARREADY
`else
        S_AR:    if (m_arready) state <= S_R;
`endif
        S_R:     if (m_rvalid && m_rready && m_rlast) state <= S_IDLE;
        S_AW:    if (m_awready) state <= S_W;
        S_W:     if (m_wvalid && m_wready && m_wlast) state <= S_B;
        S_B:     if (m_bvalid && m_bready) state <= S_IDLE;
        default: state <= S_IDLE;
      endcase
    end
  end

  // ---------------- memory-side request channels ----------------
  assign m_arvalid = (state == S_AR);
  assign m_araddr  = cmd_addr;
  assign m_arlen   = cmd_len;
  assign m_arid    = {owner, cmd_id};

  assign m_awvalid = (state == S_AW);
  assign m_awaddr  = cmd_addr;
  assign m_awlen   = cmd_len;
  assign m_awid    = {owner, cmd_id};

  // ---------------- data / response routing ----------------
  logic [MW-1:0] r_route;
`ifdef BUG_R_MISROUTE
  assign r_route = grant_idx;              // BUG: routes by live arbiter output
`else
  assign r_route = owner;
`endif

  always_comb begin
    for (int i = 0; i < N; i++) begin
      s_rvalid[i] = (state == S_R) && (r_route == MW'(i)) && m_rvalid;
      s_rdata[i]  = m_rdata;
      s_rid[i]    = m_rid[ID_W-1:0];
      s_rresp[i]  = m_rresp;
      s_rlast[i]  = m_rlast;
      s_wready[i] = (state == S_W) && (owner == MW'(i)) && m_wready;
      s_bvalid[i] = (state == S_B) && (owner == MW'(i)) && m_bvalid;
      s_bid[i]    = m_bid[ID_W-1:0];
      s_bresp[i]  = m_bresp;
    end
  end

  assign m_rready = (state == S_R) && s_rready[r_route];
  assign m_wvalid = (state == S_W) && s_wvalid[owner];
  assign m_wdata  = s_wdata[owner];
  assign m_wstrb  = s_wstrb[owner];
  assign m_wlast  = s_wlast[owner];
  assign m_bready = (state == S_B) && s_bready[owner];

`ifdef FORMAL
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume(!rst_n);
  always @(posedge clk) if (f_past_valid) assume(cfg_policy == $past(cfg_policy));

  reg f_rst_q = 1'b0;
  always @(posedge clk) f_rst_q <= rst_n;
  wire f_live = f_past_valid && f_rst_q && rst_n;

  // ---- environment: clients and memory obey AXI source rules ------------
  genvar fm;
  generate for (fm = 0; fm < N; fm++) begin : g_env
    // flatten packed-array slices so $past() sees full-width vectors
    wire [CMD_W-1:0]  f_ar = {s_araddr[fm], s_arlen[fm], s_arid[fm]};
    wire [CMD_W-1:0]  f_aw = {s_awaddr[fm], s_awlen[fm], s_awid[fm]};
    wire [DATA_W+DATA_W/8:0] f_w = {s_wdata[fm], s_wstrb[fm], s_wlast[fm]};
    wire [DATA_W+ID_W:0]     f_r = {s_rdata[fm], s_rid[fm], s_rlast[fm]};
    wire f_arv = s_arvalid[fm], f_arr = s_arready[fm];
    wire f_awv = s_awvalid[fm], f_awr = s_awready[fm];
    wire f_wv  = s_wvalid[fm],  f_wr  = s_wready[fm];
    wire f_rv  = s_rvalid[fm],  f_rr  = s_rready[fm];
    wire f_bv  = s_bvalid[fm],  f_br  = s_bready[fm];

    always @(posedge clk) if (f_live) begin
      if ($past(f_arv && !f_arr)) assume(f_arv && f_ar == $past(f_ar));
      if ($past(f_awv && !f_awr)) assume(f_awv && f_aw == $past(f_aw));
      if ($past(f_wv  && !f_wr))  assume(f_wv  && f_w  == $past(f_w));

      // outbound response channels hold until accepted
      if ($past(f_rv && !f_rr)) assert(f_rv && f_r == $past(f_r));
      if ($past(f_bv && !f_br)) assert(f_bv);
    end
  end endgenerate

  always @(posedge clk) if (f_live) begin
    if ($past(m_rvalid && !m_rready))
      assume(m_rvalid && m_rdata == $past(m_rdata) && m_rlast == $past(m_rlast) && m_rid == $past(m_rid));
    if ($past(m_bvalid && !m_bready))
      assume(m_bvalid && m_bid == $past(m_bid) && m_bresp == $past(m_bresp));
  end

  // ---- outbound request channels: VALID must hold, payload stable -------
  always @(posedge clk) if (f_live) begin
    if ($past(m_arvalid && !m_arready))
      assert(m_arvalid && m_araddr == $past(m_araddr) && m_arlen == $past(m_arlen) && m_arid == $past(m_arid));
    if ($past(m_awvalid && !m_awready))
      assert(m_awvalid && m_awaddr == $past(m_awaddr) && m_awlen == $past(m_awlen) && m_awid == $past(m_awid));
    if ($past(m_wvalid && !m_wready))
      assert(m_wvalid && m_wdata == $past(m_wdata) && m_wlast == $past(m_wlast));
  end

  // ---- single owner: never route to two clients at once -----------------
  always @(*) if (f_past_valid) begin
    assert($onehot0(s_rvalid));
    assert($onehot0(s_wready));
    assert($onehot0(s_bvalid));
    assert(!(m_arvalid && m_awvalid));
    assert(state <= S_B);
    assert(owner < N);
    assert($onehot0(ar_pop | aw_pop));
  end

  // ---- reset recovery ---------------------------------------------------
  always @(posedge clk) if (f_past_valid && !$past(rst_n))
    assert(state == S_IDLE && !m_arvalid && !m_awvalid && s_rvalid == '0 && s_bvalid == '0);

  // ---- reachability -----------------------------------------------------
  always @(*) if (f_live) cover(state == S_R && m_rvalid && m_rready && m_rlast && owner == MW'(N-1));
  always @(*) if (f_live) cover(state == S_B && m_bvalid && m_bready && owner == MW'(N-1));
`endif

endmodule
