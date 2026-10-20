// -----------------------------------------------------------------------------
// mem_subsystem_top.sv
// SoC memory subsystem: 4 clients (GPU0, GPU1, CPU, DMA) -> AXI interconnect
// -> shared memory. Policy / QoS weights are runtime configuration inputs.
// -----------------------------------------------------------------------------
module mem_subsystem_top #(
  parameter int N          = 4,
  parameter int ADDR_W     = 32,
  parameter int DATA_W     = 32,
  parameter int ID_W       = 4,
  parameter int FIFO_DEPTH = 4,
  parameter int MEM_WORDS  = 16384,
  parameter int READ_LAT   = 6
) (
  input  logic                        clk,
  input  logic                        rst_n,

  input  logic [1:0]                  cfg_policy,
  input  logic [N-1:0][3:0]           cfg_weight,
  input  logic                        cfg_mem_stall,

  input  logic [N-1:0]                s_arvalid,
  output logic [N-1:0]                s_arready,
  input  logic [N-1:0][ADDR_W-1:0]    s_araddr,
  input  logic [N-1:0][7:0]           s_arlen,
  input  logic [N-1:0][ID_W-1:0]      s_arid,

  output logic [N-1:0]                s_rvalid,
  input  logic [N-1:0]                s_rready,
  output logic [N-1:0][DATA_W-1:0]    s_rdata,
  output logic [N-1:0][ID_W-1:0]      s_rid,
  output logic [N-1:0][1:0]           s_rresp,
  output logic [N-1:0]                s_rlast,

  input  logic [N-1:0]                s_awvalid,
  output logic [N-1:0]                s_awready,
  input  logic [N-1:0][ADDR_W-1:0]    s_awaddr,
  input  logic [N-1:0][7:0]           s_awlen,
  input  logic [N-1:0][ID_W-1:0]      s_awid,

  input  logic [N-1:0]                s_wvalid,
  output logic [N-1:0]                s_wready,
  input  logic [N-1:0][DATA_W-1:0]    s_wdata,
  input  logic [N-1:0][DATA_W/8-1:0]  s_wstrb,
  input  logic [N-1:0]                s_wlast,

  output logic [N-1:0]                s_bvalid,
  input  logic [N-1:0]                s_bready,
  output logic [N-1:0][ID_W-1:0]      s_bid,
  output logic [N-1:0][1:0]           s_bresp
);
  localparam int MID_W = ID_W + $clog2(N);

  logic                m_arvalid, m_arready, m_rvalid, m_rready, m_rlast;
  logic [ADDR_W-1:0]   m_araddr, m_awaddr;
  logic [7:0]          m_arlen, m_awlen;
  logic [MID_W-1:0]    m_arid, m_rid, m_awid, m_bid;
  logic [DATA_W-1:0]   m_rdata, m_wdata;
  logic [1:0]          m_rresp, m_bresp;
  logic                m_awvalid, m_awready, m_wvalid, m_wready, m_wlast;
  logic [DATA_W/8-1:0] m_wstrb;
  logic                m_bvalid, m_bready;

  axi_interconnect #(
    .N(N), .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(ID_W),
    .FIFO_DEPTH(FIFO_DEPTH), .WW(4)
  ) u_xbar (
    .clk, .rst_n, .cfg_policy, .cfg_weight,
    .s_arvalid, .s_arready, .s_araddr, .s_arlen, .s_arid,
    .s_rvalid, .s_rready, .s_rdata, .s_rid, .s_rresp, .s_rlast,
    .s_awvalid, .s_awready, .s_awaddr, .s_awlen, .s_awid,
    .s_wvalid, .s_wready, .s_wdata, .s_wstrb, .s_wlast,
    .s_bvalid, .s_bready, .s_bid, .s_bresp,
    .m_arvalid, .m_arready, .m_araddr, .m_arlen, .m_arid,
    .m_rvalid, .m_rready, .m_rdata, .m_rid, .m_rresp, .m_rlast,
    .m_awvalid, .m_awready, .m_awaddr, .m_awlen, .m_awid,
    .m_wvalid, .m_wready, .m_wdata, .m_wstrb, .m_wlast,
    .m_bvalid, .m_bready, .m_bid, .m_bresp
  );

  axi_mem #(
    .ADDR_W(ADDR_W), .DATA_W(DATA_W), .ID_W(MID_W),
    .MEM_WORDS(MEM_WORDS), .READ_LAT(READ_LAT)
  ) u_mem (
    .clk, .rst_n, .stall_en(cfg_mem_stall),
    .s_arvalid(m_arvalid), .s_arready(m_arready), .s_araddr(m_araddr),
    .s_arlen(m_arlen), .s_arid(m_arid),
    .s_rvalid(m_rvalid), .s_rready(m_rready), .s_rdata(m_rdata),
    .s_rid(m_rid), .s_rresp(m_rresp), .s_rlast(m_rlast),
    .s_awvalid(m_awvalid), .s_awready(m_awready), .s_awaddr(m_awaddr),
    .s_awlen(m_awlen), .s_awid(m_awid),
    .s_wvalid(m_wvalid), .s_wready(m_wready), .s_wdata(m_wdata),
    .s_wstrb(m_wstrb), .s_wlast(m_wlast),
    .s_bvalid(m_bvalid), .s_bready(m_bready), .s_bid(m_bid), .s_bresp(m_bresp)
  );

endmodule
