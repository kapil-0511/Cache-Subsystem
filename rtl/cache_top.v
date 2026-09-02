`timescale 1ns/1ps
`include "cache_defines.v"

// ============================================================
// cache_top.v  —  Dual-Core CHI Coherent Cache Hierarchy
//
//   core  u_core0 ──┐
//                   ├──► chi_fabric ──► coherence_manager ──► DRAM
//   core  u_core1 ──┘
//
// CHI-B field widths:
//   TXREQ/RXREQ   : ReqOpcode[5:0]
//   TXRSP/RXRSP   : RspOpcode[3:0]
//   TXDAT/RXDAT   : DatOpcode[2:0] + Resp[2:0]
//   TXSNP/RXSNP   : SnpOpcode[4:0]
//   TXSNPDAT/RXSNPDAT : DatOpcode[2:0] + Resp[2:0]  (dirty snoop data)
// ============================================================

module cache_top (
    input  wire        clk,
    input  wire        rst_n,

    // ── Core-0 ────────────────────────────────────────────────
    input  wire [31:0]  cpu0_addr,
    input  wire [31:0]  cpu0_wdata,
    input  wire         cpu0_wen,
    input  wire         cpu0_ren,
    input  wire [3:0]   cpu0_be,
    output wire [31:0]  cpu0_rdata,
    output wire         cpu0_stall,

    // ── Core-1 ────────────────────────────────────────────────
    input  wire [31:0]  cpu1_addr,
    input  wire [31:0]  cpu1_wdata,
    input  wire         cpu1_wen,
    input  wire         cpu1_ren,
    input  wire [3:0]   cpu1_be,
    output wire [31:0]  cpu1_rdata,
    output wire         cpu1_stall,

    // ── Performance counters ──────────────────────────────────
    output wire [31:0]  l1_0_hits,
    output wire [31:0]  l1_0_misses,
    output wire [31:0]  l1_1_hits,
    output wire [31:0]  l1_1_misses,
    output wire [31:0]  l2_hits,
    output wire [31:0]  l2_misses
);

// ============================================================
// core0 ↔ Fabric wires
// ============================================================
wire         core0_txreq_valid, core0_txreq_rdy;
wire [31:0]  core0_txreq_addr;
wire [5:0]   core0_txreq_opcode;
wire [127:0] core0_txreq_wdata;

wire         core0_rxdat_valid, core0_rxdat_ack;
wire [2:0]   core0_rxdat_opcode;
wire [127:0] core0_rxdat_data;
wire [2:0]   core0_rxdat_resp;

wire         core0_rxrsp_valid, core0_rxrsp_ack;
wire [3:0]   core0_rxrsp_opcode;

wire         core0_rxsnp_valid, core0_rxsnp_rdy;
wire [31:0]  core0_rxsnp_addr;
wire [4:0]   core0_rxsnp_opcode;

wire         core0_txrsp_valid, core0_txrsp_rdy;
wire [3:0]   core0_txrsp_opcode;

wire         core0_txsnpdat_valid, core0_txsnpdat_rdy;
wire [2:0]   core0_txsnpdat_opcode;
wire [127:0] core0_txsnpdat_data;
wire [2:0]   core0_txsnpdat_resp;

// ============================================================
// core1 ↔ Fabric wires
// ============================================================
wire         core1_txreq_valid, core1_txreq_rdy;
wire [31:0]  core1_txreq_addr;
wire [5:0]   core1_txreq_opcode;
wire [127:0] core1_txreq_wdata;

wire         core1_rxdat_valid, core1_rxdat_ack;
wire [2:0]   core1_rxdat_opcode;
wire [127:0] core1_rxdat_data;
wire [2:0]   core1_rxdat_resp;

wire         core1_rxrsp_valid, core1_rxrsp_ack;
wire [3:0]   core1_rxrsp_opcode;

wire         core1_rxsnp_valid, core1_rxsnp_rdy;
wire [31:0]  core1_rxsnp_addr;
wire [4:0]   core1_rxsnp_opcode;

wire         core1_txrsp_valid, core1_txrsp_rdy;
wire [3:0]   core1_txrsp_opcode;

wire         core1_txsnpdat_valid, core1_txsnpdat_rdy;
wire [2:0]   core1_txsnpdat_opcode;
wire [127:0] core1_txsnpdat_data;
wire [2:0]   core1_txsnpdat_resp;

// ============================================================
// Fabric ↔ coherence_manager wires
// ============================================================
wire         coh_rxreq_valid, coh_rxreq_lcrd;
wire [31:0]  coh_rxreq_addr;
wire [5:0]   coh_rxreq_opcode;
wire [127:0] coh_rxreq_wdata;
wire         coh_rxreq_src_id;

wire         coh_txdat_valid, coh_txdat_ack;
wire [2:0]   coh_txdat_opcode;
wire [127:0] coh_txdat_data;
wire [2:0]   coh_txdat_resp;
wire         coh_txdat_tgt_id;

wire         coh_txrsp_valid, coh_txrsp_ack;
wire [3:0]   coh_txrsp_opcode;
wire         coh_txrsp_tgt_id;

wire         coh_txsnp_valid, coh_txsnp_rdy;
wire [31:0]  coh_txsnp_addr;
wire [4:0]   coh_txsnp_opcode;
wire         coh_txsnp_tgt_id;

wire         coh_rxrsp_valid, coh_rxrsp_lcrd;
wire [3:0]   coh_rxrsp_opcode;
wire         coh_rxrsp_src_id;

wire         coh_rxsnpdat_valid, coh_rxsnpdat_lcrd;
wire [2:0]   coh_rxsnpdat_opcode;
wire [127:0] coh_rxsnpdat_data;
wire [2:0]   coh_rxsnpdat_resp;
wire         coh_rxsnpdat_src_id;

// ============================================================
// coherence_manager ↔ DRAM wires
// ============================================================
wire [31:0]  coh_dram_addr;
wire [127:0] coh_dram_wdata;
wire         coh_dram_wen, coh_dram_ren;
wire [127:0] dram_coh_rdata;
wire         dram_coh_ack;

// ============================================================
// core0
// ============================================================
core u_core0 (
    .clk              (clk),
    .rst_n            (rst_n),
    .core_addr        (cpu0_addr),
    .core_wdata       (cpu0_wdata),
    .core_wen         (cpu0_wen),
    .core_ren         (cpu0_ren),
    .core_be          (cpu0_be),
    .core_rdata       (cpu0_rdata),
    .core_stall       (cpu0_stall),
    .txreq_valid      (core0_txreq_valid),
    .txreq_addr       (core0_txreq_addr),
    .txreq_opcode     (core0_txreq_opcode),
    .txreq_wdata      (core0_txreq_wdata),
    .txreq_rdy        (core0_txreq_rdy),
    .rxdat_valid      (core0_rxdat_valid),
    .rxdat_opcode     (core0_rxdat_opcode),
    .rxdat_data       (core0_rxdat_data),
    .rxdat_resp       (core0_rxdat_resp),
    .rxdat_ack        (core0_rxdat_ack),
    .rxrsp_valid      (core0_rxrsp_valid),
    .rxrsp_opcode     (core0_rxrsp_opcode),
    .rxrsp_ack        (core0_rxrsp_ack),
    .rxsnp_valid      (core0_rxsnp_valid),
    .rxsnp_addr       (core0_rxsnp_addr),
    .rxsnp_opcode     (core0_rxsnp_opcode),
    .rxsnp_rdy        (core0_rxsnp_rdy),
    .txrsp_valid      (core0_txrsp_valid),
    .txrsp_opcode     (core0_txrsp_opcode),
    .txrsp_rdy        (core0_txrsp_rdy),
    .txsnpdat_valid   (core0_txsnpdat_valid),
    .txsnpdat_opcode  (core0_txsnpdat_opcode),
    .txsnpdat_data    (core0_txsnpdat_data),
    .txsnpdat_resp    (core0_txsnpdat_resp),
    .txsnpdat_rdy     (core0_txsnpdat_rdy),
    .perf_hits        (l1_0_hits),
    .perf_misses      (l1_0_misses)
);

// ============================================================
// core1
// ============================================================
core u_core1 (
    .clk              (clk),
    .rst_n            (rst_n),
    .core_addr        (cpu1_addr),
    .core_wdata       (cpu1_wdata),
    .core_wen         (cpu1_wen),
    .core_ren         (cpu1_ren),
    .core_be          (cpu1_be),
    .core_rdata       (cpu1_rdata),
    .core_stall       (cpu1_stall),
    .txreq_valid      (core1_txreq_valid),
    .txreq_addr       (core1_txreq_addr),
    .txreq_opcode     (core1_txreq_opcode),
    .txreq_wdata      (core1_txreq_wdata),
    .txreq_rdy        (core1_txreq_rdy),
    .rxdat_valid      (core1_rxdat_valid),
    .rxdat_opcode     (core1_rxdat_opcode),
    .rxdat_data       (core1_rxdat_data),
    .rxdat_resp       (core1_rxdat_resp),
    .rxdat_ack        (core1_rxdat_ack),
    .rxrsp_valid      (core1_rxrsp_valid),
    .rxrsp_opcode     (core1_rxrsp_opcode),
    .rxrsp_ack        (core1_rxrsp_ack),
    .rxsnp_valid      (core1_rxsnp_valid),
    .rxsnp_addr       (core1_rxsnp_addr),
    .rxsnp_opcode     (core1_rxsnp_opcode),
    .rxsnp_rdy        (core1_rxsnp_rdy),
    .txrsp_valid      (core1_txrsp_valid),
    .txrsp_opcode     (core1_txrsp_opcode),
    .txrsp_rdy        (core1_txrsp_rdy),
    .txsnpdat_valid   (core1_txsnpdat_valid),
    .txsnpdat_opcode  (core1_txsnpdat_opcode),
    .txsnpdat_data    (core1_txsnpdat_data),
    .txsnpdat_resp    (core1_txsnpdat_resp),
    .txsnpdat_rdy     (core1_txsnpdat_rdy),
    .perf_hits        (l1_1_hits),
    .perf_misses      (l1_1_misses)
);

// ============================================================
// CHI Fabric
// ============================================================
chi_fabric u_fabric (
    // core0
    .core0_txreq_valid   (core0_txreq_valid),   .core0_txreq_addr    (core0_txreq_addr),
    .core0_txreq_opcode  (core0_txreq_opcode),  .core0_txreq_wdata   (core0_txreq_wdata),
    .core0_txreq_rdy     (core0_txreq_rdy),
    .core0_rxdat_valid   (core0_rxdat_valid),   .core0_rxdat_opcode  (core0_rxdat_opcode),
    .core0_rxdat_data    (core0_rxdat_data),    .core0_rxdat_resp    (core0_rxdat_resp),
    .core0_rxdat_ack     (core0_rxdat_ack),
    .core0_rxrsp_valid   (core0_rxrsp_valid),   .core0_rxrsp_opcode  (core0_rxrsp_opcode),
    .core0_rxrsp_ack     (core0_rxrsp_ack),
    .core0_rxsnp_valid   (core0_rxsnp_valid),   .core0_rxsnp_addr    (core0_rxsnp_addr),
    .core0_rxsnp_opcode  (core0_rxsnp_opcode),  .core0_rxsnp_rdy     (core0_rxsnp_rdy),
    .core0_txrsp_valid   (core0_txrsp_valid),   .core0_txrsp_opcode  (core0_txrsp_opcode),
    .core0_txrsp_rdy     (core0_txrsp_rdy),
    .core0_txsnpdat_valid  (core0_txsnpdat_valid), .core0_txsnpdat_opcode (core0_txsnpdat_opcode),
    .core0_txsnpdat_data   (core0_txsnpdat_data),  .core0_txsnpdat_resp   (core0_txsnpdat_resp),
    .core0_txsnpdat_rdy    (core0_txsnpdat_rdy),
    // core1
    .core1_txreq_valid   (core1_txreq_valid),   .core1_txreq_addr    (core1_txreq_addr),
    .core1_txreq_opcode  (core1_txreq_opcode),  .core1_txreq_wdata   (core1_txreq_wdata),
    .core1_txreq_rdy     (core1_txreq_rdy),
    .core1_rxdat_valid   (core1_rxdat_valid),   .core1_rxdat_opcode  (core1_rxdat_opcode),
    .core1_rxdat_data    (core1_rxdat_data),    .core1_rxdat_resp    (core1_rxdat_resp),
    .core1_rxdat_ack     (core1_rxdat_ack),
    .core1_rxrsp_valid   (core1_rxrsp_valid),   .core1_rxrsp_opcode  (core1_rxrsp_opcode),
    .core1_rxrsp_ack     (core1_rxrsp_ack),
    .core1_rxsnp_valid   (core1_rxsnp_valid),   .core1_rxsnp_addr    (core1_rxsnp_addr),
    .core1_rxsnp_opcode  (core1_rxsnp_opcode),  .core1_rxsnp_rdy     (core1_rxsnp_rdy),
    .core1_txrsp_valid   (core1_txrsp_valid),   .core1_txrsp_opcode  (core1_txrsp_opcode),
    .core1_txrsp_rdy     (core1_txrsp_rdy),
    .core1_txsnpdat_valid  (core1_txsnpdat_valid), .core1_txsnpdat_opcode (core1_txsnpdat_opcode),
    .core1_txsnpdat_data   (core1_txsnpdat_data),  .core1_txsnpdat_resp   (core1_txsnpdat_resp),
    .core1_txsnpdat_rdy    (core1_txsnpdat_rdy),
    // HN
    .hn_rxreq_valid      (coh_rxreq_valid),     .hn_rxreq_addr       (coh_rxreq_addr),
    .hn_rxreq_opcode     (coh_rxreq_opcode),    .hn_rxreq_wdata      (coh_rxreq_wdata),
    .hn_rxreq_src_id     (coh_rxreq_src_id),    .hn_rxreq_lcrd       (coh_rxreq_lcrd),
    .hn_txdat_valid      (coh_txdat_valid),     .hn_txdat_opcode     (coh_txdat_opcode),
    .hn_txdat_data       (coh_txdat_data),      .hn_txdat_resp       (coh_txdat_resp),
    .hn_txdat_tgt_id     (coh_txdat_tgt_id),    .hn_txdat_ack        (coh_txdat_ack),
    .hn_txrsp_valid      (coh_txrsp_valid),     .hn_txrsp_opcode     (coh_txrsp_opcode),
    .hn_txrsp_tgt_id     (coh_txrsp_tgt_id),    .hn_txrsp_ack        (coh_txrsp_ack),
    .hn_txsnp_valid      (coh_txsnp_valid),     .hn_txsnp_addr       (coh_txsnp_addr),
    .hn_txsnp_opcode     (coh_txsnp_opcode),    .hn_txsnp_tgt_id     (coh_txsnp_tgt_id),
    .hn_txsnp_rdy        (coh_txsnp_rdy),
    .hn_rxrsp_valid      (coh_rxrsp_valid),     .hn_rxrsp_opcode     (coh_rxrsp_opcode),
    .hn_rxrsp_src_id     (coh_rxrsp_src_id),    .hn_rxrsp_lcrd       (coh_rxrsp_lcrd),
    .hn_rxsnpdat_valid   (coh_rxsnpdat_valid),  .hn_rxsnpdat_opcode  (coh_rxsnpdat_opcode),
    .hn_rxsnpdat_data    (coh_rxsnpdat_data),   .hn_rxsnpdat_resp    (coh_rxsnpdat_resp),
    .hn_rxsnpdat_src_id  (coh_rxsnpdat_src_id), .hn_rxsnpdat_lcrd    (coh_rxsnpdat_lcrd)
);

// ============================================================
// coherence_manager
// ============================================================
coherence_manager u_coh_mgr (
    .clk                (clk),
    .rst_n              (rst_n),
    .rxreq_valid        (coh_rxreq_valid),
    .rxreq_addr         (coh_rxreq_addr),
    .rxreq_opcode       (coh_rxreq_opcode),
    .rxreq_wdata        (coh_rxreq_wdata),
    .rxreq_src_id       (coh_rxreq_src_id),
    .rxreq_lcrd         (coh_rxreq_lcrd),
    .txdat_valid        (coh_txdat_valid),
    .txdat_opcode       (coh_txdat_opcode),
    .txdat_data         (coh_txdat_data),
    .txdat_resp         (coh_txdat_resp),
    .txdat_tgt_id       (coh_txdat_tgt_id),
    .txdat_ack          (coh_txdat_ack),
    .txrsp_valid        (coh_txrsp_valid),
    .txrsp_opcode       (coh_txrsp_opcode),
    .txrsp_tgt_id       (coh_txrsp_tgt_id),
    .txrsp_ack          (coh_txrsp_ack),
    .txsnp_valid        (coh_txsnp_valid),
    .txsnp_addr         (coh_txsnp_addr),
    .txsnp_opcode       (coh_txsnp_opcode),
    .txsnp_tgt_id       (coh_txsnp_tgt_id),
    .txsnp_rdy          (coh_txsnp_rdy),
    .rxrsp_valid        (coh_rxrsp_valid),
    .rxrsp_opcode       (coh_rxrsp_opcode),
    .rxrsp_src_id       (coh_rxrsp_src_id),
    .rxrsp_lcrd         (coh_rxrsp_lcrd),
    .rxsnpdat_valid     (coh_rxsnpdat_valid),
    .rxsnpdat_opcode    (coh_rxsnpdat_opcode),
    .rxsnpdat_data      (coh_rxsnpdat_data),
    .rxsnpdat_resp      (coh_rxsnpdat_resp),
    .rxsnpdat_src_id    (coh_rxsnpdat_src_id),
    .rxsnpdat_lcrd      (coh_rxsnpdat_lcrd),
    .dram_addr          (coh_dram_addr),
    .dram_wdata         (coh_dram_wdata),
    .dram_wen           (coh_dram_wen),
    .dram_ren           (coh_dram_ren),
    .dram_rdata         (dram_coh_rdata),
    .dram_ack           (dram_coh_ack),
    .l2_perf_hits       (l2_hits),
    .l2_perf_misses     (l2_misses)
);

// ============================================================
// DRAM model
// ============================================================
dram_model u_dram (
    .clk        (clk),
    .rst_n      (rst_n),
    .dram_addr  (coh_dram_addr),
    .dram_wdata (coh_dram_wdata),
    .dram_wen   (coh_dram_wen),
    .dram_ren   (coh_dram_ren),
    .dram_rdata (dram_coh_rdata),
    .dram_ack   (dram_coh_ack)
);

endmodule
