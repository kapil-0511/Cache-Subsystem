`timescale 1ns/1ps

// ============================================================
// chi_fabric.v  —  CHI Interconnect Fabric (combinational)
//
// CHI-B field widths:
//   TXREQ/RXREQ  : ReqOpcode[5:0]
//   TXRSP/RXRSP  : RspOpcode[3:0]   (Comp / clean snoop responses)
//   TXDAT/RXDAT  : DatOpcode[2:0] + Resp[2:0]
//   TXSNP/RXSNP  : SnpOpcode[4:0]
//   TXSNPDAT/RXSNPDAT : DatOpcode[2:0] + Resp[2:0]  (dirty snoop data)
//
// Routing:
//   TXREQ    : core0|1 → HN      (core0 priority arbiter)
//   TXSNP    : HN → core0|1      (demux via tgt_id)
//   TXRSP    : core0|1 → HN      (clean snoop resp, core0 priority)
//   TXSNPDAT : core0|1 → HN      (dirty snoop resp, core0 priority)
//   TXRSP    : HN → core0|1      (Comp permission, demux via tgt_id)
//   TXDAT    : HN → core0|1      (CompData, demux via tgt_id)
// ============================================================

module chi_fabric (

    // ── core0 ─────────────────────────────────────────────────
    // TXREQ (core0 → HN)
    input  wire         core0_txreq_valid,
    input  wire [31:0]  core0_txreq_addr,
    input  wire [5:0]   core0_txreq_opcode,
    input  wire [127:0] core0_txreq_wdata,
    output wire         core0_txreq_rdy,

    // RXDAT (HN → core0, CompData)
    output wire         core0_rxdat_valid,
    output wire [2:0]   core0_rxdat_opcode,
    output wire [127:0] core0_rxdat_data,
    output wire [2:0]   core0_rxdat_resp,
    input  wire         core0_rxdat_ack,

    // RXRSP (HN → core0, Comp)
    output wire         core0_rxrsp_valid,
    output wire [3:0]   core0_rxrsp_opcode,
    input  wire         core0_rxrsp_ack,

    // RXSNP (HN → core0, snoops)
    output wire         core0_rxsnp_valid,
    output wire [31:0]  core0_rxsnp_addr,
    output wire [4:0]   core0_rxsnp_opcode,
    input  wire         core0_rxsnp_rdy,

    // TXRSP (core0 → HN, clean snoop response)
    input  wire         core0_txrsp_valid,
    input  wire [3:0]   core0_txrsp_opcode,
    output wire         core0_txrsp_rdy,

    // TXSNPDAT (core0 → HN, dirty snoop data)
    input  wire         core0_txsnpdat_valid,
    input  wire [2:0]   core0_txsnpdat_opcode,
    input  wire [127:0] core0_txsnpdat_data,
    input  wire [2:0]   core0_txsnpdat_resp,
    output wire         core0_txsnpdat_rdy,

    // ── core1 ─────────────────────────────────────────────────
    // TXREQ (core1 → HN)
    input  wire         core1_txreq_valid,
    input  wire [31:0]  core1_txreq_addr,
    input  wire [5:0]   core1_txreq_opcode,
    input  wire [127:0] core1_txreq_wdata,
    output wire         core1_txreq_rdy,

    // RXDAT (HN → core1, CompData)
    output wire         core1_rxdat_valid,
    output wire [2:0]   core1_rxdat_opcode,
    output wire [127:0] core1_rxdat_data,
    output wire [2:0]   core1_rxdat_resp,
    input  wire         core1_rxdat_ack,

    // RXRSP (HN → core1, Comp)
    output wire         core1_rxrsp_valid,
    output wire [3:0]   core1_rxrsp_opcode,
    input  wire         core1_rxrsp_ack,

    // RXSNP (HN → core1, snoops)
    output wire         core1_rxsnp_valid,
    output wire [31:0]  core1_rxsnp_addr,
    output wire [4:0]   core1_rxsnp_opcode,
    input  wire         core1_rxsnp_rdy,

    // TXRSP (core1 → HN, clean snoop response)
    input  wire         core1_txrsp_valid,
    input  wire [3:0]   core1_txrsp_opcode,
    output wire         core1_txrsp_rdy,

    // TXSNPDAT (core1 → HN, dirty snoop data)
    input  wire         core1_txsnpdat_valid,
    input  wire [2:0]   core1_txsnpdat_opcode,
    input  wire [127:0] core1_txsnpdat_data,
    input  wire [2:0]   core1_txsnpdat_resp,
    output wire         core1_txsnpdat_rdy,

    // ── Home Node (coherence_manager) ─────────────────────────
    // RXREQ
    output wire         hn_rxreq_valid,
    output wire [31:0]  hn_rxreq_addr,
    output wire [5:0]   hn_rxreq_opcode,
    output wire [127:0] hn_rxreq_wdata,
    output wire         hn_rxreq_src_id,
    input  wire         hn_rxreq_lcrd,

    // TXDAT (HN → cores, CompData)
    input  wire         hn_txdat_valid,
    input  wire [2:0]   hn_txdat_opcode,
    input  wire [127:0] hn_txdat_data,
    input  wire [2:0]   hn_txdat_resp,
    input  wire         hn_txdat_tgt_id,
    output wire         hn_txdat_ack,

    // TXRSP (HN → cores, Comp)
    input  wire         hn_txrsp_valid,
    input  wire [3:0]   hn_txrsp_opcode,
    input  wire         hn_txrsp_tgt_id,
    output wire         hn_txrsp_ack,

    // TXSNP (HN → cores, snoops)
    input  wire         hn_txsnp_valid,
    input  wire [31:0]  hn_txsnp_addr,
    input  wire [4:0]   hn_txsnp_opcode,
    input  wire         hn_txsnp_tgt_id,
    output wire         hn_txsnp_rdy,

    // RXRSP (cores → HN, clean snoop responses)
    output wire         hn_rxrsp_valid,
    output wire [3:0]   hn_rxrsp_opcode,
    output wire         hn_rxrsp_src_id,
    input  wire         hn_rxrsp_lcrd,

    // RXSNPDAT (cores → HN, dirty snoop data)
    output wire         hn_rxsnpdat_valid,
    output wire [2:0]   hn_rxsnpdat_opcode,
    output wire [127:0] hn_rxsnpdat_data,
    output wire [2:0]   hn_rxsnpdat_resp,
    output wire         hn_rxsnpdat_src_id,
    input  wire         hn_rxsnpdat_lcrd
);

// ── TXREQ: core0|1 → HN  (core0 priority) ────────────────────
assign hn_rxreq_valid   = core0_txreq_valid | core1_txreq_valid;
assign hn_rxreq_src_id  = core0_txreq_valid ? 1'b0 : 1'b1;
assign hn_rxreq_addr    = core0_txreq_valid ? core0_txreq_addr   : core1_txreq_addr;
assign hn_rxreq_opcode  = core0_txreq_valid ? core0_txreq_opcode : core1_txreq_opcode;
assign hn_rxreq_wdata   = core0_txreq_valid ? core0_txreq_wdata  : core1_txreq_wdata;

assign core0_txreq_rdy  = hn_rxreq_lcrd &&  core0_txreq_valid;
assign core1_txreq_rdy  = hn_rxreq_lcrd && !core0_txreq_valid && core1_txreq_valid;

// ── TXSNP: HN → core0|1  (demux via tgt_id) ──────────────────
assign core0_rxsnp_valid  = hn_txsnp_valid && (hn_txsnp_tgt_id == 1'b0);
assign core1_rxsnp_valid  = hn_txsnp_valid && (hn_txsnp_tgt_id == 1'b1);
assign core0_rxsnp_addr   = hn_txsnp_addr;
assign core1_rxsnp_addr   = hn_txsnp_addr;
assign core0_rxsnp_opcode = hn_txsnp_opcode;
assign core1_rxsnp_opcode = hn_txsnp_opcode;
assign hn_txsnp_rdy       = hn_txsnp_tgt_id ? core1_rxsnp_rdy : core0_rxsnp_rdy;

// ── TXRSP clean snoop resp: core0|1 → HN  (core0 priority) ───
assign hn_rxrsp_valid   = core0_txrsp_valid | core1_txrsp_valid;
assign hn_rxrsp_src_id  = core0_txrsp_valid ? 1'b0 : 1'b1;
assign hn_rxrsp_opcode  = core0_txrsp_valid ? core0_txrsp_opcode : core1_txrsp_opcode;

assign core0_txrsp_rdy  = hn_rxrsp_lcrd &&  core0_txrsp_valid;
assign core1_txrsp_rdy  = hn_rxrsp_lcrd && !core0_txrsp_valid && core1_txrsp_valid;

// ── TXSNPDAT dirty snoop data: core0|1 → HN  (core0 priority)
assign hn_rxsnpdat_valid   = core0_txsnpdat_valid | core1_txsnpdat_valid;
assign hn_rxsnpdat_src_id  = core0_txsnpdat_valid ? 1'b0 : 1'b1;
assign hn_rxsnpdat_opcode  = core0_txsnpdat_valid ? core0_txsnpdat_opcode : core1_txsnpdat_opcode;
assign hn_rxsnpdat_data    = core0_txsnpdat_valid ? core0_txsnpdat_data   : core1_txsnpdat_data;
assign hn_rxsnpdat_resp    = core0_txsnpdat_valid ? core0_txsnpdat_resp   : core1_txsnpdat_resp;

assign core0_txsnpdat_rdy  = hn_rxsnpdat_lcrd &&  core0_txsnpdat_valid;
assign core1_txsnpdat_rdy  = hn_rxsnpdat_lcrd && !core0_txsnpdat_valid && core1_txsnpdat_valid;

// ── TXRSP Comp: HN → core0|1  (demux via tgt_id) ─────────────
assign core0_rxrsp_valid  = hn_txrsp_valid && (hn_txrsp_tgt_id == 1'b0);
assign core1_rxrsp_valid  = hn_txrsp_valid && (hn_txrsp_tgt_id == 1'b1);
assign core0_rxrsp_opcode = hn_txrsp_opcode;
assign core1_rxrsp_opcode = hn_txrsp_opcode;
assign hn_txrsp_ack       = hn_txrsp_tgt_id ? core1_rxrsp_ack : core0_rxrsp_ack;

// ── TXDAT CompData: HN → core0|1  (demux via tgt_id) ─────────
assign core0_rxdat_valid  = hn_txdat_valid && (hn_txdat_tgt_id == 1'b0);
assign core1_rxdat_valid  = hn_txdat_valid && (hn_txdat_tgt_id == 1'b1);
assign core0_rxdat_opcode = hn_txdat_opcode;
assign core1_rxdat_opcode = hn_txdat_opcode;
assign core0_rxdat_data   = hn_txdat_data;
assign core1_rxdat_data   = hn_txdat_data;
assign core0_rxdat_resp   = hn_txdat_resp;
assign core1_rxdat_resp   = hn_txdat_resp;
assign hn_txdat_ack       = hn_txdat_tgt_id ? core1_rxdat_ack : core0_rxdat_ack;

endmodule
