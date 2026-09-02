// ============================================================
// chi_defines.v  —  ARM AMBA CHI-B opcode and state constants
//
// Field widths (CHI-B):
//   TXREQ/RXREQ : ReqOpcode[5:0]
//   TXRSP/RXRSP : RspOpcode[3:0]   (permission responses + clean snoop responses)
//   TXDAT/RXDAT : DatOpcode[2:0]   (data responses; cache state in Resp[2:0])
//   TXSNP/RXSNP : SnpOpcode[4:0]
//
// Snoop response channel split (CHI-correct):
//   Clean (no data) → RSP channel : SnpRespI, SnpRespSC
//   Dirty (has data) → DAT channel : SnpRespData, SnpRespDataFwd
// ============================================================

// ── Internal L1 cache-state (not a CHI field) ─────────────────
`define STATE_I   3'd0   // Invalid
`define STATE_SC  3'd1   // Shared Clean
`define STATE_UC  3'd2   // Unique Clean  (transient; → UD on write)
`define STATE_UD  3'd3   // Unique Dirty
`define STATE_SD  3'd4   // Shared Dirty

// ── REQ channel opcodes — ReqOpcode[5:0] ─────────────────────
// ARM IHI0050B Table 2-6
`define REQ_ReadShared    6'h01   // RN→HN: read, willing to share      I → SC
`define REQ_ReadUnique    6'h07   // RN→HN: read + exclusive (wr-alloc) I → UD
`define REQ_CleanUnique   6'h0B   // RN→HN: upgrade (have clean data)  SC → UD
`define REQ_MakeUnique    6'h0C   // RN→HN: upgrade (have dirty data)  SD → UD
`define REQ_Evict         6'h0D   // RN→HN: clean line silently dropped SC → I
`define REQ_WriteBackFull 6'h1B   // RN→HN: write dirty line back      UD → I
`define REQ_CleanShared   6'h08   // RN→HN: CMO, ask peer to clean

// ── RSP channel opcodes — RspOpcode[3:0] ─────────────────────
// ARM IHI0050B Table 2-9
// Snoop responses WITHOUT data (Snoopee → HN, on TXRSP/RXRSP):
`define RSP_SnpRespI      4'h1    // snoopee: line evicted      → I   (no data)
`define RSP_SnpRespSC     4'h2    // snoopee: line retained     → SC  (no data)
// Completion responses (HN → Requester, on TXRSP/RXRSP):
`define RSP_Comp          4'h4    // permission-only completion (no data transfer)

// ── DAT channel opcodes — DatOpcode[2:0] ─────────────────────
// ARM IHI0050B Table 2-8
// Snoop responses WITH dirty data (Snoopee → HN, on TXSNPDAT/RXSNPDAT):
`define DAT_SnpRespData    3'h1   // snoopee dirty data → HN  (line → I)
`define DAT_SnpRespDataFwd 3'h6   // snoopee dirty data → HN  (line → SD, HN fwds to RN)
// Data + completion (HN → Requester, on TXDAT/RXDAT):
`define DAT_CompData       3'h4   // HN→RN: data + completion; cache state in Resp[2:0]

// ── DAT Resp[2:0] — final cache state of the returned line ───
// ARM IHI0050B Table 4-3
`define RESP_I            3'b000  // line not retained (Invalid)
`define RESP_UC           3'b010  // Unique Clean
`define RESP_UD           3'b011  // Unique Dirty
`define RESP_SC           3'b110  // Shared Clean
`define RESP_SD           3'b111  // Shared Dirty

// ── SNP channel opcodes — SnpOpcode[4:0] ─────────────────────
// ARM IHI0050B Table 2-12
`define SNP_SnpShared     5'h01   // HN→Snoopee: may retain SC, must give up UD
`define SNP_SnpUnique     5'h07   // HN→Snoopee: must invalidate → I
`define SNP_SnpClean      5'h08   // HN→Snoopee: must clean dirty data, may stay SC
`define SNP_SnpSharedFwd  5'h09   // HN→Snoopee: clean/give-up UD, HN fwds data to RN
