`timescale 1ns/1ps
`include "cache_defines.v"
`include "chi_defines.v"

// ============================================================
// coherence_manager.v  —  CHI Home Node (HN-F)
//
// Contains: Shared L2 cache  +  Snoop Filter (SF)  +  CHI FSM
//
// CHI channel widths (CHI-B):
//   RXREQ  : ReqOpcode[5:0]
//   TXDAT  : DatOpcode[2:0] + Resp[2:0]   (CompData to RN)
//   TXRSP  : RspOpcode[3:0]               (Comp to RN)
//   TXSNP  : SnpOpcode[4:0]               (snoops to RN)
//   RXRSP  : RspOpcode[3:0]               (clean snoop responses: SnpRespI/SC)
//   RXSNPDAT: DatOpcode[2:0] + Resp[2:0]  (dirty snoop responses: SnpRespData/Fwd)
//
// SF encoding: sf[idx][0]=core0 has copy, sf[idx][1]=core1 has copy
// ============================================================

module coherence_manager (
    input  wire clk,
    input  wire rst_n,

    // ── RXREQ : from cores ───────────────────────────────────
    input  wire         rxreq_valid,
    input  wire [31:0]  rxreq_addr,
    input  wire [5:0]   rxreq_opcode,     // ReqOpcode[5:0]
    input  wire [127:0] rxreq_wdata,      // dirty data for WriteBackFull
    input  wire         rxreq_src_id,
    output reg          rxreq_lcrd,

    // ── TXDAT : CompData to cores ────────────────────────────
    output reg          txdat_valid,
    output reg  [2:0]   txdat_opcode,     // DatOpcode[2:0]
    output reg  [127:0] txdat_data,
    output reg  [2:0]   txdat_resp,       // Resp[2:0]: RESP_SC / RESP_UC
    output reg          txdat_tgt_id,
    input  wire         txdat_ack,

    // ── TXRSP : permission-only to cores ─────────────────────
    output reg          txrsp_valid,
    output reg  [3:0]   txrsp_opcode,     // RspOpcode[3:0]
    output reg          txrsp_tgt_id,
    input  wire         txrsp_ack,

    // ── TXSNP : snoops to cores ──────────────────────────────
    output reg          txsnp_valid,
    output reg  [31:0]  txsnp_addr,
    output reg  [4:0]   txsnp_opcode,     // SnpOpcode[4:0]
    output reg          txsnp_tgt_id,
    input  wire         txsnp_rdy,

    // ── RXRSP : clean snoop responses from cores ─────────────
    // (RSP channel: SnpRespI, SnpRespSC — no data)
    input  wire         rxrsp_valid,
    input  wire [3:0]   rxrsp_opcode,     // RspOpcode[3:0]
    input  wire         rxrsp_src_id,
    output reg          rxrsp_lcrd,

    // ── RXSNPDAT : dirty snoop responses from cores ──────────
    // (DAT channel: SnpRespData, SnpRespDataFwd — carries dirty block)
    input  wire         rxsnpdat_valid,
    input  wire [2:0]   rxsnpdat_opcode,  // DatOpcode[2:0]
    input  wire [127:0] rxsnpdat_data,
    input  wire [2:0]   rxsnpdat_resp,    // Resp[2:0]: RESP_I / RESP_SC / RESP_SD
    input  wire         rxsnpdat_src_id,
    output reg          rxsnpdat_lcrd,

    // ── DRAM interface ────────────────────────────────────────
    output wire [31:0]  dram_addr,
    output wire [127:0] dram_wdata,
    output wire         dram_wen,
    output wire         dram_ren,
    input  wire [127:0] dram_rdata,
    input  wire         dram_ack,

    // ── L2 performance counters ───────────────────────────────
    output wire [31:0]  l2_perf_hits,
    output wire [31:0]  l2_perf_misses
);

// ── L2 interface registers ────────────────────────────────────
reg  [31:0]  l2_addr_r;
reg  [127:0] l2_wdata_r;
reg          l2_wen_r;
reg          l2_ren_r;
wire [127:0] l2_rdata_i;
wire         l2_ack_i;

l2_cache u_l2 (
    .clk         (clk),
    .rst_n       (rst_n),
    .req_addr    (l2_addr_r),
    .req_wdata   (l2_wdata_r),
    .req_wen     (l2_wen_r),
    .req_ren     (l2_ren_r),
    .req_rdata   (l2_rdata_i),
    .req_ack     (l2_ack_i),
    .dram_addr   (dram_addr),
    .dram_wdata  (dram_wdata),
    .dram_wen    (dram_wen),
    .dram_ren    (dram_ren),
    .dram_rdata  (dram_rdata),
    .dram_ack    (dram_ack),
    .perf_hits   (l2_perf_hits),
    .perf_misses (l2_perf_misses)
);

// ── Snoop Filter ──────────────────────────────────────────────
reg [1:0] sf [0:1023];

// ── Latched request ───────────────────────────────────────────
reg         req_src_id_r;
reg [31:0]  req_addr_r;
reg [5:0]   req_opc_r;
reg [127:0] req_wdata_r;

wire        peer_id    = ~req_src_id_r;
wire [9:0]  req_idx    = req_addr_r[13:4];
wire        peer_in_sf = (peer_id == 1'b0) ? sf[req_idx][0] : sf[req_idx][1];

// What kind of completion to send the requester (set in DECODE)
// true = send data (DAT_CompData), false = send permission only (RSP_Comp)
reg         resp_has_data;
reg [2:0]   resp_dat_resp;   // RESP_SC or RESP_UC, used when resp_has_data=1

// ── FSM states ────────────────────────────────────────────────
localparam HNF_IDLE      = 4'd0;
localparam HNF_DECODE    = 4'd1;
localparam HNF_SNOOP     = 4'd2;
localparam HNF_WAIT_SRSP = 4'd3;
localparam HNF_WB_L2     = 4'd4;
localparam HNF_FETCH_L2  = 4'd5;
localparam HNF_COMP      = 4'd6;
localparam HNF_WAIT_ACK  = 4'd7;
localparam HNF_CU_WB     = 4'd8;  // CleanUnique: write requester data to L2, then snoop/comp

reg [3:0] state;
integer i;

// ── Snoop response received helper ────────────────────────────
// Either a clean RSP or a dirty DAT constitutes the snoop response
wire snp_resp_arrived = rxrsp_valid | rxsnpdat_valid;

// Dirty data is available when rxsnpdat_valid
wire snp_resp_has_data = rxsnpdat_valid;
wire snp_resp_fwd      = rxsnpdat_valid &&
                         (rxsnpdat_opcode == `DAT_SnpRespDataFwd);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= HNF_IDLE;
        rxreq_lcrd      <= 1'b0;
        rxrsp_lcrd      <= 1'b0;
        rxsnpdat_lcrd   <= 1'b0;
        txdat_valid     <= 1'b0;
        txrsp_valid     <= 1'b0;
        txsnp_valid     <= 1'b0;
        l2_wen_r        <= 1'b0;
        l2_ren_r        <= 1'b0;
        for (i = 0; i < 1024; i = i + 1) sf[i] <= 2'b0;
    end else begin
        rxreq_lcrd    <= 1'b0;
        rxrsp_lcrd    <= 1'b0;
        rxsnpdat_lcrd <= 1'b0;

        case (state)

            HNF_IDLE: begin
                if (rxreq_valid) begin
                    req_src_id_r <= rxreq_src_id;
                    req_addr_r   <= rxreq_addr;
                    req_opc_r    <= rxreq_opcode;
                    req_wdata_r  <= rxreq_wdata;
                    rxreq_lcrd   <= 1'b1;
                    state        <= HNF_DECODE;
                end
            end

            HNF_DECODE: begin
                case (req_opc_r)

                    `REQ_ReadShared: begin
                        // Requester gets SC; data response
                        resp_has_data <= 1'b1;
                        resp_dat_resp <= `RESP_SC;
                        if (peer_in_sf) begin
                            txsnp_valid  <= 1'b1;
                            txsnp_addr   <= req_addr_r;
                            txsnp_opcode <= `SNP_SnpSharedFwd;
                            txsnp_tgt_id <= peer_id;
                            state        <= HNF_SNOOP;
                        end else begin
                            l2_addr_r <= req_addr_r;
                            l2_ren_r  <= 1'b1;
                            state     <= HNF_FETCH_L2;
                        end
                    end

                    `REQ_ReadUnique: begin
                        // Requester gets UC; data response
                        resp_has_data <= 1'b1;
                        resp_dat_resp <= `RESP_UC;
                        if (peer_in_sf) begin
                            txsnp_valid  <= 1'b1;
                            txsnp_addr   <= req_addr_r;
                            txsnp_opcode <= `SNP_SnpUnique;
                            txsnp_tgt_id <= peer_id;
                            state        <= HNF_SNOOP;
                        end else begin
                            l2_addr_r <= req_addr_r;
                            l2_ren_r  <= 1'b1;
                            state     <= HNF_FETCH_L2;
                        end
                    end

                    `REQ_CleanUnique: begin
                        // Permission only; requester SC→UD
                        resp_has_data <= 1'b0;
                        if (peer_in_sf) begin
                            txsnp_valid  <= 1'b1;
                            txsnp_addr   <= req_addr_r;
                            txsnp_opcode <= `SNP_SnpUnique;
                            txsnp_tgt_id <= peer_id;
                            state        <= HNF_SNOOP;
                        end else begin
                            state <= HNF_COMP;
                        end
                    end

                    `REQ_MakeUnique: begin
                        // Full-line store from I; requester does NOT need old data.
                        // Spec: peer's dirty copy must be DISCARDED, not written back.
                        // We still send SnpUnique but in HNF_WAIT_SRSP we drop any
                        // dirty data received — no L2 writeback for MakeUnique.
                        resp_has_data <= 1'b0;
                        if (peer_in_sf) begin
                            txsnp_valid  <= 1'b1;
                            txsnp_addr   <= req_addr_r;
                            txsnp_opcode <= `SNP_SnpUnique;
                            txsnp_tgt_id <= peer_id;
                            state        <= HNF_SNOOP;
                        end else begin
                            state <= HNF_COMP;
                        end
                    end

                    `REQ_WriteBackFull: begin
                        // Write dirty data to L2; permission only Comp back
                        l2_addr_r     <= req_addr_r;
                        l2_wdata_r    <= req_wdata_r;
                        l2_wen_r      <= 1'b1;
                        resp_has_data <= 1'b0;
                        state         <= HNF_WB_L2;
                    end

                    `REQ_Evict: begin
                        // Clean eviction: just clear SF; Comp back
                        resp_has_data <= 1'b0;
                        state         <= HNF_COMP;
                    end

                    `REQ_CleanShared: begin
                        resp_has_data <= 1'b0;
                        if (peer_in_sf) begin
                            txsnp_valid  <= 1'b1;
                            txsnp_addr   <= req_addr_r;
                            txsnp_opcode <= `SNP_SnpClean;
                            txsnp_tgt_id <= peer_id;
                            state        <= HNF_SNOOP;
                        end else begin
                            state <= HNF_COMP;
                        end
                    end

                    default: state <= HNF_IDLE;
                endcase
            end

            HNF_SNOOP: begin
                if (txsnp_rdy) begin
                    txsnp_valid <= 1'b0;
                    state       <= HNF_WAIT_SRSP;
                end
            end

            HNF_WAIT_SRSP: begin
                if (snp_resp_arrived) begin
                    // Acknowledge whichever channel arrived
                    if (rxrsp_valid)    rxrsp_lcrd    <= 1'b1;
                    if (rxsnpdat_valid) rxsnpdat_lcrd <= 1'b1;

                    // Clear peer SF: SnpUnique always forces I.
                    // SnpSharedFwd/SnpShared: clear only if peer responded I.
                    if (txsnp_opcode == `SNP_SnpUnique ||
                        (rxrsp_valid && rxrsp_opcode == `RSP_SnpRespI)) begin
                        if (peer_id == 1'b0) sf[req_idx] <= sf[req_idx] & 2'b10;
                        else                  sf[req_idx] <= sf[req_idx] & 2'b01;
                    end

                    if (snp_resp_has_data) begin
                        if (req_opc_r == `REQ_ReadUnique) begin
                            // Peer had dirty data: forward directly to requester as UD.
                            // No L2 write — requester owns the dirty line; L2 updated
                            // later via WriteBackFull when requester evicts.
                            txdat_data    <= rxsnpdat_data;
                            resp_dat_resp <= `RESP_UD;
                            state         <= HNF_COMP;
                        end else if (req_opc_r == `REQ_MakeUnique) begin
                            // MakeUnique: spec requires peer dirty data to be DISCARDED
                            state <= HNF_COMP;
                        end else begin
                            // ReadShared / CleanUnique: write dirty data to L2
                            l2_addr_r  <= req_addr_r;
                            l2_wdata_r <= rxsnpdat_data;
                            l2_wen_r   <= 1'b1;
                            state      <= HNF_WB_L2;
                        end
                    end else if (resp_has_data) begin
                        // Clean snoop response but requester needs data → fetch from L2
                        l2_addr_r <= req_addr_r;
                        l2_ren_r  <= 1'b1;
                        state     <= HNF_FETCH_L2;
                    end else begin
                        state <= HNF_COMP;
                    end
                end
            end

            HNF_WB_L2: begin
                l2_wen_r <= 1'b0;
                if (l2_ack_i) begin
                    if (resp_has_data) begin
                        // After writing dirty data, fetch clean copy for requester
                        l2_addr_r <= req_addr_r;
                        l2_ren_r  <= 1'b1;
                        state     <= HNF_FETCH_L2;
                    end else begin
                        state <= HNF_COMP;
                    end
                end
            end

            HNF_FETCH_L2: begin
                l2_ren_r <= 1'b0;
                if (l2_ack_i) begin
                    txdat_data <= l2_rdata_i;
                    state      <= HNF_COMP;
                end
            end

            HNF_COMP: begin
                if (resp_has_data) begin
                    txdat_valid  <= 1'b1;
                    txdat_opcode <= `DAT_CompData;
                    txdat_resp   <= resp_dat_resp;   // RESP_SC or RESP_UC
                    txdat_tgt_id <= req_src_id_r;
                end else begin
                    txrsp_valid  <= 1'b1;
                    txrsp_opcode <= `RSP_Comp;
                    txrsp_tgt_id <= req_src_id_r;
                end
                state <= HNF_WAIT_ACK;
            end

            HNF_WAIT_ACK: begin
                if ((txdat_valid && txdat_ack) ||
                    (txrsp_valid && txrsp_ack)) begin
                    txdat_valid <= 1'b0;
                    txrsp_valid <= 1'b0;

                    // Update SF based on final requester state
                    case (req_opc_r)
                        `REQ_ReadShared: begin
                            // Requester gets SC
                            if (req_src_id_r == 1'b0) sf[req_idx] <= sf[req_idx] | 2'b01;
                            else                        sf[req_idx] <= sf[req_idx] | 2'b10;
                        end
                        `REQ_ReadUnique, `REQ_CleanUnique, `REQ_MakeUnique: begin
                            // Requester gets exclusive (UD); only it has the line
                            sf[req_idx] <= (req_src_id_r == 1'b0) ? 2'b01 : 2'b10;
                        end
                        `REQ_WriteBackFull, `REQ_Evict: begin
                            // Requester drops the line
                            if (req_src_id_r == 1'b0) sf[req_idx] <= sf[req_idx] & 2'b10;
                            else                        sf[req_idx] <= sf[req_idx] & 2'b01;
                        end
                        default: ;
                    endcase

                    state <= HNF_IDLE;
                end
            end

            default: state <= HNF_IDLE;
        endcase
    end
end

endmodule
