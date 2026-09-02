`timescale 1ns/1ps
`include "cache_defines.v"
`include "chi_defines.v"

// ============================================================
// core.v  —  CHI Request Node (Core + L1 Cache + CHI Engine)
//
// Hierarchy:
//   core
//   └── l1_cache  u_l1   (4KB direct-mapped, 128-bit blocks)
//
// CHI Channel widths (CHI-B):
//   TXREQ : ReqOpcode[5:0]
//   RXDAT : DatOpcode[2:0] + Resp[2:0]   (CompData with cache state)
//   RXRSP : RspOpcode[3:0]               (Comp — permission only)
//   RXSNP : SnpOpcode[4:0]
//   TXRSP : RspOpcode[3:0]               (SnpRespI / SnpRespSC — no data)
//   TXSNPDAT : DatOpcode[2:0] + Resp[2:0] (SnpRespData / SnpRespDataFwd — dirty data)
//
// Snoop response split:
//   Clean response  → TXRSP     (RSP channel, no dirty data)
//   Dirty response  → TXSNPDAT  (DAT channel, carries dirty block to HN)
// ============================================================

module core (
    input  wire        clk,
    input  wire        rst_n,

    // ── Core CPU interface ────────────────────────────────────
    input  wire [31:0]  core_addr,
    input  wire [31:0]  core_wdata,
    input  wire         core_wen,
    input  wire         core_ren,
    input  wire [3:0]   core_be,
    output reg  [31:0]  core_rdata,
    output wire         core_stall,

    // ── TXREQ : RN → HN requests ─────────────────────────────
    output reg          txreq_valid,
    output reg  [31:0]  txreq_addr,
    output reg  [5:0]   txreq_opcode,     // ReqOpcode[5:0]
    output reg  [127:0] txreq_wdata,      // dirty data for WriteBackFull (simplified)
    input  wire         txreq_rdy,

    // ── RXDAT : HN → RN data responses ───────────────────────
    input  wire         rxdat_valid,
    input  wire [2:0]   rxdat_opcode,     // DatOpcode[2:0]
    input  wire [127:0] rxdat_data,
    input  wire [2:0]   rxdat_resp,       // Resp[2:0]: RESP_SC / RESP_UC / RESP_UD
    output reg          rxdat_ack,

    // ── RXRSP : HN → RN permission responses ─────────────────
    input  wire         rxrsp_valid,
    input  wire [3:0]   rxrsp_opcode,     // RspOpcode[3:0]
    output reg          rxrsp_ack,

    // ── RXSNP : HN → RN snoops ───────────────────────────────
    input  wire         rxsnp_valid,
    input  wire [31:0]  rxsnp_addr,
    input  wire [4:0]   rxsnp_opcode,     // SnpOpcode[4:0]
    output wire         rxsnp_rdy,

    // ── TXRSP : RN → HN snoop response (clean, no data) ──────
    output reg          txrsp_valid,
    output reg  [3:0]   txrsp_opcode,     // RSP_SnpRespI | RSP_SnpRespSC
    input  wire         txrsp_rdy,

    // ── TXSNPDAT : RN → HN snoop response (dirty, has data) ──
    output reg          txsnpdat_valid,
    output reg  [2:0]   txsnpdat_opcode,  // DAT_SnpRespData | DAT_SnpRespDataFwd
    output reg  [127:0] txsnpdat_data,
    output reg  [2:0]   txsnpdat_resp,    // RESP_I (went I) | RESP_SD (went SD)
    input  wire         txsnpdat_rdy,

    // ── Performance counters ──────────────────────────────────
    output reg  [31:0]  perf_hits,
    output reg  [31:0]  perf_misses
);

// ── Latched core request ──────────────────────────────────────
reg [31:0]  req_core_addr;
reg [31:0]  req_core_wdata;
reg         req_core_wen;
reg [3:0]   req_core_be;

wire [19:0] req_tag  = req_core_addr[31:12];
wire [7:0]  req_idx  = req_core_addr[11:4];
wire [1:0]  req_woff = req_core_addr[3:2];

// ── Latched snoop ─────────────────────────────────────────────
reg [31:0] snp_addr_r;
reg [4:0]  snp_opc_r;
wire [19:0] snp_tag = snp_addr_r[31:12];
wire [7:0]  snp_idx = snp_addr_r[11:4];

// ── FSM states ────────────────────────────────────────────────
localparam S_IDLE             = 4'd0;
localparam S_CHECK            = 4'd1;
localparam S_EVICT_REQ        = 4'd2;
localparam S_EVICT_WAIT       = 4'd3;
localparam S_REQ              = 4'd4;
localparam S_WAIT_COMP        = 4'd5;
localparam S_RESP             = 4'd6;
localparam S_SNP              = 4'd7;
localparam S_SNP_RSP          = 4'd8;
localparam S_EVICT_CLEAN      = 4'd9;
localparam S_EVICT_CLEAN_WAIT = 4'd10;
localparam S_LOCAL_WR         = 4'd11;  // apply pending CPU write to UC line locally

reg [3:0] state;

assign core_stall = ((state != S_IDLE) && (state != S_RESP)) ||
                    ((state == S_IDLE) && (core_ren | core_wen));
assign rxsnp_rdy  = (state == S_IDLE);

// ── Eviction context ──────────────────────────────────────────
reg [31:0]  evict_addr_r;
reg [127:0] evict_data_r;
reg [31:0]  evict_clean_addr_r;
reg [5:0]   pending_opcode;

// ============================================================
// L1 cache instance
// ============================================================
reg [7:0]   l1_wr_idx;
reg [127:0] l1_wr_data;
reg         l1_wr_data_en;
reg [19:0]  l1_wr_tag;
reg         l1_wr_tag_en;
reg [2:0]   l1_wr_cs;
reg         l1_wr_cs_en;

wire         l1_a_hit;
wire [127:0] l1_a_line;
wire [2:0]   l1_a_cs;
wire [19:0]  l1_a_stored_tag;
wire         l1_b_hit;
wire [127:0] l1_b_line;
wire [2:0]   l1_b_cs;

l1_cache u_l1 (
    .clk          (clk),
    .rst_n        (rst_n),
    .a_idx        (req_idx),
    .a_tag        (req_tag),
    .a_hit        (l1_a_hit),
    .a_line       (l1_a_line),
    .a_cs         (l1_a_cs),
    .a_stored_tag (l1_a_stored_tag),
    .b_idx        (snp_idx),
    .b_tag        (snp_tag),
    .b_hit        (l1_b_hit),
    .b_line       (l1_b_line),
    .b_cs         (l1_b_cs),
    .wr_idx       (l1_wr_idx),
    .wr_data      (l1_wr_data),
    .wr_data_en   (l1_wr_data_en),
    .wr_tag       (l1_wr_tag),
    .wr_tag_en    (l1_wr_tag_en),
    .wr_cs        (l1_wr_cs),
    .wr_cs_en     (l1_wr_cs_en)
);

// ── Word extraction from cache line ───────────────────────────
reg [31:0] data_word_sel;
always @(*) begin
    case (req_woff)
        2'd0:    data_word_sel = l1_a_line[31:0];
        2'd1:    data_word_sel = l1_a_line[63:32];
        2'd2:    data_word_sel = l1_a_line[95:64];
        default: data_word_sel = l1_a_line[127:96];
    endcase
end

reg [31:0] rxdat_word_sel;
always @(*) begin
    case (req_woff)
        2'd0:    rxdat_word_sel = rxdat_data[31:0];
        2'd1:    rxdat_word_sel = rxdat_data[63:32];
        2'd2:    rxdat_word_sel = rxdat_data[95:64];
        default: rxdat_word_sel = rxdat_data[127:96];
    endcase
end

// ── Byte-enable write merge ───────────────────────────────────
reg [127:0] merged_from_arr;
always @(*) begin
    merged_from_arr = l1_a_line;
    case (req_woff)
        2'd0: begin
            if (req_core_be[0]) merged_from_arr[7:0]     = req_core_wdata[7:0];
            if (req_core_be[1]) merged_from_arr[15:8]    = req_core_wdata[15:8];
            if (req_core_be[2]) merged_from_arr[23:16]   = req_core_wdata[23:16];
            if (req_core_be[3]) merged_from_arr[31:24]   = req_core_wdata[31:24];
        end
        2'd1: begin
            if (req_core_be[0]) merged_from_arr[39:32]   = req_core_wdata[7:0];
            if (req_core_be[1]) merged_from_arr[47:40]   = req_core_wdata[15:8];
            if (req_core_be[2]) merged_from_arr[55:48]   = req_core_wdata[23:16];
            if (req_core_be[3]) merged_from_arr[63:56]   = req_core_wdata[31:24];
        end
        2'd2: begin
            if (req_core_be[0]) merged_from_arr[71:64]   = req_core_wdata[7:0];
            if (req_core_be[1]) merged_from_arr[79:72]   = req_core_wdata[15:8];
            if (req_core_be[2]) merged_from_arr[87:80]   = req_core_wdata[23:16];
            if (req_core_be[3]) merged_from_arr[95:88]   = req_core_wdata[31:24];
        end
        default: begin
            if (req_core_be[0]) merged_from_arr[103:96]  = req_core_wdata[7:0];
            if (req_core_be[1]) merged_from_arr[111:104] = req_core_wdata[15:8];
            if (req_core_be[2]) merged_from_arr[119:112] = req_core_wdata[23:16];
            if (req_core_be[3]) merged_from_arr[127:120] = req_core_wdata[31:24];
        end
    endcase
end

// ============================================================
// Combinational L1 write port driver
// ============================================================
always @(*) begin
    l1_wr_idx     = req_idx;
    l1_wr_data    = 128'd0;
    l1_wr_data_en = 1'b0;
    l1_wr_tag     = req_tag;
    l1_wr_tag_en  = 1'b0;
    l1_wr_cs      = `STATE_I;
    l1_wr_cs_en   = 1'b0;

    case (state)

        S_CHECK: begin
            l1_wr_idx = req_idx;
            if (l1_a_hit) begin
                if (req_core_wen &&
                    (l1_a_cs == `STATE_UD || l1_a_cs == `STATE_UC)) begin
                    l1_wr_data    = merged_from_arr;
                    l1_wr_data_en = 1'b1;
                    l1_wr_cs      = `STATE_UD;
                    l1_wr_cs_en   = 1'b1;
                end
            end else begin
                if (l1_a_cs != `STATE_I) begin
                    l1_wr_cs    = `STATE_I;
                    l1_wr_cs_en = 1'b1;
                end
            end
        end

        S_WAIT_COMP: begin
            l1_wr_idx = req_idx;
            if (rxdat_valid) begin
                l1_wr_tag    = req_tag;
                l1_wr_tag_en = 1'b1;
                if (rxdat_resp == `RESP_SC) begin
                    // ReadShared: install data as SC
                    l1_wr_data    = rxdat_data;
                    l1_wr_data_en = 1'b1;
                    l1_wr_cs      = `STATE_SC;
                    l1_wr_cs_en   = 1'b1;
                end else begin
                    // ReadUnique: install as UD if HN forwarded dirty data (RESP_UD),
                    // or UC if data came from clean L2 (RESP_UC).
                    // CPU write applied separately in S_LOCAL_WR.
                    l1_wr_data    = rxdat_data;
                    l1_wr_data_en = 1'b1;
                    l1_wr_cs      = (rxdat_resp == `RESP_UD) ? `STATE_UD : `STATE_UC;
                    l1_wr_cs_en   = 1'b1;
                end
            end else if (rxrsp_valid) begin
                // RSP_Comp for CleanUnique: data already in L1; upgrade cs to UC only.
                // CPU write applied separately in S_LOCAL_WR.
                l1_wr_cs    = `STATE_UC;
                l1_wr_cs_en = 1'b1;
            end
        end

        S_LOCAL_WR: begin
            // Line is UC in L1; apply pending CPU write locally → UD
            l1_wr_idx     = req_idx;
            l1_wr_data    = merged_from_arr;
            l1_wr_data_en = 1'b1;
            l1_wr_cs      = `STATE_UD;
            l1_wr_cs_en   = 1'b1;
        end

        S_SNP: begin
            l1_wr_idx = snp_idx;
            if (l1_b_hit) begin
                case (snp_opc_r)
                    `SNP_SnpShared: begin
                        l1_wr_cs    = `STATE_SC;
                        l1_wr_cs_en = 1'b1;
                    end
                    `SNP_SnpUnique: begin
                        l1_wr_cs    = `STATE_I;
                        l1_wr_cs_en = 1'b1;
                    end
                    `SNP_SnpSharedFwd: begin
                        // UD→SD; SC stays SC
                        l1_wr_cs    = (l1_b_cs == `STATE_UD) ? `STATE_SD : `STATE_SC;
                        l1_wr_cs_en = 1'b1;
                    end
                    `SNP_SnpClean: begin
                        l1_wr_cs    = `STATE_SC;
                        l1_wr_cs_en = 1'b1;
                    end
                    default: ;
                endcase
            end
        end

        default: ;
    endcase
end

// ============================================================
// FSM — sequential
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= S_IDLE;
        txreq_valid     <= 1'b0;
        rxdat_ack       <= 1'b0;
        rxrsp_ack       <= 1'b0;
        txrsp_valid     <= 1'b0;
        txsnpdat_valid  <= 1'b0;
        perf_hits       <= 32'd0;
        perf_misses     <= 32'd0;
        core_rdata      <= 32'd0;
    end else begin
        txreq_valid    <= 1'b0;
        rxdat_ack      <= 1'b0;
        rxrsp_ack      <= 1'b0;
        txrsp_valid    <= 1'b0;
        txsnpdat_valid <= 1'b0;

        case (state)

            S_IDLE: begin
                if (rxsnp_valid) begin
                    snp_addr_r <= rxsnp_addr;
                    snp_opc_r  <= rxsnp_opcode;
                    state      <= S_SNP;
                end else if (core_ren | core_wen) begin
                    req_core_addr  <= core_addr;
                    req_core_wdata <= core_wdata;
                    req_core_wen   <= core_wen;
                    req_core_be    <= core_be;
                    state          <= S_CHECK;
                end
            end

            S_CHECK: begin
                if (l1_a_hit) begin
                    perf_hits <= perf_hits + 1;
                    if (req_core_wen) begin
                        case (l1_a_cs)
                            `STATE_UD, `STATE_UC: begin
                                state <= S_RESP;
                            end
                            `STATE_SD: begin
                                // SD→UC: requester holds dirty data; must write it back
                                // to HN so it becomes clean before going unique.
                                pending_opcode <= `REQ_CleanUnique;
                                txreq_valid    <= 1'b1;
                                txreq_addr     <= {req_tag, req_idx, 4'b0000};
                                txreq_opcode   <= `REQ_CleanUnique;
                                txreq_wdata    <= l1_a_line;  // dirty data → HN writes to L2
                                state          <= S_REQ;
                            end
                            default: begin
                                // SC→UC: send current clean line to HN so it can
                                // update L2 (harmless overwrite; needed for protocol).
                                pending_opcode <= `REQ_CleanUnique;
                                txreq_valid    <= 1'b1;
                                txreq_addr     <= {req_tag, req_idx, 4'b0000};
                                txreq_opcode   <= `REQ_CleanUnique;
                                txreq_wdata    <= l1_a_line;
                                state          <= S_REQ;
                            end
                        endcase
                    end else begin
                        core_rdata <= data_word_sel;
                        state      <= S_RESP;
                    end
                end else begin
                    perf_misses    <= perf_misses + 1;
                    // Always ReadUnique for write miss: our CPU interface writes one
                    // 32-bit word at a time, so we always need the other 96 bits from
                    // the existing cache line before we can merge the write.
                    // MakeUnique (spec: full cache-line store, all 16 bytes overwritten)
                    // is never appropriate here — the CPU BE granularity is 4 bytes.
                    pending_opcode <= req_core_wen ? `REQ_ReadUnique : `REQ_ReadShared;

                    if (l1_a_cs == `STATE_UD || l1_a_cs == `STATE_SD) begin
                        // Dirty eviction: WriteBackFull
                        evict_addr_r <= {l1_a_stored_tag, req_idx, 4'b0000};
                        evict_data_r <= l1_a_line;
                        txreq_valid  <= 1'b1;
                        txreq_addr   <= {l1_a_stored_tag, req_idx, 4'b0000};
                        txreq_opcode <= `REQ_WriteBackFull;
                        txreq_wdata  <= l1_a_line;
                        state        <= S_EVICT_REQ;
                    end else if (l1_a_cs != `STATE_I) begin
                        // SC/UC: clean eviction via Evict
                        evict_clean_addr_r <= {l1_a_stored_tag, req_idx, 4'b0000};
                        txreq_valid  <= 1'b1;
                        txreq_addr   <= {l1_a_stored_tag, req_idx, 4'b0000};
                        txreq_opcode <= `REQ_Evict;
                        txreq_wdata  <= 128'd0;
                        state        <= S_EVICT_CLEAN;
                    end else begin
                        txreq_valid  <= 1'b1;
                        txreq_addr   <= {req_tag, req_idx, 4'b0000};
                        txreq_opcode <= req_core_wen ? `REQ_ReadUnique : `REQ_ReadShared;
                        txreq_wdata  <= 128'd0;
                        state        <= S_REQ;
                    end
                end
            end

            S_EVICT_REQ: begin
                txreq_valid  <= 1'b1;
                txreq_addr   <= evict_addr_r;
                txreq_opcode <= `REQ_WriteBackFull;
                txreq_wdata  <= evict_data_r;
                if (txreq_rdy) begin
                    txreq_valid <= 1'b0;
                    state       <= S_EVICT_WAIT;
                end
            end

            S_EVICT_WAIT: begin
                if (rxrsp_valid) begin
                    rxrsp_ack    <= 1'b1;
                    txreq_valid  <= 1'b1;
                    txreq_addr   <= {req_tag, req_idx, 4'b0000};
                    txreq_opcode <= pending_opcode;
                    txreq_wdata  <= 128'd0;
                    state        <= S_REQ;
                end
            end

            S_EVICT_CLEAN: begin
                txreq_valid  <= 1'b1;
                txreq_addr   <= evict_clean_addr_r;
                txreq_opcode <= `REQ_Evict;
                txreq_wdata  <= 128'd0;
                if (txreq_rdy) begin
                    txreq_valid <= 1'b0;
                    state       <= S_EVICT_CLEAN_WAIT;
                end
            end

            S_EVICT_CLEAN_WAIT: begin
                if (rxrsp_valid) begin
                    rxrsp_ack    <= 1'b1;
                    txreq_valid  <= 1'b1;
                    txreq_addr   <= {req_tag, req_idx, 4'b0000};
                    txreq_opcode <= pending_opcode;
                    txreq_wdata  <= 128'd0;
                    state        <= S_REQ;
                end
            end

            S_REQ: begin
                txreq_valid <= 1'b1;
                if (txreq_rdy) begin
                    txreq_valid <= 1'b0;
                    state       <= S_WAIT_COMP;
                end
            end

            S_WAIT_COMP: begin
                // Comp with data (ReadShared / ReadUnique)
                if (rxdat_valid) begin
                    rxdat_ack <= 1'b1;
                    if (rxdat_resp == `RESP_SC) begin
                        // ReadShared: line installed as SC; return read data to CPU
                        core_rdata <= rxdat_word_sel;
                        state      <= S_RESP;
                    end else begin
                        // ReadUnique: line installed as UC; apply CPU write in S_LOCAL_WR
                        state <= S_LOCAL_WR;
                    end
                end
                // Comp without data (CleanUnique / Evict / WBF)
                else if (rxrsp_valid) begin
                    rxrsp_ack <= 1'b1;
                    if (pending_opcode == `REQ_CleanUnique)
                        state <= S_LOCAL_WR;  // line now UC; apply CPU write locally
                    else
                        state <= S_RESP;
                end
            end

            S_LOCAL_WR: begin
                // One cycle to register the CPU write into the UC line; then done
                state <= S_RESP;
            end

            S_RESP: state <= S_IDLE;

            // ── Snoop handling ───────────────────────────────────
            S_SNP: begin
                if (l1_b_hit) begin
                    case (snp_opc_r)
                        `SNP_SnpShared: begin
                            if (l1_b_cs == `STATE_UD || l1_b_cs == `STATE_SD) begin
                                // Dirty data → HN on DAT channel; line → SC
                                txsnpdat_valid  <= 1'b1;
                                txsnpdat_opcode <= `DAT_SnpRespData;
                                txsnpdat_data   <= l1_b_line;
                                txsnpdat_resp   <= `RESP_SC;  // snoopee retains SC
                            end else begin
                                // Clean SC → stays SC, no data needed
                                txrsp_valid  <= 1'b1;
                                txrsp_opcode <= `RSP_SnpRespSC;
                            end
                        end
                        `SNP_SnpUnique: begin
                            if (l1_b_cs == `STATE_UD || l1_b_cs == `STATE_SD) begin
                                // Dirty data → HN; line → I
                                txsnpdat_valid  <= 1'b1;
                                txsnpdat_opcode <= `DAT_SnpRespData;
                                txsnpdat_data   <= l1_b_line;
                                txsnpdat_resp   <= `RESP_I;
                            end else begin
                                txrsp_valid  <= 1'b1;
                                txrsp_opcode <= `RSP_SnpRespI;
                            end
                        end
                        `SNP_SnpSharedFwd: begin
                            if (l1_b_cs == `STATE_UD) begin
                                // UD: forward dirty data; line → SD
                                txsnpdat_valid  <= 1'b1;
                                txsnpdat_opcode <= `DAT_SnpRespDataFwd;
                                txsnpdat_data   <= l1_b_line;
                                txsnpdat_resp   <= `RESP_SD;  // snoopee keeps SD
                            end else begin
                                // SC: stays SC, no data forwarded
                                txrsp_valid  <= 1'b1;
                                txrsp_opcode <= `RSP_SnpRespSC;
                            end
                        end
                        `SNP_SnpClean: begin
                            if (l1_b_cs == `STATE_UD || l1_b_cs == `STATE_SD) begin
                                txsnpdat_valid  <= 1'b1;
                                txsnpdat_opcode <= `DAT_SnpRespData;
                                txsnpdat_data   <= l1_b_line;
                                txsnpdat_resp   <= `RESP_SC;
                            end else begin
                                txrsp_valid  <= 1'b1;
                                txrsp_opcode <= `RSP_SnpRespSC;
                            end
                        end
                        default: begin
                            txrsp_valid  <= 1'b1;
                            txrsp_opcode <= `RSP_SnpRespI;
                        end
                    endcase
                end else begin
                    // No hit: respond I on RSP channel
                    txrsp_valid  <= 1'b1;
                    txrsp_opcode <= `RSP_SnpRespI;
                end
                state <= S_SNP_RSP;
            end

            S_SNP_RSP: begin
                // Hold until whichever response channel was used is accepted
                if (txrsp_valid) begin
                    txrsp_valid <= 1'b1;
                    if (txrsp_rdy) begin
                        txrsp_valid <= 1'b0;
                        state       <= S_IDLE;
                    end
                end else if (txsnpdat_valid) begin
                    txsnpdat_valid <= 1'b1;
                    if (txsnpdat_rdy) begin
                        txsnpdat_valid <= 1'b0;
                        state          <= S_IDLE;
                    end
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
