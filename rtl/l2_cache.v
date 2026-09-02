`timescale 1ns/1ps
`include "cache_defines.v"

// ============================================================
// l2_cache.v  —  L2 4-Way Set-Associative Write-Back Cache
//
// Capacity   : 64KB, 1024 sets, 4 ways, 16-byte (128-bit) blocks
// Policy     : Write-back + write-allocate
// Replacement: 3-bit Pseudo-LRU (PLRU) tree per set
// Interface  : coherence_manager (block-level, 128-bit) ↔ DRAM
//
// Address decomposition (32-bit byte address):
//   [31:14]  tag        (18 bits)
//   [13:4]   set index  (10 bits → 1024 sets)
//   [3:2]    word offset(2  bits → 32-bit word within 128-bit block)
//   [1:0]    byte offset(2  bits)
//
// PLRU tree per set (3 bits: {b2, b1, b0}):
//           b2
//          /  \
//        b1    b0
//        /\    /\
//      W0 W1  W2 W3
//
// FSM: IDLE → CHECK → [EVICT →] [DRAM_FILL →] INSTALL → RESP → IDLE
// ============================================================

module l2_cache (
    input  wire        clk,
    input  wire        rst_n,

    // ── Upstream interface (from coherence_manager, 128-bit) ─────
    input  wire [31:0]  req_addr,
    input  wire [127:0] req_wdata,
    input  wire         req_wen,
    input  wire         req_ren,
    output reg  [127:0] req_rdata,
    output reg          req_ack,

    // ── DRAM interface (block-granular, 128-bit) ──────────────
    output reg  [31:0]  dram_addr,
    output reg  [127:0] dram_wdata,
    output reg          dram_wen,
    output reg          dram_ren,
    input  wire [127:0] dram_rdata,
    input  wire         dram_ack,

    // ── Performance counters ──────────────────────────────────
    output reg  [31:0]  perf_hits,
    output reg  [31:0]  perf_misses
);

// ── Latched request ───────────────────────────────────────────
reg [31:0]  req_addr_r;
reg [127:0] req_wdata_r;
reg         req_is_write;

wire [17:0] req_tag = req_addr_r[31:14];
wire [9:0]  req_idx = req_addr_r[13:4];

// ── Storage arrays (1024 sets × 4 ways = 4096 entries) ───────
// Flat index = {req_idx[9:0], way[1:0]}
reg [127:0]  data_arr [0:4095];
reg [17:0]   tag_arr  [0:4095];
reg [4095:0] l2_valid;
reg [4095:0] l2_dirty;
reg [2:0]    plru     [0:1023];

// ── Hit detection ─────────────────────────────────────────────
wire [3:0] way_hit;
assign way_hit[0] = l2_valid[{req_idx, 2'd0}] && (tag_arr[{req_idx, 2'd0}] == req_tag);
assign way_hit[1] = l2_valid[{req_idx, 2'd1}] && (tag_arr[{req_idx, 2'd1}] == req_tag);
assign way_hit[2] = l2_valid[{req_idx, 2'd2}] && (tag_arr[{req_idx, 2'd2}] == req_tag);
assign way_hit[3] = l2_valid[{req_idx, 2'd3}] && (tag_arr[{req_idx, 2'd3}] == req_tag);

wire l2_hit = |way_hit;

reg [1:0] hit_way;
always @(*) begin
    casex (way_hit)
        4'bxxx1: hit_way = 2'd0;
        4'bxx10: hit_way = 2'd1;
        4'bx100: hit_way = 2'd2;
        4'b1000: hit_way = 2'd3;
        default: hit_way = 2'd0;
    endcase
end

// Victim selection: prefer invalid ways, then PLRU
reg [1:0] victim_way;
always @(*) begin
    if      (!l2_valid[{req_idx, 2'd0}]) victim_way = 2'd0;
    else if (!l2_valid[{req_idx, 2'd1}]) victim_way = 2'd1;
    else if (!l2_valid[{req_idx, 2'd2}]) victim_way = 2'd2;
    else if (!l2_valid[{req_idx, 2'd3}]) victim_way = 2'd3;
    else if (plru[req_idx][2] == 1'b0)
        victim_way = (plru[req_idx][0] == 1'b0) ? 2'd2 : 2'd3;
    else
        victim_way = (plru[req_idx][1] == 1'b0) ? 2'd0 : 2'd1;
end

// ── FSM ───────────────────────────────────────────────────────
localparam S_IDLE      = 3'd0;
localparam S_CHECK     = 3'd1;
localparam S_EVICT     = 3'd2;
localparam S_DRAM_FILL = 3'd3;
localparam S_INSTALL   = 3'd4;
localparam S_RESP      = 3'd5;

reg [2:0]   state;
reg [1:0]   sel_way;
reg [127:0] install_data;

integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        req_ack     <= 1'b0;
        req_rdata   <= 128'b0;
        dram_wen    <= 1'b0;
        dram_ren    <= 1'b0;
        l2_valid    <= 4096'b0;
        l2_dirty    <= 4096'b0;
        perf_hits   <= 32'd0;
        perf_misses <= 32'd0;
        for (i = 0; i < 1024; i = i + 1) plru[i] <= 3'd0;
    end else begin
        req_ack <= 1'b0;

        case (state)

            S_IDLE: begin
                dram_wen <= 1'b0;
                dram_ren <= 1'b0;
                if (req_ren | req_wen) begin
                    req_addr_r   <= req_addr;
                    req_wdata_r  <= req_wdata;
                    req_is_write <= req_wen;
                    state        <= S_CHECK;
                end
            end

            S_CHECK: begin
                if (l2_hit) begin
                    perf_hits <= perf_hits + 1;
                    sel_way   <= hit_way;
                    if (req_is_write) begin
                        data_arr[{req_idx, hit_way}] <= req_wdata_r;
                        l2_dirty[{req_idx, hit_way}] <= 1'b1;
                    end else begin
                        req_rdata <= data_arr[{req_idx, hit_way}];
                    end
                    case (hit_way)
                        2'd0: plru[req_idx] <= {1'b0, 1'b0, plru[req_idx][0]};
                        2'd1: plru[req_idx] <= {1'b0, 1'b1, plru[req_idx][0]};
                        2'd2: plru[req_idx] <= {1'b1, plru[req_idx][1], 1'b0};
                        2'd3: plru[req_idx] <= {1'b1, plru[req_idx][1], 1'b1};
                    endcase
                    state <= S_RESP;
                end else begin
                    perf_misses <= perf_misses + 1;
                    sel_way     <= victim_way;
                    if (l2_dirty[{req_idx, victim_way}] &&
                        l2_valid[{req_idx, victim_way}]) begin
                        dram_addr  <= {tag_arr[{req_idx, victim_way}], req_idx, 4'b0};
                        dram_wdata <= data_arr[{req_idx, victim_way}];
                        dram_wen   <= 1'b1;
                        state      <= S_EVICT;
                    end else if (req_is_write) begin
                        install_data <= req_wdata_r;
                        state        <= S_INSTALL;
                    end else begin
                        dram_addr <= {req_tag, req_idx, 4'b0};
                        dram_ren  <= 1'b1;
                        state     <= S_DRAM_FILL;
                    end
                end
            end

            S_EVICT: begin
                if (dram_ack) begin
                    dram_wen <= 1'b0;
                    if (req_is_write) begin
                        install_data <= req_wdata_r;
                        state        <= S_INSTALL;
                    end else begin
                        dram_addr <= {req_tag, req_idx, 4'b0};
                        dram_ren  <= 1'b1;
                        state     <= S_DRAM_FILL;
                    end
                end
            end

            S_DRAM_FILL: begin
                if (dram_ack) begin
                    dram_ren     <= 1'b0;
                    install_data <= dram_rdata;
                    req_rdata    <= dram_rdata;
                    state        <= S_INSTALL;
                end
            end

            S_INSTALL: begin
                data_arr[{req_idx, sel_way}] <= install_data;
                tag_arr [{req_idx, sel_way}] <= req_tag;
                l2_valid[{req_idx, sel_way}] <= 1'b1;
                l2_dirty[{req_idx, sel_way}] <= req_is_write;
                case (sel_way)
                    2'd0: plru[req_idx] <= {1'b0, 1'b0, plru[req_idx][0]};
                    2'd1: plru[req_idx] <= {1'b0, 1'b1, plru[req_idx][0]};
                    2'd2: plru[req_idx] <= {1'b1, plru[req_idx][1], 1'b0};
                    2'd3: plru[req_idx] <= {1'b1, plru[req_idx][1], 1'b1};
                endcase
                state <= S_RESP;
            end

            S_RESP: begin
                req_ack <= 1'b1;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
