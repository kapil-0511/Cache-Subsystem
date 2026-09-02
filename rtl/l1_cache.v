`timescale 1ns/1ps
`include "cache_defines.v"
`include "chi_defines.v"

// ============================================================
// l1_cache.v  —  L1 Cache Storage Array
//
// Capacity : 4KB, 256 lines, direct-mapped, 128-bit blocks
// Provides two combinational read ports (request + snoop)
// and one registered write port driven by the core FSM.
//
// Address decomposition (32-bit byte address):
//   [31:12]  tag         (20 bits)
//   [11:4]   line index  (8 bits → 256 lines)
//   [3:2]    word offset (2 bits)
//   [1:0]    byte offset
// ============================================================

module l1_cache (
    input  wire        clk,
    input  wire        rst_n,

    // ── Port A: request context (combinational read) ──────────
    input  wire [7:0]   a_idx,          // line index for core request
    input  wire [19:0]  a_tag,          // tag to compare for hit check
    output wire         a_hit,          // 1 = valid line with matching tag
    output wire [127:0] a_line,         // full 128-bit cache line at a_idx
    output wire [2:0]   a_cs,           // coherence state at a_idx
    output wire [19:0]  a_stored_tag,   // tag currently stored at a_idx

    // ── Port B: snoop context (combinational read) ────────────
    input  wire [7:0]   b_idx,          // line index for incoming snoop
    input  wire [19:0]  b_tag,          // tag to compare for snoop hit
    output wire         b_hit,          // 1 = valid line matches snoop address
    output wire [127:0] b_line,         // full cache line at b_idx (for dirty data)
    output wire [2:0]   b_cs,           // coherence state at b_idx

    // ── Write port (one active per clock cycle) ───────────────
    input  wire [7:0]   wr_idx,         // line index to update
    input  wire [127:0] wr_data,        // new line data
    input  wire         wr_data_en,     // write data_arr when asserted
    input  wire [19:0]  wr_tag,         // new tag
    input  wire         wr_tag_en,      // write tag_arr when asserted
    input  wire [2:0]   wr_cs,          // new coherence state
    input  wire         wr_cs_en        // write cs_arr when asserted
);

reg [127:0] data_arr [0:255];
reg [19:0]  tag_arr  [0:255];
reg [2:0]   cs_arr   [0:255];
integer     i;

// ── Combinational read port A ─────────────────────────────────
assign a_line        = data_arr[a_idx];
assign a_stored_tag  = tag_arr[a_idx];
assign a_cs          = cs_arr[a_idx];
assign a_hit         = (cs_arr[a_idx] != `STATE_I) &&
                       (tag_arr[a_idx] == a_tag);

// ── Combinational read port B ─────────────────────────────────
assign b_line = data_arr[b_idx];
assign b_cs   = cs_arr[b_idx];
assign b_hit  = (cs_arr[b_idx] != `STATE_I) &&
                (tag_arr[b_idx] == b_tag);

// ── Registered write port ─────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 256; i = i + 1)
            cs_arr[i] <= `STATE_I;
    end else begin
        if (wr_data_en) data_arr[wr_idx] <= wr_data;
        if (wr_tag_en)  tag_arr[wr_idx]  <= wr_tag;
        if (wr_cs_en)   cs_arr[wr_idx]   <= wr_cs;
    end
end

endmodule
