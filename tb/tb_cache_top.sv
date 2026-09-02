`timescale 1ns/1ps
`include "cache_defines.v"

// ============================================================
// tb_cache_top.sv  —  Dual-L1 CHI Coherent Cache Testbench
//
// Geometry:
//   L1  : 4KB, 256 lines, direct-mapped, 16-byte (128-bit) blocks
//           idx = addr[11:4]  tag = addr[31:12]  woff = addr[3:2]
//   L2  : 64KB, 1024 sets, 4-way PLRU, 128-bit blocks
//           idx = addr[13:4]  tag = addr[31:14]
//   DRAM: 1MB, 10-cycle latency, 128-bit transfers
//
// L1 set stride  = 0x1000 (same L1 idx, different tag)
// L2 set stride  = 0x4000 (same L2 idx, stress PLRU)
//
// Tests:
//   1.  Cold read CPU0             I→SC
//   2.  CPU1 reads same line       SnpSharedFwd: CPU0 UC→SD, CPU1→SC
//   3.  CPU0 writes shared         CleanUnique → CPU1 snooped I, CPU0→UD
//   4.  CPU1 reads dirty (SD)      SnpSharedFwd: CPU0 UD→SD, CPU1→SC
//   5.  CPU0 dirty eviction        WriteBackFull → L2 updated, CPU0→I
//   6.  CPU1 reads evicted line    L2 hit (after WBF)
//   7.  CPU0 ReadUnique (write)    SnpUnique CPU1→I, CPU0→UD
//   8.  Write-allocate CPU0        ReadUnique + write-allocate
//   9.  Byte-enable partial write
//  10.  PLRU stress (5 blocks into same L2 set)
//  12.  CPU1 write-allocate + hit coverage
//  13.  False sharing (two CPUs, same block)
//  14.  Three-way coherence chain
//  15.  Evict opcode clears SF proactively
//  16.  CPU1 WBF eviction visible to CPU0
//  21.  Evict enables snoop-free fill
//  23.  SnpSharedFwd: UD→SD (holder keeps dirty, requester gets SC)
//  24.  MakeReadUnique: SD→UD (holder upgrades, no data fetch)
//  25.  Performance counter readback
// ============================================================

module tb_cache_top;

// ── Clock & reset ─────────────────────────────────────────────
logic clk, rst_n;
initial clk = 0;
always #5 clk = ~clk;   // 100 MHz

// ── DUT signals ───────────────────────────────────────────────
logic [31:0]  cpu0_addr,  cpu0_wdata,  cpu0_rdata;
logic         cpu0_wen,   cpu0_ren,    cpu0_stall;
logic [3:0]   cpu0_be;

logic [31:0]  cpu1_addr,  cpu1_wdata,  cpu1_rdata;
logic         cpu1_wen,   cpu1_ren,    cpu1_stall;
logic [3:0]   cpu1_be;

logic [31:0] l1_0_hits, l1_0_misses;
logic [31:0] l1_1_hits, l1_1_misses;
logic [31:0] l2_hits,   l2_misses;

cache_top dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .cpu0_addr      (cpu0_addr),      .cpu0_wdata     (cpu0_wdata),
    .cpu0_wen       (cpu0_wen),       .cpu0_ren       (cpu0_ren),
    .cpu0_be        (cpu0_be),        .cpu0_rdata     (cpu0_rdata),
    .cpu0_stall     (cpu0_stall),
    .cpu1_addr      (cpu1_addr),      .cpu1_wdata     (cpu1_wdata),
    .cpu1_wen       (cpu1_wen),       .cpu1_ren       (cpu1_ren),
    .cpu1_be        (cpu1_be),        .cpu1_rdata     (cpu1_rdata),
    .cpu1_stall     (cpu1_stall),
    .l1_0_hits      (l1_0_hits),      .l1_0_misses    (l1_0_misses),
    .l1_1_hits      (l1_1_hits),      .l1_1_misses    (l1_1_misses),
    .l2_hits        (l2_hits),        .l2_misses      (l2_misses)
);

// ── Test statistics ───────────────────────────────────────────
int pass_cnt, fail_cnt;

// ── CPU access tasks ──────────────────────────────────────────
task automatic cpu0_read(input logic [31:0] addr, output logic [31:0] rdata);
    @(negedge clk);
    cpu0_addr = addr; cpu0_ren = 1'b1; cpu0_wen = 1'b0; cpu0_be = 4'hF;
    @(posedge clk);
    while (cpu0_stall) @(posedge clk);
    rdata = cpu0_rdata;
    @(negedge clk); cpu0_ren = 1'b0;
    @(posedge clk);
endtask

task automatic cpu0_write(input logic [31:0] addr, data);
    @(negedge clk);
    cpu0_addr = addr; cpu0_wdata = data; cpu0_wen = 1'b1; cpu0_ren = 1'b0; cpu0_be = 4'hF;
    @(posedge clk);
    while (cpu0_stall) @(posedge clk);
    @(negedge clk); cpu0_wen = 1'b0;
    @(posedge clk);
endtask

task automatic cpu0_write_be(input logic [31:0] addr, data, input logic [3:0] be);
    @(negedge clk);
    cpu0_addr = addr; cpu0_wdata = data; cpu0_wen = 1'b1; cpu0_ren = 1'b0; cpu0_be = be;
    @(posedge clk);
    while (cpu0_stall) @(posedge clk);
    @(negedge clk); cpu0_wen = 1'b0;
    @(posedge clk);
endtask

task automatic cpu1_read(input logic [31:0] addr, output logic [31:0] rdata);
    @(negedge clk);
    cpu1_addr = addr; cpu1_ren = 1'b1; cpu1_wen = 1'b0; cpu1_be = 4'hF;
    @(posedge clk);
    while (cpu1_stall) @(posedge clk);
    rdata = cpu1_rdata;
    @(negedge clk); cpu1_ren = 1'b0;
    @(posedge clk);
endtask

task automatic cpu1_write(input logic [31:0] addr, data);
    @(negedge clk);
    cpu1_addr = addr; cpu1_wdata = data; cpu1_wen = 1'b1; cpu1_ren = 1'b0; cpu1_be = 4'hF;
    @(posedge clk);
    while (cpu1_stall) @(posedge clk);
    @(negedge clk); cpu1_wen = 1'b0;
    @(posedge clk);
endtask

// ── Check helper ──────────────────────────────────────────────
task automatic check(
    input string  desc,
    input logic [31:0] got,
    input logic [31:0] exp
);
    if (got === exp) begin
        $display("  [PASS] %-55s  got=0x%08h", desc, got);
        pass_cnt++;
    end else begin
        $display("  [FAIL] %-55s  got=0x%08h  exp=0x%08h", desc, got, exp);
        fail_cnt++;
    end
endtask

// ── DRAM init value ─────────────────────────────────────────
// DRAM model: mem[idx] = {16'hDA00, idx[15:0]}
// Word index for byte address A = A >> 2
function automatic [31:0] dram_init_val(input int widx);
    return {16'hDA00, widx[15:0]};
endfunction

// ── VCD dump ──────────────────────────────────────────────────
initial begin
    $dumpfile("tb_cache_top.vcd");
    $dumpvars(0, tb_cache_top);
end

// ═══════════════════════════════════════════════════════════════
//  Transaction Monitor — prints one line per CHI event
// ═══════════════════════════════════════════════════════════════

// Decode helpers
function automatic string req_opc_name(input logic [5:0] opc);
    case (opc)
        6'h01: req_opc_name = "ReadShared   ";
        6'h07: req_opc_name = "ReadUnique   ";
        6'h0B: req_opc_name = "CleanUnique  ";
        6'h0C: req_opc_name = "MakeUnique   ";
        6'h0D: req_opc_name = "Evict        ";
        6'h1B: req_opc_name = "WriteBackFull";
        default: req_opc_name = $sformatf("REQ_0x%02h     ", opc);
    endcase
endfunction

function automatic string snp_opc_name(input logic [4:0] opc);
    case (opc)
        5'h01: snp_opc_name = "SnpShared    ";
        5'h07: snp_opc_name = "SnpUnique    ";
        5'h09: snp_opc_name = "SnpSharedFwd ";
        default: snp_opc_name = $sformatf("SNP_0x%02h     ", opc);
    endcase
endfunction

function automatic string dat_opc_name(input logic [2:0] opc);
    case (opc)
        3'h1: dat_opc_name = "SnpRespData   ";
        3'h2: dat_opc_name = "CopyBackWrData";
        3'h4: dat_opc_name = "CompData      ";
        3'h6: dat_opc_name = "SnpRespDataFwd";
        default: dat_opc_name = $sformatf("DAT_0x%1h       ", opc);
    endcase
endfunction

function automatic string rsp_opc_name(input logic [3:0] opc);
    case (opc)
        4'h1: rsp_opc_name = "SnpResp_I ";
        4'h2: rsp_opc_name = "SnpResp_SC";
        4'h4: rsp_opc_name = "Comp      ";
        4'h5: rsp_opc_name = "CompDBIDRsp";
        default: rsp_opc_name = $sformatf("RSP_0x%1h   ", opc);
    endcase
endfunction

function automatic string resp_name(input logic [2:0] r);
    case (r)
        3'b000: resp_name = "I ";
        3'b010: resp_name = "UC";
        3'b011: resp_name = "UD";
        3'b110: resp_name = "SC";
        3'b111: resp_name = "SD";
        default: resp_name = "??";
    endcase
endfunction

function automatic string cs_name(input logic [2:0] cs);
    case (cs)
        3'd0: cs_name = "I ";
        3'd1: cs_name = "SC";
        3'd2: cs_name = "UC";
        3'd3: cs_name = "UD";
        3'd4: cs_name = "SD";
        default: cs_name = "??";
    endcase
endfunction

// L2 hit/miss tracking registers
reg [31:0] mon_l2h_prev, mon_l2m_prev;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin mon_l2h_prev <= 0; mon_l2m_prev <= 0; end
    else        begin mon_l2h_prev <= l2_hits; mon_l2m_prev <= l2_misses; end
end

// ── L1 lookup: print once per access (S_CHECK is exactly 1 cycle) ──
localparam MON_S_CHECK = 4'd1;

always @(posedge clk) begin
    if (dut.u_core0.state == MON_S_CHECK)
        $display("[L1-0] t=%8t  addr=0x%08h  L1_state=%-2s  %s",
                 $time,
                 {dut.u_core0.req_tag, dut.u_core0.req_idx, 4'b0000},
                 cs_name(dut.u_core0.l1_a_cs),
                 dut.u_core0.l1_a_hit ? "HIT " : "MISS");
    if (dut.u_core1.state == MON_S_CHECK)
        $display("[L1-1] t=%8t  addr=0x%08h  L1_state=%-2s  %s",
                 $time,
                 {dut.u_core1.req_tag, dut.u_core1.req_idx, 4'b0000},
                 cs_name(dut.u_core1.l1_a_cs),
                 dut.u_core1.l1_a_hit ? "HIT " : "MISS");
end

// ── CHI REQ: core → HN (txreq handshake) ──────────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_rxreq_valid && dut.u_fabric.hn_rxreq_lcrd)
        $display("[REQ]  t=%8t  %s→HN  %-13s  addr=0x%08h  wdata[31:0]=0x%08h",
                 $time,
                 dut.u_fabric.hn_rxreq_src_id ? "CPU1" : "CPU0",
                 req_opc_name(dut.u_fabric.hn_rxreq_opcode),
                 dut.u_fabric.hn_rxreq_addr,
                 dut.u_fabric.hn_rxreq_wdata[31:0]);
end

// ── CHI SNP: HN → core (snoop dispatch) ───────────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_txsnp_valid && dut.u_fabric.hn_txsnp_rdy)
        $display("[SNP]  t=%8t  HN→%s   %-13s  addr=0x%08h",
                 $time,
                 dut.u_fabric.hn_txsnp_tgt_id ? "CPU1" : "CPU0",
                 snp_opc_name(dut.u_fabric.hn_txsnp_opcode),
                 dut.u_fabric.hn_txsnp_addr);
end

// ── CHI SRSP: core → HN (clean snoop response) ────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_rxrsp_valid && dut.u_fabric.hn_rxrsp_lcrd)
        $display("[SRSP] t=%8t  %s→HN   %-10s  (no data)",
                 $time,
                 dut.u_fabric.hn_rxrsp_src_id ? "CPU1" : "CPU0",
                 rsp_opc_name(dut.u_fabric.hn_rxrsp_opcode));
end

// ── CHI SDAT: core → HN (dirty snoop data) ────────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_rxsnpdat_valid && dut.u_fabric.hn_rxsnpdat_lcrd)
        $display("[SDAT] t=%8t  %s→HN   %-14s  resp=%-2s  data[31:0]=0x%08h",
                 $time,
                 dut.u_fabric.hn_rxsnpdat_src_id ? "CPU1" : "CPU0",
                 dat_opc_name(dut.u_fabric.hn_rxsnpdat_opcode),
                 resp_name(dut.u_fabric.hn_rxsnpdat_resp),
                 dut.u_fabric.hn_rxsnpdat_data[31:0]);
end

// ── CHI CompData: HN → core (data response) ───────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_txdat_valid && dut.u_fabric.hn_txdat_ack)
        $display("[CDAT] t=%8t  HN→%s   %-14s  resp=%-2s  data[31:0]=0x%08h",
                 $time,
                 dut.u_fabric.hn_txdat_tgt_id ? "CPU1" : "CPU0",
                 dat_opc_name(dut.u_fabric.hn_txdat_opcode),
                 resp_name(dut.u_fabric.hn_txdat_resp),
                 dut.u_fabric.hn_txdat_data[31:0]);
end

// ── CHI Comp RSP: HN → core (permission grant) ────────────────
always @(posedge clk) begin
    if (dut.u_fabric.hn_txrsp_valid && dut.u_fabric.hn_txrsp_ack)
        $display("[COMP] t=%8t  HN→%s   %-10s",
                 $time,
                 dut.u_fabric.hn_txrsp_tgt_id ? "CPU1" : "CPU0",
                 rsp_opc_name(dut.u_fabric.hn_txrsp_opcode));
end

// ── L2 hit/miss events ─────────────────────────────────────────
always @(posedge clk) begin
    if (l2_hits   != mon_l2h_prev)
        $display("[L2  ] t=%8t  HIT    (cumulative hits=%0d)",   $time, l2_hits);
    if (l2_misses != mon_l2m_prev)
        $display("[L2  ] t=%8t  MISS   (cumulative misses=%0d)", $time, l2_misses);
end

// ── Main test sequence ────────────────────────────────────────
logic [31:0] r0, r1;
int          midx;

initial begin
    pass_cnt = 0; fail_cnt = 0;
    cpu0_addr = 0; cpu0_wdata = 0; cpu0_wen = 0; cpu0_ren = 0; cpu0_be = 4'hF;
    cpu1_addr = 0; cpu1_wdata = 0; cpu1_wen = 0; cpu1_ren = 0; cpu1_be = 4'hF;

    // Initialize DRAM contents before reset
    for (midx = 0; midx < `DRAM_WORDS; midx = midx + 1)
        dut.u_dram.mem[midx] = {16'hDA00, midx[15:0]};

    // Reset
    rst_n = 0;
    repeat(4) @(posedge clk); #1;
    rst_n = 1;
    @(posedge clk); #1;

    $display("");
    $display("================================================================");
    $display("  Dual-L1 CHI Coherence Test — UC/UD/SC/SD states");
    $display("  L1: 4KB DM  |  L2: 64KB 4-way PLRU  |  DRAM: 1MB");
    $display("  Block size: 128-bit (16 bytes)");
    $display("  L1 stride: 0x1000  |  L2 stride: 0x4000");
    $display("================================================================");

    // ── Test 1: Cold read CPU0 (I → SC) ───────────────────────
    // addr=0x100: L1 idx=(0x100>>4)&0xFF=0x10=16, woff=addr[3:2]=0
    $display("\n[TEST 1]  Cold read CPU0 — I→SC");
    cpu0_read(32'h0000_0100, r0);
    check("CPU0 cold read addr=0x100 (I→SC)", r0, dram_init_val(32'h100 >> 2));

    // ── Test 2: CPU1 reads same line → ICN SnpSharedFwd ───────
    // CPU0 is UC (cold read gave UC? Actually ReadShared → SC, not UC)
    // After cold read: CPU0 → SC (ReadShared response is CompData = SC)
    // CPU1 read → sf[idx][0]=1 → SnpSharedFwd to CPU0
    // CPU0 is SC → responds SnpRespS (stays SC), no dirty forwarding
    // CPU1 gets SC from L2
    $display("\n[TEST 2]  CPU1 reads same line — SnpSharedFwd to CPU0 (SC peer), both SC");
    cpu1_read(32'h0000_0100, r1);
    check("CPU1 read same block gets same data (SC)", r1, dram_init_val(32'h100 >> 2));

    // ── Test 3: CPU0 writes shared (SC→UD via CleanUnique) ────
    $display("\n[TEST 3]  CPU0 write hit on SC — CleanUnique, CPU1 snooped to I");
    cpu0_write(32'h0000_0100, 32'hDEAD_BEEF);
    cpu0_read(32'h0000_0100, r0);
    check("CPU0 sees its own write (UD state)", r0, 32'hDEAD_BEEF);

    // ── Test 4: CPU1 reads CPU0's dirty line ──────────────────
    // CPU0 has UD (dirty); ICN sends SnpSharedFwd → CPU0 goes UD→SD, forwards data
    // CPU1 gets CompData (SC) with the dirty data value
    $display("\n[TEST 4]  CPU1 reads UD line — SnpSharedFwd (CPU0 UD→SD, CPU1→SC)");
    cpu1_read(32'h0000_0100, r1);
    check("CPU1 gets forwarded dirty data via SnpSharedFwd", r1, 32'hDEAD_BEEF);
    // CPU0 is now SD — read should still return valid (dirty) data
    cpu0_read(32'h0000_0100, r0);
    check("CPU0 still valid after SnpSharedFwd (SD state)", r0, 32'hDEAD_BEEF);

    // ── Test 5: CPU0 dirty eviction → WriteBackFull ───────────
    // Make CPU0's 0x100 dirty again first via MakeReadUnique
    // (CPU0 is SD; write → MakeReadUnique → UD)
    $display("\n[TEST 5]  CPU0 dirty eviction — WriteBackFull to L2");
    cpu0_write(32'h0000_0100, 32'hCAFE_0001);   // SD → MakeReadUnique → UD
    // Force eviction: address with same L1 idx=0x10 but different tag
    // L1 stride = 0x1000; conflict: 0x100 + 0x1000 = 0x1100
    cpu0_write(32'h0000_1100, 32'hBBBB_BBBB);   // same L1 idx=0x10, evicts 0x100 via WBF
    // Read 0x100 from CPU0 → L2 hit (written by WBF)
    cpu0_read(32'h0000_0100, r0);
    check("Evicted dirty data readable via L2 (WBF worked)", r0, 32'hCAFE_0001);

    // ── Test 6: CPU1 reads after eviction (L2 hit) ────────────
    $display("\n[TEST 6]  CPU1 reads evicted address — L2 hit");
    cpu1_read(32'h0000_0100, r1);
    check("CPU1 reads WBF'd data from L2", r1, 32'hCAFE_0001);

    // ── Test 7: CPU0 ReadUnique (write miss on SC) ─────────────
    $display("\n[TEST 7]  CPU0 ReadUnique write — ICN SnpUnique to CPU1, CPU1→I");
    // CPU1 has 0x100 in SC; CPU0 write → miss (CPU0's was evicted) → ReadUnique
    // ICN snoops CPU1 (SnpUnique) → CPU1→I; CPU0 gets UD with fill from L2
    cpu0_write(32'h0000_0100, 32'hA5A5_A5A5);
    cpu0_read(32'h0000_0100, r0);
    check("CPU0 has unique ownership after ReadUnique (UD)", r0, 32'hA5A5_A5A5);
    cpu1_read(32'h0000_0100, r1);
    check("CPU1 reads updated data after CPU0's ReadUnique",  r1, 32'hA5A5_A5A5);

    // ── Test 8: Write-allocate ────────────────────────────────
    // New addr 0x200: L1 idx=(0x200>>4)&0xFF=0x20=32
    $display("\n[TEST 8]  Write-allocate (write miss → ReadUnique + write)");
    cpu0_write(32'h0000_0200, 32'h1111_2222);
    cpu0_read(32'h0000_0200, r0);
    check("CPU0 write-allocate: written word readable",         r0, 32'h1111_2222);
    // Word 1 of same 128-bit block (offset +4): should be DRAM init value
    cpu0_read(32'h0000_0204, r0);
    check("CPU0 write-allocate: adjacent word is DRAM init",
          r0, dram_init_val(32'h204 >> 2));

    // ── Test 9: Byte-enable partial write ────────────────────
    $display("\n[TEST 9]  Byte-enable partial write");
    cpu0_write(32'h0000_0300, 32'h1234_5678);
    cpu0_write_be(32'h0000_0300, 32'hAB00_0000, 4'b1000);  // upper byte only
    cpu0_read(32'h0000_0300, r0);
    check("Byte-enable: upper byte overwritten, lower 3 unchanged", r0, 32'hAB34_5678);

    // ── Test 10: PLRU stress (5 blocks → same L2 set) ─────────
    // L2 idx = addr[13:4]; L2 stride = 0x4000
    // All addresses with tag+idx differing only in tag → same L2 set idx=0
    $display("\n[TEST 10]  PLRU stress (5 blocks into same L2 set)");
    cpu0_write(32'h0000_0000, 32'h0000_1111);   // L2 idx=0, way 0
    cpu0_write(32'h0000_4000, 32'h0000_2222);   // L2 idx=0, way 1
    cpu0_write(32'h0000_8000, 32'h0000_3333);   // L2 idx=0, way 2
    cpu0_write(32'h0000_C000, 32'h0000_4444);   // L2 idx=0, way 3
    cpu0_write(32'h0001_0000, 32'h0000_5555);   // L2 idx=0, evicts PLRU victim
    // All blocks accessible somewhere in hierarchy
    cpu0_read(32'h0000_4000, r0);
    check("PLRU: block 0x4000 (2222) readable after 5-way stress", r0, 32'h0000_2222);
    cpu0_read(32'h0000_8000, r0);
    check("PLRU: block 0x8000 (3333) readable after 5-way stress", r0, 32'h0000_3333);
    cpu0_read(32'h0001_0000, r0);
    check("PLRU: newest block 0x10000 (5555) accessible",           r0, 32'h0000_5555);

    // ── Test 12: CPU1 write-allocate + L1-1 hit coverage ──────
    // addr=0x4080: L1 idx=(0x4080>>4)&0xFF=0x408&0xFF=0x08=8
    $display("\n[TEST 12]  CPU1 write-allocate and read/write hit");
    cpu1_write(32'h0000_4080, 32'hBEEF_CAFE);
    cpu1_read (32'h0000_4080, r1);
    check("L1-1 write-allocate: data readable (hit on UD)",       r1, 32'hBEEF_CAFE);
    cpu1_write(32'h0000_4080, 32'hCAFE_BEEF);   // UD hit → local update
    cpu1_read (32'h0000_4080, r1);
    check("L1-1 write hit on UD: data updated locally",           r1, 32'hCAFE_BEEF);

    // ── Test 13: False sharing ────────────────────────────────
    // addr=0x4180: L1 idx=(0x4180>>4)&0xFF=0x418&0xFF=0x18=24
    // 0x4180 and 0x4184 are in the same 128-bit block (block-aligned at 0x4180)
    $display("\n[TEST 13]  False sharing — CPU0 and CPU1 each write one word");
    cpu0_write(32'h0000_4180, 32'hAAAA_AAAA);   // CPU0 UD (word 0)
    cpu1_write(32'h0000_4184, 32'hBBBB_BBBB);   // CPU1 ReadUnique: SnpUnique CPU0→I
                                                  // CPU1 gets dirty block, updates word 1
    cpu0_read(32'h0000_4180, r0);   // CPU0 misses → ReadShared: SnpSharedFwd CPU1→SD
    check("False sharing: CPU0 word-0 intact",                    r0, 32'hAAAA_AAAA);
    cpu1_read(32'h0000_4184, r1);   // CPU1 still has line (SD or SC hit)
    check("False sharing: CPU1 word-1 intact",                    r1, 32'hBBBB_BBBB);

    // ── Test 14: Three-way coherence chain ────────────────────
    // addr=0x4280: L1 idx=(0x4280>>4)&0xFF=0x428&0xFF=0x28=40
    $display("\n[TEST 14]  Three-way coherence: UD → shared → exclusive → UD");
    cpu0_write(32'h0000_4280, 32'h1111_1111);   // CPU0 UD
    cpu1_read (32'h0000_4280, r1);               // SnpSharedFwd: CPU0 UD→SD, CPU1 SC
    check("Three-way: CPU1 gets CPU0's dirty data", r1, 32'h1111_1111);
    cpu1_write(32'h0000_4280, 32'h2222_2222);   // CPU1 CleanUnique: CPU0→I/SD→I, CPU1 UD
    cpu0_read (32'h0000_4280, r0);               // SnpSharedFwd: CPU1 UD→SD, CPU0 SC
    check("Three-way: CPU0 sees CPU1's write",      r0, 32'h2222_2222);

    // ── Test 15: Evict clears SF proactively ──────────────────
    // addr=0x4380: L1 idx=0x38=56; conflict=0x4380+0x1000=0x5380 (same idx=0x38)
    $display("\n[TEST 15]  Evict opcode — clean eviction clears SF proactively");
    cpu0_read(32'h0000_4380, r0);               // CPU0 SC at idx=56; SF[56][0]=1
    // Reading 0x5380 (same L1 idx, diff tag) causes Evict for 0x4380 before fill
    cpu0_read(32'h0000_5380, r0);               // ICN clears SF[56][0]=0
    // CPU1 writes 0x4380: SF[56][0]=0 → no snoop to CPU0
    cpu1_write(32'h0000_4380, 32'hDEAD_FACE);
    cpu1_read (32'h0000_4380, r1);
    check("Evict: SF cleared, CPU1 gets UD with no stale snoop", r1, 32'hDEAD_FACE);

    // ── Test 16: CPU1 WBF eviction visible to CPU0 ────────────
    // addr=0x4040: L1 idx=(0x4040>>4)&0xFF=0x04=4; conflict=0x4040+0x1000=0x5040 (idx=0x04)
    $display("\n[TEST 16]  CPU1 dirty eviction — WBF updates L2, visible to CPU0");
    cpu1_write(32'h0000_4040, 32'hF00D_CAFE);   // CPU1 UD at idx=4
    cpu1_read (32'h0000_5040, r1);               // evicts 0x4040 via WBF
    cpu0_read (32'h0000_4040, r0);               // L2 hit on WBF'd data
    check("CPU1 WBF eviction: CPU0 reads dirty data via L2",      r0, 32'hF00D_CAFE);

    // ── Test 21: Evict enables snoop-free fill ────────────────
    // addr=0x6340: L1 idx=(0x6340>>4)&0xFF=0x34=52; conflict=0x6340+0x1000=0x7340 (idx=0x34)
    $display("\n[TEST 21]  Evict — proactive SF clearance enables snoop-free fill");
    cpu0_read(32'h0000_6340, r0);               // CPU0 SC; SF[6340][0]=1
    cpu0_read(32'h0000_7340, r0);               // L1 evicts 0x6340 via Evict; SF[6340][0]=0
    cpu1_write(32'h0000_6340, 32'hFEED_CAFE);  // SF=0 → no snoop to CPU0; CPU1 UD
    cpu1_read (32'h0000_6340, r1);
    check("Evict+snoop-free fill: CPU1 UD, data correct",          r1, 32'hFEED_CAFE);

    // ── Test 23: SnpSharedFwd UD→SD ───────────────────────────
    // CPU0 gets UD on a fresh block; CPU1 reads → SnpSharedFwd →
    // CPU0 goes UD→SD (keeps dirty data, L2 gets clean copy); CPU1 gets SC
    // addr=0x0000_D000: L1 idx=(0xD000>>4)&0xFF=0xD00&0xFF=0x00=0
    $display("\n[TEST 23]  SnpSharedFwd: CPU0 UD→SD, CPU1 gets SC with forwarded data");
    cpu0_write(32'h0000_D000, 32'hBEEF_1234);  // CPU0 UD (fresh block)
    cpu1_read (32'h0000_D000, r1);              // ICN: SnpSharedFwd → CPU0 UD→SD; CPU1 SC
    check("SnpSharedFwd: CPU1 gets CPU0's dirty data (SC)",        r1, 32'hBEEF_1234);
    cpu0_read (32'h0000_D000, r0);              // CPU0 has SD — read returns valid dirty data
    check("SnpSharedFwd: CPU0 still valid in SD state",            r0, 32'hBEEF_1234);

    // ── Test 24: MakeReadUnique SD→UD ─────────────────────────
    // CPU0 is SD, CPU1 is SC (from test 23)
    // CPU0 write → SD → sends MakeReadUnique → ICN snoops CPU1 (SnpUnique)
    // CPU1 goes SC→I; ICN sends Comp to CPU0 → CPU0: SD→UD + local write
    $display("\n[TEST 24]  MakeReadUnique: CPU0 SD→UD, write without data fetch");
    cpu0_write(32'h0000_D000, 32'hDEAD_5678);  // SD → MakeReadUnique → UD
    cpu0_read (32'h0000_D000, r0);
    check("MakeReadUnique: CPU0 upgraded SD→UD, write visible",    r0, 32'hDEAD_5678);
    cpu1_read (32'h0000_D000, r1);              // CPU1 was SnpUnique'd to I → ReadShared
    check("MakeReadUnique: CPU1 re-read gets updated value",        r1, 32'hDEAD_5678);

    // ── Test 25: Performance counter readback ─────────────────
    $display("\n[TEST 25]  Performance Counters");
    $display("  L1-0 hits=%0d  misses=%0d", l1_0_hits, l1_0_misses);
    $display("  L1-1 hits=%0d  misses=%0d", l1_1_hits, l1_1_misses);
    $display("  L2   hits=%0d  misses=%0d", l2_hits,   l2_misses);
    if (l1_0_hits > 0 && l1_0_misses > 0 &&
        l1_1_hits > 0 && l1_1_misses > 0 &&
        l2_hits   > 0 && l2_misses   > 0)
        $display("  [PASS]  All counters non-zero");
    else begin
        $display("  [FAIL]  One or more counters stuck at zero");
        fail_cnt++;
    end

    // ── Test 26: Byte-enable individual byte writes ───────────
    $display("\n[TEST 26]  Byte-enable — each byte and 2-byte combinations");
    cpu0_write(32'h0000_0510, 32'h1234_5678);
    cpu0_read (32'h0000_0510, r0);
    check("T26: initial full write (UD)",                         r0, 32'h1234_5678);
    cpu0_write_be(32'h0000_0510, 32'hAB00_0000, 4'b1000);
    cpu0_read (32'h0000_0510, r0);
    check("T26: byte3 only → 0xAB345678",                        r0, 32'hAB34_5678);
    cpu0_write_be(32'h0000_0510, 32'h00CD_0000, 4'b0100);
    cpu0_read (32'h0000_0510, r0);
    check("T26: byte2 only → 0xABCD5678",                        r0, 32'hABCD_5678);
    cpu0_write_be(32'h0000_0510, 32'h0000_EF00, 4'b0010);
    cpu0_read (32'h0000_0510, r0);
    check("T26: byte1 only → 0xABCDEF78",                        r0, 32'hABCD_EF78);
    cpu0_write_be(32'h0000_0510, 32'h0000_0012, 4'b0001);
    cpu0_read (32'h0000_0510, r0);
    check("T26: byte0 only → 0xABCDEF12",                        r0, 32'hABCD_EF12);
    cpu0_write_be(32'h0000_0510, 32'h0000_3456, 4'b0011);
    cpu0_read (32'h0000_0510, r0);
    check("T26: lower 2 bytes → 0xABCD3456",                     r0, 32'hABCD_3456);
    cpu0_write_be(32'h0000_0510, 32'h7890_0000, 4'b1100);
    cpu0_read (32'h0000_0510, r0);
    check("T26: upper 2 bytes → 0x78903456",                     r0, 32'h7890_3456);

    // ── Test 27: All 4 words in a 128-bit cache line ──────────
    $display("\n[TEST 27]  All 4 words within one 128-bit cache block");
    cpu0_write(32'h0000_0520, 32'h1111_AAAA);
    cpu0_write(32'h0000_0524, 32'h2222_BBBB);
    cpu0_write(32'h0000_0528, 32'h3333_CCCC);
    cpu0_write(32'h0000_052C, 32'h4444_DDDD);
    cpu0_read(32'h0000_0520, r0); check("T27: word0=0x1111AAAA", r0, 32'h1111_AAAA);
    cpu0_read(32'h0000_0524, r0); check("T27: word1=0x2222BBBB", r0, 32'h2222_BBBB);
    cpu0_read(32'h0000_0528, r0); check("T27: word2=0x3333CCCC", r0, 32'h3333_CCCC);
    cpu0_read(32'h0000_052C, r0); check("T27: word3=0x4444DDDD", r0, 32'h4444_DDDD);
    cpu1_read(32'h0000_0520, r1);
    check("T27: CPU1 sees CPU0 word0 via SnpSharedFwd",          r1, 32'h1111_AAAA);

    // ── Test 28: ReadUnique dirty forwarding (RESP_UD path) ───
    $display("\n[TEST 28]  ReadUnique dirty forwarding (RESP_UD from HN to L1)");
    cpu0_write(32'h0000_0530, 32'hBEEF_0001);
    cpu1_write(32'h0000_0530, 32'hBEEF_0002);
    cpu1_read (32'h0000_0530, r1);
    check("T28: CPU1 UD after RESP_UD forward, write visible",    r1, 32'hBEEF_0002);
    cpu0_read (32'h0000_0530, r0);
    check("T28: CPU0 SC via SnpSharedFwd (CPU1 UD→SD)",          r0, 32'hBEEF_0002);
    cpu1_read (32'h0000_0530, r1);
    check("T28: CPU1 SD hit after SnpSharedFwd",                  r1, 32'hBEEF_0002);

    // ── Test 29: Round-robin write ownership ──────────────────
    $display("\n[TEST 29]  Round-robin write ownership — alternating ReadUnique");
    cpu0_write(32'h0000_0540, 32'hAAAA_0001);
    cpu1_write(32'h0000_0540, 32'hBBBB_0002);
    cpu0_write(32'h0000_0540, 32'hCCCC_0003);
    cpu1_read (32'h0000_0540, r1);
    check("T29: CPU1 SC gets CPU0 latest write",                  r1, 32'hCCCC_0003);
    cpu0_read (32'h0000_0540, r0);
    check("T29: CPU0 SD hit returns valid data",                  r0, 32'hCCCC_0003);
    cpu1_write(32'h0000_0540, 32'hDDDD_0004);
    cpu1_read (32'h0000_0540, r1);
    check("T29: CPU1 UD after CleanUnique upgrade",               r1, 32'hDDDD_0004);
    cpu0_read (32'h0000_0540, r0);
    check("T29: CPU0 SC sees CPU1 latest write",                  r0, 32'hDDDD_0004);

    // ── Test 30: I → SC → UD sequence ────────────────────────
    $display("\n[TEST 30]  I→SC→UD: cold read, CleanUnique upgrade, UD hit");
    cpu0_read (32'h0000_0550, r0);
    check("T30: cold read → SC, DRAM init",         r0, dram_init_val(32'h0550 >> 2));
    cpu0_write(32'h0000_0550, 32'hFACE_CAFE);
    cpu0_read (32'h0000_0550, r0);
    check("T30: SC→UD via CleanUnique, write visible",            r0, 32'hFACE_CAFE);
    cpu0_write(32'h0000_0550, 32'hDEAD_BEEF);
    cpu0_read (32'h0000_0550, r0);
    check("T30: UD hit, second write visible",                    r0, 32'hDEAD_BEEF);

    // ── Test 31: SD CleanUnique sends dirty data to HN ────────
    $display("\n[TEST 31]  SD CleanUnique — dirty data written to HN L2");
    cpu0_write(32'h0000_0560, 32'h1234_0001);
    cpu1_read (32'h0000_0560, r1);
    check("T31: CPU1 SC sees CPU0 dirty via SnpSharedFwd",        r1, 32'h1234_0001);
    cpu0_read (32'h0000_0560, r0);
    check("T31: CPU0 SD hit valid",                               r0, 32'h1234_0001);
    cpu0_write(32'h0000_0560, 32'h5678_0002);
    cpu0_read (32'h0000_0560, r0);
    check("T31: CPU0 UD after SD CleanUnique, write visible",     r0, 32'h5678_0002);
    cpu1_read (32'h0000_0560, r1);
    check("T31: CPU1 SC sees updated value via SnpSharedFwd",     r1, 32'h5678_0002);

    // ── Test 32: Two independent addresses, no interference ───
    $display("\n[TEST 32]  Independent addresses — no cross-line coherence");
    cpu0_write(32'h0000_0570, 32'hAABB_1111);
    cpu1_write(32'h0000_0580, 32'hCCDD_2222);
    cpu0_read (32'h0000_0570, r0);
    check("T32: CPU0 UD at 0x0570 unaffected",                    r0, 32'hAABB_1111);
    cpu1_read (32'h0000_0580, r1);
    check("T32: CPU1 UD at 0x0580 unaffected",                    r1, 32'hCCDD_2222);
    cpu0_read (32'h0000_0580, r0);
    check("T32: CPU0 SC reads CPU1 0x0580 via SnpSharedFwd",      r0, 32'hCCDD_2222);
    cpu1_read (32'h0000_0570, r1);
    check("T32: CPU1 SC reads CPU0 0x0570 via SnpSharedFwd",      r1, 32'hAABB_1111);

    // ── Test 33: L1 capacity — 8 distinct sets ───────────────
    $display("\n[TEST 33]  L1 capacity sweep — 8 distinct non-conflicting sets");
    cpu0_write(32'h0000_0590, 32'hA590_0001);
    cpu0_write(32'h0000_05A0, 32'hA5A0_0002);
    cpu0_write(32'h0000_05B0, 32'hA5B0_0003);
    cpu0_write(32'h0000_05C0, 32'hA5C0_0004);
    cpu0_write(32'h0000_05D0, 32'hA5D0_0005);
    cpu0_write(32'h0000_05E0, 32'hA5E0_0006);
    cpu0_write(32'h0000_05F0, 32'hA5F0_0007);
    cpu0_write(32'h0000_0600, 32'hA600_0008);
    cpu0_read(32'h0000_0590, r0); check("T33: set 0x59", r0, 32'hA590_0001);
    cpu0_read(32'h0000_05A0, r0); check("T33: set 0x5A", r0, 32'hA5A0_0002);
    cpu0_read(32'h0000_05B0, r0); check("T33: set 0x5B", r0, 32'hA5B0_0003);
    cpu0_read(32'h0000_05C0, r0); check("T33: set 0x5C", r0, 32'hA5C0_0004);
    cpu0_read(32'h0000_05D0, r0); check("T33: set 0x5D", r0, 32'hA5D0_0005);
    cpu0_read(32'h0000_05E0, r0); check("T33: set 0x5E", r0, 32'hA5E0_0006);
    cpu0_read(32'h0000_05F0, r0); check("T33: set 0x5F", r0, 32'hA5F0_0007);
    cpu0_read(32'h0000_0600, r0); check("T33: set 0x60", r0, 32'hA600_0008);

    // ── Test 34: CPU0 dirty eviction, CPU1 reads from L2 ─────
    $display("\n[TEST 34]  CPU0 dirty eviction (WBF) then CPU1 reads from L2");
    cpu0_write(32'h0000_0700, 32'hF0F0_A1A1);
    cpu0_write(32'h0000_1700, 32'h0000_0000);
    cpu1_read (32'h0000_0700, r1);
    check("T34: CPU1 reads WBF data from L2",                     r1, 32'hF0F0_A1A1);
    cpu0_read (32'h0000_0700, r0);
    check("T34: CPU0 SC after eviction, reads L2 value",          r0, 32'hF0F0_A1A1);
    cpu0_write(32'h0000_0700, 32'hB2B2_C3C3);
    cpu0_read (32'h0000_0700, r0);
    check("T34: CPU0 UD after post-eviction CleanUnique",         r0, 32'hB2B2_C3C3);

    // ── Test 35: CPU1 writes after CPU0 dirty eviction ────────
    $display("\n[TEST 35]  CPU1 write after CPU0 dirty eviction — L2 hit path");
    cpu0_write(32'h0000_0710, 32'hAAAA_BBBB);
    cpu0_read (32'h0000_1710, r0);
    cpu1_write(32'h0000_0710, 32'hCCCC_DDDD);
    cpu1_read (32'h0000_0710, r1);
    check("T35: CPU1 UD after eviction+ReadUnique",               r1, 32'hCCCC_DDDD);
    cpu0_read (32'h0000_0710, r0);
    check("T35: CPU0 SC via SnpSharedFwd sees CPU1 write",        r0, 32'hCCCC_DDDD);
    cpu1_read (32'h0000_0710, r1);
    check("T35: CPU1 SD hit after SnpSharedFwd",                  r1, 32'hCCCC_DDDD);

    // ── Test 36: Sparse address coverage — 10 unique sets ─────
    $display("\n[TEST 36]  Sparse coverage — 10 unique L1 sets (idx 0x80..0x89)");
    cpu0_write(32'h0000_0800, 32'hB800_0000);
    cpu0_write(32'h0000_0810, 32'hB800_0001);
    cpu0_write(32'h0000_0820, 32'hB800_0002);
    cpu0_write(32'h0000_0830, 32'hB800_0003);
    cpu0_write(32'h0000_0840, 32'hB800_0004);
    cpu0_write(32'h0000_0850, 32'hB800_0005);
    cpu0_write(32'h0000_0860, 32'hB800_0006);
    cpu0_write(32'h0000_0870, 32'hB800_0007);
    cpu0_write(32'h0000_0880, 32'hB800_0008);
    cpu0_write(32'h0000_0890, 32'hB800_0009);
    cpu0_read(32'h0000_0800, r0); check("T36: addr 0x0800", r0, 32'hB800_0000);
    cpu0_read(32'h0000_0810, r0); check("T36: addr 0x0810", r0, 32'hB800_0001);
    cpu0_read(32'h0000_0820, r0); check("T36: addr 0x0820", r0, 32'hB800_0002);
    cpu0_read(32'h0000_0830, r0); check("T36: addr 0x0830", r0, 32'hB800_0003);
    cpu0_read(32'h0000_0840, r0); check("T36: addr 0x0840", r0, 32'hB800_0004);
    cpu0_read(32'h0000_0850, r0); check("T36: addr 0x0850", r0, 32'hB800_0005);
    cpu0_read(32'h0000_0860, r0); check("T36: addr 0x0860", r0, 32'hB800_0006);
    cpu0_read(32'h0000_0870, r0); check("T36: addr 0x0870", r0, 32'hB800_0007);
    cpu0_read(32'h0000_0880, r0); check("T36: addr 0x0880", r0, 32'hB800_0008);
    cpu0_read(32'h0000_0890, r0); check("T36: addr 0x0890", r0, 32'hB800_0009);

    // ── Test 37: SC→UD multiple writes, eviction, re-read ─────
    $display("\n[TEST 37]  SC→UD writes then eviction and re-read from L2");
    cpu0_read (32'h0000_0A00, r0);
    cpu0_write(32'h0000_0A00, 32'h5A5A_0001);
    cpu0_read (32'h0000_0A00, r0);
    check("T37: first write after SC→UD",                         r0, 32'h5A5A_0001);
    cpu0_write(32'h0000_0A00, 32'h5A5A_0002);
    cpu0_read (32'h0000_0A00, r0);
    check("T37: second UD-hit write",                             r0, 32'h5A5A_0002);
    cpu0_read (32'h0000_1A00, r0);
    cpu1_read (32'h0000_0A00, r1);
    check("T37: CPU1 reads evicted dirty from L2",                r1, 32'h5A5A_0002);
    cpu0_read (32'h0000_0A00, r0);
    check("T37: CPU0 SC re-read after eviction",                  r0, 32'h5A5A_0002);

    // ── Test 38: CPU1 dirty eviction visible to CPU0 ──────────
    $display("\n[TEST 38]  CPU1 dirty eviction (WBF) visible to CPU0");
    cpu1_write(32'h0000_0B00, 32'hF1F1_F2F2);
    cpu1_read (32'h0000_1B00, r1);
    cpu0_read (32'h0000_0B00, r0);
    check("T38: CPU0 reads CPU1 evicted dirty via L2",            r0, 32'hF1F1_F2F2);
    cpu1_read (32'h0000_0B00, r1);
    check("T38: CPU1 re-reads via SnpSharedFwd (both SC)",        r1, 32'hF1F1_F2F2);
    cpu1_write(32'h0000_0B00, 32'hE3E3_E4E4);
    cpu1_read (32'h0000_0B00, r1);
    check("T38: CPU1 UD after CleanUnique upgrade",               r1, 32'hE3E3_E4E4);

    // ── Test 39: Adjacent blocks, no cross-line interference ──
    $display("\n[TEST 39]  Adjacent blocks 0xC00–0xC30 — no cross-line interference");
    cpu0_write(32'h0000_0C00, 32'hC000_0001);
    cpu0_write(32'h0000_0C10, 32'hC001_0002);
    cpu1_write(32'h0000_0C20, 32'hC002_0003);
    cpu1_write(32'h0000_0C30, 32'hC003_0004);
    cpu0_read(32'h0000_0C00, r0); check("T39: CPU0 UD 0xC00", r0, 32'hC000_0001);
    cpu0_read(32'h0000_0C10, r0); check("T39: CPU0 UD 0xC10", r0, 32'hC001_0002);
    cpu1_read(32'h0000_0C20, r1); check("T39: CPU1 UD 0xC20", r1, 32'hC002_0003);
    cpu1_read(32'h0000_0C30, r1); check("T39: CPU1 UD 0xC30", r1, 32'hC003_0004);

    // ── Test 40: CPU1 cold reads from DRAM ────────────────────
    $display("\n[TEST 40]  CPU1 cold reads from DRAM — I→SC via ReadShared");
    cpu1_read(32'h0000_0D00, r1);
    check("T40: CPU1 cold read 0x0D00 → DRAM init", r1, dram_init_val(32'h0D00 >> 2));
    cpu1_read(32'h0000_0E00, r1);
    check("T40: CPU1 cold read 0x0E00 → DRAM init", r1, dram_init_val(32'h0E00 >> 2));
    cpu1_read(32'h0000_0F00, r1);
    check("T40: CPU1 cold read 0x0F00 → DRAM init", r1, dram_init_val(32'h0F00 >> 2));

    // ── Test 41: CPU1 CleanUnique from SC (not just CPU0) ────────
    // addr=0x0E10: L1 idx=0xE1. Both cores go SC, then CPU1 upgrades.
    $display("\n[TEST 41]  CPU1 CleanUnique from SC — symmetric upgrade path");
    cpu0_read (32'h0000_0E10, r0);              // CPU0 I→SC
    cpu1_read (32'h0000_0E10, r1);              // CPU1 I→SC (both SC, SnpSharedFwd)
    cpu1_write(32'h0000_0E10, 32'h4141_4141);  // CPU1 SC→CleanUnique→UC→UD; CPU0 SC→I
    cpu1_read (32'h0000_0E10, r1);
    check("T41: CPU1 UD after CleanUnique from SC",            r1, 32'h4141_4141);
    cpu0_read (32'h0000_0E10, r0);              // ReadShared → SnpSharedFwd; CPU1 UD→SD, CPU0 SC
    check("T41: CPU0 SC gets CPU1 written value",              r0, 32'h4141_4141);
    cpu0_write(32'h0000_0E10, 32'h4242_4242);  // CPU0 SC→CleanUnique; CPU1 SD→I (dirty to HN)
    cpu0_read (32'h0000_0E10, r0);
    check("T41: CPU0 UD after CleanUnique; CPU1 SD evicted",   r0, 32'h4242_4242);

    // ── Test 42: All 5 opcodes on a single cache line ─────────
    // addr=0x0E20: L1 idx=0xE2. Conflict at 0x1E20 (same idx, diff tag).
    // Exercises: ReadShared, CleanUnique, WriteBackFull, ReadUnique, Evict
    $display("\n[TEST 42]  All 5 CHI opcodes on address 0x0E20");
    // (a) ReadShared: I→SC
    cpu0_read (32'h0000_0E20, r0);
    check("T42: ReadShared I→SC, DRAM init",                   r0, dram_init_val(32'h0E20 >> 2));
    // (b) CleanUnique: SC→UD
    cpu0_write(32'h0000_0E20, 32'h4200_0001);
    cpu0_read (32'h0000_0E20, r0);
    check("T42: CleanUnique SC→UD, write visible",             r0, 32'h4200_0001);
    // (c) WriteBackFull: conflict read evicts dirty 0xE20 → L2
    cpu0_read (32'h0000_1E20, r0);              // WBF for 0xE20 (UD); fills 0x1E20
    cpu0_read (32'h0000_0E20, r0);              // L2 hit (after WBF) → SC
    check("T42: WBF eviction; L2 has dirty data",              r0, 32'h4200_0001);
    // (d) ReadUnique: CPU1 write miss (CPU0 has SC, SnpUnique → CPU0 I; CPU1 UC→UD)
    cpu1_write(32'h0000_0E20, 32'h4200_0002);
    cpu1_read (32'h0000_0E20, r1);
    check("T42: ReadUnique I→UD; CPU1 has exclusive dirty",    r1, 32'h4200_0002);
    // (e) Evict: CPU0 re-reads → SC; then conflict evicts SC cleanly via Evict
    cpu0_read (32'h0000_0E20, r0);              // SnpSharedFwd; CPU1 UD→SD; CPU0 SC
    cpu0_read (32'h0000_1E20, r0);              // Evict for 0xE20 (SC); fill 0x1E20
    cpu0_read (32'h0000_0E20, r0);              // refill 0xE20 from L2/HN → SC
    check("T42: Evict SC; refill gives correct data",          r0, 32'h4200_0002);

    // ── Test 43: Two eviction+refill cycles on the same line ──
    // addr=0x0E30: L1 idx=0xE3. Conflict at 0x1E30 (same idx, diff tag).
    $display("\n[TEST 43]  Two dirty eviction+refill cycles on same address");
    // Cycle 1
    cpu0_write(32'h0000_0E30, 32'h4300_CAFE);  // CPU0 I→UD (write-alloc)
    cpu0_read (32'h0000_1E30, r0);              // WBF for 0xE30 (dirty); fill 0x1E30
    cpu0_read (32'h0000_0E30, r0);              // L2 hit → SC
    check("T43: Cycle-1 eviction; L2 has 0x4300CAFE",         r0, 32'h4300_CAFE);
    // Cycle 2: write again, evict again
    cpu0_write(32'h0000_0E30, 32'h4300_BEEF);  // SC→CleanUnique→UD
    cpu0_read (32'h0000_1E30, r0);              // WBF for 0xE30 (dirty again)
    cpu0_read (32'h0000_0E30, r0);              // L2 hit → SC with 0x4300_BEEF
    check("T43: Cycle-2 eviction; L2 has 0x4300BEEF",         r0, 32'h4300_BEEF);
    // Cross-core read after two cycles
    cpu1_read (32'h0000_0E30, r1);              // SnpSharedFwd; CPU0 SC stays SC; CPU1 SC
    check("T43: CPU1 sees correct data after 2 evictions",    r1, 32'h4300_BEEF);
    // CPU1 upgrades from SC
    cpu1_write(32'h0000_0E30, 32'h4300_DEAD);  // CleanUnique; CPU0 SC→I; CPU1 UD
    cpu1_read (32'h0000_0E30, r1);
    check("T43: CPU1 UD after upgrade post-eviction cycles",   r1, 32'h4300_DEAD);

    // ── Test 44: CPU1 SD→UD CleanUnique (symmetric to Test 24) ─
    // addr=0x0E40: L1 idx=0xE4.
    // CPU1 gets UD; CPU0 shares → CPU1 UD→SD; CPU1 upgrades from SD.
    $display("\n[TEST 44]  CPU1 SD→UD via CleanUnique (symmetric SD upgrade)");
    cpu1_write(32'h0000_0E40, 32'h4400_0001);  // CPU1 I→UD (write-alloc)
    cpu0_read (32'h0000_0E40, r0);              // ReadShared → SnpSharedFwd; CPU1 UD→SD, CPU0 SC
    check("T44: CPU0 SC gets CPU1 dirty value via SnpSharedFwd", r0, 32'h4400_0001);
    cpu1_write(32'h0000_0E40, 32'h4400_0002);  // CPU1 SD→CleanUnique (dirty wdata to HN); CPU0 SC→I; CPU1 UC→UD
    cpu1_read (32'h0000_0E40, r1);
    check("T44: CPU1 UD after SD CleanUnique, write visible",  r1, 32'h4400_0002);
    cpu0_read (32'h0000_0E40, r0);              // ReadShared → SnpSharedFwd; CPU1 UD→SD, CPU0 SC
    check("T44: CPU0 SC reads CPU1's post-upgrade value",      r0, 32'h4400_0002);

    // ── Summary ───────────────────────────────────────────────
    $display("");
    $display("================================================================");
    $display("  Results:  %0d PASS  |  %0d FAIL", pass_cnt, fail_cnt);
    $display("================================================================");
    $display("");
    $stop;
end

// ── Timeout watchdog ──────────────────────────────────────────
initial begin
    #20_000_000;
    $display("TIMEOUT: simulation exceeded 20ms");
    $stop;
end

endmodule
