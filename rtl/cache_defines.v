`ifndef CACHE_DEFINES_V
`define CACHE_DEFINES_V

// ============================================================
// L1 Cache  —  4KB, direct-mapped, 16-byte (128-bit) blocks
// ============================================================
`define L1_LINES   256    // 4096B / 16B per block
`define L1_TAG_W   20     // addr[31:12]
`define L1_IDX_W   8      // addr[11:4]
`define L1_WOFF_W  2      // addr[3:2]  (32-bit word select within 128-bit block)
`define L1_BOFF_W  2      // addr[1:0]  (byte offset)

// ============================================================
// L2 Cache  —  64KB, 4-way PLRU, 16-byte (128-bit) blocks
// ============================================================
`define L2_SETS    1024   // 65536B / (16B * 4 ways)
`define L2_WAYS    4
`define L2_TAG_W   18     // addr[31:14]
`define L2_IDX_W   10     // addr[13:4]  — different from L1's addr[11:4]

// ============================================================
// DRAM  —  1MB, 10-cycle fixed latency, 128-bit block transfers
// ============================================================
`define DRAM_WORDS    262144  // 1MB / 4 bytes per word
`define DRAM_LATENCY  10

`endif
