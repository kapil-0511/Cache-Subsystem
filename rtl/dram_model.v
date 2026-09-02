`timescale 1ns/1ps
`include "cache_defines.v"

// ============================================================
// dram_model.v  —  Behavioral DRAM Model
//
// Capacity : 1 MB  (262144 × 32-bit words)
// Latency  : 10 clock cycles (fixed)
// Transfers: 128-bit block (4 consecutive 32-bit words, block-aligned)
//
// Interface:
//   Caller asserts dram_ren or dram_wen with block-aligned dram_addr.
//   After DRAM_LATENCY cycles, dram_ack pulses for one cycle.
//   For reads, dram_rdata is valid when dram_ack=1.
// ============================================================

module dram_model (
    input  wire         clk,
    input  wire         rst_n,

    input  wire [31:0]  dram_addr,    // block-aligned byte address
    input  wire [127:0] dram_wdata,
    input  wire         dram_wen,
    input  wire         dram_ren,
    output reg  [127:0] dram_rdata,
    output reg          dram_ack
);

// Memory: 262144 × 32-bit = 1MB
reg [31:0] mem [0:`DRAM_WORDS - 1];

// Block word address: first 32-bit word of the 128-bit (4-word) block
// addr[19:4] = 16-bit block index → × 4 words = 18-bit word address
wire [17:0] blk_waddr = {dram_addr[19:4], 2'b00};

localparam D_IDLE  = 2'd0;
localparam D_READ  = 2'd1;
localparam D_WRITE = 2'd2;

reg [1:0] d_state;
reg [3:0] lat_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        d_state    <= D_IDLE;
        lat_cnt    <= 4'd0;
        dram_ack   <= 1'b0;
        dram_rdata <= 128'd0;
    end else begin
        dram_ack <= 1'b0;

        case (d_state)
            D_IDLE: begin
                lat_cnt <= 4'd0;
                if (dram_ren)      d_state <= D_READ;
                else if (dram_wen) d_state <= D_WRITE;
            end

            D_READ: begin
                if (lat_cnt == `DRAM_LATENCY - 1) begin
                    dram_rdata <= {mem[blk_waddr + 3], mem[blk_waddr + 2],
                                   mem[blk_waddr + 1], mem[blk_waddr + 0]};
                    dram_ack   <= 1'b1;
                    d_state    <= D_IDLE;
                end else begin
                    lat_cnt <= lat_cnt + 1;
                end
            end

            D_WRITE: begin
                if (lat_cnt == `DRAM_LATENCY - 1) begin
                    mem[blk_waddr + 0] <= dram_wdata[31:0];
                    mem[blk_waddr + 1] <= dram_wdata[63:32];
                    mem[blk_waddr + 2] <= dram_wdata[95:64];
                    mem[blk_waddr + 3] <= dram_wdata[127:96];
                    dram_ack  <= 1'b1;
                    d_state   <= D_IDLE;
                end else begin
                    lat_cnt <= lat_cnt + 1;
                end
            end

            default: d_state <= D_IDLE;
        endcase
    end
end

endmodule
