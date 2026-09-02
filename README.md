# Dual-Core CHI-B Cache Coherence Subsystem

A fully RTL-implemented, simulation-verified dual-core shared-memory cache hierarchy conforming to the **ARM AMBA CHI Issue B** protocol. Includes a private L1 per core, a shared L2 with integrated Home Node Function and Snoop Filter, a CHI fabric, and a behavioural DRAM model.

**Simulation result: 111 PASS | 0 FAIL** across 44 test groups.

---

## Repository Layout

```
cache_subsystem/
├── rtl/
│   ├── cache_defines.v          # L1 / L2 / DRAM geometry constants
│   ├── chi_defines.v            # CHI-B opcode & cache-state constants
│   ├── l1_cache.v               # 4 KB direct-mapped L1 array
│   ├── l2_cache.v               # 64 KB 4-way PLRU L2 + DRAM FSM
│   ├── core.v                   # RNF FSM + L1 controller (per core)
│   ├── coherence_manager.v      # HN-F FSM + L2 + Snoop Filter
│   ├── chi_fabric.v             # CHI crossbar (6 channels, 2 cores)
│   ├── cache_top.v              # Top-level instantiation
│   └── dram_model.v             # Behavioural DRAM (10-cycle latency)
├── tb/
│   └── tb_cache_top.sv          # SystemVerilog testbench (44 tests, 111 checks)
├── sim/vivado_proj/cache_coherent.sim/sim_1/behav/xsim/
│   ├── tb_cache_top_vlog.prj    # xvlog compile list
│   └── tb_cache_top.tcl         # xsim run script
└── cache_coherence_report.pdf   # Full design report (block diagrams, FSMs, opcodes, tests)
```

---

## System Architecture

```
 CPU0 ──► ┌─────────────────────┐        ┌──────────────────────────────────┐
          │  Core 0  (RNF)      │        │  Coherence Manager  (HN-F)       │
          │  ┌───────────────┐  │        │  ┌────────────────────────────┐  │
          │  │  L1 Cache     │  │◄──────►│  │  L2 Cache  (64KB, 4-way)   │  │
          │  │  4KB · DM     │  │  CHI   │  │  + Snoop Filter (1024 sets)│  │        ┌──────────┐
          │  └───────────────┘  │ Fabric │  └────────────────────────────┘  │◄──────►│  DRAM    │
          │  CHI RNF FSM        │        │  HNF FSM                         │        │  1 MB    │
          └─────────────────────┘        └──────────────────────────────────┘        └──────────┘
 CPU1 ──► ┌─────────────────────┐                ▲
          │  Core 1  (RNF)      │◄───────────────┘
          │  ┌───────────────┐  │
          │  │  L1 Cache     │  │
          │  │  4KB · DM     │  │
          │  └───────────────┘  │
          │  CHI RNF FSM        │
          └─────────────────────┘
```

---

## Cache Geometry

| Parameter       | L1 Cache              | L2 Cache                 | DRAM               |
|-----------------|-----------------------|--------------------------|--------------------|
| Size            | 4 KB                  | 64 KB                    | 1 MB               |
| Organization    | 256 sets, direct-map  | 1024 sets, 4-way PLRU    | —                  |
| Block size      | 128 bits (16 bytes)   | 128 bits (16 bytes)      | 128-bit transfers  |
| Tag             | `addr[31:12]` (20 b)  | `addr[31:14]` (18 b)     | —                  |
| Index           | `addr[11:4]`  (8 b)   | `addr[13:4]`  (10 b)     | —                  |
| Word offset     | `addr[3:2]`   (2 b)   | `addr[3:2]`   (2 b)      | —                  |
| Conflict stride | `0x1000`              | `0x4000`                 | —                  |
| Latency         | 1 cycle (hit)         | 2–3 cycles (tag + data)  | 10 cycles (fixed)  |
| Init pattern    | —                     | —                        | `{0xDA00, widx[15:0]}` |

---

## Coherence States

Five-state MOESI subset tracked per L1 cache line in a 3-bit `cs[]` field:

| State | Code   | Description                                                  |
|-------|--------|--------------------------------------------------------------|
| **I** | `3'd0` | Invalid — line not present                                   |
| **SC**| `3'd1` | Shared Clean — clean copy, may be shared with peer           |
| **UC**| `3'd2` | Unique Clean — exclusive clean; transitional before write    |
| **UD**| `3'd3` | Unique Dirty — exclusive dirty; L2 stale; must write back    |
| **SD**| `3'd4` | Shared Dirty — master dirty copy; peer may hold SC           |

---

## Supported CHI-B Opcodes

### REQ Channel (RN → HN)

| Opcode | Hex    | Name              | Trigger              | Final L1 state |
|--------|--------|-------------------|----------------------|----------------|
| `6'h01`| `0x01` | ReadShared        | Read miss (I)        | SC             |
| `6'h07`| `0x07` | ReadUnique        | Write miss (I)       | UD             |
| `6'h0B`| `0x0B` | CleanUnique       | Write hit (SC or SD) | UD             |
| `6'h0C`| `0x0C` | MakeUnique        | Full-line store (I)  | UD *(not triggered at 32-bit interface)* |
| `6'h0D`| `0x0D` | Evict             | Clean eviction       | I              |
| `6'h1B`| `0x1B` | WriteBackFull     | Dirty eviction       | I              |

### SNP Channel (HN → RN)

| Opcode | Hex    | Name            | Action required                              |
|--------|--------|-----------------|----------------------------------------------|
| `5'h01`| `0x01` | SnpShared       | Retain SC or downgrade from UD               |
| `5'h07`| `0x07` | SnpUnique       | Invalidate; write dirty data to HN if dirty  |
| `5'h08`| `0x08` | SnpClean        | Clean dirty data; may stay SC                |
| `5'h09`| `0x09` | SnpSharedFwd    | UD→SD, forward dirty data to requester       |

### DAT `Resp[2:0]` — Granted Cache State

| Resp    | Binary    | State | Meaning                                         |
|---------|-----------|-------|-------------------------------------------------|
| RESP_UC | `3'b010`  | UC    | ReadUnique from clean L2                        |
| RESP_UD | `3'b011`  | UD    | ReadUnique; dirty data forwarded from peer      |
| RESP_SC | `3'b110`  | SC    | ReadShared granted                              |
| RESP_SD | `3'b111`  | SD    | SnpSharedFwd; snoopee keeps SD master copy      |

---

## RTL Module Summary

### `core.v` — Request Node Function (per core)

12-state FSM handling the full RNF protocol:

```
S_IDLE → S_CHECK ──── L1 hit (read)  ──────────────────────────► S_RESP → S_IDLE
              │
              ├── L1 hit (write SC/SD) ──► S_REQ ──► S_WAIT_COMP ──► S_LOCAL_WR ──► S_RESP
              │
              ├── L1 miss, dirty evict ──► S_EVICT_REQ ──► S_EVICT_WAIT ──► S_REQ ──► ...
              │
              └── L1 miss, clean evict ──► S_EVICT_CLEAN ──► S_EVICT_CLEAN_WAIT ──► S_REQ ──► ...

S_IDLE ──── rxsnp_valid ──► S_SNP ──► S_SNP_RSP ──► S_IDLE
```

**Key design decision — `S_LOCAL_WR`:** After receiving CompData (RESP_UC) for ReadUnique or Comp for CleanUnique, the line is installed as UC. `S_LOCAL_WR` then applies the pending CPU write (byte-enabled merge) and promotes the line to UD, without any additional CHI traffic.

### `coherence_manager.v` — Home Node Function

8-state FSM, single-issue (one request in-flight at a time):

```
HNF_IDLE → HNF_DECODE ─┬─ peer in SF ──► HNF_SNOOP ──► HNF_WAIT_SRSP ─┬─ dirty data ──► HNF_WB_L2 ──► HNF_FETCH_L2 ──► HNF_COMP
                        │                                                 ├─ RU dirty fwd ──────────────────────────────────► HNF_COMP
                        │                                                 └─ clean, perm-only ──────────────────────────────► HNF_COMP
                        ├─ no peer, ReadSh/RU ──► HNF_FETCH_L2 ──────────────────────────────────────────────────────────► HNF_COMP
                        ├─ WriteBackFull ──► HNF_WB_L2 ─────────────────────────────────────────────────────────────────► HNF_COMP
                        └─ Evict / CU no peer ───────────────────────────────────────────────────────────────────────────► HNF_COMP
                                                                                                                              │
                                                                                                                         HNF_WAIT_ACK → HNF_IDLE
```

**ReadUnique dirty forward:** When the peer holds UD and a ReadUnique arrives, the HNF forwards the dirty data directly to the requester as `RESP_UD` without writing to L2. L2 is updated later when the requester evicts via WriteBackFull.

### `chi_fabric.v` — CHI Crossbar

Passive, zero-latency mux routing 6 CHI channels between 2 cores and the HN-F. Fixed priority: core 0 wins contention on shared uplink channels.

### `l2_cache.v` — L2 Cache

4-way PLRU (3-bit PLRU tree per set). Interfaces with the coherence manager and drives the DRAM model on misses. Performance counters (`perf_hits`, `perf_misses`) exposed as top-level outputs.

---

## Running the Simulation

Requires **Vivado 2025.1** (or compatible) installed at `C:\Xilinx\2025.1\`.

```powershell
cd sim\vivado_proj\cache_coherent.sim\sim_1\behav\xsim

# 1. Compile
xvlog.bat --incr --relax -L uvm -prj tb_cache_top_vlog.prj

# 2. Elaborate
xelab.bat --incr --debug typical --relax --mt 2 `
  -L xil_defaultlib -L uvm -L unisims_ver -L unimacro_ver -L secureip -L xpm `
  --snapshot tb_cache_top_behav `
  xil_defaultlib.tb_cache_top xil_defaultlib.glbl

# 3. Simulate
xsim.bat tb_cache_top_behav -tclbatch tb_cache_top.tcl
```

Expected output (last lines):
```
================================================================
  Results:  111 PASS  |  0 FAIL
================================================================
```

---

## Transaction Monitor Output

The testbench prints one line per CHI channel handshake, making it easy to follow the coherence protocol in the log:

```
[L1-0] t=     65000  addr=0x00000100  L1_state=I   MISS
[REQ]  t=     85000  CPU0→HN  ReadShared     addr=0x00000100  wdata[31:0]=0x00000000
[L2  ] t=    115000  MISS   (cumulative misses=1)
[CDAT] t=    285000  HN→CPU0   CompData        resp=SC  data[31:0]=0xda000040

[L1-0] t=    465000  addr=0x00000100  L1_state=SC  HIT
[REQ]  t=    485000  CPU0→HN  CleanUnique    addr=0x00000100  wdata[31:0]=0xda000040
[SNP]  t=    495000  HN→CPU1   SnpUnique      addr=0x00000100
[SRSP] t=    525000  CPU1→HN   SnpResp_I   (no data)
[COMP] t=    545000  HN→CPU0   Comp
```

| Tag     | Trigger                            | Shows                                        |
|---------|------------------------------------|----------------------------------------------|
| `[L1-x]`| Core enters S_CHECK                | Address, L1 state, HIT/MISS                  |
| `[REQ]` | `hn_rxreq_valid & lcrd`            | Source, opcode, address, wdata[31:0]         |
| `[SNP]` | `hn_txsnp_valid & rdy`             | Target, snoop opcode, address                |
| `[SRSP]`| `hn_rxrsp_valid & lcrd`            | Source, RSP opcode (clean, no data)          |
| `[SDAT]`| `hn_rxsnpdat_valid & lcrd`         | Source, DAT opcode, Resp (state), data[31:0] |
| `[CDAT]`| `hn_txdat_valid & ack`             | Target, DAT opcode, Resp (granted state), data |
| `[COMP]`| `hn_txrsp_valid & ack`             | Target, RSP opcode (permission grant)        |
| `[L2  ]`| `l2_hits` or `l2_misses` change    | HIT or MISS, cumulative count                |

---

## Test Suite Overview

44 test groups covering the full CHI-B state machine:

| Tests      | Focus area                                                         |
|------------|--------------------------------------------------------------------|
| 1–9        | Core protocol: cold read, shared read, SC→UD, SnpSharedFwd, WBF, write-allocate, byte-enable |
| 10         | L2 PLRU eviction stress (5 blocks → same L2 set)                  |
| 12–16      | CPU1 write-allocate, false sharing, three-way coherence chain, Evict, CPU1 WBF |
| 21, 23, 24 | Evict for snoop-free fill; UD→SD via SnpSharedFwd; SD→UD via CleanUnique |
| 25         | Performance counter readback                                       |
| 26–27      | All byte-enable combinations; all 4 words in a 128-bit cache line  |
| 28         | ReadUnique RESP_UD dirty forwarding (peer UD → requester UD)       |
| 29–31      | Round-robin ownership; I→SC→UD sequence; SD CleanUnique            |
| 32–33      | Independent addresses; L1 capacity sweep (8 sets)                  |
| 34–39      | Dirty eviction + CPU1 reads, sparse coverage, SC→UD+WBF, adjacent blocks |
| 40         | CPU1 cold reads from DRAM                                          |
| 41         | CPU1 CleanUnique from SC (symmetric upgrade)                       |
| 42         | All 5 CHI REQ opcodes exercised on a single cache line             |
| 43         | Two dirty eviction+refill cycles on the same address               |
| 44         | CPU1 SD→UD via CleanUnique (symmetric to test 24)                  |

---

## Design Report

A full design report with block diagrams, FSM state diagrams (SVG), opcode tables, transaction flow traces, and test coverage matrix is available at:

```
cache_coherence_report_redesigned.pdf
```

---

## Protocol Compliance Notes

- **CHI-B field widths:** REQ `[5:0]`, SNP `[4:0]`, RSP `[3:0]`, DAT `[2:0]` + Resp `[2:0]`
- **Snoop response split:** clean responses use the RSP channel (`SnpRespI`, `SnpRespSC`); dirty responses use the DAT channel (`SnpRespData`, `SnpRespDataFwd`) — matching CHI-B §4.4
- **MakeUnique:** defined and handled in HNF (peer dirty data is discarded per spec) but not reachable from the 32-bit CPU interface
- **No request pipelining:** HNF is single-issue; a second REQ is held off until completion acknowledgement
- **Dirty forward (RESP_UD):** ReadUnique with a UD peer bypasses L2 — dirty data goes directly to requester; L2 updated on subsequent WBF eviction
