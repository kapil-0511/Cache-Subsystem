# =============================================================================
# setup_project.tcl  —  Dual-L1 CHI Coherent Cache  |  Vivado Project Setup
# =============================================================================
# Vivado Tcl console:
#   source C:/Users/kapily/Downloads/cache_subsystem/sim/setup_project.tcl
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set rtl_dir    [file normalize [file join $script_dir .. rtl]]
set tb_dir     [file normalize [file join $script_dir .. tb]]

puts ""
puts "==================================================================="
puts "  Cache Subsystem  |  Dual-L1 CHI-B Coherent Cache  |  Project Setup"
puts "  L1×2: 4KB DM  |  L2: 64KB 4-way PLRU  |  DRAM: 1MB"
puts "  Block: 128-bit  |  Protocol: ARM AMBA CHI-B"
puts "==================================================================="

# ── Detect simulator ──────────────────────────────────────────────────────────
if {[info commands create_project] ne ""} {
    set SIM_TOOL "vivado"
} elseif {[info commands vlib] ne ""} {
    set SIM_TOOL "modelsim"
} else {
    puts "\[ERROR\]  Cannot detect simulator."
    return
}
puts "\[INFO\]  Simulator : $SIM_TOOL"
puts "\[INFO\]  RTL dir   : $rtl_dir"
puts "\[INFO\]  TB dir    : $tb_dir"
puts ""

# =============================================================================
# ── VIVADO ────────────────────────────────────────────────────────────────────
# =============================================================================
if {$SIM_TOOL eq "vivado"} {

    set proj_dir [file normalize [file join $script_dir vivado_proj]]
    create_project -force cache_coherent $proj_dir -part xc7a35tcpg236-1

    set_property simulator_language Mixed         [current_project]
    set_property target_language   Verilog        [current_project]
    set_property default_lib       xil_defaultlib [current_project]

    # ── RTL sources ───────────────────────────────────────────────────────────
    set rtl_files [list \
        [file join $rtl_dir cache_defines.v     ] \
        [file join $rtl_dir chi_defines.v       ] \
        [file join $rtl_dir dram_model.v        ] \
        [file join $rtl_dir l1_cache.v          ] \
        [file join $rtl_dir l2_cache.v          ] \
        [file join $rtl_dir core.v              ] \
        [file join $rtl_dir coherence_manager.v ] \
        [file join $rtl_dir chi_fabric.v        ] \
        [file join $rtl_dir cache_top.v         ] \
    ]
    add_files -fileset sources_1 $rtl_files

    # Header files — Vivado skips solo compilation for these
    set_property file_type {Verilog Header} [get_files */cache_defines.v]
    set_property file_type {Verilog Header} [get_files */chi_defines.v]
    set_property include_dirs [list $rtl_dir] [get_filesets sources_1]
    set_property top cache_top [get_filesets sources_1]
    update_compile_order -fileset sources_1
    puts "\[INFO\]  RTL sources added (9 files)."

    # ── Testbench ─────────────────────────────────────────────────────────────
    set tb_files [list \
        [file join $tb_dir tb_cache_top.sv] \
    ]
    add_files -fileset sim_1 -norecurse $tb_files
    set_property file_type SystemVerilog \
        [get_files -of_objects [get_filesets sim_1] -filter {NAME =~ *.sv}]
    set_property include_dirs [list $rtl_dir] [get_filesets sim_1]
    set_property xsim.simulate.runtime "" [get_filesets sim_1]
    set_property top     tb_cache_top  [get_filesets sim_1]
    set_property top_lib xil_defaultlib [get_filesets sim_1]
    update_compile_order -fileset sim_1
    puts "\[INFO\]  Testbench added (1 file)."

    puts ""
    puts "==================================================================="
    puts "  Setup complete (Vivado).  Open project and run simulation, or:"
    puts "    launch_simulation"
    puts "    run all"
    puts "    close_simulation"
    puts "==================================================================="

# =============================================================================
# ── MODELSIM / QUESTA ─────────────────────────────────────────────────────────
# =============================================================================
} else {

    if {[file exists work]} { vdel -lib work -all }
    vlib work
    vmap work work
    puts "\[INFO\]  'work' library created."

    puts "\[COMPILE\]  RTL..."
    vlog +incdir+$rtl_dir \
        $rtl_dir/cache_defines.v      \
        $rtl_dir/chi_defines.v        \
        $rtl_dir/dram_model.v         \
        $rtl_dir/l1_cache.v           \
        $rtl_dir/l2_cache.v           \
        $rtl_dir/core.v               \
        $rtl_dir/coherence_manager.v  \
        $rtl_dir/chi_fabric.v         \
        $rtl_dir/cache_top.v

    puts "\[COMPILE\]  Testbench..."
    vlog -sv +incdir+$rtl_dir \
        $tb_dir/tb_cache_top.sv

    puts ""
    puts "==================================================================="
    puts "  Setup complete (ModelSim).  Run simulation:"
    puts "    vsim -c tb_cache_top -do \"run -all\""
    puts "==================================================================="
}

puts ""
