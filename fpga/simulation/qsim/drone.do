onerror {quit -f}
vlib work
vlog -work work drone.vo
vlog -work work drone.vt
vsim -novopt -c -t 1ps -L cycloneii_ver -L altera_ver -L altera_mf_ver -L 220model_ver -L sgate work.drone_vlg_vec_tst
vcd file -direction drone.msim.vcd
vcd add -internal drone_vlg_vec_tst/*
vcd add -internal drone_vlg_vec_tst/i1/*
add wave /*
run -all
