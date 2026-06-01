onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_viga_circuit/dut/clock
add wave -noupdate /tb_viga_circuit/dut/reset
add wave -noupdate -radix decimal /tb_viga_circuit/dut/xr
add wave -noupdate -radix decimal /tb_viga_circuit/dut/xe
add wave -noupdate /tb_viga_circuit/dut/start
add wave -noupdate /tb_viga_circuit/dut/sample_tick
add wave -noupdate /tb_viga_circuit/dut/coeff_tick
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/coeff_in
add wave -noupdate -radix decimal /tb_viga_circuit/dut/data_out
add wave -noupdate /tb_viga_circuit/dut/LEDR
add wave -noupdate /tb_viga_circuit/dut/valid_fir_s
add wave -noupdate /tb_viga_circuit/dut/busy_fir_s
add wave -noupdate /tb_viga_circuit/dut/coeff_ok_fir_s
add wave -noupdate /tb_viga_circuit/dut/enable_fir_s
add wave -noupdate /tb_viga_circuit/dut/mode_fir_s
add wave -noupdate /tb_viga_circuit/dut/configured_s
add wave -noupdate /tb_viga_circuit/dut/running_s
add wave -noupdate -divider Datapath
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/xr
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/xe
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/coeff_in
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/enable_fir
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/mode_fir
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/data_out
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/valid_fir
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/busy_fir
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/coeff_ok_fir
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/fc
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/yf
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/xc
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/xfx
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/valid_wf
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/valid_ws
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/busy_wf
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/busy_ws
add wave -noupdate -divider control
add wave -noupdate /tb_viga_circuit/dut/control_inst/start
add wave -noupdate /tb_viga_circuit/dut/control_inst/sample_tick
add wave -noupdate /tb_viga_circuit/dut/control_inst/coeff_tick
add wave -noupdate /tb_viga_circuit/dut/control_inst/coeff_ok_fir
add wave -noupdate /tb_viga_circuit/dut/control_inst/busy_fir
add wave -noupdate /tb_viga_circuit/dut/control_inst/valid_fir
add wave -noupdate /tb_viga_circuit/dut/control_inst/enable_fir
add wave -noupdate /tb_viga_circuit/dut/control_inst/mode_fir
add wave -noupdate /tb_viga_circuit/dut/control_inst/configured
add wave -noupdate /tb_viga_circuit/dut/control_inst/running
add wave -noupdate /tb_viga_circuit/dut/control_inst/state
TreeUpdate [SetDefaultTree]
quietly WaveActivateNextPane
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/xc
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/xfx
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/xe
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/clock
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/reset
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/en
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/fc
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/q_dados1
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/q_dados2
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/q_coef
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/controle
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/controle2
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/controle3
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/data_coef_ram
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/data_xfx_ram
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/data_xc_ram
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/endereco
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/endereco2
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/endereco3
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/clear_addr
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/p
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/i
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/j
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/cont_div
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/num
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/denum
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/mi
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/coef
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/saida_adap
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/pre_filtrado
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/energ
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/energ16
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/fator
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/fator16
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/mult32
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/mult16
add wave -noupdate -radix decimal /tb_viga_circuit/dut/datapath_inst/wc_inst/erro
add wave -noupdate -radix hexadecimal /tb_viga_circuit/dut/datapath_inst/wc_inst/fc_reg
add wave -noupdate /tb_viga_circuit/dut/datapath_inst/wc_inst/estado
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {9811425 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 339
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 100
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits us
update
WaveRestoreZoom {9427978 ps} {10105100 ps}
