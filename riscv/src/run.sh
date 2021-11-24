cd ..
./build_test.sh test.c
cd src
iverilog common/block_ram/block_ram.v common/fifo/fifo.v common/uart/uart_baud_clk.v common/uart/uart.v common/uart/uart_rx.v common/uart/uart_tx.v cpu.v hci.v ram.v riscv_top.v testbench.v -o cpu_run.out
vvp cpu_run.out