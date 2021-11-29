// RISCV32I CPU top module
// port modification allowed for debugging purposes
`include "alu.v"
`include "decoder.v"
`include "dispatcher.v"
`include "fetcher.v"
`include "lsb.v"
`include "pc.v"
`include "register.v"
`include "rob.v"
`include "rs.v"
module cpu(
  input  wire                 clk_in,			// system clock signal
  input  wire                 rst_in,			// reset signal
  input  wire				  rdy_in,			// ready signal, pause cpu when low

  input  wire [ 7:0]          mem_din,		// data input bus
  output wire [ 7:0]          mem_dout,		// data output bus
  output wire [31:0]          mem_a,			// address bus (only 17:0 is used)
  output wire                 mem_wr,			// write/read signal (1 for write)
	
  input  wire                 io_buffer_full, // 1 if uart buffer is full
	
  output wire [31:0]			dbgreg_dout		// cpu register output (debugging demo)
);
  
// implementation goes here

// Specifications:
// - Pause cpu(freeze pc, registers, etc.) when rdy_in is low
// - Memory read result will be returned in the next cycle. Write takes 1 cycle(no need to wait)
// - Memory is of size 128KB, with valid address ranging from 0x0 to 0x20000
// - I/O port is mapped to address higher than 0x30000 (mem_a[17:16]==2'b11)
// - 0x30000 read: read a byte from input
// - 0x30000 write: write a byte to output (write 0x00 is ignored)
// - 0x30004 read: read clocks passed since cpu starts (in dword, 4 bytes)
// - 0x30004 write: indicates program stop (will output '\0' through uart tx)
    wire entry_full = lsb_full || rs_full || rob_full;
    wire lsb_full, rs_full, rob_full;
    wire cdb_flush_enable;
    wire rob_to_lsb_store_enable, rob_to_lsb_io_read_enable;
    wire lsb_to_rob_store_over;
    wire fetch_to_pc_stall;
    wire fetch_to_pc_last_enable;
    wire [`INSTRUCTION_WIDTH] fetch_to_pc_last_inst;
    wire pc_to_fetch_enable;
    wire [`ADDRESS_WIDTH] pc_to_fetch_pc;
    wire pc_to_fetch_predict;
    wire rob_to_pc_feedback;
    wire [`ADDRESS_WIDTH] rob_to_pc_commit_pc;
    wire [`ADDRESS_WIDTH] rob_to_pc_branch_pc;
    wire fetch_to_decode_enable;
    wire [`INSTRUCTION_WIDTH] fetch_to_decode_inst;
    wire [`ADDRESS_WIDTH]     fetch_to_decode_pc;
    wire fetch_to_decode_predict;
    wire [`INSTRUCTION_WIDTH] dispatch_to_fetch_inst;
    wire pc_to_dispatch_req;              wire [`ADDRESS_WIDTH] pc_to_dispatch_req_addr;
    wire lsb_to_dispatch_load_req;        wire [`ADDRESS_WIDTH] lsb_to_dispatch_load_addr;    wire [`RW_STYLE_WIDTH] lsb_to_dispatch_load_style;
    wire lsb_to_dispatch_store_req;       wire [`ADDRESS_WIDTH] lsb_to_dispatch_store_addr;   wire [`RW_STYLE_WIDTH] lsb_to_dispatch_store_style;  wire [`DATA_WIDTH] lsb_to_dispatch_store_data;
    wire dispatch_pc_req_enable;          wire dispatch_pc_data_enable;
    wire dispatch_to_lsb_load_req_enable; wire dispatch_to_lsb_load_data_enable;
    wire dispatch_to_lsb_store_req_enable;
    wire [`DATA_WIDTH] dispatch_to_lsb_load_data;

    wire decode_to_lsb_assign_enable, decode_to_rs_assign_enable, decode_to_rob_assign_enable, decode_to_reg_write_enable;
    wire [`OPERATOR_WIDTH] decode_type;
    wire [`DATA_WIDTH]     decode_imm;
    wire [`ROB_WIDTH]      decode_qj, decode_qk, decode_reorder;
    wire [`DATA_WIDTH]     decode_vj, decode_vk;
    wire [`ADDRESS_WIDTH]  decode_pc;
    wire [`REG_WIDTH]      decode_rd, decode_rs, decode_rt;
    wire                   decode_to_rob_predict;
    wire [`ROB_WIDTH]      decode_to_rob_rs_reorder, decode_to_rob_rt_reorder;
    wire rob_to_decode_rs_ready, rob_to_decode_rt_ready;
    wire [`DATA_WIDTH]     rob_to_decode_rs_value,   rob_to_decode_rt_value;
    wire [`ROB_WIDTH]      rob_to_decode_tail;

    wire reg_to_decode_rs_busy, reg_to_decode_rt_busy;
    wire [`DATA_WIDTH]     reg_to_decode_rs_data, reg_to_decode_rt_data;
    wire [`ROB_WIDTH]      reg_to_decode_rs_reorder, reg_to_decode_rt_reorder;
    
    wire cdb_alu_broadcast_enable,  cdb_lsb_broadcast_enable;
    wire [`ROB_WIDTH]  cdb_alu_broadcast_reorder, cdb_lsb_broadcast_reorder;
    wire [`DATA_WIDTH] cdb_alu_broadcast_result,  cdb_lsb_broadcast_result;
    wire [`ADDRESS_WIDTH] cdb_alu_broadcast_branch;
    wire cdb_lsb_broadcast_io_read;

    wire rs_to_alu_enable;
    wire [`OPERATOR_WIDTH] rs_to_alu_type;
    wire [`ADDRESS_WIDTH]  rs_to_alu_pc;
    wire [`DATA_WIDTH]     rs_to_alu_imm, rs_to_alu_left_oprand, rs_to_alu_right_oprand;
    wire [`ROB_WIDTH]      rs_to_alu_reorder;

    wire rob_to_reg_commit_enable;
    wire [`REG_WIDTH]  rob_to_reg_commit_rd;
    wire [`DATA_WIDTH] rob_to_reg_commit_value;
    wire [`ROB_WIDTH]  rob_to_reg_commit_reorder;
    
    pc PC_CTRL(
        .in_clk                     (clk_in),
        .in_rst                     (rst_in),
        .in_rdy                     (rdy_in),
        .in_stall                   (entry_full || fetch_to_pc_stall),
 
        .in_flush_enable            (cdb_flush_enable),
        .in_feedback_enable         (rob_to_pc_feedback),
        .in_rob_commit_pc           (rob_to_pc_commit_pc),
        .in_rob_branch_pc           (rob_to_pc_branch_pc),
 
        .in_fetcher_last_enable     (fetch_to_pc_last_enable),
        .in_fetcher_last_inst       (fetch_to_pc_last_inst),
        .out_fetcher_enable         (pc_to_fetch_enable),
        .out_fetcher_pc             (pc_to_fetch_pc),
        .out_fetcher_predict        (pc_to_fetch_predict)
    );

    fetcher FETCH(
        .in_clk                     (clk_in),
        .in_rst                     (rst_in),
        .in_rdy                     (rdy_in),
        .in_stall                   (entry_full),
        .in_flush_enable            (cdb_flush_enable),

        .in_fetcher_enable          (pc_to_fetch_enable),
        .in_pc                      (pc_to_fetch_pc),
        .in_predict                 (pc_to_fetch_predict),
        .out_pc_fetch_full          (fetch_to_pc_stall),
        .out_pc_last_enable         (fetch_to_pc_last_enable),
        .out_pc_last_inst           (fetch_to_pc_last_inst),

        .out_decoder_decode_enable  (fetch_to_decode_enable),
        .out_decoder_pc_inst        (fetch_to_decode_inst),
        .out_decoder_pc_addr        (fetch_to_decode_pc),
        .out_decoder_pc_pre         (fetch_to_decode_predict),

        .out_dispatch_pc_requesting (pc_to_dispatch_req),
        .out_dispatch_pc_addr       (pc_to_dispatch_req_addr),
        .in_dispatch_pc_inst        (dispatch_to_fetch_inst),
        .in_pc_req_enable           (dispatch_pc_req_enable),
        .in_pc_data_enable          (dispatch_pc_data_enable)
    );

    dispatcher MEM_CTRL(
        .in_clk                   (clk_in),
        .in_rst                   (rst_in),
        .in_rdy                   (rdy_in),
        .in_flush_enable          (cdb_flush_enable),
        .io_buffer_full           (io_buffer_full),

        .in_mem_data              (mem_din),
        .out_mem_data             (mem_dout),
        .out_mem_addr             (mem_a),
        .out_mem_wr_signal        (mem_wr),

        .in_pc_requesting         (pc_to_dispatch_req),
        .in_pc_addr               (pc_to_dispatch_req_addr),
        .out_pc_inst              (dispatch_to_fetch_inst),
        .out_pc_req_enable        (dispatch_pc_req_enable),
        .out_pc_data_enable       (dispatch_pc_data_enable),

        .in_load_requesting       (lsb_to_dispatch_load_req),
        .in_load_addr             (lsb_to_dispatch_load_addr),
        .in_load_style            (lsb_to_dispatch_load_style),
        .out_load_value           (dispatch_to_lsb_load_data),
        .out_load_req_enable      (dispatch_to_lsb_load_req_enable),
        .out_load_data_enable     (dispatch_to_lsb_load_data_enable),

        .in_store_requesting      (lsb_to_dispatch_store_req),
        .in_store_addr            (lsb_to_dispatch_store_addr),
        .in_store_style           (lsb_to_dispatch_store_style),
        .in_store_value           (lsb_to_dispatch_store_data),
        .out_store_req_enable     (dispatch_to_lsb_store_req_enable)
    );
    
    decoder DECODE(
        .in_decode_enable         (fetch_to_decode_enable),
        .inst                     (fetch_to_decode_inst),
        .pc                       (fetch_to_decode_pc),
        .predict                  (fetch_to_decode_predict),

        .out_type                 (decode_type),
        .out_imm                  (decode_imm),
        .out_Qj                   (decode_qj),
        .out_Qk                   (decode_qk),
        .out_Vj                   (decode_vj),
        .out_Vk                   (decode_vk),
        .out_reorder              (decode_reorder),
        .out_pc                   (decode_pc),
        .out_rd                   (decode_rd),
        .out_rs                   (decode_rs),
        .out_rt                   (decode_rt),

        .out_lsb_assign_enable    (decode_to_lsb_assign_enable),
        .out_rs_assign_enable     (decode_to_rs_assign_enable),
        .out_rob_assign_enable    (decode_to_rob_assign_enable),

        .out_rob_predict          (decode_to_rob_predict),
        .out_rob_rs_reorder       (decode_to_rob_rs_reorder),
        .out_rob_rt_reorder       (decode_to_rob_rt_reorder),
        .in_rob_rs_ready          (rob_to_decode_rs_ready),
        .in_rob_rt_ready          (rob_to_decode_rt_ready),
        .in_rob_rs_value          (rob_to_decode_rs_value),
        .in_rob_rt_value          (rob_to_decode_rt_value),
        .in_rob_tail              (rob_to_decode_tail),

        .out_reg_write_enable     (decode_to_reg_write_enable),
        .in_reg_rs_busy           (reg_to_decode_rs_busy),
        .in_reg_rt_busy           (reg_to_decode_rt_busy),
        .in_reg_rs_data           (reg_to_decode_rs_data),
        .in_reg_rt_data           (reg_to_decode_rt_data),
        .in_reg_rs_reorder        (reg_to_decode_rs_reorder),
        .in_reg_rt_reorder        (reg_to_decode_rt_reorder)
    );    

    lsb LSB(
        .in_clk                   (clk_in),
        .in_rst                   (rst_in),
        .in_rdy                   (rdy_in),
        .in_flush_enable          (cdb_flush_enable),
        .out_capacity_full        (lsb_full),

        .in_decoder_assign_enable (decode_to_lsb_assign_enable),
        .in_decoder_type          (decode_type),
        .in_decoder_imm           (decode_imm),
        .in_decoder_Qj            (decode_qj),
        .in_decoder_Qk            (decode_qk),
        .in_decoder_Vj            (decode_vj),
        .in_decoder_Vk            (decode_vk),
        .in_decoder_dest          (decode_reorder),
        .in_decoder_pc            (decode_pc),

        .in_rob_store_enable      (rob_to_lsb_store_enable),
        .in_rob_io_read_commit    (rob_to_lsb_io_read_enable),
        .out_rob_store_over       (lsb_to_rob_store_over),
        
        .out_dispatch_load_requesting   (lsb_to_dispatch_load_req),
        .out_dispatch_load_addr         (lsb_to_dispatch_load_addr),
        .out_dispatch_load_style        (lsb_to_dispatch_load_style),
        .in_dispatch_load_value         (dispatch_to_lsb_load_data),
        .in_load_req_enable             (dispatch_to_lsb_load_req_enable),
        .in_load_data_enable            (dispatch_to_lsb_load_data_enable),

        .out_dispatch_store_requesting  (lsb_to_dispatch_store_req),
        .out_dispatch_store_addr        (lsb_to_dispatch_store_addr),
        .out_dispatch_store_style       (lsb_to_dispatch_store_style),
        .out_dispatch_store_value       (lsb_to_dispatch_store_data),
        .in_store_req_enable            (dispatch_to_lsb_store_req_enable),

        .in_alu_broadcast_enable        (cdb_alu_broadcast_enable),
        .in_alu_broadcast_reorder       (cdb_alu_broadcast_reorder),
        .in_alu_broadcast_result        (cdb_alu_broadcast_result),

        .out_cdb_broadcast_enable       (cdb_lsb_broadcast_enable),
        .out_cdb_reorder                (cdb_lsb_broadcast_reorder),
        .out_cdb_result                 (cdb_lsb_broadcast_result),
        .out_cdb_io_read                (cdb_lsb_broadcast_io_read)
    );
    
    rs RS(
        .in_clk                   (clk_in),
        .in_rst                   (rst_in || cdb_flush_enable),
        .in_rdy                   (rdy_in),
        .out_capacity_full        (rs_full),

        .in_decoder_assign_enable (decode_to_rs_assign_enable),
        .in_decoder_type          (decode_type),
        .in_decoder_imm           (decode_imm),
        .in_decoder_Qj            (decode_qj),
        .in_decoder_Qk            (decode_qk),
        .in_decoder_Vj            (decode_vj),
        .in_decoder_Vk            (decode_vk),
        .in_decoder_dest          (decode_reorder),
        .in_decoder_pc_addr       (decode_pc),

        .out_alu_enable           (rs_to_alu_enable),
        .out_alu_type             (rs_to_alu_type),
        .out_alu_pc               (rs_to_alu_pc),
        .out_alu_imm              (rs_to_alu_imm),
        .out_alu_left_oprand      (rs_to_alu_left_oprand),
        .out_alu_right_oprand     (rs_to_alu_right_oprand),
        .out_alu_dest             (rs_to_alu_reorder),

        .in_alu_broadcast_enable  (cdb_alu_broadcast_enable),
        .in_alu_broadcast_reorder (cdb_alu_broadcast_reorder),
        .in_alu_broadcast_result  (cdb_alu_broadcast_result),

        .in_lsb_broadcast_enable  (cdb_lsb_broadcast_enable),
        .in_lsb_broadcast_reorder (cdb_lsb_broadcast_reorder),
        .in_lsb_broadcast_result  (cdb_lsb_broadcast_result)
    );

    rob ROB(
        .in_clk                   (clk_in),
        .in_rst                   (rst_in || (rdy_in && cdb_flush_enable)), //self-flush. flush signal maintain 1 cycle
        .in_rdy                   (rdy_in),
        .out_flush_enable         (cdb_flush_enable),
        .out_capacity_full        (rob_full),

        .in_decoder_assign_enable (decode_to_rob_assign_enable),
        .in_decoder_predict       (decode_to_rob_predict),
        .in_decoder_type          (decode_type),
        .in_decoder_rd            (decode_rd),
        .in_decoder_pc            (decode_pc),
        .in_decoder_rs_reorder    (decode_to_rob_rs_reorder),
        .in_decoder_rt_reorder    (decode_to_rob_rt_reorder),
        .out_decoder_rs_ready     (rob_to_decode_rs_ready),
        .out_decoder_rt_ready     (rob_to_decode_rt_ready),
        .out_decoder_rs_value     (rob_to_decode_rs_value),
        .out_decoder_rt_value     (rob_to_decode_rt_value),
        .out_decoder_tail         (rob_to_decode_tail),

        .out_lsb_io_read_commit   (rob_to_lsb_io_read_enable),
        .out_lsb_store_enable     (rob_to_lsb_store_enable),
        .in_lsb_store_over        (lsb_to_rob_store_over),

        .out_reg_commit_enable    (rob_to_reg_commit_enable),
        .out_reg_commit_rd        (rob_to_reg_commit_rd),
        .out_reg_commit_value     (rob_to_reg_commit_value),
        .out_reg_commit_reorder   (rob_to_reg_commit_reorder),

        .out_pc_feedback_enable   (rob_to_pc_feedback),
        .out_pc_commit_pc         (rob_to_pc_commit_pc),
        .out_pc_branch_pc         (rob_to_pc_branch_pc),

        .in_alu_broadcast_enable  (cdb_alu_broadcast_enable),
        .in_alu_broadcast_reorder (cdb_alu_broadcast_reorder),
        .in_alu_broadcast_result  (cdb_alu_broadcast_result),
        .in_alu_broadcast_branch  (cdb_alu_broadcast_branch),

        .in_lsb_broadcast_enable  (cdb_lsb_broadcast_enable),
        .in_lsb_broadcast_reorder (cdb_lsb_broadcast_reorder),
        .in_lsb_broadcast_result  (cdb_lsb_broadcast_result),
        .in_lsb_braodcast_io_read (cdb_lsb_broadcast_io_read)
    );

    alu ALU (
        .in_clk                   (clk_in),
        .in_rst                   (rst_in),
        .in_rdy                   (rdy_in),

        .in_enable                (rs_to_alu_enable),
        .type                     (rs_to_alu_type),
        .pc                       (rs_to_alu_pc),
        .imm                      (rs_to_alu_imm),
        .rs                       (rs_to_alu_left_oprand),
        .rt                       (rs_to_alu_right_oprand),
        .reorder                  (rs_to_alu_reorder),

        .out_cdb_broadcast_enable (cdb_alu_broadcast_enable),
        .out_cdb_broadcast_reorder(cdb_alu_broadcast_reorder),
        .out_cdb_broadcast_result (cdb_alu_broadcast_result),
        .out_cdb_broadcast_branch (cdb_alu_broadcast_branch)
    );

    register REG_FILE(
        .in_clk                   (clk_in),
        .in_rst                   (rst_in),
        .in_rdy                   (rdy_in),
        .in_flush_enable          (cdb_flush_enable),

        .in_decoder_write_enable  (decode_to_reg_write_enable),
        .in_decoder_rd            (decode_rd),
        .in_decoder_rd_reorder    (decode_reorder),
        .in_decoder_rs            (decode_rs),
        .in_decoder_rt            (decode_rt),
        .out_decoder_rs_busy      (reg_to_decode_rs_busy),
        .out_decoder_rs_data      (reg_to_decode_rs_data),
        .out_decoder_rs_reorder   (reg_to_decode_rs_reorder),
        .out_decoder_rt_busy      (reg_to_decode_rt_busy),
        .out_decoder_rt_data      (reg_to_decode_rt_data),
        .out_decoder_rt_reorder   (reg_to_decode_rt_reorder),

        .in_rob_commit_enable     (rob_to_reg_commit_enable),
        .in_rob_rd_addr           (rob_to_reg_commit_rd),
        .in_rob_rd_value          (rob_to_reg_commit_value),
        .in_rob_reorder           (rob_to_reg_commit_reorder)
    );
endmodule