//A Register File with #Reorder tag.
`include "def.v"
module register (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    input  wire     in_flush_enable,
    //connect with control-signal

    input  wire     in_decoder_write_enable,

    input  wire     [`REG_WIDTH] in_decoder_rs,
    input  wire     [`REG_WIDTH] in_decoder_rt,
    input  wire     [`REG_WIDTH] in_decoder_rd,
    input  wire     [`ROB_WIDTH] in_decoder_rd_reorder,

    output wire     out_decoder_rs_busy,
    output wire     [`DATA_WIDTH] out_decoder_rs_data,
    output wire     [`ROB_WIDTH] out_decoder_rs_reorder,

    output wire     out_decoder_rt_busy,
    output wire     [`DATA_WIDTH] out_decoder_rt_data,
    output wire     [`ROB_WIDTH] out_decoder_rt_reorder,
    //connect with Decoder

    input  wire     in_rob_commit_enable,
    input  wire     [`REG_WIDTH] in_rob_rd_addr,
    input  wire     [`DATA_WIDTH] in_rob_rd_value,
    input  wire     [`ROB_WIDTH] in_rob_reorder
    //connect with ROB (commit)
);
    reg [`ROB_WIDTH]  reg_reorder [`REG_ENTRY];
    reg [`DATA_WIDTH] reg_data    [`REG_ENTRY];
    reg reg_busy [`REG_ENTRY];
    integer iter;
    
    assign out_decoder_rs_busy    = reg_busy[in_decoder_rs];
    assign out_decoder_rs_data    = reg_data[in_decoder_rs];
    assign out_decoder_rs_reorder = reg_reorder[in_decoder_rs];
    assign out_decoder_rt_busy    = reg_busy[in_decoder_rt];
    assign out_decoder_rt_data    = reg_data[in_decoder_rt];
    assign out_decoder_rt_reorder = reg_reorder[in_decoder_rt];
wire [`DATA_WIDTH] reg_a0 = reg_data[5'd10];
wire [`DATA_WIDTH] reg_a1 = reg_data[5'd11];
wire [`DATA_WIDTH] reg_a3 = reg_data[5'd13];
wire [`DATA_WIDTH] reg_a5 = reg_data[5'd15];
wire [`DATA_WIDTH] reg_a6 = reg_data[5'd16];
wire [`DATA_WIDTH] reg_s0 = reg_data[5'd8];
wire [`DATA_WIDTH] reg_s2 = reg_data[5'd18];
wire [`DATA_WIDTH] reg_zero = reg_data[5'd0];
wire               reg_zero_busy = reg_busy[5'd0];
wire [`DATA_WIDTH] reg_s4 = reg_data[5'd20];
wire [`DATA_WIDTH] reg_s6 = reg_data[5'd22];
wire [`DATA_WIDTH] reg_s1 = reg_data[5'd9];
wire [`DATA_WIDTH] reg_s11 = reg_data[5'd27];
wire [`DATA_WIDTH] reg_sp = reg_data[5'd1];
wire [`ROB_WIDTH]  reg_sp_reorder = reg_reorder[5'd1];
wire               reg_sp_busy = reg_busy[5'd1];
wire [`ROB_WIDTH]  reg_a0_reorder = reg_reorder[5'd10];
wire               reg_a0_busy = reg_busy[5'd10];
    always @(posedge in_clk) begin
        if (in_rst) begin
            for (iter = 0 ; iter < `REG_SIZE ; iter = iter+1) begin
                reg_data[iter]    <= `ZERO_DATA;
                reg_busy[iter]    <= `FALSE;
                reg_reorder[iter] <= `ZERO_ROB;
            end
        end
        else if(in_rdy) begin
            if (in_flush_enable) begin
                for (iter = 0; iter < `REG_SIZE ; iter=iter+1) begin
                    reg_busy[iter]    <= `FALSE;
                    reg_reorder[iter] <= `ZERO_ROB;
                end
                if (in_rob_commit_enable && in_rob_rd_addr != `ZERO_REG) begin //jalr
                    reg_data[in_rob_rd_addr] <= in_rob_rd_value;
                end
            end
            else begin
                if (in_rob_commit_enable && in_rob_rd_addr != `ZERO_REG) begin
                    reg_data[in_rob_rd_addr] <= in_rob_rd_value;
                    if (reg_reorder[in_rob_rd_addr] == in_rob_reorder) begin
                        reg_busy[in_rob_rd_addr] <= `FALSE;
                        reg_reorder[in_rob_rd_addr] <= `ZERO_ROB;
                    end
                end
                if (in_decoder_write_enable) begin
                    reg_busy[in_decoder_rd] <= `TRUE;
                    reg_reorder[in_decoder_rd] <= in_decoder_rd_reorder;
                end
            end
        end
    end
endmodule //register. write into x[0] is forbidden