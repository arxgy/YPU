//Reserve Station module (single module supported)
`include "def.v"
module rs (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    output wire     out_capacity_full,
    //connect with control-signal

    input  wire     in_decoder_assign_enable,
    input  wire     [`OPERATOR_WIDTH] in_decoder_type,
    input  wire     [`DATA_WIDTH]in_decoder_imm,
    input  wire     [`ROB_WIDTH] in_decoder_Qj,
    input  wire     [`ROB_WIDTH] in_decoder_Qk,
    input  wire     [`DATA_WIDTH] in_decoder_Vj,
    input  wire     [`DATA_WIDTH] in_decoder_Vk,
    input  wire     [`ROB_WIDTH] in_decoder_dest,
    input  wire     [`ADDRESS_WIDTH] in_decoder_pc_addr,
    //connect with Decoder

    output reg      out_alu_enable,
    output reg      [`OPERATOR_WIDTH] out_alu_type,
    output reg      [`ADDRESS_WIDTH] out_alu_pc,
    output reg      [`DATA_WIDTH] out_alu_imm,
    output reg      [`DATA_WIDTH] out_alu_left_oprand,
    output reg      [`DATA_WIDTH] out_alu_right_oprand,
    output reg      [`ROB_WIDTH] out_alu_dest,
    //connect with ALU (arithmetic output)

    input  wire     in_alu_broadcast_enable,
    input  wire     [`ROB_WIDTH] in_alu_broadcast_reorder,
    input  wire     [`DATA_WIDTH] in_alu_broadcast_result,
    //cdb from alu
    input  wire     in_lsb_broadcast_enable,
    input  wire     [`ROB_WIDTH] in_lsb_broadcast_reorder,
    input  wire     [`DATA_WIDTH] in_lsb_broadcast_result
    //cdb from lsb
);
    integer iter, lsb_iter, alu_iter;
    
    reg [`OPERATOR_WIDTH]   type_entry  [`RS_ENTRY];
    reg [`DATA_WIDTH]       imm_entry   [`RS_ENTRY];
    reg [`ROB_WIDTH]        qj_entry    [`RS_ENTRY];
    reg [`ROB_WIDTH]        qk_entry    [`RS_ENTRY];
    reg [`DATA_WIDTH]       vj_entry    [`RS_ENTRY];
    reg [`DATA_WIDTH]       vk_entry    [`RS_ENTRY];
    reg [`ROB_WIDTH]        dest_entry  [`RS_ENTRY];
    reg [`ADDRESS_WIDTH]    pc_entry    [`RS_ENTRY];
    reg                     busy_entry  [`RS_ENTRY];
    wire                    ready_entry [`RS_ENTRY];
    wire [`ROB_WIDTH]       assign_index;
    wire [`ROB_WIDTH]       ready_index;

    assign out_capacity_full = (assign_index == `ZERO_RS);

    generate
        genvar i;
        for (i = 1 ; i < `RS_SIZE; i = i+1 ) begin : ready_assign
            assign ready_entry[i] = (busy_entry[i] && (qj_entry[i] == `ZERO_ROB) && (qk_entry[i] == `ZERO_ROB));
        end        
    endgenerate

    always @(posedge in_clk) begin
        if (in_rst) begin
            out_alu_enable <= `FALSE;
            for (iter = 1 ; iter < `RS_SIZE ; iter = iter+1) begin
                busy_entry[iter] <= `FALSE;
            end
        end
        else if(in_rdy) begin
            if (in_decoder_assign_enable) begin
                busy_entry[assign_index] <= `TRUE;
                type_entry[assign_index] <= in_decoder_type;
                imm_entry[assign_index]  <= in_decoder_imm;
                qj_entry[assign_index]   <= in_decoder_Qj;
                qk_entry[assign_index]   <= in_decoder_Qk;
                vj_entry[assign_index]   <= in_decoder_Vj;
                vk_entry[assign_index]   <= in_decoder_Vk;
                dest_entry[assign_index] <= in_decoder_dest;
                pc_entry[assign_index]   <= in_decoder_pc_addr;
            end
            //accept broadcast
            if (in_lsb_broadcast_enable) begin
                for (lsb_iter = 1 ; lsb_iter < `RS_SIZE ; lsb_iter = lsb_iter+1) begin
                    if (in_decoder_assign_enable && lsb_iter == assign_index) begin
                        if (in_decoder_Qj == in_lsb_broadcast_reorder) begin 
                            qj_entry[lsb_iter] <= `ZERO_ROB;
                            vj_entry[lsb_iter] <= in_lsb_broadcast_reorder;
                        end
                        if (in_decoder_Qk == in_lsb_broadcast_reorder) begin
                            qk_entry[lsb_iter] <= `ZERO_ROB;
                            vk_entry[lsb_iter] <= in_lsb_broadcast_reorder;
                        end
                    end
                    else begin
                        if (qj_entry[lsb_iter] == in_lsb_broadcast_reorder) begin
                            qj_entry[lsb_iter] <= `ZERO_ROB;
                            vj_entry[lsb_iter] <= in_lsb_broadcast_result;
                        end
                        if (qk_entry[lsb_iter] == in_lsb_broadcast_reorder) begin
                            qk_entry[lsb_iter] <= `ZERO_ROB;
                            vk_entry[lsb_iter] <= in_lsb_broadcast_result;
                        end
                    end
                end
            end
            if (in_alu_broadcast_enable) begin
                for (alu_iter = 1 ; alu_iter < `RS_SIZE ; alu_iter = alu_iter+1) begin
                    if (in_decoder_assign_enable && alu_iter == assign_index) begin
                         if (in_decoder_Qj == in_alu_broadcast_reorder) begin
                            qj_entry[alu_iter] <= `ZERO_ROB;
                            vj_entry[alu_iter] <= in_alu_broadcast_result;
                        end
                        if (in_decoder_Qk == in_alu_broadcast_reorder) begin
                            qk_entry[alu_iter] <= `ZERO_ROB;
                            vk_entry[alu_iter] <= in_alu_broadcast_result;
                        end
                    end
                    else begin
                        if (qj_entry[alu_iter] == in_alu_broadcast_reorder) begin
                            qj_entry[alu_iter] <= `ZERO_ROB;
                            vj_entry[alu_iter] <= in_alu_broadcast_result;
                        end
                        if (qk_entry[alu_iter] == in_alu_broadcast_reorder) begin
                            qk_entry[alu_iter] <= `ZERO_ROB;
                            vk_entry[alu_iter] <= in_alu_broadcast_result;
                        end
                    end
                end
            end
            //check ready, push to alu, pop
            if (ready_index != `ZERO_RS) begin
                out_alu_enable          <= `TRUE;
                busy_entry[ready_index] <= `FALSE;
                out_alu_type            <= type_entry[ready_index];
                out_alu_pc              <= pc_entry[ready_index];
                out_alu_imm             <= imm_entry[ready_index];
                out_alu_left_oprand     <= vj_entry[ready_index];
                out_alu_right_oprand    <= vk_entry[ready_index];
                out_alu_dest            <= dest_entry[ready_index];
            end 
            else begin
                out_alu_enable <= `FALSE;
            end
        end
    end

    assign assign_index =  !busy_entry[1]  ? 1  : 
                           !busy_entry[2]  ? 2  : 
                           !busy_entry[3]  ? 3  : 
                           !busy_entry[4]  ? 4  : 
                           !busy_entry[5]  ? 5  : 
                           !busy_entry[6]  ? 6  : 
                           !busy_entry[7]  ? 7  : 
                           !busy_entry[8]  ? 8  : 
                           !busy_entry[9]  ? 9  : 
                           !busy_entry[10] ? 10 : 
                           !busy_entry[11] ? 11 : 
                           !busy_entry[12] ? 12 : 
                           !busy_entry[13] ? 13 : 
                           !busy_entry[14] ? 14 : 
                           !busy_entry[15] ? 15 :
                           `ZERO_RS; 

    assign ready_index =    ready_entry[1]  ? 1  : 
                            ready_entry[2]  ? 2  : 
                            ready_entry[3]  ? 3  : 
                            ready_entry[4]  ? 4  : 
                            ready_entry[5]  ? 5  : 
                            ready_entry[6]  ? 6  : 
                            ready_entry[7]  ? 7  : 
                            ready_entry[8]  ? 8  : 
                            ready_entry[9]  ? 9  : 
                            ready_entry[10] ? 10 : 
                            ready_entry[11] ? 11 : 
                            ready_entry[12] ? 12 : 
                            ready_entry[13] ? 13 : 
                            ready_entry[14] ? 14 : 
                            ready_entry[15] ? 15 :
                            `ZERO_RS; 
endmodule //rs