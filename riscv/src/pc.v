//PC Address Controller for MIPS-32.
//with a quite toy-like saturating predictor of 1024 size 
`include "def.v"
module pc (
    input  wire     in_clk,
    input  wire     in_rst, 
    input  wire     in_rdy,
    input  wire     in_stall,    
    //connect with control-signal
    
    input  wire     in_flush_enable,        //when commit and predict failed
    input  wire     in_feedback_enable,     //when commit and predict succeeded
    input  wire     [`ADDRESS_WIDTH] in_rob_commit_pc,
    input  wire     [`ADDRESS_WIDTH] in_rob_branch_pc,
    //connect with ROB

    input  wire                          in_fetcher_last_enable,
    input  wire     [`INSTRUCTION_WIDTH] in_fetcher_last_inst,
    output reg                           out_fetcher_enable,
    output reg      [`ADDRESS_WIDTH]     out_fetcher_pc,
    output reg                           out_fetcher_predict
    //connect with fetcher
);
    integer iter;
    reg [`TAKEN_WIDTH]       predict_table [`PREDICT_ENTRY];    //1024, [9:0]
    reg [`ADDRESS_WIDTH]     last_pc_add_4;
    reg                      last_enable;
    reg [`INSTRUCTION_WIDTH] last_inst;
    
    wire [`DATA_WIDTH] jal_imm    = {{12{last_inst[31]}}, last_inst[19:12], last_inst[20], last_inst[30:21], 1'b0};
    wire [`DATA_WIDTH] branch_imm = {{20{last_inst[31]}}, last_inst[7], last_inst[30:25], last_inst[11:8], 1'b0};
    
    always @(posedge in_clk) begin
        if (in_rst) begin
            last_enable         <= `FALSE;
            last_pc_add_4       <= `ZERO_ADDR;
            out_fetcher_enable  <= `FALSE;
            for (iter = 0; iter < `PREDICT_SIZE; iter = iter+1 ) begin
                predict_table[iter] <= `STRONG_NOT_TAKEN;
            end
        end 
        else if(in_rdy) begin   //1. update table; 2.push pc to fetch
            if (in_flush_enable) begin              
                case (predict_table[in_rob_commit_pc[9:0]]) 
                    `STRONG_NOT_TAKEN: predict_table[in_rob_commit_pc[9:0]] <= `WEAK_NOT_TAKEN;
                    `WEAK_NOT_TAKEN:   predict_table[in_rob_commit_pc[9:0]] <= `STRONG_TAKEN;
                    `WEAK_TAKEN:       predict_table[in_rob_commit_pc[9:0]] <= `STRONG_NOT_TAKEN;
                    `STRONG_TAKEN:     predict_table[in_rob_commit_pc[9:0]] <= `WEAK_TAKEN;
                endcase
                out_fetcher_enable  <= `TRUE;
                out_fetcher_pc      <=  in_rob_branch_pc;
                last_pc_add_4       <=  in_rob_branch_pc + 4;
                last_enable         <= `FALSE;
            end
            else begin
                if (in_feedback_enable) begin
                    case (predict_table[in_rob_commit_pc[9:0]]) 
                        `WEAK_NOT_TAKEN:   predict_table[in_rob_commit_pc[9:0]] <= `STRONG_NOT_TAKEN;
                        `WEAK_TAKEN:       predict_table[in_rob_commit_pc[9:0]] <= `STRONG_TAKEN;
                    endcase
                end
                if (in_fetcher_last_enable) begin
                    last_enable         <= `TRUE;
                    last_inst           <=  in_fetcher_last_inst;
                end

                if (in_stall) begin
                    out_fetcher_enable  <= `FALSE;
                end
                else if (last_pc_add_4 == `ZERO_ADDR) begin   //at beginning
                    out_fetcher_enable  <= `TRUE;
                    out_fetcher_pc      <=  last_pc_add_4;
                    last_pc_add_4       <=  last_pc_add_4 + 4;
                    out_fetcher_predict <= `NOT_TAKEN;
                end
                else if (last_enable) begin             //get last pc's inst, push next inst to fetcher
                    out_fetcher_enable  <= `TRUE;
                    last_enable         <= `FALSE;

                    if (last_inst[6:0] == `BRANCH_OP) begin
                        if (predict_table[out_fetcher_pc[9:0]] == `STRONG_NOT_TAKEN || predict_table[out_fetcher_pc[9:0] == `WEAK_NOT_TAKEN]) begin
                            out_fetcher_pc      <=  last_pc_add_4;
                            last_pc_add_4       <=  last_pc_add_4 + 4;
                            out_fetcher_predict <= `NOT_TAKEN;
                        end
                        else begin
                            out_fetcher_pc      <=  out_fetcher_pc + branch_imm;
                            last_pc_add_4       <=  out_fetcher_pc + branch_imm + 4;
                            out_fetcher_predict <= `TAKEN;
                        end
                    end
                    else if (last_inst[6:0] == `JAL_OP) begin
                        out_fetcher_pc      <=  out_fetcher_pc + jal_imm;
                        last_pc_add_4       <=  out_fetcher_pc + jal_imm + 4;
                        out_fetcher_predict <= `TAKEN;
                    end
                    else begin  //not branch (including jalr)
                        out_fetcher_pc      <=  last_pc_add_4;
                        last_pc_add_4       <=  last_pc_add_4 + 4;
                        out_fetcher_predict <= `NOT_TAKEN;
                    end
                end
                else begin
                    out_fetcher_enable <= `FALSE;
                end
            end
        end
    end
endmodule //pc