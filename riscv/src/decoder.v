//Decode instr, issue instr into LSbuffer/RS and ROB.
//conbinational circuit
`include "def.v"
module decoder (
    input  wire     in_decode_enable,   //fetch_enable & !stall & in_rdy
    //connect with control-signal

    input  wire     [`INSTRUCTION_WIDTH] inst,
    input  wire     [`ADDRESS_WIDTH] pc,
    //connect with fetcher

    output wire     [`OPERATOR_WIDTH] out_type,
    output wire     [`DATA_WIDTH]     out_imm,
    output wire     [`ROB_WIDTH]      out_Qj,   //for store & load
    output wire     [`ROB_WIDTH]      out_Qk,   //for store
    output wire     [`DATA_WIDTH]     out_Vj,   //for store & load
    output wire     [`DATA_WIDTH]     out_Vk,   //for store
    output wire     [`ROB_WIDTH]      out_reorder, 
    output wire     [`ADDRESS_WIDTH]  out_pc,
    output wire     [`REG_WIDTH]      out_rd,
    output wire     [`REG_WIDTH]      out_rs,
    output wire     [`REG_WIDTH]      out_rt,

    output wire     out_lsb_assign_enable,
    output wire     out_rs_assign_enable,
    output wire     out_rob_assign_enable,
    
    output wire     [`ROB_WIDTH] out_rob_rs_reorder, 
    output wire     [`ROB_WIDTH] out_rob_rt_reorder,
    input  wire     in_rob_rs_ready,
    input  wire     in_rob_rt_ready,
    input  wire     [`DATA_WIDTH] in_rob_rs_value,
    input  wire     [`DATA_WIDTH] in_rob_rt_value,
    input  wire     [`ROB_WIDTH]  in_rob_tail,     
    //connect with ROB

    output wire     out_reg_write_enable,
    input  wire     in_reg_rs_busy,
    input  wire     in_reg_rt_busy,
    input  wire     [`DATA_WIDTH] in_reg_rs_data,
    input  wire     [`DATA_WIDTH] in_reg_rt_data,
    input  wire     [`ROB_WIDTH]  in_reg_rs_reorder,
    input  wire     [`ROB_WIDTH]  in_reg_rt_reorder
    //connect with Reg
);  
    assign out_type = get_inst_type(inst);
    assign out_imm  = get_inst_imm(inst);
    //rs/qj: except u-type & j-type
    //rt/qk: r-type s-type b-type
    assign out_Qj   = (inst[6:0] == `LUI_OP || inst[6:0] == `AUIPC_OP || inst[6:0] == `JAL_OP )  ? `ZERO_ROB : 
                      (in_reg_rs_busy ? (in_rob_rs_ready ? `ZERO_ROB : in_reg_rs_reorder ) : `ZERO_ROB);
    assign out_Qk   = (inst[6:0] == `IA_OP || inst[6:0] == `STORE_OP || inst[6:0] == `BRANCH_OP) ? 
                      (in_reg_rt_busy ? (in_rob_rt_ready ? `ZERO_ROB : in_reg_rt_reorder ) : `ZERO_ROB) : `ZERO_ROB;
                      
    assign out_Vj   = (inst[6:0] == `LUI_OP || inst[6:0] == `AUIPC_OP || inst[6:0] == `JAL_OP )  ? `ZERO_DATA : 
                      (in_reg_rs_busy ? (in_rob_rs_ready ? in_rob_rs_value : `ZERO_DATA  ) : in_reg_rs_data );
    assign out_Vk   = (inst[6:0] == `IA_OP || inst[6:0] == `STORE_OP || inst[6:0] == `BRANCH_OP) ? 
                      (in_reg_rt_busy ? (in_rob_rt_ready ? in_rob_rt_value : `ZERO_DATA  ) : in_reg_rt_data ) : `ZERO_DATA;
    assign out_reorder = in_rob_tail;
    assign out_pc   = pc;
    assign out_rd   = inst[11:7];
    assign out_rs   = inst[19:15];
    assign out_rt   = inst[24:20];
    assign out_rs_assign_enable  = in_decode_enable ? ((inst[6:0] == `LOAD_OP || inst[6:0] == `STORE_OP)?  `FALSE: `TRUE) : `FALSE;
    assign out_lsb_assign_enable = in_decode_enable ? ((inst[6:0] == `LOAD_OP || inst[6:0] == `STORE_OP)?  `TRUE : `FALSE) : `FALSE;

    assign out_rob_assign_enable = in_decode_enable ? `TRUE : `FALSE;
    assign out_rob_rs_reorder    = in_reg_rs_busy ? in_reg_rs_reorder : `ZERO_ROB;
    assign out_rob_rt_reorder    = in_reg_rt_busy ? in_reg_rt_reorder : `ZERO_ROB;
    
    assign out_reg_write_enable  = in_decode_enable ? ((inst[6:0] == `STORE_OP || inst[6:0] == `BRANCH_OP)? `FALSE : `TRUE ) : `FALSE;
    
    function [`OPERATOR_WIDTH] get_inst_type(input [`INSTRUCTION_WIDTH] inst);
        begin
            case (inst[6:0])
                `LUI_OP:    get_inst_type = `LUI;
                `AUIPC_OP:  get_inst_type = `AUIPC;
                `JAL_OP:    get_inst_type = `JAL;
                `JALR_OP:   get_inst_type = `JALR;
                `BRANCH_OP: 
                    case (inst[14:12])
                        `BEQ_TAG:   get_inst_type = `BEQ;
                        `BNE_TAG:   get_inst_type = `BNE;
                        `BLT_TAG:   get_inst_type = `BLT;
                        `BGE_TAG:   get_inst_type = `BGE;
                        `BLTU_TAG:  get_inst_type = `BLTU;
                        `BGEU_TAG:  get_inst_type = `BGEU;
                    endcase
                `LOAD_OP:
                    case (inst[14:12])
                        `LB_TAG:    get_inst_type = `LB;
                        `LH_TAG:    get_inst_type = `LH;
                        `LW_TAG:    get_inst_type = `LW;
                        `LBU_TAG:   get_inst_type = `LBU;
                        `LHU_TAG:   get_inst_type = `LHU;
                    endcase
                `STORE_OP:
                    case (inst[14:12])
                        `SB_TAG:    get_inst_type = `SB;
                        `SH_TAG:    get_inst_type = `SH;
                        `SW_TAG:    get_inst_type = `SW;
                    endcase
                `IA_IMM_OP: begin
                    case (inst[14:12])
                        `ADD_or_SUB_TAG :get_inst_type = `ADDI;
                        `SLT_TAG        :get_inst_type = `SLTI;
                        `SLTU_TAG       :get_inst_type = `SLTIU;
                        `XOR_TAG        :get_inst_type = `XORI;
                        `OR_TAG         :get_inst_type = `ORI;
                        `AND_TAG        :get_inst_type = `ANDI;
                        `SLL_TAG        :get_inst_type = `SLLI;
                        `SRL_or_SRA_TAG :
                            case (inst[31:25])
                                `SRL_TAG:get_inst_type = `SRLI;
                                `SRA_TAG:get_inst_type = `SRAI;
                            endcase
                    endcase
                end
                `IA_OP:
                    case (inst[14:12])
                        `ADD_or_SUB_TAG :
                            case (inst[31:25])
                                `ADD_TAG:get_inst_type = `ADD;
                                `SUB_TAG:get_inst_type = `SUB;
                            endcase
                        `SLL_TAG        :get_inst_type = `SLL;
                        `SLT_TAG        :get_inst_type = `SLT;
                        `SLTU_TAG       :get_inst_type = `SLTU;
                        `XOR_TAG        :get_inst_type = `XOR;
                        `SRL_or_SRA_TAG :   
                            case (inst[31:25])
                                `SRL_TAG:get_inst_type = `SRL;
                                `SRA_TAG:get_inst_type = `SRA;
                            endcase
                        `OR_TAG         :get_inst_type = `OR;
                        `AND_TAG        :get_inst_type = `AND;
                    endcase
            endcase
        end
    endfunction
    function [`DATA_WIDTH] get_inst_imm(input [`INSTRUCTION_WIDTH] instr);
        begin
            case (instr[6:0])
                `LUI_OP:    get_inst_imm = {instr[31:12], {12{1'b0}}};
                `AUIPC_OP:  get_inst_imm = {instr[31:12], {12{1'b0}}};
                `JAL_OP:    get_inst_imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
                `JALR_OP:   get_inst_imm = {{21{instr[31]}}, instr[30:20]};
                `BRANCH_OP: get_inst_imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
                `LOAD_OP:   get_inst_imm = {{21{instr[31]}}, instr[30:20]};
                `STORE_OP:  get_inst_imm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
                `IA_IMM_OP: 
                        if (instr[14:12] == `SRL_or_SRA_TAG || instr[14:12] == `SLL_TAG) 
                            get_inst_imm = {{27{1'b0}} , instr[24:20]};  //shamt
                        else 
                            get_inst_imm = {{21{instr[31]}}, instr[30:20]};
            endcase
        end
    endfunction
endmodule //decoder