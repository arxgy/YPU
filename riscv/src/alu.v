//Just a Calculator. designed for calculate x[rd] value
//pc is handled by adder in reserve station.
`include "def.v"
module alu (
    input  wire                       in_enable,
    input  wire     [`OPERATOR_WIDTH] type,
    input  wire     [`ADDRESS_WIDTH]  pc,
    input  wire     [`DATA_WIDTH]     imm,
    input  wire     [`DATA_WIDTH]     rs,   
    input  wire     [`DATA_WIDTH]     rt,  
    input  wire     [`ROB_WIDTH]      reorder,
    //connect with ReserveStation (arithmetic step)

    output wire                      out_cdb_broadcast_enable,              //to rs, lsb,
    output wire     [`ROB_WIDTH]     out_cdb_broadcast_reorder,
    output reg      [`DATA_WIDTH]    out_cdb_broadcast_result,
    output reg      [`ADDRESS_WIDTH] out_cdb_broadcast_branch                //only to rob
    //cdb: connect with lsb, rs, rob
);
    assign out_cdb_broadcast_enable  = in_enable;
    assign out_cdb_broadcast_reorder = reorder;
    // assign out_cdb_broadcast_result = get_alu_result(type);
    
    always @(*) begin
        if (in_enable) begin
            case (type)
                `NOP: begin
                    out_cdb_broadcast_result = `ZERO_DATA;
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `LUI: begin
                    out_cdb_broadcast_result = imm;  
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `AUIPC: begin
                    out_cdb_broadcast_result= pc + imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `JAL: begin
                    out_cdb_broadcast_result = pc + 4;
                    out_cdb_broadcast_branch = pc + imm;
                end
                `JALR:begin
                    out_cdb_broadcast_result = pc + 4;
                    out_cdb_broadcast_branch = (rs + imm)&(~1);
                end
                `BEQ: begin
                    out_cdb_broadcast_result = rs == rt;
                    out_cdb_broadcast_branch = pc + imm;
                end
                `BNE: begin
                    out_cdb_broadcast_result = rs != rt;
                    out_cdb_broadcast_branch = pc + imm;
                end
                `BLT: begin
                    out_cdb_broadcast_result = $signed(rs) <  $signed(rt);
                    out_cdb_broadcast_branch = pc + imm;
                end
                `BGE: begin
                    out_cdb_broadcast_result = $signed(rs) >= $signed(rt);
                    out_cdb_broadcast_branch = pc + imm;
                end
                `BLTU:begin
                    out_cdb_broadcast_result = rs <  rt;
                    out_cdb_broadcast_branch = pc + imm;
                end
                `BGEU:begin
                    out_cdb_broadcast_result = rs >= rt;
                    out_cdb_broadcast_branch = pc + imm;
                end
                `ADDI: begin
                    out_cdb_broadcast_result = rs + imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end 
                `SLTI: begin 
                    out_cdb_broadcast_result = $signed(rs) < $signed(imm);
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `SLTIU: begin
                    out_cdb_broadcast_result= rs < imm;                    
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `XORI: begin
                    out_cdb_broadcast_result = rs ^ imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;
                end
                `ORI: begin
                    out_cdb_broadcast_result = rs | imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;                    
                end
                `ANDI:begin
                    out_cdb_broadcast_result = rs & imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;                    
                end
                `SLLI: begin 
                    out_cdb_broadcast_result = rs << imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;                  
                end
                `SRLI: begin
                    out_cdb_broadcast_result = rs >> imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SRAI: begin
                    out_cdb_broadcast_result = rs >>> imm;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `ADD: begin
                    out_cdb_broadcast_result = rs + rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SUB: begin
                    out_cdb_broadcast_result = rs - rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SLL: begin
                    out_cdb_broadcast_result = rs << rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SLT: begin
                    out_cdb_broadcast_result = $signed(rs) < $signed(rt);
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SLTU: begin
                    out_cdb_broadcast_result = rs < rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `XOR: begin
                    out_cdb_broadcast_result = rs ^ rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SRL: begin
                    out_cdb_broadcast_result = rs >> rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `SRA: begin
                    out_cdb_broadcast_result = rs >>> rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `OR:  begin
                    out_cdb_broadcast_result = rs | rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                `AND: begin
                    out_cdb_broadcast_result = rs & rt;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                default: begin
                    out_cdb_broadcast_result = `ZERO_DATA;
                    out_cdb_broadcast_branch = `ZERO_ADDR;     
                end
                    
            endcase
        end
    end
endmodule //alu