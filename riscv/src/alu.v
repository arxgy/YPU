//Just a Calculator. designed for calculate x[rd] value
//pc is handled by adder in reserve station.
`include "def.v"
module alu (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    //connect with control-signal

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
                `NOP: 
                    out_cdb_broadcast_result = `ZERO_DATA;
                `LUI: 
                    out_cdb_broadcast_result = imm;  
                `AUIPC:
                    out_cdb_broadcast_result= pc + imm;
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
                `ADDI: out_cdb_broadcast_result = rs + imm;
                `SLTI:out_cdb_broadcast_result = $signed(rs) < $signed(imm);
                `SLTIU:out_cdb_broadcast_result= rs < imm;
                `XORI:out_cdb_broadcast_result = rs ^ imm;
                `ORI: out_cdb_broadcast_result = rs | imm;
                `ANDI:out_cdb_broadcast_result = rs & imm;
                `SLLI:out_cdb_broadcast_result = rs << imm;
                `SRLI:out_cdb_broadcast_result = rs >> imm;
                `SRAI:out_cdb_broadcast_result = rs >>> imm;

                `ADD: out_cdb_broadcast_result = rs + rt;
                `SUB: out_cdb_broadcast_result = rs - rt;
                `SLL: out_cdb_broadcast_result = rs << rt;
                `SLT: out_cdb_broadcast_result = $signed(rs) < $signed(rt);
                `SLTU:out_cdb_broadcast_result = rs < rt;
                `XOR: out_cdb_broadcast_result = rs ^ rt;
                `SRL: out_cdb_broadcast_result = rs >> rt;
                `SRA: out_cdb_broadcast_result = rs >>> rt;
                `OR:  out_cdb_broadcast_result = rs | rt;
                `AND: out_cdb_broadcast_result = rs & rt;
            endcase
        end
    end
endmodule //alu