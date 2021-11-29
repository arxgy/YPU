// Record all macro define.
`timescale 1ns/1ps

`define ADDRESS_WIDTH 31:0 
`define INSTRUCTION_WIDTH 31:0

`define OPERATOR_WIDTH 5:0
`define DATA_WIDTH 31:0
`define MEM_BUFFER_DATA_WIDTH 23:0
`define DISPATCH_STATUS_WIDTH 2:0
`define LSB_STATUS_WIDTH 1:0
`define RW_STYLE_WIDTH 1:0
`define CYCLE_COUNTER_WIDTH 2:0 
//define type
`define REG_WIDTH 4:0
`define RS_WIDTH 3:0
`define LSB_WIDTH 3:0
`define ROB_WIDTH 3:0
`define MEM_WIDTH 7:0
`define FETCH_WIDTH 2:0
`define QUEUE_WIDTH 3:0
`define TAG_WIDTH 9:0
`define INDEX_WIDTH 7:0
`define TAKEN_WIDTH 1:0
//define array
`define REG_ENTRY 31:0
`define RS_ENTRY  15:0
`define LSB_ENTRY 15:0
`define ROB_ENTRY 15:0
`define FETCH_ENTRY 7:0
`define QUEUE_ENTRY 15:0
`define CACHE_ENTRY 255:0
`define PREDICT_ENTRY 1023:0

`define INST_TYPE_WIDTH 6:0
`define INST_FUNC3_WIDTH 2:0
`define INST_FUNC7_WIDTH 6:0
`define INST_SHAMT_WIDTH 4:0

`define REG_SIZE 32
`define RS_SIZE  16
`define LSB_SIZE 16
`define ROB_SIZE 16
`define MEM_SIZE 256
`define FETCH_SIZE 8
`define QUEUE_SIZE 16
`define CACHE_SIZE 256
`define PREDICT_SIZE 1024
//default value
`define NOP_INSTR 32'b0 
`define ZERO_DATA 32'b0
`define ZERO_ADDR 32'b0
`define ZERO_REG 5'b0
`define ZERO_RS  4'b0
`define ZERO_LSB 4'b0
`define ZERO_ROB 4'b0

`define FULL_ROB 4'd15
`define HEAD_ROB_ENTRY 4'd1       //head entry
`define TAIL_ROB_ENTRY 4'd15      //tail entry
`define HEAD_LSB_ENTRY 4'd0
`define TAIL_LSB_ENTRY 4'd15
`define HEAD_FETCH_ENTRY 3'd0
`define TAIL_FETCH_ENTRY 3'd7
`define HEAD_QUEUE_ENTRY 4'd0
`define TAIL_QUEUE_ENTRY 4'd15

`define TRUE 1'b1
`define FALSE 1'b0
`define WRITE_SIGNAL 1'b1
`define READ_SIGNAL  1'b0
//define opcode
`define LUI_OP 7'b0110111
`define AUIPC_OP 7'b0010111
`define JAL_OP 7'b1101111
`define JALR_OP 7'b1100111
`define BRANCH_OP 7'b1100011    //b-type
`define LOAD_OP 7'b0000011
`define STORE_OP 7'b0100011     //s-type
`define IA_IMM_OP 7'b0010011
`define IA_OP 7'b0110011        //r-type
//Integer Arithmetic, IA for short

//define funct3
`define BEQ_TAG 3'b000
`define BNE_TAG 3'b001
`define BLT_TAG 3'b100
`define BGE_TAG 3'b101
`define BLTU_TAG 3'b110
`define BGEU_TAG 3'b111
`define LB_TAG 3'b000
`define LH_TAG 3'b001
`define LW_TAG 3'b010
`define LBU_TAG 3'b100
`define LHU_TAG 3'b101
`define SB_TAG 3'b000
`define SH_TAG 3'b001
`define SW_TAG 3'b010
`define ADD_or_SUB_TAG 3'b000
`define SLL_TAG 3'b001
`define SLT_TAG 3'b010
`define SLTU_TAG 3'b011
`define XOR_TAG 3'b100
`define SRL_or_SRA_TAG 3'b101
`define OR_TAG 3'b110
`define AND_TAG 3'b111

`define SRL_TAG 7'b0000000
`define SRA_TAG 7'b0100000 
`define ADD_TAG 7'b0000000
`define SUB_TAG 7'b0100000 
//define operator in ROB
`define LUI 6'd0
`define AUIPC 6'd1
`define JAL 6'd2
`define JALR 6'd3

`define BEQ 6'd4
`define BNE 6'd5    
`define BLT 6'd6
`define BGE 6'd7
`define BLTU 6'd8
`define BGEU 6'd9

`define LB 6'd10
`define LH 6'd11
`define LW 6'd12
`define LBU 6'd13
`define LHU 6'd14

`define SB 6'd15
`define SH 6'd16
`define SW 6'd17

`define ADDI 6'd18
`define SLTI 6'd19
`define SLTIU 6'd20
`define XORI 6'd21
`define ORI 6'd22
`define ANDI 6'd23  
`define SLLI 6'd24
`define SRLI 6'd25
`define SRAI 6'd26

`define ADD 6'd27
`define SUB 6'd28
`define SLL 6'd29
`define SLT 6'd30
`define SLTU 6'd31
`define XOR 6'd32
`define SRL 6'd33
`define SRA 6'd34
`define OR 6'd35
`define AND 6'd36

`define NOP 6'd37

`define IDLE_STATUS 2'b00
`define PC_STATUS 2'b01
`define LOAD_STATUS 2'b10
`define STORE_STATUS 2'b11

`define ZERO_REG 5'd0

//dispatcher read/write-method
`define RW_NONE 2'b00
`define RW_BYTE 2'b01
`define RW_HALF_WORD  2'b10
`define RW_WORD 2'b11 

//predictor
`define STRONG_NOT_TAKEN 2'b00
`define WEAK_NOT_TAKEN 2'b01
`define WEAK_TAKEN 2'b10
`define STRONG_TAKEN 2'b11

`define TAKEN  1'b1
`define NOT_TAKEN 1'b0
//for inst queue sync
`define NULL_PRE 2'b00
`define TAKE_PRE 2'b01
`define NOT_TAKE_PRE 2'b10