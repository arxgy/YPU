//ReOrder Buffer module
`include "def.v"
module rob (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    output reg      out_flush_enable,
    output wire     out_capacity_full,
    //connect with control-signal

    input  wire     in_decoder_assign_enable,
    input  wire     in_decoder_predict,
    input  wire     [`OPERATOR_WIDTH] in_decoder_type,
    input  wire     [`REG_WIDTH]      in_decoder_rd,    
    input  wire     [`ADDRESS_WIDTH]  in_decoder_pc,   
        //assign new rob entry
    input  wire     [`ROB_WIDTH]  in_decoder_rs_reorder,
    input  wire     [`ROB_WIDTH]  in_decoder_rt_reorder,
    output wire                   out_decoder_rs_ready,
    output wire                   out_decoder_rt_ready,
    output wire     [`DATA_WIDTH] out_decoder_rs_value,
    output wire     [`DATA_WIDTH] out_decoder_rt_value,
    output wire     [`ROB_WIDTH]  out_decoder_tail,
        //query entry message (conbinational)
    //connect with Decoder  
    output reg      out_lsb_io_read_commit,
    output wire     out_lsb_store_enable,
    input  wire     in_lsb_store_over,
    input  wire     [`ROB_WIDTH] in_lsb_store_reorder,
    //connect with lsb 

    output reg      out_reg_commit_enable,
    output reg      [`REG_WIDTH]     out_reg_commit_rd,
    output reg      [`DATA_WIDTH]    out_reg_commit_value,
    output reg      [`ROB_WIDTH]     out_reg_commit_reorder,
    //connect with Reg

    output reg      out_pc_feedback_enable,
    output reg      [`ADDRESS_WIDTH] out_pc_commit_pc,
    output reg      [`ADDRESS_WIDTH] out_pc_branch_pc,
    //connect with PC_control

    input  wire     in_alu_broadcast_enable,
    input  wire     [`ROB_WIDTH]     in_alu_broadcast_reorder,
    input  wire     [`DATA_WIDTH]    in_alu_broadcast_result,
    input  wire     [`ADDRESS_WIDTH] in_alu_broadcast_branch,
    //alu broadcast

    input  wire     in_lsb_broadcast_enable,
    input  wire     [`ROB_WIDTH]    in_lsb_broadcast_reorder,
    input  wire     [`DATA_WIDTH]   in_lsb_broadcast_result,
    input  wire     in_lsb_braodcast_io_read
    //lsb broadcast
);
// initial begin
//     fd = $fopen("inst.out", "w");
// end
//     integer fd;
    integer iter;
    reg                   empty;
    reg [`ROB_WIDTH]      head, tail;      //next entry
    reg [`ADDRESS_WIDTH]  pc_entry     [`ROB_ENTRY];
    reg [`OPERATOR_WIDTH] type_entry   [`ROB_ENTRY];
    reg [`REG_WIDTH]      dest_entry   [`ROB_ENTRY];
    reg [`DATA_WIDTH]     value_entry  [`ROB_ENTRY];
    reg [`ADDRESS_WIDTH]  branch_entry [`ROB_ENTRY]; //default: 4, jal: sext(imm)
    reg                   io_read_entry[`ROB_ENTRY];
    reg                   ready_entry  [`ROB_ENTRY];
    reg                   predict_entry[`ROB_ENTRY];
    reg [31:0]  inst_counter;
    wire [`ROB_WIDTH] head_next = head == `TAIL_ROB_ENTRY ? `HEAD_ROB_ENTRY : head + 4'd1;
    assign out_decoder_rs_ready = ready_entry[in_decoder_rs_reorder];
    assign out_decoder_rt_ready = ready_entry[in_decoder_rt_reorder];
    assign out_decoder_rs_value = value_entry[in_decoder_rs_reorder];
    assign out_decoder_rt_value = value_entry[in_decoder_rt_reorder];
    assign out_decoder_tail     = tail;
    assign out_lsb_store_enable = is_store(type_entry[head]);
    assign out_capacity_full = (head == tail + 4'd1 || (head == `HEAD_ROB_ENTRY && tail == `TAIL_ROB_ENTRY) || (tail == head) && !empty);
    initial begin
        inst_counter <= 0;
    end
    wire [`OPERATOR_WIDTH] dbg_head_tp = type_entry[head];
    wire [`ADDRESS_WIDTH]  dbg_head_pc = pc_entry  [head];
    wire [`DATA_WIDTH]     dbg_head_value = value_entry[head];
    wire                   dbg_head_predict = predict_entry[head];
    wire dbg_commit_reg_a0 = out_reg_commit_rd == 5'd10;
    wire dbg_commit_reg_a1 = out_reg_commit_rd == 5'd11;
    wire dbg_commit_reg_a3 = out_reg_commit_rd == 5'd13;
    wire dbg_commit_reg_a5 = out_reg_commit_rd == 5'd15;
    wire dbg_commit_reg_a6 = out_reg_commit_rd == 5'd16;
    wire dbg_commit_reg_s0 = out_reg_commit_rd == 5'd8;
    wire dbg_commit_reg_s1 = out_reg_commit_rd == 5'd9;
    wire dbg_commit_reg_s4 = out_reg_commit_rd == 5'd20;
    wire dbg_commit_reg_s6 = out_reg_commit_rd == 5'd22;
    wire dbg_commit_reg_s11 = out_reg_commit_rd == 5'd27;
    
    
    always @(posedge in_clk) begin
        if (in_rst) begin
            head  <= `HEAD_ROB_ENTRY;
            tail  <= `HEAD_ROB_ENTRY;
            empty <= `TRUE;
            out_flush_enable       <= `FALSE;
            out_pc_feedback_enable <= `FALSE;
            out_reg_commit_enable  <= `FALSE;
            out_lsb_io_read_commit <= `FALSE;
            for (iter = 1 ; iter < `ROB_SIZE ; iter = iter+1 ) begin
                ready_entry[iter]  <= `FALSE;
                pc_entry[iter] <= `ZERO_ADDR;
                predict_entry[iter] <= `NOT_TAKEN;
            end
        end
        else if(in_rdy) begin            
            out_flush_enable       <= `FALSE;
            out_pc_feedback_enable <= `FALSE;
            out_reg_commit_enable  <= `FALSE;
            out_lsb_io_read_commit <= `FALSE;

            //assign entry
            if (in_decoder_assign_enable) begin
                if (tail == `TAIL_ROB_ENTRY) tail <= `HEAD_ROB_ENTRY;
                else tail <= tail + 4'd1;
                empty <= `FALSE;
                ready_entry[tail]   <= `FALSE;
                type_entry [tail]   <= in_decoder_type;
                dest_entry [tail]   <= in_decoder_rd;
                pc_entry[tail]      <= in_decoder_pc;
                predict_entry[tail] <= in_decoder_predict;
            end
            //accept lsb/alu broadcast (no conflict)
            if (in_lsb_broadcast_enable) begin //store: value, load:
                ready_entry  [in_lsb_broadcast_reorder] <= `TRUE;
                value_entry  [in_lsb_broadcast_reorder] <= in_lsb_broadcast_result;
                io_read_entry[in_lsb_broadcast_reorder] <= in_lsb_braodcast_io_read;
            end
            if (in_alu_broadcast_enable) begin
                ready_entry  [in_alu_broadcast_reorder] <= `TRUE;
                value_entry  [in_alu_broadcast_reorder] <=  in_alu_broadcast_result;
                branch_entry [in_alu_broadcast_reorder] <=  in_alu_broadcast_branch;
                io_read_entry[in_alu_broadcast_reorder] <= `FALSE;
            end
            if (in_lsb_store_over) begin
                ready_entry  [in_lsb_store_reorder] <= `TRUE;
                io_read_entry[in_lsb_store_reorder] <= `FALSE;
            end

            if (!empty && ready_entry[head]) begin
                // $fdisplay(fd, "%h",pc_entry[head], " ", inst_counter);    
                inst_counter = inst_counter + 1;
                if (head == `TAIL_ROB_ENTRY) begin
                    head <= `HEAD_ROB_ENTRY;
                    if (tail == `HEAD_ROB_ENTRY) empty <= `TRUE;
                end
                else begin
                    head <= head + 4'd1;
                    if (head + 4'd1 == tail) empty <= `TRUE;
                end                 
                //branch commit
                if (is_branch(type_entry[head])) begin
                    if (type_entry[head] == `JAL) begin         //always predict succeed
                        // $fdisplay(fd, "commit rd: ", dest_entry[head], "; commit value ", value_entry[head]);
                        out_reg_commit_enable  <= `TRUE;                    
                        out_reg_commit_rd      <= dest_entry[head];
                        out_reg_commit_reorder <= head;
                        out_reg_commit_value   <= value_entry[head];
                    end
                    else if (type_entry[head] == `JALR) begin   //always predict fail
                        out_flush_enable       <= `TRUE;
                        out_pc_branch_pc       <= branch_entry[head];
                        // $fdisplay(fd, "commit rd: ", dest_entry[head], "; commit value ", value_entry[head]);
                        out_reg_commit_enable  <= `TRUE;                    
                        out_reg_commit_rd      <= dest_entry[head];
                        out_reg_commit_reorder <= head;
                        out_reg_commit_value   <= value_entry[head];
                    end
                    else begin  
                        out_pc_feedback_enable <= `TRUE;
                        out_pc_commit_pc       <=  pc_entry[head];

                        if (value_entry[head][0] != predict_entry[head]) begin  //not match, pc redirect
                            out_flush_enable <= `TRUE;
                            if (predict_entry[head] == `TAKEN) out_pc_branch_pc <= pc_entry[head] + 4;
                            else out_pc_branch_pc <= branch_entry[head];
                        end
                    end
                end             
                else if (!is_store(type_entry[head]))begin                                //normal commit
                    if (type_entry[head] != `NOP) begin                        
                        // $fdisplay(fd, "commit rd: ", dest_entry[head], "; commit value ", value_entry[head]);
                        out_reg_commit_enable  <= `TRUE;
                        out_reg_commit_rd      <= dest_entry[head];
                        out_reg_commit_reorder <= head;
                        out_reg_commit_value   <= value_entry[head];
                    end

                    if (is_load(type_entry[head]) && io_read_entry[head]) begin
                        out_lsb_io_read_commit <= `TRUE;
                    end
                end
            end
        end
    end
    function reg is_branch(input [`OPERATOR_WIDTH] type) ;
        begin
            is_branch = (type == `JAL || type == `JALR||type == `BEQ || type == `BNE || type == `BLT || type == `BGE || type == `BLTU|| type == `BGEU);
        end
    endfunction
    function reg is_store(input [`OPERATOR_WIDTH] type) ;
        begin
            is_store =  (type == `SB || type == `SH || type == `SW);
        end
    endfunction
    function reg is_load (input [`OPERATOR_WIDTH] type);
        begin
            is_load =  (type == `LB || type == `LH ||  type == `LW || type == `LBU|| type == `LHU);
        end
    endfunction
endmodule //rob