//A load/Store Queue for execute stage with inner Address Adder.
`include "def.v"
module lsb (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    input  wire     in_flush_enable,
    output wire     out_capacity_full,
    //collect with control-signal

    input  wire     in_decoder_assign_enable,
    input  wire     [`OPERATOR_WIDTH] in_decoder_type,
    input  wire     [`DATA_WIDTH] in_decoder_imm,
    input  wire     [`ROB_WIDTH]  in_decoder_Qj,
    input  wire     [`ROB_WIDTH]  in_decoder_Qk,
    input  wire     [`DATA_WIDTH] in_decoder_Vj,
    input  wire     [`DATA_WIDTH] in_decoder_Vk,
    input  wire     [`ROB_WIDTH]  in_decoder_dest,
    input  wire     [`ADDRESS_WIDTH] in_decoder_pc,
    //connect with Decoder

    input  wire     in_rob_store_enable,    //pop 
    input  wire     in_rob_io_read_commit,
    output reg      out_rob_store_over,
    //connect with ROB

    output reg      out_dispatch_load_requesting,
    output reg      [`ADDRESS_WIDTH] out_dispatch_load_addr,
    output reg      [`RW_STYLE_WIDTH] out_dispatch_load_style,
    input  wire     [`DATA_WIDTH] in_dispatch_load_value,
    input  wire     in_load_req_enable,
    input  wire     in_load_data_enable,

    output reg      out_dispatch_store_requesting,
    output reg      [`ADDRESS_WIDTH] out_dispatch_store_addr,
    output reg      [`RW_STYLE_WIDTH] out_dispatch_store_style,
    output reg      [`DATA_WIDTH] out_dispatch_store_value, 
    input  wire     in_store_req_enable,
    //connect with dispatcher 

    input  wire     in_alu_broadcast_enable,
    input  wire     [`ROB_WIDTH]  in_alu_broadcast_reorder,
    input  wire     [`DATA_WIDTH] in_alu_broadcast_result,

    output reg      out_cdb_broadcast_enable,
    output reg      [`ROB_WIDTH]  out_cdb_reorder,
    output reg      [`DATA_WIDTH] out_cdb_result,
    output reg      out_cdb_io_read 
    //cdb
);
    integer iter, lsb_iter, alu_iter;
    reg                    queue_empty, io_misbranched;
    reg  [`QUEUE_WIDTH]    queue_head, queue_tail;
    reg  [`DATA_WIDTH]     io_queue [`QUEUE_ENTRY];
    reg                    entry_empty;
    reg  [`LSB_WIDTH]      entry_head, entry_tail;
    reg  [`OPERATOR_WIDTH] type_entry  [`LSB_ENTRY];
    reg  [`DATA_WIDTH]     imm_entry   [`LSB_ENTRY];
    reg  [`ROB_WIDTH]      qj_entry    [`LSB_ENTRY];
    reg  [`ROB_WIDTH]      qk_entry    [`LSB_ENTRY];
    reg  [`DATA_WIDTH]     vj_entry    [`LSB_ENTRY];
    reg  [`DATA_WIDTH]     vk_entry    [`LSB_ENTRY];
    reg  [`ROB_WIDTH]      dest_entry  [`LSB_ENTRY];
    reg                    busy_entry  [`LSB_ENTRY];
    reg                    store_wait ;
    reg  [`ADDRESS_WIDTH]  pc_entry    [`LSB_ENTRY];

    wire cdb_lsb_broadcast_enable  = out_cdb_broadcast_enable;
    wire [`ROB_WIDTH]  cdb_lsb_broadcast_reorder = out_cdb_reorder; 
    wire [`DATA_WIDTH] cdb_lsb_broadcast_result  = out_cdb_result;
    //as lsb broadcast (last cycle)
    reg  [`OPERATOR_WIDTH] last_load_type;
    reg  [`ROB_WIDTH]      last_load_dest;
    reg                    last_load_io;

    wire [`DATA_WIDTH]     head_entry_addr;
    assign out_capacity_full = (entry_head == entry_tail + 4'd1 || ((entry_tail == entry_head) && !entry_empty));
    assign head_entry_addr   = vj_entry[entry_head]+imm_entry[entry_head];

    wire [`OPERATOR_WIDTH] dbg_head_type = type_entry[entry_head];
    wire [`DATA_WIDTH] dbg_head_imm = imm_entry[entry_head];
    wire [`ROB_WIDTH] dbg_head_qj = qj_entry[entry_head];
    wire [`ROB_WIDTH] dbg_head_qk = qk_entry[entry_head];
    wire [`DATA_WIDTH] dbg_head_vj = vj_entry[entry_head];
    wire [`DATA_WIDTH] dbg_head_vk = vk_entry[entry_head];
    wire [`ADDRESS_WIDTH] dbg_head_pc = pc_entry[entry_head];
    wire [`ROB_WIDTH]   dbg_head_dest = dest_entry[entry_head];
    always @(posedge in_clk) begin          //clear all
        if (in_rst) begin
            io_misbranched     <= `FALSE;
            store_wait         <= `FALSE;
            out_rob_store_over <= `FALSE;
            queue_head <= 4'd0;     queue_tail <= 4'd0;     queue_empty <= `TRUE;
            entry_head <= 4'd0;     entry_tail <= 4'd0;     entry_empty <= `TRUE;
            last_load_dest     <= `ZERO_ROB;    //no load
            last_load_io       <= `FALSE;
            out_rob_store_over <= `FALSE;
            out_dispatch_load_requesting  <= `FALSE;
            out_dispatch_store_requesting <= `FALSE;
            out_cdb_broadcast_enable      <= `FALSE;
            for (iter = 0 ; iter < `LSB_SIZE ; iter = iter+1 ) begin
                busy_entry[iter]  <= `FALSE;
            end
        end 
        else if (in_rdy) begin
            if (in_flush_enable) begin      //clear, but maintain queue, maintain dispatch
                if (queue_empty) io_misbranched <= `FALSE;
                else             io_misbranched <= `TRUE;
                store_wait <= `FALSE;
                entry_head <= 4'd0;     entry_tail <= 4'd0;     entry_empty <= `TRUE;
                last_load_dest <= `ZERO_ROB;
                out_rob_store_over <= `FALSE;
                out_dispatch_load_requesting  <= `FALSE;
                out_dispatch_store_requesting <= `FALSE;
                out_cdb_broadcast_enable      <= `FALSE;
                for (iter = 0 ; iter < `LSB_SIZE ; iter = iter+1 ) begin
                    busy_entry[iter]  <= `FALSE;
                end

                if (in_load_data_enable && last_load_io) begin
                    queue_empty <= `FALSE;
                    queue_tail  <= queue_tail + 4'd1;
                    case (last_load_type)
                        `LB: io_queue[queue_tail] <= {{24{in_dispatch_load_value[7]}}, in_dispatch_load_value[7:0] };
                        `LBU:io_queue[queue_tail] <= {{24{1'b0}}, in_dispatch_load_value[7:0] };
                    endcase 
                end
            end
            else begin
                //queue pop
                if (in_rob_io_read_commit) begin
                    queue_head <= queue_head + 4'd1;
                    if (queue_tail == queue_head + 4'd1) begin
                        queue_empty    <= `TRUE;
                        io_misbranched <= `FALSE;
                    end
                end
                //assign entry
                if (in_decoder_assign_enable) begin
                    entry_tail  <= entry_tail + 4'd1;
                    entry_empty <= `FALSE;
                    type_entry[entry_tail] <= in_decoder_type;
                    imm_entry[entry_tail]  <= in_decoder_imm;
                    qj_entry[entry_tail]   <= in_decoder_Qj;
                    qk_entry[entry_tail]   <= in_decoder_Qk;
                    vj_entry[entry_tail]   <= in_decoder_Vj;
                    vk_entry[entry_tail]   <= in_decoder_Vk;
                    dest_entry[entry_tail] <= in_decoder_dest;
                    busy_entry[entry_tail] <= `TRUE;
                    pc_entry[entry_tail]   <= in_decoder_pc;
                end

                //accept lsb/alu broadcast 
                if (cdb_lsb_broadcast_enable) begin
                    for (lsb_iter = 0 ; lsb_iter < `LSB_SIZE ; lsb_iter = lsb_iter+1) begin
                        if (in_decoder_assign_enable && lsb_iter == entry_tail) begin      //special judge
                            if (in_decoder_Qj == cdb_lsb_broadcast_reorder) begin 
                                qj_entry[lsb_iter] <= `ZERO_ROB;
                                vj_entry[lsb_iter] <= cdb_lsb_broadcast_result;
                            end
                            if (in_decoder_Qk == cdb_lsb_broadcast_reorder) begin
                                qk_entry[lsb_iter] <= `ZERO_ROB;
                                vk_entry[lsb_iter] <= cdb_lsb_broadcast_result;
                            end
                        end
                        else if (busy_entry[lsb_iter]) begin    //normal broadcast
                            if (qj_entry[lsb_iter] == cdb_lsb_broadcast_reorder) begin
                                qj_entry[lsb_iter] <= `ZERO_ROB;
                                vj_entry[lsb_iter] <= cdb_lsb_broadcast_result;
                            end
                            if (qk_entry[lsb_iter] == cdb_lsb_broadcast_reorder) begin
                                qk_entry[lsb_iter] <= `ZERO_ROB;
                                vk_entry[lsb_iter] <= cdb_lsb_broadcast_result;
                            end
                        end
                    end
                end
                if (in_alu_broadcast_enable)  begin
                    for (alu_iter = 0 ; alu_iter < `RS_SIZE ; alu_iter = alu_iter+1) begin
                        if (in_decoder_assign_enable && alu_iter == entry_tail) begin
                            if (in_decoder_Qj == in_alu_broadcast_reorder) begin
                                qj_entry[alu_iter] <= `ZERO_ROB;
                                vj_entry[alu_iter] <= in_alu_broadcast_result;
                            end
                            if (in_decoder_Qk == in_alu_broadcast_reorder) begin
                                qk_entry[alu_iter] <= `ZERO_ROB;
                                vk_entry[alu_iter] <= in_alu_broadcast_result;
                            end
                        end
                        else if (busy_entry[alu_iter]) begin
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

                if (in_load_data_enable) begin    
                    out_cdb_broadcast_enable <= `TRUE;
                    out_cdb_reorder <= last_load_dest;
                    out_cdb_io_read <= last_load_io;
                    last_load_dest <= `ZERO_ROB;
                    case (last_load_type)
                        `LB: out_cdb_result <= {{24{in_dispatch_load_value[7]}}, in_dispatch_load_value[7:0] };
                        `LH: out_cdb_result <= {{16{in_dispatch_load_value[15]}},in_dispatch_load_value[15:0]};
                        `LW: out_cdb_result <= in_dispatch_load_value;
                        `LBU:out_cdb_result <= {{24{1'b0}}, in_dispatch_load_value[7:0] };
                        `LHU:out_cdb_result <= {{16{1'b0}}, in_dispatch_load_value[15:0]};
                    endcase

                    if (last_load_io) begin     //io_load: add to io_queue
                        queue_empty <= `FALSE;
                        queue_tail  <= queue_tail + 4'd1;
                        case (last_load_type)
                            `LB: io_queue[queue_tail] <= {{24{in_dispatch_load_value[7]}}, in_dispatch_load_value[7:0] };
                            `LBU:io_queue[queue_tail] <= {{24{1'b0}}, in_dispatch_load_value[7:0] };
                        endcase 
                    end
                end
                else begin
                    out_cdb_broadcast_enable <= `FALSE;
                end

                out_rob_store_over            <= `FALSE;
                out_dispatch_load_requesting  <= `FALSE;
                out_dispatch_store_requesting <= `FALSE;
                if (!entry_empty) begin
                    if (is_store(type_entry[entry_head]) && qj_entry[entry_head] == `ZERO_ROB && qk_entry[entry_head] == `ZERO_ROB && in_rob_store_enable && in_store_req_enable) begin
                        if (!store_wait) begin      //dispatch is empty, push to dispatch
                            out_rob_store_over <= `FALSE;
                            store_wait <= `TRUE;
                            out_dispatch_store_requesting <= `TRUE;
                            out_dispatch_store_addr  <= head_entry_addr;
                            out_dispatch_store_value <= vk_entry[entry_head];
                            case (type_entry[entry_head])
                                `SB: out_dispatch_store_style <= `RW_BYTE;
                                `SH: out_dispatch_store_style <= `RW_HALF_WORD;
                                `SW: out_dispatch_store_style <= `RW_WORD;
                            endcase    
                        end
                        else begin                 //this store is over, inform rob to commit
                            out_rob_store_over <= `TRUE;
                            store_wait <= `FALSE;
                            busy_entry[entry_head] <= `FALSE;
                            entry_head <= entry_head + 4'd1;
                            if (entry_tail == entry_head + 4'd1 && !in_decoder_assign_enable) entry_empty <= `TRUE;
                        end
                    end
                    else if (is_load(type_entry[entry_head]) && qj_entry[entry_head] == `ZERO_ROB) begin    
                        if (head_entry_addr[17:16] == 2'b11 && io_misbranched) begin       //read from queue
                            if (!in_load_data_enable) begin          //to avoid data broadcast collision
                                out_cdb_broadcast_enable  <= `TRUE;
                                out_cdb_reorder           <=  dest_entry[entry_head];
                                out_cdb_result            <=  io_queue[entry_head];
                                out_cdb_io_read           <= `TRUE;

                                busy_entry[entry_head]    <= `FALSE;
                                entry_head <= entry_head + 4'd1;
                                if (entry_tail == entry_head + 4'd1 && !in_decoder_assign_enable) entry_empty <= `TRUE;
                                queue_head <= queue_head + 4'd1;
                                if (queue_tail == queue_head + 4'd1) begin
                                    queue_empty <= `TRUE;
                                    io_misbranched <= `FALSE;
                                end
                            end
                            else begin
                                out_cdb_broadcast_enable <= `FALSE;
                            end
                        end
                        else if (in_load_req_enable && last_load_dest == `ZERO_ROB) begin      
                            //last load execute over! another delay, so sad
                            out_dispatch_load_requesting <= `TRUE;
                            out_dispatch_load_addr <= head_entry_addr;
                            case (type_entry[entry_head])
                                `LB: out_dispatch_load_style <= `RW_BYTE;
                                `LH: out_dispatch_load_style <= `RW_HALF_WORD;
                                `LW: out_dispatch_load_style <= `RW_WORD;
                                `LBU:out_dispatch_load_style <= `RW_BYTE;
                                `LHU:out_dispatch_load_style <= `RW_HALF_WORD;
                            endcase
                            last_load_io   <= head_entry_addr[17:16] == 2'b11;
                            last_load_dest <= dest_entry[entry_head];
                            last_load_type <= type_entry[entry_head];
                            busy_entry[entry_head] <= `FALSE;

                            entry_head     <= entry_head + 4'd1;
                            if (entry_tail == entry_head + 4'd1 && !in_decoder_assign_enable) entry_empty <= `TRUE;
                        end
                    end
                end
            end
        end
    end
    function reg is_store(input [`OPERATOR_WIDTH] type);
        begin
            is_store = (type == `SB || type == `SH ||  type == `SW);
        end
    endfunction
    function reg is_load (input [`OPERATOR_WIDTH] type);
        begin
            is_load =  (type == `LB || type == `LH ||  type == `LW || type == `LBU|| type == `LHU);
        end
    endfunction
endmodule //LSbuffer