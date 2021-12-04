
//A port for Mem access from Instr Access and Data Access, issue Instr to decoder.
//with 256 size icache
`include "def.v"

module fetcher (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    input  wire     in_stall,
    input  wire     in_flush_enable,
    //connect with control-signal

    input  wire     in_fetcher_enable,
    input  wire     [`ADDRESS_WIDTH] in_pc,  
    input  wire     in_predict, 
    output wire     out_pc_fetch_full,
    output reg      out_pc_last_enable,
    output reg      [`INSTRUCTION_WIDTH] out_pc_last_inst,
    //connect with PC controller

    output reg      out_decoder_decode_enable,
    output reg      [`INSTRUCTION_WIDTH] out_decoder_pc_inst,
    output reg      [`ADDRESS_WIDTH] out_decoder_pc_addr,
    output reg      out_decoder_pc_pre, 
    //connect with decoder

    output reg      out_dispatch_pc_requesting,
    output reg      [`ADDRESS_WIDTH] out_dispatch_pc_addr,
    input  wire     [`INSTRUCTION_WIDTH] in_dispatch_pc_inst,
    input  wire     in_pc_req_enable,
    input  wire     in_pc_data_enable
    //connect with dispatcher (request instruction)
);
    //inst queue
    reg  [`ADDRESS_WIDTH]     pc_entry    [`FETCH_ENTRY];     
    reg  [`INSTRUCTION_WIDTH] inst_entry  [`FETCH_ENTRY];    
    reg  [1:0]                pre_entry   [`FETCH_ENTRY];
    reg                       wait_entry  [`FETCH_ENTRY];
    reg  [`FETCH_WIDTH]       head, tail, next;    
    // wire [`ADDRESS_WIDTH]     dbg_head_pc = pc_entry[head];
    // wire [1:0]                dbg_head_predict = pre_entry[head];
    // wire [1:0]                dbg_1138_predict = pre_entry[3'd3];
    reg  empty;    
    //icache
    reg  [`TAG_WIDTH]         tag_cache   [`CACHE_ENTRY];
    reg                       valid_cache [`CACHE_ENTRY];
    reg  [`INSTRUCTION_WIDTH] inst_cache  [`CACHE_ENTRY];

    integer                   queue_iter, cache_iter;
    assign                    out_pc_fetch_full = (head == tail + 3'd1 || (head == tail && !empty));
   
    wire [`INDEX_WIDTH]       pc_index   = pc_entry[next][10:0];
    wire                      cache_hit  = valid_cache[pc_index] && tag_cache[pc_index] == pc_entry[next][17:11];
    wire [`INSTRUCTION_WIDTH] cache_inst = inst_cache[pc_index];
    wire [`FETCH_WIDTH] tail_prev = tail == 3'd0 ? 3'd7 : tail-3'd1;
    always @(posedge in_clk) begin
        if (in_rst) begin
            empty <= `TRUE;
            head  <= `HEAD_FETCH_ENTRY;
            tail  <= `HEAD_FETCH_ENTRY;
            next  <= `HEAD_FETCH_ENTRY;
            out_pc_last_enable         <= `FALSE;
            out_decoder_decode_enable  <= `FALSE;
            out_dispatch_pc_requesting <= `FALSE;
            for (queue_iter = 0 ; queue_iter < `FETCH_SIZE ; queue_iter = queue_iter+1) begin
                pre_entry[queue_iter]  <= `NULL_PRE;
                wait_entry[queue_iter] <= `FALSE;
            end
            for (cache_iter = 0 ; cache_iter < `CACHE_SIZE ; cache_iter = cache_iter+1) begin
                valid_cache[cache_iter] <= `FALSE;
            end
        end
        else if(in_rdy) begin
            if (in_flush_enable) begin  //maintain cache
                empty<= `TRUE;
                head <= `HEAD_FETCH_ENTRY;
                tail <= `HEAD_FETCH_ENTRY;
                next <= `HEAD_FETCH_ENTRY;
                out_pc_last_enable <= `FALSE;
                out_decoder_decode_enable  <= `FALSE;
                out_dispatch_pc_requesting <= `FALSE;
                for (queue_iter = 0 ; queue_iter < `FETCH_SIZE ; queue_iter = queue_iter+1) begin
                    pre_entry[queue_iter] <= `NULL_PRE;
                    wait_entry[queue_iter] <= `FALSE;
                end
            end
            else begin
                out_pc_last_enable <= `FALSE;
                out_dispatch_pc_requesting <= `FALSE;
                //get pc from pc_ctrl
                if (in_fetcher_enable) begin
                    pc_entry[tail]   <= in_pc;
                    wait_entry[tail] <= `FALSE;
                    tail  <= tail + 3'd1;
                    empty <= `FALSE;
                    if (!empty) begin
                        pre_entry[tail_prev] <= in_predict ? `TAKE_PRE : `NOT_TAKE_PRE;
                    end
                end       
                //get inst from cache/dispatcher
                if (next != tail && !wait_entry[next]) begin 
                    if (cache_hit) begin
                        out_pc_last_enable <= `TRUE;
                        out_pc_last_inst   <=  cache_inst;
                        inst_entry[next]   <=  cache_inst;
                        next <= next + 3'd1;
                    end  
                    else if(in_pc_req_enable) begin
                        out_dispatch_pc_requesting <= `TRUE;    
                        wait_entry[next] <= `TRUE;
                        out_dispatch_pc_addr <= pc_entry[next];
                    end        
                end
                
                if (in_pc_data_enable) begin
                    out_pc_last_enable <= `TRUE;
                    out_pc_last_inst   <=  in_dispatch_pc_inst;
                    wait_entry[next]   <= `FALSE;
                    inst_entry[next]   <=  in_dispatch_pc_inst;
                    
                    valid_cache[pc_index] <= `TRUE;
                    tag_cache  [pc_index] <=  pc_entry[next][17:11];
                    inst_cache [pc_index] <=  in_dispatch_pc_inst;
                    next <= next + 3'd1;
                end
                
                if (in_stall) begin    //issue inst
                    out_decoder_decode_enable <= `FALSE;
                end
                else begin             //1 cycle delay? so sad
                    if (next == head) begin         
                        out_decoder_decode_enable <= `FALSE;
                    end
                    else if (pre_entry[head] != `NULL_PRE)begin
                        out_decoder_decode_enable <= `TRUE;
                        out_decoder_pc_addr <= pc_entry[head];
                        out_decoder_pc_inst <= inst_entry[head];
                        out_decoder_pc_pre  <= pre_entry[head] == `TAKE_PRE ? `TAKEN : `NOT_TAKEN;
                        head <= head + 3'd1;
                        if (tail == head + 3'd1) empty <= `TRUE;
                        pre_entry[head] <= `NULL_PRE;
                    end
                    else begin
                        out_decoder_decode_enable <= `FALSE;
                    end
                end
            end
        end
    end
endmodule //fetcher