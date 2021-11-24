//A port for Mem access from Instr Access and Data Access, issue Instr to decoder.
//waiting for instruction-cache implement
`include "def.v"

module fetcher (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy,
    input  wire     in_stall,
    //connect with control-signal

    input  wire     in_fetcher_enable,
    input  wire     [`ADDRESS_WIDTH] in_pc,   
    output wire     out_pc_fetch_full,
    //connect with PC controller

    output reg      out_decoder_decode_enable,
    output reg      [`INSTRUCTION_WIDTH] out_decoder_pc_inst,
    output reg      [`ADDRESS_WIDTH] out_decoder_pc_addr,
    //connect with decoder

    output reg      out_dispatch_pc_requesting,
    output reg      [`ADDRESS_WIDTH] out_dispatch_pc_addr,
    input  wire     [`INSTRUCTION_WIDTH] in_dispatch_pc_inst,
    input  wire     in_pc_req_enable,
    input  wire     in_pc_data_enable
    //connect with dispatcher (request instruction)
);
    reg [`ADDRESS_WIDTH]     pc_entry   [`FETCH_ENTRY];     
    reg [`INSTRUCTION_WIDTH] inst_entry [`FETCH_ENTRY];    
    reg                      wait_entry [`FETCH_ENTRY];
    reg [`FETCH_WIDTH] head, tail, next;    //next pc that push to dispatcher 
    wire [`ADDRESS_WIDTH]    dbg_head_pc = pc_entry[head];
    reg empty;
    integer iter;
    assign out_pc_fetch_full = (head == tail + 3'd1 || (head == tail && !empty));


    always @(posedge in_clk) begin
        if (in_rst) begin
            empty<= `TRUE;
            head <= `HEAD_FETCH_ENTRY;
            tail <= `HEAD_FETCH_ENTRY;
            next <= `HEAD_FETCH_ENTRY;
            out_decoder_decode_enable  <= `FALSE;
            out_dispatch_pc_requesting <= `FALSE;
            for (iter = 0 ; iter < `FETCH_SIZE ; iter = iter+1) begin
                wait_entry[iter] <= `FALSE;
            end
        end
        else if(in_rdy) begin
            //with pc_ctrl
            if (in_fetcher_enable) begin
                pc_entry[tail]   <= in_pc;
                wait_entry[tail] <= `FALSE;
                tail  <= tail + 3'd1;
                empty <= `FALSE;
            end       
            //with dispatcher
            if (in_pc_req_enable && next != tail && !wait_entry[next]) begin
                out_dispatch_pc_requesting <= `TRUE;    
                wait_entry[next] <= `TRUE;
                out_dispatch_pc_addr <= pc_entry[next];
            end
            else begin
                out_dispatch_pc_requesting <= `FALSE;
            end

            if (in_pc_data_enable) begin
                inst_entry[next] <= in_dispatch_pc_inst;
                next <= next + 3'd1;
            end
            //with decoder
            if (in_stall) begin   
                out_decoder_decode_enable <= `FALSE;
            end
            else begin             //1 cycle delay? so sad
                if (next == head) begin         
                    out_decoder_decode_enable <= `FALSE;
                end
                else begin
                    out_decoder_decode_enable <= `TRUE;
                    out_decoder_pc_addr <= pc_entry[head];
                    out_decoder_pc_inst <= inst_entry[head];
                    head <= head + 3'd1;
                    if (tail == head + 3'd1) empty <= `TRUE;
                end
            end
        end
    end
endmodule //fetcher