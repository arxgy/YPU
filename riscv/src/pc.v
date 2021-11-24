//PC Address Controller for MIPS-32.
//without predictor for now...
`include "def.v"
module pc (
    input  wire     in_clk,
    input  wire     in_rst, 
    input  wire     in_rdy,
    input  wire     in_stall,    
    //connect with control-signal
    
    input  wire     in_flush_enable,   
    input  wire     [`ADDRESS_WIDTH] in_rob_branch_pc,
    //connect with ROB

    output reg      out_fetcher_enable,
    output reg      [`ADDRESS_WIDTH] out_fetcher_pc
    //connect with fetcher
);
    reg [`ADDRESS_WIDTH] last_push_pc;
    reg                  flush;
    always @(posedge in_clk) begin
        if (in_rst) begin
            last_push_pc       <= `ZERO_DATA-4;
            out_fetcher_enable <= `FALSE;
            flush <= `FALSE;
        end 
        else if(in_rdy) begin
            if (in_flush_enable) begin
                flush <= `TRUE;
                out_fetcher_enable <= `FALSE;
            end
            else begin
                if (in_stall) begin
                    out_fetcher_enable <= `FALSE;
                end
                else begin
                    out_fetcher_enable <= `TRUE;//
                    if (flush) begin
                        out_fetcher_pc      <= in_rob_branch_pc;
                        last_push_pc        <= in_rob_branch_pc;
                        flush <= `FALSE;
                    end
                    else begin
                        out_fetcher_pc      <= last_push_pc + 4;
                        last_push_pc        <= last_push_pc + 4;
                    end
                end
            end
        end
    end
endmodule //pc