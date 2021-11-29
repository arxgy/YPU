//ignore io_buffer_full signal when performming simulation, 
`include "def.v"
module dispatcher (
    input  wire     in_clk,
    input  wire     in_rst,
    input  wire     in_rdy, 
    input  wire     in_flush_enable,
    input  wire     io_buffer_full,
    //connect with control-signal

    input  wire     [`MEM_WIDTH]     in_mem_data,
    output reg      [`MEM_WIDTH]     out_mem_data,
    output reg      [`ADDRESS_WIDTH] out_mem_addr,
    output reg      out_mem_wr_signal,
    //connect with MEM

    input  wire     in_pc_requesting,
    input  wire     [`ADDRESS_WIDTH] in_pc_addr,
    output reg      [`DATA_WIDTH]    out_pc_inst,
    output wire     out_pc_req_enable,
    output reg      out_pc_data_enable,
    //connect with fetcher (pc request)

    input  wire     in_load_requesting,
    input  wire     [`ADDRESS_WIDTH]  in_load_addr,
    input  wire     [`RW_STYLE_WIDTH] in_load_style,
    output reg      [`DATA_WIDTH]     out_load_value,
    output wire     out_load_req_enable,
    output reg      out_load_data_enable,

    input  wire     in_store_requesting,
    input  wire     [`ADDRESS_WIDTH]  in_store_addr,
    input  wire     [`RW_STYLE_WIDTH] in_store_style,    
    input  wire     [`DATA_WIDTH]     in_store_value,
    output wire     out_store_req_enable
    //connect with lsb (load & store)

);
//dispatcher instruction def
`define GRANT_PC\
begin\
    push_status <= `PC_STATUS;\
    push_cycle  <=  3'd1;\
    out_mem_addr      <= pc_request_addr;\
    out_mem_wr_signal <= `READ_SIGNAL;\
end
`define GRANT_LOAD\
begin\
    push_status <= `LOAD_STATUS;\
    push_cycle  <=  3'd1;\
    out_mem_addr <= load_request_addr;\
    out_mem_wr_signal <= `READ_SIGNAL;\
end
`define GRANT_STORE\
begin\
    push_status <= `STORE_STATUS;\
    push_cycle  <=  3'd1;\
    out_mem_addr <= store_request_addr;\
    out_mem_data <= store_request_data[7:0];\
    out_mem_wr_signal <= `WRITE_SIGNAL;\
end
`define GRANT_IDLE\
begin\
    out_mem_addr <= `ZERO_ADDR;\
    out_mem_wr_signal <= `READ_SIGNAL;\
    push_status <= `IDLE_STATUS;\
    push_cycle  <=  3'd0;\
end

`define PUSH_PC\
begin\
    push_status <= `PC_STATUS;\
    push_cycle  <=  push_cycle + 3'd1;\
    out_mem_addr <= pc_request_addr + push_cycle;\
    out_mem_wr_signal <= `READ_SIGNAL;\
end
`define PUSH_LOAD\
begin\
    push_status <= `LOAD_STATUS;\
    push_cycle  <= push_cycle + 3'd1;\
    out_mem_addr <= load_request_addr + push_cycle;\
    out_mem_wr_signal <= `READ_SIGNAL;\
end

`define ARBIT_IDLE\
begin\
    if (pc_buffering) `GRANT_PC\
    else if (load_buffering) begin\
        if (load_request_addr[17:16] != 2'b11)       `GRANT_LOAD\
        else if (store_buffering && store_request_addr[17:16] != 2'b11) `GRANT_STORE\
        else                                                            `GRANT_IDLE\
    end\
    else if (store_buffering) begin\
        if (store_request_addr[17:16] != 2'b11)                         `GRANT_STORE\   //normal store
        else if (mem_status == `IDLE_STATUS)         `GRANT_STORE\   //io store
        else                                                            `GRANT_IDLE\
    end\
    else `GRANT_IDLE\
end
`define ARBIT_PC\
begin\
    pc_buffering <= `FALSE;\
    if (load_buffering) begin\
        if (load_request_addr[17:16] != 2'b11 )       `GRANT_LOAD\
        else if (store_buffering && store_request_addr[17:16] != 2'b11) `GRANT_STORE\
        else                                                            `GRANT_IDLE\
    end\
    else if(store_buffering) begin\
        if (store_request_addr[17:16] != 2'b11)                         `GRANT_STORE\   //normal store
        else if (mem_status == `IDLE_STATUS)         `GRANT_STORE\   //io store
        else                                                            `GRANT_IDLE\
    end\
    else `GRANT_IDLE\
end

`define ARBIT_LOAD\
begin\
    load_buffering <= `FALSE;\
    if(store_buffering) begin\
        if (store_request_addr[17:16] != 2'b11)                         `GRANT_STORE\   //normal store
        else if (mem_status == `IDLE_STATUS)         `GRANT_STORE\   //io store
        else                                                            `GRANT_IDLE\
    end\
    else if (pc_buffering) `GRANT_PC\
    else `GRANT_IDLE\
end


`define ARBIT_STORE\
begin\
    store_buffering <= `FALSE;\
    if (pc_buffering) `GRANT_PC\
    else if (load_buffering) begin\
        if (load_request_addr[17:16] != 2'b11)       `GRANT_LOAD\
        else                                                            `GRANT_IDLE\
    end\
    else `GRANT_IDLE\
end

    reg  [`DISPATCH_STATUS_WIDTH] mem_status, push_status;
    reg  [`CYCLE_COUNTER_WIDTH]   mem_cycle,  push_cycle;  
    reg  [`MEM_BUFFER_DATA_WIDTH] buffer_data;       

    reg                    pc_buffering;
    reg  [`ADDRESS_WIDTH]  pc_request_addr;
    reg                    load_buffering;
    reg  [`ADDRESS_WIDTH]  load_request_addr;
    reg  [`RW_STYLE_WIDTH] load_request_style;
    reg                    store_buffering;
    reg  [`ADDRESS_WIDTH]  store_request_addr;
    reg  [`DATA_WIDTH]     store_request_data;
    reg  [`RW_STYLE_WIDTH] store_request_style;

    assign out_pc_req_enable    = !in_pc_requesting    && !pc_buffering;        
    assign out_load_req_enable  = !in_load_requesting  && !load_buffering;
    assign out_store_req_enable = !in_store_requesting && !store_buffering;
    
    wire dbg_output_store = in_store_requesting && in_store_addr[17:16] == 2'b11;
    integer fd;
    initial begin
        fd = $fopen("tableOut.out", "w");
    end

    always @(posedge in_pc_requesting) begin
        pc_buffering        <= `TRUE;
        pc_request_addr     <= in_pc_addr;
    end
    always @(posedge in_load_requesting) begin
        load_buffering      <= `TRUE;
        load_request_addr   <= in_load_addr;
        load_request_style  <= in_load_style;
    end
    always @(posedge in_store_requesting) begin
        store_buffering     <= `TRUE;
        store_request_addr  <= in_store_addr;
        store_request_style <= in_store_style;
        store_request_data  <= in_store_value;
    end

    always @(posedge in_clk) begin
        if (in_rst) begin 
            mem_status            <= `IDLE_STATUS;    
            mem_cycle             <=  3'd0;
            push_status           <= `IDLE_STATUS;
            push_cycle            <=  3'd0;

            buffer_data           <= `ZERO_DATA;
            pc_buffering          <= `FALSE;
            load_buffering        <= `FALSE;
            store_buffering       <= `FALSE;
            out_pc_data_enable    <= `FALSE;
            out_load_data_enable  <= `FALSE;
        end
        else if(in_rdy) begin
            if (in_flush_enable) begin
                if (push_status == `LOAD_STATUS && load_request_addr[17:16] == 2'b11) begin //need to receive data of next cycle
                    mem_status <= push_status;
                    mem_cycle  <= push_cycle;
                end
                else begin //ignore data of next cycle
                    mem_status <= `IDLE_STATUS;
                    mem_cycle  <=  3'd0;
                end
                push_status <= `IDLE_STATUS;
                push_cycle  <=  3'd0;
                
                pc_buffering          <= `FALSE;
                load_buffering        <= `FALSE;
                store_buffering       <= `FALSE;
                out_pc_data_enable    <= `FALSE;
                if (mem_status == `LOAD_STATUS && load_request_addr[17:16] == 2'b11) begin //need to output data of this cycle
                    out_load_data_enable <= `TRUE;
                    out_load_value       <= {{24{1'b0}}, in_mem_data};
                end
                else begin //ignore data of this cycle
                    out_load_data_enable <= `FALSE;
                end
            end
            else begin
                mem_status <= push_status;
                mem_cycle  <= push_cycle;

                case (mem_status)
                    `IDLE_STATUS: begin
                        buffer_data <= `ZERO_DATA;
                        out_pc_data_enable   <= `FALSE;
                        out_load_data_enable <= `FALSE;
                    end
                    `PC_STATUS: begin
                        case (mem_cycle)
                            3'd1: begin
                                buffer_data[7:0]     <= in_mem_data;
                                out_load_data_enable <= `FALSE;
                            end
                            3'd2: buffer_data[15:8]  <= in_mem_data;
                            3'd3: buffer_data[23:16] <= in_mem_data;
                            3'd4: begin
                                out_pc_data_enable <= `TRUE;
                                out_pc_inst <= {in_mem_data, buffer_data[23:0]};
                            end
                        endcase
                    end
                    `LOAD_STATUS: begin
                        case (mem_cycle)
                            3'd1: begin
                                out_pc_data_enable <= `FALSE;
                                if (load_request_addr[17:16] == 2'b11 || load_request_style == `RW_BYTE) begin
                                    load_buffering <= `FALSE;
                                    out_load_data_enable <= `TRUE;
                                    out_load_value <= {{24{1'b0}}, in_mem_data};
                                end
                                else begin
                                    out_load_data_enable <= `FALSE;
                                    buffer_data[7:0] <= in_mem_data;
                                end
                            end
                            3'd2: begin
                                if (load_request_style == `RW_HALF_WORD) begin
                                    load_buffering <= `FALSE;
                                    out_load_data_enable <= `TRUE;
                                    out_load_value <= {{16{1'b0}}, in_mem_data, buffer_data[7:0]};
                                end
                                else begin                                   
                                    out_load_data_enable <= `FALSE;
                                    buffer_data[15:8] <= in_mem_data;
                                end
                            end
                            3'd3: begin
                                buffer_data[23:16] <= in_mem_data;
                            end
                            3'd4: begin
                                out_load_data_enable <= `TRUE;
                                out_load_value <= {in_mem_data, buffer_data[23:0]};
                            end
                        endcase
                    end
                    `STORE_STATUS: begin
                        out_pc_data_enable   <= `FALSE;
                        out_load_data_enable <= `FALSE;
                    end
                endcase

                case (push_status)
                    `IDLE_STATUS: `ARBIT_IDLE
                    `PC_STATUS: begin
                        case (push_cycle)
                            3'd1: `PUSH_PC
                            3'd2: `PUSH_PC
                            3'd3: `PUSH_PC
                            3'd4: `ARBIT_PC
                        endcase
                    end
                    `LOAD_STATUS: begin
                        case (push_cycle)
                            3'd1: begin
                                if (load_request_style == `RW_BYTE) `ARBIT_LOAD
                                else `PUSH_LOAD
                            end
                            3'd2: begin
                                if (load_request_style == `RW_HALF_WORD) `ARBIT_LOAD
                                else `PUSH_LOAD
                            end
                            3'd3: `PUSH_LOAD
                            3'd4: `ARBIT_LOAD
                        endcase
                    end
                    `STORE_STATUS: begin
                        case (push_cycle)
                            3'd1: begin
                                if (store_request_style == `RW_BYTE) `ARBIT_STORE
                                else begin
                                    push_status <= `STORE_STATUS;
                                    push_cycle  <= push_cycle + 3'd1;
                                    out_mem_addr <= store_request_addr + push_cycle;
                                    out_mem_data <= store_request_data[15:8];
                                    out_mem_wr_signal <= `WRITE_SIGNAL;
                                end
                            end
                            3'd2: begin
                                if (store_request_style == `RW_HALF_WORD) `ARBIT_STORE
                                else begin
                                    push_status <= `STORE_STATUS;
                                    push_cycle  <= push_cycle + 3'd1;
                                    out_mem_addr <= store_request_addr + push_cycle;
                                    out_mem_data <= store_request_data[23:16];
                                    out_mem_wr_signal <= `WRITE_SIGNAL;
                                end
                            end
                            3'd3: begin
                                push_status <= `STORE_STATUS;
                                push_cycle  <= push_cycle + 3'd1;
                                out_mem_addr <= store_request_addr + push_cycle;
                                out_mem_data <= store_request_data[31:24];
                                out_mem_wr_signal <= `WRITE_SIGNAL;
                            end
                            3'd4: `ARBIT_STORE
                        endcase
                    end
                endcase  
            end
        end
    end
endmodule //dispatcher


