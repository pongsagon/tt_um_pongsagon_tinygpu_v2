/* re-implementation and extension of from
   https://github.com/MichaelBell/tt07-rle-vga
   
   Copyright 2023-2024 (c) Michael Bell
   SPDX-License-Identifier: Apache-2.0
   

   To perform a read:
   - Set addr_in and set start_read high for 1 cycle
   - Wait for ready to go hi
   - The read data is now available on data_out

   If the controller is configured to transfer multiple bytes, then
   note that the word transferred in data_out is in big
   endian order, i.e. the byte with the lowest address is aligned to 
   the MSB of the word. 

*/



/*  Matt Pongsagon
   
   Flash
       - use EBh command instead of 6Bh, change timing in addr/wait state
            cmd 8, addr 6, wait 2+4, 4bit data each clk,...
       - only support 4-bit data read
       - sclk only stop in the idle state -> will read data more than needed, 
         just ignore excess data
       - FSM will finally stay at hold state and output data until user stop read
    
    RAM (use only RAM A)
        - enter quad mode, h35: cmd 8 
            after get at_quadmode=1, caller must send stop_txn
        - read hOB: cmd 2, addr 6, wait 4, 4bit data
        - write h02: cmd 2, addr 6, -, 4bit data
*/

//`default_nettype wire
//`timescale 1ns / 1ps

module spi_flash_controller #(parameter DATA_WIDTH_BITS=4, parameter ADDR_BITS=24) (
    input clk,
    input rstn,

    // External SPI interface
    input  [3:0]        spi_data_in,
    output reg [3:0]    spi_data_out,
    output reg [3:0]    spi_data_oe,
    output              spi_clk_out,
    // low to select
    output reg          spi_flash_select,
    output reg          spi_ram_a_select,
    output reg          spi_ram_b_select,   // always=1, use only RAM A
    

    // Configuration
    input [2:0] latency,

    // Internal interface for reading data
    // cmd part
    input                           select_ROM,     // 1 = ROM, 0 = RAM
    input                           enter_quadmode,
    input                           start_read,
    input                           start_write,
    input                           stop_txn,       // stop read or write
    //
    input [ADDR_BITS-1:0]           addr_in,
    input [DATA_WIDTH_BITS-1:0]     data_in,        // for writing
    //    
    output [DATA_WIDTH_BITS-1:0]    data_out,       // for reading
    output reg                      data_req,       // for writing
    output reg                      data_ready,     // for reading
    output reg                      at_quadmode
  
    // debug
  	//output [3:0] debug_spi_data_in,
  	//output [3:0] debug_spi_miso_in
);

`define max(a, b) (a > b) ? a : b

    //localparam DATA_WIDTH_BITS = DATA_WIDTH_BYTES * 8;
    localparam BITS_REM_BITS = $clog2(`max(DATA_WIDTH_BITS,ADDR_BITS));

    // read ROM / RAM / write RAM
    localparam FSM_IDLE = 0;
    localparam FSM_CMD  = 1;
    localparam FSM_ADDR = 2;
    localparam FSM_DUMMY = 3;
    localparam FSM_DATA = 4;
    localparam FSM_LAT1 = 5;
    localparam FSM_LAT2 = 6;
    localparam FSM_STREAM = 7;
    // enter quad


    reg [2:0] fsm_state;
    reg       doing_quadmode;
    reg       is_writing;
    reg [7:0] spi_miso_buf_n;
    reg [7:0] spi_miso_buf_p;
    reg [ADDR_BITS-1:0]       addr;
    reg [DATA_WIDTH_BITS-1:0] data;
    reg [BITS_REM_BITS-1:0] bits_remaining;     // of each state


    assign data_out = data;
    //assign spi_select = fsm_state == FSM_IDLE;
    assign spi_clk_out = !clk && (fsm_state > 0);


    always @(posedge clk) begin
        if (!rstn || stop_txn ) begin
            fsm_state <= FSM_IDLE;
            is_writing <= 0;
            bits_remaining <= 0;
            data_ready <= 0;
            data_req <= 0;
            at_quadmode <= 0; 
            doing_quadmode <= 0;
            spi_data_oe <= 4'b0000;
            spi_flash_select <= 1;
            spi_ram_a_select <= 1;
            spi_ram_b_select <= 1;
        end else begin
            data_ready <= 0;
            data_req <= 0;
            if (fsm_state == FSM_IDLE) begin
                if (start_read || start_write || enter_quadmode) begin
                    if(select_ROM || enter_quadmode)begin
                        spi_data_oe <= 4'b0001;
                        bits_remaining <= 8-1;
                        doing_quadmode <= (enter_quadmode)? 1:0;
                    end else begin
                        is_writing <= !start_read;
                        spi_data_oe <= 4'b1111;
                        bits_remaining <= 2-1;
                    end
                    fsm_state <= FSM_CMD;
                    //
                    spi_flash_select <= (select_ROM)? 0:1;
                    spi_ram_a_select <= (select_ROM)? 1:0;
                end
            end else if (fsm_state == FSM_STREAM) begin
                // do nothing, keep reading/writing data until stop_txn
                data_ready <= !is_writing;
                data_req <= is_writing;
            end else begin
                if (bits_remaining == 0) begin
                    fsm_state <= fsm_state + 1;
                    if (fsm_state == FSM_CMD) begin
                        if(doing_quadmode)begin
                            at_quadmode <= 1;       
                            fsm_state <= FSM_IDLE;
                            spi_ram_a_select <= 1;
                        end else begin
                            // change to FSM_ADDR
                            bits_remaining <= (ADDR_BITS >> 2)-1;
                            spi_data_oe <= 4'b1111;
                        end
                    end
                    else if (fsm_state == FSM_ADDR) begin
                        // change to FSM_DUMMY or FSM_STREAM
                        if(select_ROM)begin
                            bits_remaining <= 6-1;
                        end else if(is_writing)begin
                            data_req <= 1;
                            fsm_state <= FSM_STREAM;
                        end else begin
                            bits_remaining <= 4-1;
                        end
                    end 
                    else if (fsm_state == FSM_DUMMY) begin
                        spi_data_oe <= 4'b0000;
                     	bits_remaining <= 0;
                    end 
                    else if (fsm_state == FSM_DATA) begin
                     	bits_remaining <= 0;
                    end 
                    else if (fsm_state == FSM_LAT1) begin
                        bits_remaining <= 0;
                    end 
                    else if (fsm_state == FSM_LAT2)  begin
                        // 1st 4-bit data ready after this state
                        data_ready <= 1;
                    end 
                end else begin
                    bits_remaining <= bits_remaining - 1;
                end
            end
        end
    end


    // spi_data_out                
    always @(*) begin
        case (fsm_state)
            FSM_CMD: begin // CMD only used for the PSRAM, the flash is always in continuous read mode
                if (is_writing) begin
                    // RAM Write command is 02h
                    spi_data_out = (bits_remaining == 1)? 4'b0000 : 4'b0010;
                end else begin
                    if(select_ROM) begin
                        // ROM Read command is EBh, single mode
                        spi_data_out = {3'b000, !(bits_remaining == 4 || bits_remaining == 2)};
                    end else if(doing_quadmode) begin
                        // Quadmode command is 35h, single mode
                        spi_data_out = {3'b000, (bits_remaining == 0 || bits_remaining == 2 || bits_remaining == 4 || bits_remaining == 5)};
                    end else begin
                        // RAM Read command is 0Bh
                        spi_data_out = (bits_remaining == 1)? 4'b0000 : 4'b1011;
                    end
                end
            end
            FSM_ADDR:   spi_data_out = addr[ADDR_BITS-1:ADDR_BITS-4];
            // for write only, chk spi_data_oe
            FSM_STREAM:   spi_data_out = data_in;
            default:    spi_data_out = 4'b0000;
        endcase
    end


    // addr used in spi_data_out
    always @(posedge clk) begin
        if (fsm_state == FSM_IDLE && (start_read || start_write)) begin
            addr <= addr_in;
        end else if (fsm_state == FSM_ADDR) begin
            addr <= {addr[ADDR_BITS-5:0], 4'b0000};
        end
    end


    // handle read latency
    always @(negedge clk) begin
        spi_miso_buf_n <= {spi_miso_buf_n[3:0], spi_data_in};
    end
    always @(posedge clk) begin
        spi_miso_buf_p <= {spi_miso_buf_p[3:0], spi_data_in};
    end
    reg [3:0] spi_miso_in;
    always @(*) begin
        if (latency[0]) begin
            if (latency[1]) spi_miso_in = spi_miso_buf_p[3:0];
            else spi_miso_in = spi_miso_buf_p[7:4];
        end else begin
            if (latency[2]) spi_miso_in = spi_miso_buf_n[3:0];
            else spi_miso_in = spi_miso_buf_n[7:4];
        end
    end

    // data_out to read caller
    always @(posedge clk) begin
        // only valid when data_ready == 1
        data <= spi_miso_in;
    end

    

  	//assign debug_spi_miso_in = spi_miso_in;
  	//assign debug_spi_data_in = spi_data_in;
endmodule