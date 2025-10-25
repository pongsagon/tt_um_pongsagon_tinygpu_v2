
/* Matt pongsagon

  Task: raster screen space tri
    0. enter Quad mode
    1. copy flash->ram 
      - read numtri (2 byte)
      - each vertex: XY screen Q16.0, Z screen 0.16
    2. read front, clear back
    3. clear z
    4. swap

  Include module
    - spi_flash_controller
    - vga
    - vsfs

 
  Addr RAM space
    - after reset, FSM will drive start read
    - 0:      front, FLASH tri
    - 38400:  back
    - 76800:  z
    - 153600: tri

  - ui_in[2:0]: latency, 
  - ui_in[6:4]: gamepad pmod


*/

//`timescale 1ns / 1ps

module tt_um_pongsagon_tinygpu_v2 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    
    // // sim version
    // output wire [9:0] sim_x,
    // output wire [9:0] sim_y,
    // output wire sim_blank,

    // // things to watch
    // output wire debug_do_swap,
    // output wire debug_ram_notbusy,
    // output  wire [19:0] debug_x_screen_v0,
    // output  wire [19:0] debug_x_screen_v1,
    // output  wire [19:0] debug_x_screen_v2,
    // output  wire [19:0] debug_y_screen_v0,
    // output  wire [19:0] debug_y_screen_v1,
    // output  wire [19:0] debug_y_screen_v2,
    // output  wire [21:0] debug_z_screen_v0,
    // output  wire [21:0] debug_z_screen_v1,
    // output  wire [21:0] debug_z_screen_v2,
    // output wire [8:0] debug_numtri,
    // output wire [7:0] debug_vsfs_fsm_state,
    // output wire [4:0] debug_fsm_state,
    // output wire debug_start_printing,
    // output wire [3:0] debug_spi_data,
    // output wire [7:0] debug_sub_frame,
    // output reg [21:0] debug_clk,           // >2.11M   

    input  wire       rst_n    // reset_n - low to reset
  );


  // vga
  wire HS,VS;
  wire setblack;
  wire setblack_;
  wire [9:0] x;
  wire [9:0] y;
  wire blank;
  assign setblack = ((x > 320) | (y > 239));
  assign setblack_ = ((x > 319) | (y > 239));

  vga v(
      .clk(clk),
      .reset(!rst_n),
      .HS(HS),
      .VS(VS),
      .blank(blank),
      .x(x),
      .y(y)
  );
    

  // external SPI interface
  wire [3:0] qspi_data_in = {uio_in[5:4], uio_in[2:1]};
  wire [3:0] qspi_data_out;
  wire [3:0] qspi_data_oe;
  wire       qspi_clk_out;
  wire       qspi_flash_select;
  wire       qspi_ram_a_select;
  wire       qspi_ram_b_select;
  assign uio_out = {qspi_ram_b_select, qspi_ram_a_select, qspi_data_out[3:2], 
                    qspi_clk_out, qspi_data_out[1:0], qspi_flash_select};
  assign uio_oe = rst_n ? {2'b11, qspi_data_oe[3:2], 1'b1, qspi_data_oe[1:0], 1'b1} : 8'h00;  
  

  // internal SPI
  reg spi_select_ROM;
  reg spi_enter_quadmode;
  wire spi_start_read;
  wire spi_start_write;
  wire spi_stop_txn;
  wire [23:0] spi_addr;
  wire [3:0] spi_data_in;
  wire [3:0] spi_data;
  wire spi_data_req;
  wire spi_data_ready;
  wire spi_at_quadmode;


  // vsfs
  wire vsfs_start_read;
  wire vsfs_start_write;
  wire vsfs_stop_txn;
  wire [23:0] vsfs_addr;
  wire [3:0] vsfs_data_in;
  wire do_swap;
  reg start_vsfs;
  reg [8:0] numtri;         // <512 tri


  // main FSM
  parameter   SLOWEST_STATE       = 75;   // 55 RW time + 16 wait + 4 safe = 75
  reg [4:0] fsm_state;        // 0-31: state
  reg [4:0] read_delay;       // flash: 24, RAM R: 16,
  reg [17:0] numread;         // #4bit read, >153600 (#z pixel)
  reg [3:0] pixels [3:0];     // do 4pixel at atime
  reg [7:0] sub_frame;        // must < 255, start with 1 not 0
  reg evenframe;              // 1: front -> 0, 0: front -> 38400
  reg [16:0] drawPixel;       // >76800 (320x240), draw @pixel
  wire [9:0] yplus1;          // to set addr for the next line
  assign yplus1 = y + 1;
  reg [13:0] i_numtri_byte;   // for loop read numtri
  wire [13:0] numtri_byte;     // numtri * 18 (2 xyz x 3);
  assign numtri_byte = {1'b0,numtri,4'b0000} + {4'b0,numtri,1'b0};
  //
  reg display_start_read;
  reg display_start_write;
  reg display_stop_txn;
  reg [23:0] display_addr;
  reg [3:0] display_data_in;
  //
  wire vsfs_running;
  wire eol;               // stop access mem to display image
  wire eof;               // stop access mem to display image
  wire eol_e;
  wire eof_e;
  assign eol = (y < 239) & (x > (799-SLOWEST_STATE)) & (sub_frame > 1);
  assign eof = (y == 524) & (x > (799-SLOWEST_STATE));    // true(725,524)->, false(0,0)
  assign eol_e = (y < 239) & (x > 782) & (sub_frame > 1);
  assign eof_e = (y == 524) & (x > 782);    
  // mux between VSFS & display to ram
  wire ram_notbusy;
  wire ram_notbusy_end;
  assign ram_notbusy = ((sub_frame > 1) & setblack & !eol & !eof) | 
                        ((sub_frame < 2) & (!eof) & (y > 431)) ;
  assign ram_notbusy_end = ((sub_frame > 1) & setblack & !eol_e & !eof_e) | 
                          ((sub_frame < 2) & (!eof_e) & (y > 431)) ;
  assign spi_addr =         (ram_notbusy_end & vsfs_running)? vsfs_addr : display_addr;
  assign spi_data_in =      (ram_notbusy_end & vsfs_running)? vsfs_data_in : display_data_in;
  assign spi_start_read =   (ram_notbusy_end & vsfs_running)? vsfs_start_read : display_start_read;
  assign spi_start_write =  (ram_notbusy_end & vsfs_running)? vsfs_start_write : display_start_write;
  assign spi_stop_txn =     (ram_notbusy_end & vsfs_running)? vsfs_stop_txn : display_stop_txn;


  vsfs _vsfs(
      .clk(clk),
      .reset(!rst_n),
      .vsfs_addr(vsfs_addr),
      .vsfs_data_in(vsfs_data_in),
      .vsfs_start_read(vsfs_start_read),
      .vsfs_start_write(vsfs_start_write),
      .vsfs_stop_txn(vsfs_stop_txn),
      .spi_data(spi_data),
      .spi_data_req(spi_data_req),
      .start_vsfs(start_vsfs),
      .do_swap(do_swap),
      .vsfs_running(vsfs_running),
      .x(x),
      .y(y),
      .numtri(numtri),
      .evenframe(evenframe),
      
      //
      // .debug_x_screen_v0(debug_x_screen_v0),
      // .debug_x_screen_v1(debug_x_screen_v1),
      // .debug_x_screen_v2(debug_x_screen_v2),
      // .debug_y_screen_v0(debug_y_screen_v0),
      // .debug_y_screen_v1(debug_y_screen_v1),
      // .debug_y_screen_v2(debug_y_screen_v2),
      // .debug_z_screen_v0(debug_z_screen_v0),
      // .debug_z_screen_v1(debug_z_screen_v1),
      // .debug_z_screen_v2(debug_z_screen_v2),
      // .debug_vsfs_fsm_state(debug_vsfs_fsm_state),

      .ram_notbusy(ram_notbusy)
  );

  spi_flash_controller  i_spi (
      .clk        (clk),
      .rstn       (rst_n),

      .spi_data_in(qspi_data_in),
      .spi_data_out(qspi_data_out),
      .spi_data_oe(qspi_data_oe),
      .spi_clk_out(qspi_clk_out),
      .spi_flash_select (qspi_flash_select),
      .spi_ram_a_select (qspi_ram_a_select),
      .spi_ram_b_select (qspi_ram_b_select),

      .latency    (ui_in[2:0]),

      .select_ROM (spi_select_ROM),
      .enter_quadmode (spi_enter_quadmode),
      .start_read (spi_start_read),
      .start_write (spi_start_write),
      .stop_txn (spi_stop_txn),
      .addr_in (spi_addr),
      .data_in (spi_data_in),
      .data_out (spi_data),
      .data_req (spi_data_req),
      .data_ready (spi_data_ready),
      .at_quadmode (spi_at_quadmode)
  );

  

  // pixel spi data to uo_out
  assign uo_out[0] = (setblack_?0:spi_data[3]);
  assign uo_out[1] = (setblack_?0:spi_data[2]);
  assign uo_out[5] = (setblack_?0:spi_data[1]);
  assign uo_out[2] = (setblack_?0:spi_data[0]);
  assign uo_out[3] = VS;
  assign uo_out[7] = HS;
  assign uo_out[4] = (setblack_?0:spi_data[3]);
  assign uo_out[6] = (setblack_?0:spi_data[0]);
  

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      fsm_state <= 0;
      read_delay <= 0;
      numread <= 0;
      sub_frame <= 0;
      evenframe <= 1;
      drawPixel <= 0;
      i_numtri_byte <= 0;
      //
      //debug_clk <= 0;
      // SPI
      spi_select_ROM <= 0;
      spi_enter_quadmode <= 0;
      display_start_read <= 0;
      display_start_write <= 0;
      display_stop_txn <= 0;
      display_addr <= 0;  
      display_data_in <= 0;
      //
      start_vsfs <= 0;
      numtri <= 0;
    end else begin
      //debug_clk <= debug_clk + 1;
      case (fsm_state)
      // 0. enter Quad mode (fsm 0-1)
        0: begin
          if ((y == 524) & (x == 775)) begin 
            spi_enter_quadmode <= 1;
            fsm_state <= 1;
          end
        end
        //    wait for at_quadmode, around 8 clk
        1: begin
          spi_enter_quadmode <= 0;
          if(spi_at_quadmode)begin
            display_stop_txn <= 1;
            fsm_state <= 2;
          end
        end
      // 1. copy Flash -> RAM (fsm 2 - 10)
        //  - read numtri (2 byte), at addr 153600
        2: begin
          display_stop_txn <= 0;
          numread <= 0;
          spi_select_ROM <= 1;      // flash
          display_start_read <= 1;
          display_addr <= 0;
          fsm_state <= 3;
        end
        //   -- wait for the first flash data to be ready
        3: begin
          display_start_read <= 0;
          if(read_delay == 24) begin
            read_delay <= 0;
            pixels[numread[1:0]] <= spi_data;
            numread <= 1;
            fsm_state <= 4;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
        end
        //   -- read 3 more 4bit
        4: begin
          pixels[numread[1:0]] <= spi_data;
          numread <= numread + 1;
          if(numread == 3)begin
            numread <= 0;
            display_stop_txn <= 1;
            fsm_state <= 5;
          end

          // if(numread == 4)begin
          //   numread <= 0;
          //   display_stop_txn <= 1;
          //   fsm_state <= 5;
          // end else begin
          //   pixels[numread[1:0]] <= spi_data;
          //   numread <= numread + 1;
          // end
        end
        //   -- save to numtri, little endian -> big endian
        5: begin
          display_stop_txn <= 0;
          numtri[8] <= pixels[3][0];
          numtri[3:0] <= pixels[1];
          numtri[7:4] <= pixels[0];
          i_numtri_byte <= 0;
          display_addr <= 2;
          fsm_state <= 6;
        end
        //  - for loop copy each vertex: XY screen Q16.0, Z screen 0.16
        6: begin
          display_stop_txn <= 0;
          numread <= 0;
          if(i_numtri_byte == numtri_byte)begin
            i_numtri_byte <= 0;
            fsm_state <= 11;
          end else begin
            spi_select_ROM <= 1;
            display_start_read <= 1;
            fsm_state <= 7;
          end
        end
        //   -- wait for the first flash data to be ready
        7: begin
          display_start_read <= 0;
          if(read_delay == 24) begin
            read_delay <= 0;
            pixels[numread[1:0]] <= spi_data;
            numread <= 1;
            fsm_state <= 8;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
        end
        //   -- read 3 more 4bit
        8: begin
          pixels[numread[1:0]] <= spi_data;
          numread <= numread + 1;
          if(numread == 3)begin
            numread <= 0;
            display_stop_txn <= 1;
            fsm_state <= 9;
          end

          // if(numread == 4)begin
          //   numread <= 0;
          //   display_stop_txn <= 1;
          //   fsm_state <= 9;
          // end else begin
          //   pixels[numread[1:0]] <= spi_data;
          //   numread <= numread + 1;
          // end
        end
        //   -- init to write to RAM tri, little endian -> big endian
        9: begin
          display_stop_txn <= 0;
          spi_select_ROM <= 0;
          display_start_write <= 1;
          display_addr <= 24'd153600 + {10'b0,i_numtri_byte};
          display_data_in <= pixels[numread[1:0] + 2'b10];
          numread <= numread + 1;
          fsm_state <= 10;
        end
        //   -- wait & write 2 byte
        10: begin
          display_start_write <= 0;
          if (spi_data_req) begin
            display_data_in <= pixels[numread[1:0] + 2'b10];
            numread <= numread + 1;
            if(numread == 3) begin
              numread <= 0;
              display_stop_txn <= 1;
              display_addr <= {10'b0,i_numtri_byte} + 4; // 4 = offset #tri 2byte + next 2 byte
              i_numtri_byte <= i_numtri_byte + 2;
              fsm_state <= 6;
            end
          end


          // if(numread == 4)begin
          //   numread <= 0;
          //   display_stop_txn <= 1;
          //   display_addr <= {10'b0,i_numtri_byte} + 4; // 4 = offset #tri 2byte + next 2 byte
          //   i_numtri_byte <= i_numtri_byte + 2;
          //   fsm_state <= 6;
          // end else if(spi_data_req)begin
          //   display_data_in <= pixels[numread[1:0] + 2'b10];
          //   if(numread == 3) begin
          //     display_stop_txn <= 1;
          //   end
          //   numread <= numread + 1;
          // end
        end

      // 2. RAM front -> vga uo_out[] + clear Back (fsm 11 - 16)
        //  - wait for eof last line y & x 16 clk ahead to read the first pixel 
        11: begin
          display_stop_txn <= 0;
          start_vsfs <= 0;
          if (do_swap) begin
            // 4. swap, set for 1clk from VSFS
            evenframe <= !evenframe;
            sub_frame <= 0;
          end
          else if ((y == 524) & (x == 783)) begin     //mark1, eof
            spi_select_ROM <= 0;
            display_start_read <= 1;
            display_addr <= (evenframe)?0:38400;
            //debug_clk <= 0;
            sub_frame <= sub_frame + 1;
            fsm_state <= 12;
          end
        end
        // - wait for the first data to be ready, display the first pixel
        12: begin
          display_start_read <= 0;
          if(read_delay == 16) begin
            read_delay <= 0;
            numread <= 1;
            fsm_state <= 13;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
        end
        // - keep display pixels for 1 line, until hblank 
        13: begin
          numread <= numread + 1;
          if(numread == 319) begin
            numread <= 0;
            display_stop_txn <= 1;
            if (sub_frame == 1) begin
              // do clear back
              fsm_state <= 14;
            end else  if (y < 239) begin
              // go to wait for eol 
              fsm_state <= 16;
            end else begin
              // go to wait for eof
              fsm_state <= 11;
            end
          end

          // if(numread == 320) begin              //mark2,
          //   numread <= 0;
          //   display_stop_txn <= 1;
          //   if (sub_frame == 1) begin
          //     // do clear back
          //     fsm_state <= 14;
          //   end else  if (y < 239) begin
          //     // go to wait for eol 
          //     fsm_state <= 16;
          //   end else begin
          //     // go to wait for eof
          //     fsm_state <= 11;
          //   end
          // end
          // else begin
          //   numread <= numread + 1;
          // end
        end
        // - switch to clear back 1 line
        14: begin
            display_stop_txn <= 0;
            display_start_write <= 1;
            display_data_in <= 4'b0000;     // black
            //addr <= y * 160 + offset of back[0];
            display_addr <= (evenframe)?{7'b0,y,7'b0} + {9'b0,y,5'b0} + 38400:
                                {7'b0,y,7'b0} + {9'b0,y,5'b0};
            numread <= numread + 1;
            fsm_state <= 15;
        end
        // - wait & clear back 1 line
        15: begin
          display_start_write <= 0;
          if (spi_data_req) begin
            display_data_in <= 4'b0000;
            numread <= numread + 1;
            if(numread == 319)begin 
              numread <= 0;            
              display_stop_txn <= 1;
              if (y == 239) begin               //mark5: to clearZ
                fsm_state <= 17;
              end else begin                    //mark3
                fsm_state <= 16;
              end
            end
          end

          // if(numread == 319)begin 
          //   numread <= 0;            
          //   display_stop_txn <= 1;
          //   if (y == 239) begin               //mark5: to clearZ
          //     fsm_state <= 17;
          //   end else begin                    //mark3
          //     fsm_state <= 16;
          //   end
          // end else if(spi_data_req)begin
          //   display_data_in <= 4'b0000;
          //   numread <= numread + 1;
          // end
        end
        // - wait for eol state, start read 16 clk ahead of the next line
        // enter here from 13 OR 15
        16: begin
          display_stop_txn <= 0;
          if (x == 783) begin               //mark4
            display_start_read <= 1;
            display_addr <= (evenframe)?{7'b0,yplus1,7'b0} + {9'b0,yplus1,5'b0}:
                                {7'b0,yplus1,7'b0} + {9'b0,yplus1,5'b0} + 38400;
            fsm_state <= 12;
          end
        end

      // 3. clearZ (fsm 17-18)
        17: begin
          display_stop_txn <= 0;
          display_start_write <= 1;
          display_addr <= 76800;
          display_data_in <= 4'b1111;   // 0.996 fartest
          numread <= numread + 1;
          fsm_state <= 18;
        end
        18: begin
          display_start_write <= 0;
          if (spi_data_req) begin
            display_data_in <= 4'b1111;
            numread <= numread + 1;
            if (numread == 153599)begin       //mark6
              numread <= 0;
              display_stop_txn <= 1;
              drawPixel <= 0;          
              start_vsfs <= 1;
              fsm_state <= 11;
            end
          end

          // if (numread == 153599)begin       //mark6
          //   numread <= 0;
          //   display_stop_txn <= 1;
          //   drawPixel <= 0;          
          //   start_vsfs <= 1;
          //   fsm_state <= 11;
          // end else if(spi_data_req)begin
          //   display_data_in <= 4'b1111;
          //   numread <= numread + 1;
          // end
        end



        default: begin
          fsm_state <= 0;
        end
      endcase
    end
  end

  // need for sim with verilator
  // assign sim_x = x;
  // assign sim_y = y;
  // assign sim_blank = blank;
  // // debug
  // assign debug_numtri = numtri;
  // assign debug_fsm_state = fsm_state;
  // assign debug_start_printing = ((fsm_state == 11) & ((y == 524) & (x == 783)));
  // assign debug_spi_data = spi_data;
  // assign debug_sub_frame = sub_frame;
  // assign debug_ram_notbusy = ram_notbusy;
  // assign debug_do_swap = do_swap;

    
 endmodule