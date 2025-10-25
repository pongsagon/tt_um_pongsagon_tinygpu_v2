
/* Matt pongsagon

  - addr Flash & RAM front start at 0
  - addr RAM back start at 38400 (320x240=76800/2, 2pixel:1byte)
  - addr RAM z start at 76800 (1pixel:1byte)
  	- front 4bit, back 4bit, z 8bit
		- cal z in Q2.20 save in Q0.8 [0,1]
  - addr Tri start at 153600

  color
  	1: blue
  	2: dark green


  VSFS
		//0. user input -> [M][MVP][M]-1, 40 states
    1. for each tri
    	- READ tri 
    //2. VS
    3. bbox
      - bbox (mul of 4, not chk outofrange, 3clk)   
    4. e0, bar, zbar
      4.1 x3: e_init
      4.2 denom
      4.3 x3: bar_init, bar_dx, bar_dy
      4.4 x3: z_bar, z_bar_dx, z_bar_dy
    5. for pixel y in bbox (y < bboxMax_Y)
      - e0 = e0_init, z = z_bar / e0_init += dy, z_bar += z_bar_dy
      - for x in bbox (x < bboxMax_X)
        - READ x4 Z+B (42 clk)
        // x4 pixel
        - if ((e0 < 0) && (e1 < 0) && (e2 < 0)) / e0 += dx, z += z_bar_dx
          - pixel[0-4].cz = (Z < Zbuffer)? cz: pixel[0-4].cz
        //
        - WRITE x4 Z+B (28 clk) 
 

 	Don't know why?, not cause problems
 		- state 33, spi_data still output another 16bit(4clk) after stop read from RAM

*/

//`timescale 1ns / 1ps


module vsfs (
    input  	wire       	clk,     
    input  	wire       	reset,   
    output  reg [23:0] 	vsfs_addr,
  	output  reg [3:0] 	vsfs_data_in,
  	output  reg 				vsfs_start_read,
  	output  reg					vsfs_start_write,
  	output  reg 				vsfs_stop_txn,
  	input 	wire [3:0]	spi_data,
  	input   wire 				spi_data_req,
  	input		wire 				start_vsfs,
  	output  reg 				do_swap,
  	output  reg 				vsfs_running,
  	input 	wire [9:0]  x,
  	input 	wire [9:0]  y,
  	input		wire [8:0]	numtri,
  	input   wire 				evenframe,
  	

    // things to watch
    // output  wire [19:0] debug_x_screen_v0,
    // output  wire [19:0] debug_x_screen_v1,
    // output  wire [19:0] debug_x_screen_v2,
    // output  wire [19:0] debug_y_screen_v0,
    // output  wire [19:0] debug_y_screen_v1,
    // output  wire [19:0] debug_y_screen_v2,
    // output  wire [21:0] debug_z_screen_v0,
    // output  wire [21:0] debug_z_screen_v1,
    // output  wire [21:0] debug_z_screen_v2,
    // output  wire [7:0]  debug_vsfs_fsm_state,

    input		wire 				ram_notbusy

  );
    

  // used by div w
	reg signed [31:0] div_a;  
	reg signed [31:0] div_b;  
	wire signed [31:0] div_result;
  reg div_start;
	wire div_done;
	wire div_busy;
	wire div_valid;
	wire div_dbz;
	wire div_ovf;
	// Q16.16
  div div1 (.clk (clk), .rst(reset),.start(div_start),.done(div_done)
  		  ,.a(div_a),.b(div_b),.val(div_result)
  		  ,.busy(div_busy),.valid(div_valid),.dbz(div_dbz),.ovf(div_ovf));

  // used by denom, (div w and denom can be computed at the same time)
  // Q20.20
  reg signed [39:0] div2_a;  
	reg signed [39:0] div2_b;  
	wire signed [39:0] div2_result;
  reg div2_start;
	wire div2_done;
	wire div2_busy;
	wire div2_valid;
	wire div2_dbz;
	wire div2_ovf;
  div #(.WIDTH(40),.FBITS(20)) div2 
    		  (.clk (clk), .rst(reset),.start(div2_start),.done(div2_done)
    		  ,.a(div2_a),.b(div2_b),.val(div2_result)
    		  ,.busy(div2_busy),.valid(div2_valid),.dbz(div2_dbz),.ovf(div2_ovf));

	// mul 22-bit used to compute ei_init, bar
	reg signed [21:0] mul_a;  
	reg signed [21:0] mul_b;  
  wire signed [43:0] mul_result;  
  reg mul_start;
  wire mul_done;
  wire mul_busy;
  wire mul_aux;
  slowmpy #(.LGNA(5),.NA(22)) mul 
  			(.i_clk (clk), .i_reset(reset), .i_stb(mul_start),.i_a(mul_a)
  			,.i_b(mul_b),.i_aux(1'b0),.o_done(mul_done),.o_p(mul_result)
  			,.o_busy(mul_busy),.o_aux(mul_aux));

  // reg dot_start;
  // wire dot_done;
  // // use in always @(*), not infer registers (that's not what declare the signal of reg type means), 
  // // it infers a multiplexer with constant assignment 
  // reg signed [15:0] v1_x;			
	// reg signed [15:0] v1_y;
	// reg signed [15:0] v1_z;
	// reg signed [15:0] v1_w;
	// reg signed [15:0] v2_x;
	// reg signed [15:0] v2_y;
	// reg signed [15:0] v2_z;
	// reg signed [15:0] v2_w;
  // wire signed [15:0] dot_result;
  // dot4 dot (.clk (clk), .reset(reset),.start(dot_start)
  			// ,.v1_x(v1_x),.v1_y(v1_y),.v1_z(v1_z),.v1_w(v1_w)
  			// ,.v2_x(v2_x),.v2_y(v2_y),.v2_z(v2_z),.v2_w(v2_w)
  			// ,.done(dot_done),.result(dot_result));

 
 	// main FSM
	reg [7:0] fsm_state; 
	reg [4:0] read_delay;
	reg [17:0] numread;  
	reg [8:0] tri_idx;
	wire [13:0] tri_idx_addr;     		// numtri * 18 (2 xyz x 3);
  assign tri_idx_addr = {1'b0,tri_idx,4'b0000} + {4'b0,tri_idx,1'b0};
	reg signed [15:0] tri_xyz [8:0];		// read screenspace for now xy Q16.0, z Q0.16, final version is model space
	//
	reg signed [19:0] x_screen_v0;			// Q20.0, Q8.8 from file
	reg signed [19:0] x_screen_v1;
	reg signed [19:0] x_screen_v2;
	reg signed [19:0] y_screen_v0;
	reg signed [19:0] y_screen_v1;
	reg signed [19:0] y_screen_v2;
	reg signed [21:0] z_screen_v0;			// Q2.20, to match with bar, Q0.16 from file
	reg signed [21:0] z_screen_v1;
	reg signed [21:0] z_screen_v2;
	reg signed [19:0] bboxMin_X;				// Q20.0
	reg signed [19:0] bboxMin_Y;
	reg signed [19:0] bboxMax_X;
	reg signed [19:0] bboxMax_Y;
	reg [3:0] tri_color;
	//
	reg signed [19:0] e0_init;					// Q20.0
	reg signed [19:0] e1_init;
	reg signed [19:0] e2_init;
	// for compute ei_int,
  reg signed [19:0] tmp_ei_mul1;
  reg signed [19:0] tmp_ei_mul2;
  // bar_iy, bar_iz, denom
  reg signed [21:0] denom;						// Q2.20  [-1,0.999]
  reg signed [21:0] bar_ix;						// Q2.20
  reg signed [21:0] bar_ix_dx;
  reg signed [21:0] bar_ix_dy;
  reg signed [21:0] bar_iy;						// Q2.20
  reg signed [21:0] bar_iy_dx;
  reg signed [21:0] bar_iy_dy;				
  reg signed [21:0] bar_iz;
  reg signed [21:0] bar_iz_dx;
  reg signed [21:0] bar_iz_dy;
  // bar interpolate z
  reg signed [21:0] z_bar;						// Q2.20
  reg signed [21:0] z_bar_dx;
  reg signed [21:0] z_bar_dy;
  // in for loop
  reg [9:0] pixel_y;									// Q10.0
  reg [9:0] pixel_x;
  reg signed [21:0] pixel_z;					// Q2.20
  reg signed [19:0] e0;
  reg signed [19:0] e1;
  reg signed [19:0] e2;
  // 4-pixel Z, Color buffer
  reg [7:0] Z_buffer [3:0];						// Q0.8
  reg [3:0] C_buffer [3:0];						// Q4.0



    
	always @(posedge clk) begin
    if(reset) begin
    	fsm_state <= 0;
    	read_delay <= 0;
    	numread <= 0;
    	tri_idx <= 0;
    	vsfs_running <= 0;
    	// mul, div
			div_a <= 0;
			div_b <= 0;
			div_start <= 0;
			div2_a <= 0;
			div2_b <= 0;
			div2_start <= 0;
			mul_a <= 0;
			mul_b <= 0;
			mul_start <= 0;
			//
			tri_color <= 4'b1010;
			// compute e0_init
			tmp_ei_mul1 <= 0;
			tmp_ei_mul2 <= 0;
			// bar, denom
			denom <= 0;
			bar_ix <= 0;
	    bar_ix_dy <= 0;
	    bar_ix_dx <= 0;
			bar_iy <= 0;
	    bar_iy_dy <= 0;
	    bar_iy_dx <= 0;
	    bar_iz <= 0;
	    bar_iz_dy <= 0;
	    bar_iz_dx <= 0;
	    //
	    z_bar <= 0;
	    z_bar_dx <= 0;
	    z_bar_dy <= 0;
	    //
	    pixel_y <= 0;
  		pixel_x <= 0;
  		pixel_z <= 0;
  		e0 <= 0;
  		e1 <= 0;
  		e2 <= 0;
    	//
    	vsfs_addr <= 0;
    	vsfs_data_in <= 0;
    	vsfs_start_read <= 0;
    	vsfs_start_write <= 0;
    	vsfs_stop_txn <= 0;
    	do_swap <= 0;
    end else begin
			case (fsm_state)
			// wait for start_vsfs, after clear z on the 1st subframe 
				0: begin
					do_swap <= 0;
					vsfs_running <= 0;
					if (start_vsfs) begin
						tri_idx <= 0;
						vsfs_running <= 1;
						fsm_state <= 1;
					end
				end

			///////////////////////////////
			// 0. user input -> [M][MVP][M]-1, 30 states
				1: begin
						fsm_state <= 31;
				end 
			///////////////////////////////

			// 1. for each tri
				//	- READ tri
				31: begin
					if(tri_idx == numtri)begin
						if (y > 500) begin
							do_swap <= 1;
							fsm_state <= 0;

							//fsm_state <= 156;
						end
					end else begin
						if (ram_notbusy) begin
							vsfs_stop_txn <= 0;
							vsfs_start_read <= 1;
							vsfs_addr <= 24'd153600 + {10'b0,tri_idx_addr};
							numread <= 0;
							read_delay <= 0;
							fsm_state <= 32;
						end
					end
				end
				//   -- wait for the first flash data to be ready
				32: begin
					vsfs_start_read <= 0;
          if(read_delay == 16) begin
            read_delay <= 0;
            tri_xyz[numread[5:2]][{~numread[1:0],2'b00} +: 4] <= spi_data;
            numread <= 1;
            fsm_state <= 33;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
				end
				//   -- read 35 more 4bit
        33: begin
        	tri_xyz[numread[5:2]][{~numread[1:0],2'b00} +: 4] <= spi_data;
          numread <= numread + 1;
          if(numread == 35) begin
          	numread <= 0;
            vsfs_stop_txn <= 1;
            fsm_state <= 34;
          end

          // if(numread == 36)begin
          //   numread <= 0;
          //   vsfs_stop_txn <= 1;
          //   fsm_state <= 34;
          // end else begin
          //   tri_xyz[numread[5:2]][{numread[1:0],2'b00} +: 4] <= spi_data;
          //   numread <= numread + 1;
          // end
        end
        //   -- convert bit format to compute e0,bar,z
				34: begin
					vsfs_stop_txn <= 0;
					x_screen_v0 <= {4'b0000,tri_xyz[0]};
					y_screen_v0 <= {4'b0000,tri_xyz[1]};
					z_screen_v0 <= {2'b00,tri_xyz[2],4'b0000};
					x_screen_v1 <= {4'b0000,tri_xyz[3]};
					y_screen_v1 <= {4'b0000,tri_xyz[4]};
					z_screen_v1 <= {2'b00,tri_xyz[5],4'b0000};
					x_screen_v2 <= {4'b0000,tri_xyz[6]};
					y_screen_v2 <= {4'b0000,tri_xyz[7]};
					z_screen_v2 <= {2'b00,tri_xyz[8],4'b0000};
					fsm_state <= 35;
				end

			///////////////////////////////
			// 2. VS, 40 states
				35:begin
					fsm_state <= 74;
				end
			///////////////////////////////

			// 3. bbox
				// x mul of 4, floor-Min, floor-Max (will do 3 more pixels)
				74: begin
					bboxMin_X <= (x_screen_v0 < x_screen_v1)? x_screen_v0 : x_screen_v1;
					bboxMin_Y <= (y_screen_v0 < y_screen_v1)? y_screen_v0 : y_screen_v1;
					bboxMax_X <= (x_screen_v0 > x_screen_v1)? x_screen_v0 : x_screen_v1;
					bboxMax_Y <= (y_screen_v0 > y_screen_v1)? y_screen_v0 : y_screen_v1;
					fsm_state <= 75;
				end
				75: begin
					bboxMin_X <= (bboxMin_X < x_screen_v2)? bboxMin_X : x_screen_v2;
					bboxMin_Y <= (bboxMin_Y < y_screen_v2)? bboxMin_Y : y_screen_v2;
					bboxMax_X <= (bboxMax_X > x_screen_v2)? bboxMax_X : x_screen_v2;
					bboxMax_Y <= (bboxMax_Y > y_screen_v2)? bboxMax_Y : y_screen_v2;
					fsm_state <= 76;
				end
				76: begin
					bboxMin_X[1:0] <= 2'b00;
					bboxMax_X[1:0] <= 2'b00;
					fsm_state <= 77;

					// DEBUG state 150-155
					//pixel_y <= bboxMin_Y[9:0];
					//fsm_state <= 150; 
				end


			//DEBUGGING STATE 150-155: draw bbox
				// 150: begin
				// 	if (pixel_y > bboxMax_Y[9:0]) begin
				// 		tri_idx <= tri_idx + 1;
				// 		fsm_state <= 31;
				// 	end else begin
				// 		pixel_x <= bboxMin_X[9:0];
				// 		fsm_state <= 151;
				// 	end
				// end
				// 151: begin
				// 	if (pixel_x > bboxMax_X[9:0]) begin
				// 		fsm_state <= 155;
				// 	end else begin
				// 		C_buffer[0] <= tri_idx[3:0] + 1;
				// 		C_buffer[1] <= tri_idx[3:0] + 1;
				// 		C_buffer[2] <= tri_idx[3:0] + 1;
				// 		C_buffer[3] <= tri_idx[3:0] + 1;
				// 		numread <= 0;
				// 		fsm_state <= 152;
				// 	end
				// end
				// 152: begin
				// 	if (ram_notbusy) begin
				// 		vsfs_stop_txn <= 0;
				// 		vsfs_start_write <= 1;
				// 		// y * 160 + (x/2) + (offset back[0])
				// 		vsfs_addr <= (evenframe)?{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]} + 38400:
        //                         	{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]};
				// 		vsfs_data_in <= C_buffer[numread[1:0]];
				// 		numread <= numread + 1;
				// 		fsm_state <= 153;
				// 	end
				// end
				// 153: begin
				// 	vsfs_start_write <= 0;
				// 	if(spi_data_req)begin
        //     vsfs_data_in <= C_buffer[numread[1:0]];
        //     numread <= numread + 1;
        //     if(numread == 3)begin
	      //       numread <= 0;
	      //       vsfs_stop_txn <= 1;
	      //       fsm_state <= 154;
	      //     end
        //   end

        //   // if(numread == 4)begin
        //   //   numread <= 0;
        //   //   vsfs_stop_txn <= 1;
        //   //   fsm_state <= 154;
        //   // end else if(spi_data_req)begin
        //   //   vsfs_data_in <= C_buffer[numread[1:0]];
        //   //   numread <= numread + 1;
        //   // end
				// end
				// 154: begin
				// 	vsfs_stop_txn <= 0;
				// 	pixel_x <= pixel_x + 4;
				// 	fsm_state <= 151;
				// end
				// 155: begin
				// 	pixel_y <= pixel_y + 1;
				// 	fsm_state <= 150;
				// end



			// 4.1 e0_init, e1_init, e2_init 
				// e0_init = (bboxmin.x - pts[0].x)*(pts[1].y-pts[0].y) + (pts[0].y - bboxmin.y ) * (pts[1].x-pts[0].x);
    		// e1_init = (bboxmin.x - pts[1].x)*(pts[2].y-pts[1].y) + (pts[1].y - bboxmin.y ) * (pts[2].x-pts[1].x);
    		// e2_init = (bboxmin.x - pts[2].x)*(pts[0].y-pts[2].y) + (pts[2].y - bboxmin.y ) * (pts[0].x-pts[2].x);
    		// Q20.2 x Q20.2 = Q40.4->Q20.0

    		//	- e0_init
				77: begin
					mul_a <= {bboxMin_X - x_screen_v0,2'b00};				// bboxmin.x - pts[0].x
					mul_b <= {y_screen_v1 - y_screen_v0,2'b00};			// pts[1].y-pts[0].y (a0)
					mul_start <= 1;
					fsm_state <= 78;
				end
				78: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];							// ready in 23clk for 20bit mul
						mul_a <= {y_screen_v0 - bboxMin_Y,2'b00};			// pts[0].y - bboxmin.y
						mul_b <= {x_screen_v1 - x_screen_v0,2'b00};		// pts[1].x-pts[0].x (b0)
						mul_start <= 1;
						fsm_state <= 79;
					end
				end
				79: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul2 <= mul_result[23:4];							// ready in 23clk for 20bit mul
						fsm_state <= 80;
					end 
				end
				//	- e1_init
				80: begin
					e0_init <= tmp_ei_mul2 + tmp_ei_mul1;						// fin e0_init
					mul_a <= {bboxMin_X - x_screen_v1,2'b00};						
					mul_b <= {y_screen_v2 - y_screen_v1,2'b00};			
					mul_start <= 1;
					fsm_state <= 81;
				end
				81: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];		
						mul_a <= {y_screen_v1 - bboxMin_Y,2'b00};					
						mul_b <= {x_screen_v2 - x_screen_v1,2'b00};	
						mul_start <= 1;
						fsm_state <= 82;
					end
				end
				82: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul2 <= mul_result[23:4];						// ready in 23clk for 20bit mul
						fsm_state <= 83;
					end
				end
				//	- e2_init
				83: begin
					e1_init <= tmp_ei_mul2 + tmp_ei_mul1;					// fin e1_init
					mul_a <= {bboxMin_X - x_screen_v2,2'b00};						
					mul_b <= {y_screen_v0 - y_screen_v2,2'b00};			
					mul_start <= 1;
					fsm_state <= 84;
				end
				84: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];		
						mul_a <= {y_screen_v2 - bboxMin_Y,2'b00};					
						mul_b <= {x_screen_v0 - x_screen_v2,2'b00};		
						mul_start <= 1;
						fsm_state <= 85;
					end
				end
				85: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul2 <= mul_result[23:4];		
						fsm_state <= 86;
					end 
				end
				86: begin
					e2_init <= tmp_ei_mul2 + tmp_ei_mul1;				// fin e2_init
					fsm_state <= 87;
				end

			// 4.2 denom
				// Q20.0 denom_i = (y1-y2)(x0-x2)+(x2-x1)(y0-y2)
				// Q2.20 denom = float2fix14(1.0f/denom_i);
				// 640x640 x2 = 819,200      2^20
				// 1/819,200 = 0.00000122,   1/2^20   
				
				87: begin
						mul_a <= {y_screen_v1 - y_screen_v2,2'b00};			
						mul_b <= {x_screen_v0 - x_screen_v2,2'b00};	
						mul_start <= 1;
						fsm_state <= 88;
					end
				88: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];		// ready in 23clk for 20bit mul
						mul_a <= {y_screen_v0 - y_screen_v2,2'b00};		
						mul_b <= {x_screen_v2 - x_screen_v1,2'b00};	
						mul_start <= 1;
						fsm_state <= 89;
					end
				end
				89: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul2 <= tmp_ei_mul1 + mul_result[23:4];		// denom_i
						fsm_state <= 90;
					end 
				end
				90: begin
					// Q20.0->Q20.20 -> Q20.20/Q20.20 
					div2_a <= { 20'b0000_0000_0000_0000_0001, 20'b0000_0000_0000_0000_0000};	// 1.0f/denom_i
					div2_b <= { tmp_ei_mul2, 20'b0000_0000_0000_0000_0000};
					div2_start <= 1;
					fsm_state <= 91;
				end
				91: begin
					div2_start <= 0;
					if (div2_done) begin
						// Q20.20->Q2.20
						denom <= {div2_result[21:0]};
						fsm_state <= 92;
					end
				end

			//4.3 bar_ix, bar_iy, bar_iz  
				// 	Q2.20 = (Q20.0->)Q20.2 * Q2.20
				//	      = 500,000 * 0.000002 = 1
				//        = 10,000 * 0.000002 = 0.02
				//	Q2.20 = 500 * 0.000002 = 0.001, 2^10
				//		  = 100 * 0.000002 = 0.0002, 2^16 = 0.000015
				//		
				//		bar_iy <= {(y2-y0)(bboxMin_X-x2)+(x0-x2)(bboxMin_Y-y2)} * denom;	// 3 mul
				//		bar_iy_dy <= x0x2 * denom;		// 1 mul
				//		bar_iy_dx <= y2y0 * denom;		// 1 mul
				//		bar_iz <= {(y0-y1)(bboxMin_X-x0)+(x1-x0)(bboxMin_Y-y0)} * denom;	// 3 mul
				//		bar_iz_dy <= x1x0 * denom;		// 1 mul
				//		bar_iz_dx <= y0y1 * denom;		// 1 mul
				//		bar_ix <= 1 - (bar_iy + bar_iz)
				//		bar_ix <= {(y1-y2)(bboxMin_X-x2)+(x2-x1)(bboxMin_Y-y2)} * denom;
				//		bar_ix_dy <= x2x1 * denom;
				//		bar_ix_dx <= y1y2 * denom;

				92: begin
					mul_a <= {y_screen_v2 - y_screen_v0,2'b00};						
					mul_b <= {bboxMin_X - x_screen_v2,2'b00};			
					mul_start <= 1;
					fsm_state <= 93;
				end
				93: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];						// ready in 23clk for 20bit mul
						mul_a <= {x_screen_v0 - x_screen_v2,2'b00};		
						mul_b <= {bboxMin_Y - y_screen_v2,2'b00};					
						mul_start <= 1;
						fsm_state <= 94;
					end
				end
				94: begin
					mul_start <= 0;
					if (mul_done) begin
						mul_a <= {tmp_ei_mul1 + mul_result[23:4],2'b00};		// ready in 23clk for 20bit mul
						mul_b <= denom;							
						mul_start <= 1;
						fsm_state <= 95;
					end
				end
				95: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iy <= mul_result[23:2];								// ready in 23clk for 20bit mul
						//
						mul_a <= {x_screen_v0 - x_screen_v2,2'b00};		
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 96;
					end
				end
				96: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iy_dy <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						mul_a <= {y_screen_v2 - y_screen_v0,2'b00};			
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 97;
					end
				end
				97: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iy_dx <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						mul_a <= {y_screen_v0 - y_screen_v1,2'b00};						
						mul_b <= {bboxMin_X - x_screen_v0,2'b00};				
						mul_start <= 1;
						fsm_state <= 98;
					end
				end
				98: begin
					mul_start <= 0;
					if (mul_done) begin
						tmp_ei_mul1 <= mul_result[23:4];						// ready in 23clk for 20bit mul
						mul_a <= {x_screen_v1 - x_screen_v0,2'b00};		
						mul_b <= {bboxMin_Y - y_screen_v0,2'b00};					
						mul_start <= 1;
						fsm_state <= 99;
					end
				end
				99: begin
					mul_start <= 0;
					if (mul_done) begin
						mul_a <= {tmp_ei_mul1 + mul_result[23:4],2'b00};		// ready in 23clk for 20bit mul
						mul_b <= denom;							
						mul_start <= 1;
						fsm_state <= 100;
					end
				end
				100: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iz <= mul_result[23:2];								// ready in 23clk for 20bit mul
						//
						mul_a <= {x_screen_v1 - x_screen_v0,2'b00};		
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 101;
					end
				end
				101: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iz_dy <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						bar_ix <= bar_iy + bar_iz;
						//
						mul_a <= {y_screen_v0 - y_screen_v1,2'b00};			
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 102;
					end
				end
				102: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_iz_dx <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						bar_ix <= 22'b01_0000_0000_0000_0000_0000 - bar_ix;
						//
						mul_a <= {x_screen_v2 - x_screen_v1,2'b00};		
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 103;
					end
				end
				103: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_ix_dy <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						mul_a <= {y_screen_v1 - y_screen_v2,2'b00};			
						mul_b <= denom;					
						mul_start <= 1;
						fsm_state <= 104;
					end
				end
				104: begin
					mul_start <= 0;
					if (mul_done) begin
						bar_ix_dx <= mul_result[23:2];							// ready in 23clk for 20bit mul
						//
						fsm_state <= 105;
					end
				end

			//4.4 Z_bar
				// 		z_bar = z_screen_v0*bar_ix + z_screen_v1*bar_iy + z_screen_v2*bar_iz
				// 		z_bar_dx = z_screen_v0*bar_ix_dx + z_screen_v1*bar_iy_dx + z_screen_v2*bar_iz_dx
				// 		z_bar_dy = z_screen_v0*bar_ix_dy + z_screen_v1*bar_iy_dy + z_screen_v2*bar_iz_dy
				//		Q2.20 * Q2.20 = Q4.40 -> Q2.20
				105: begin
					mul_a <= z_screen_v0;						
					mul_b <= bar_ix;			
					mul_start <= 1;
					fsm_state <= 106;
				end
				106: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar <= mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v1;		
						mul_b <= bar_iy;					
						mul_start <= 1;
						fsm_state <= 107;
					end
				end
				107: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar <= z_bar + mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v2;		
						mul_b <= bar_iz;					
						mul_start <= 1;
						fsm_state <= 108;
					end
				end
				108: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar <= z_bar + mul_result[41:20];						// ready in 23clk for 20bit mul
						//
						mul_a <= z_screen_v0;	;		
						mul_b <= bar_ix_dx;					
						mul_start <= 1;
						fsm_state <= 109;
					end
				end
				109: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dx <= mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v1;		
						mul_b <= bar_iy_dx;					
						mul_start <= 1;
						fsm_state <= 110;
					end
				end
				110: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dx <= z_bar_dx + mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v2;		
						mul_b <= bar_iz_dx;					
						mul_start <= 1;
						fsm_state <= 111;
					end
				end
				111: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dx <= z_bar_dx + mul_result[41:20];						// ready in 23clk for 20bit mul
						//
						mul_a <= z_screen_v0;	;		
						mul_b <= bar_ix_dy;					
						mul_start <= 1;
						fsm_state <= 112;
					end
				end
				112: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dy <= mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v1;		
						mul_b <= bar_iy_dy;					
						mul_start <= 1;
						fsm_state <= 113;
					end
				end
				113: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dy <= z_bar_dy + mul_result[41:20];						// ready in 23clk for 20bit mul
						mul_a <= z_screen_v2;		
						mul_b <= bar_iz_dy;					
						mul_start <= 1;
						fsm_state <= 114;
					end
				end
				114: begin
					mul_start <= 0;
					if (mul_done) begin
						z_bar_dy <= z_bar_dy + mul_result[41:20];						// ready in 23clk for 20bit mul
						//
						pixel_y <= bboxMin_Y[9:0];
						fsm_state <= 115;
					end
				end
			
			//5. for pixel y in bbox
				// - e0 = e0_init, z = z_bar / e0_init += dy, z_bar += z_bar_dy
				115: begin
					if (pixel_y > bboxMax_Y[9:0]) begin
						tri_idx <= tri_idx + 1;
						fsm_state <= 31;
					end else begin
						e0 <= e0_init;
						e1 <= e1_init;
						e2 <= e2_init;
						pixel_z <= z_bar;
						//
						e0_init <= e0_init - (x_screen_v1 - x_screen_v0);
						e1_init <= e1_init - (x_screen_v2 - x_screen_v1);
						e2_init <= e2_init - (x_screen_v0 - x_screen_v2);
						z_bar <= z_bar + z_bar_dy;
						//
						pixel_x <= bboxMin_X[9:0];
						fsm_state <= 116;
					end
				end
				// - for x in bbox (x < bboxMax_X)
				//	- READ x4 Z (23 clk)
				116: begin
					if (pixel_x > bboxMax_X[9:0]) begin
						fsm_state <= 255;
					end else begin
						if (ram_notbusy) begin
							vsfs_stop_txn <= 0;
							vsfs_start_read <= 1;
							// y*320 + x + z_start
							vsfs_addr <= {6'b0,pixel_y,8'b0} + {8'b0,pixel_y,6'b0} + {14'b0,pixel_x} + 76800;
							numread <= 0;
							read_delay <= 0;
							fsm_state <= 117;
						end
					end
				end
				//   -- wait for the first flash data to be ready
				117: begin
					vsfs_start_read <= 0;
          if(read_delay == 16) begin
            read_delay <= 0;
            Z_buffer[numread[2:1]][{~numread[0],2'b00} +: 4] <= spi_data;
            numread <= 1;
            fsm_state <= 118;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
				end
				//   -- read 7 more 4bit
				118: begin
					Z_buffer[numread[2:1]][{~numread[0],2'b00} +: 4] <= spi_data;
          numread <= numread + 1;
          if(numread == 7)begin
            numread <= 0;
            vsfs_stop_txn <= 1;
            fsm_state <= 119;
          end
            
					// if(numread == 8)begin
          //   numread <= 0;
          //   vsfs_stop_txn <= 1;
          //   fsm_state <= 119;
          // end else begin
          //   Z_buffer[numread[2:1]][{numread[0],2'b00} +: 4] <= spi_data;
          //   numread <= numread + 1;
          // end
				end
				//	- READ x4 B (19 clk)
				119: begin
					vsfs_stop_txn <= 0;
					vsfs_start_read <= 1;
					// y * 160 + x/2 + (offset back[0])
					vsfs_addr <= (evenframe)?{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]} + 38400:
                                	{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]};
					numread <= 0;
					read_delay <= 0;
					fsm_state <= 120;
				end
				//   -- wait for the first flash data to be ready
				120: begin
					vsfs_start_read <= 0;
          if(read_delay == 16) begin
            read_delay <= 0;
            C_buffer[numread[1:0]] <= spi_data;
            numread <= 1;
            fsm_state <= 121;
          end 
          else begin
            read_delay <= read_delay + 1;
          end
				end
				//   -- read 3 more 4bit
				121: begin
					C_buffer[numread[1:0]] <= spi_data;
          numread <= numread + 1;
          if(numread == 3)begin
            numread <= 0;
            vsfs_stop_txn <= 1;
            fsm_state <= 122;
          end

					// if(numread == 4)begin
          //   numread <= 0;
          //   vsfs_stop_txn <= 1;
          //   fsm_state <= 122;
          // end else begin
          //   C_buffer[numread[1:0]] <= spi_data;
          //   numread <= numread + 1;
          // end
				end
				// x4 pixel
				//	- if ((e0 > 0) && (e1 > 0) && (e2 > 0)) / e0 += dx, z += z_bar_dx
				//		- pixel[0-4].cz = (Z < Zbuffer)? cz: pixel[0-4].cz
				//	screen y, 0 at the top (reverse opengl), use e0 > 0
				122: begin
					vsfs_stop_txn <= 0;
					if ((e0 > 0) & (e1 > 0) & (e2 > 0)) begin
					//if (((e0 > 0) & (e1 > 0) & (e2 > 0)) | ((e0 < 0) & (e1 < 0) & (e2 < 0))) begin
						C_buffer[0] <= (pixel_z[19:12] < Z_buffer[0])? tri_idx[3:0]+1 : C_buffer[0]; 
						Z_buffer[0] <= (pixel_z[19:12] < Z_buffer[0])? pixel_z[19:12] : Z_buffer[0]; 
						//C_buffer[0] <= tri_idx[3:0]+1;
					end 
					e0 <= e0 + (y_screen_v1 - y_screen_v0);
					e1 <= e1 + (y_screen_v2 - y_screen_v1);
					e2 <= e2 + (y_screen_v0 - y_screen_v2);
					pixel_z <= pixel_z + z_bar_dx;
					fsm_state <= 123;
				end
				123: begin
					if ((e0 > 0) & (e1 > 0) & (e2 > 0)) begin
					// if (((e0 > 0) & (e1 > 0) & (e2 > 0)) | ((e0 < 0) & (e1 < 0) & (e2 < 0))) begin
						C_buffer[1] <= (pixel_z[19:12] < Z_buffer[1])? tri_idx[3:0] : C_buffer[1]; 
						Z_buffer[1] <= (pixel_z[19:12] < Z_buffer[1])? pixel_z[19:12] : Z_buffer[1]; 
						// C_buffer[1] <= tri_idx[3:0]; 
					end 
					e0 <= e0 + (y_screen_v1 - y_screen_v0);
					e1 <= e1 + (y_screen_v2 - y_screen_v1);
					e2 <= e2 + (y_screen_v0 - y_screen_v2);
					pixel_z <= pixel_z + z_bar_dx;
					fsm_state <= 124;
				end
				124: begin
					if ((e0 > 0) & (e1 > 0) & (e2 > 0)) begin
					// if (((e0 > 0) & (e1 > 0) & (e2 > 0)) | ((e0 < 0) & (e1 < 0) & (e2 < 0))) begin
						C_buffer[2] <= (pixel_z[19:12] < Z_buffer[2])? tri_idx[3:0]+1 : C_buffer[2]; 
						Z_buffer[2] <= (pixel_z[19:12] < Z_buffer[2])? pixel_z[19:12] : Z_buffer[2]; 
						// C_buffer[2] <= tri_idx[3:0]+1;
					end 
					e0 <= e0 + (y_screen_v1 - y_screen_v0);
					e1 <= e1 + (y_screen_v2 - y_screen_v1);
					e2 <= e2 + (y_screen_v0 - y_screen_v2);
					pixel_z <= pixel_z + z_bar_dx;
					fsm_state <= 125;
				end
				125: begin
					if ((e0 > 0) & (e1 > 0) & (e2 > 0)) begin
					// if (((e0 > 0) & (e1 > 0) & (e2 > 0)) | ((e0 < 0) & (e1 < 0) & (e2 < 0))) begin
						C_buffer[3] <= (pixel_z[19:12] < Z_buffer[3])? tri_idx[3:0] : C_buffer[3]; 
						Z_buffer[3] <= (pixel_z[19:12] < Z_buffer[3])? pixel_z[19:12] : Z_buffer[3]; 
						// C_buffer[3] <= tri_idx[3:0];
					end 
					e0 <= e0 + (y_screen_v1 - y_screen_v0);
					e1 <= e1 + (y_screen_v2 - y_screen_v1);
					e2 <= e2 + (y_screen_v0 - y_screen_v2);
					pixel_z <= pixel_z + z_bar_dx;
					fsm_state <= 126;
				end
				// - WRITE x4 Z (16 clk) 
				126: begin
					if (ram_notbusy) begin
						vsfs_stop_txn <= 0;
						vsfs_start_write <= 1;
						// y*320 + x + z_start
						vsfs_addr <= {6'b0,pixel_y,8'b0} + {8'b0,pixel_y,6'b0} + {14'b0,pixel_x} + 76800;
						vsfs_data_in <= Z_buffer[numread[2:1]][{~numread[0],2'b00} +: 4];
						numread <= numread + 1;
						fsm_state <= 127;
					end
				end
				//   -- wait & write 4 Z
				127: begin
					vsfs_start_write <= 0;
					if(spi_data_req)begin
            vsfs_data_in <= Z_buffer[numread[2:1]][{~numread[0],2'b00} +: 4];
            numread <= numread + 1;
            if(numread == 7)begin
	            numread <= 0;
	            vsfs_stop_txn <= 1;
	            fsm_state <= 128;
	          end
          end

          // if(numread == 8)begin
          //   numread <= 0;
          //   vsfs_stop_txn <= 1;
          //   fsm_state <= 128;
          // end else if(spi_data_req)begin
          //   vsfs_data_in <= Z_buffer[numread[2:1]][{numread[0],2'b00} +: 4];
          //   numread <= numread + 1;
          // end
				end
				// - WRITE x4 B (12 clk) 
				128: begin
					vsfs_stop_txn <= 0;
					vsfs_start_write <= 1;
					// y * 160 + x/2 + (offset back[0])
					vsfs_addr <= (evenframe)?{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]} + 38400:
                              	{7'b0,pixel_y,7'b0} + {9'b0,pixel_y,5'b0} + {15'b0,pixel_x[9:1]};
					vsfs_data_in <= C_buffer[numread[1:0]];
					numread <= numread + 1;
					fsm_state <= 129;
				end
				//   -- wait wait & write 4 color
				129: begin
					vsfs_start_write <= 0;
					if(spi_data_req)begin
            vsfs_data_in <= C_buffer[numread[1:0]];
            numread <= numread + 1;
            if(numread == 3)begin
	            numread <= 0;
	            vsfs_stop_txn <= 1;
	            fsm_state <= 254;
	          end
          end

          // if(numread == 4)begin
          //   numread <= 0;
          //   vsfs_stop_txn <= 1;
          //   fsm_state <= 254;
          // end else if(spi_data_req)begin
          //   vsfs_data_in <= C_buffer[numread[1:0]];
          //   numread <= numread + 1;
          // end
				end



				// - last line for pixel x
				254: begin
					vsfs_stop_txn <= 0;
					pixel_x <= pixel_x + 4;
					fsm_state <= 116;
				end
				// - last line for pixel y, max 255 state
				255: begin
					pixel_y <= pixel_y + 1;
					fsm_state <= 115;
				end

				default: begin
					fsm_state <= 0;
				end
			endcase
    end
  end



  // debug
  // assign debug_vsfs_fsm_state = fsm_state;
  // assign debug_x_screen_v0 = x_screen_v0;
  // assign debug_x_screen_v1 = x_screen_v1;
  // assign debug_x_screen_v2 = x_screen_v2;
  // assign debug_y_screen_v0 = y_screen_v0;
  // assign debug_y_screen_v1 = y_screen_v1;
  // assign debug_y_screen_v2 = y_screen_v2;
  // assign debug_z_screen_v0 = z_screen_v0;
  // assign debug_z_screen_v1 = z_screen_v1;
  // assign debug_z_screen_v2 = z_screen_v2;

  
endmodule