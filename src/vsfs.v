
/* Matt pongsagon

  - addr Flash & RAM front start at 0
  - addr RAM back start at 38400 (320x240=76800/2, 2pixel:1byte)
  - addr RAM z start at 76800 (1pixel:1byte)
  	- front 4bit, back 4bit, z 8bit
		- cal z in Q2.20 save in Q0.8 [0,1]
  - addr Tri start at 153600

  color
  	0: black
  	1: blue
  	2: dark green
		15:
 
  VSFS
		//0. perframe, state [150-250]
			- Tz, rotx2 -> [M] ([Tz*Rx*Ry]) // cos table, set premul mat manually 
			- dir light rot 1axis -> update formula
			- [M]*[VP] 			// no cam fix [VP]
			- [M]-1 ([Ryt*Rxt*Tz-1]) // override [M], for light-world, campos-world -> model 
			- [M-1]*campos 
			- [M-1]*lightpos 

    1. for each tri, state [31-129, 254,255]
    	- READ tri (pos.xyz x3 Q8.8, face normal Q2.8 x3, color 2bit)

    //2. VS
			- x1: backface culling (view-model dot faceNormal-model)
			- x3: [MVP]*v, clip->NDC (div), NDC->screen
			- x1: light-model dot faceNormal-model

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
  	input		wire [9:0]	numtri,
  	input   wire 				evenframe,

  	//+ gamepad input
  	

    // things to watch
    output  wire [15:0] debug_x_model_v0,
    output  wire [15:0] debug_x_model_v1,
    output  wire [15:0] debug_x_model_v2,
    output  wire [15:0] debug_y_model_v0,
    output  wire [15:0] debug_y_model_v1,
    output  wire [15:0] debug_y_model_v2,
    output  wire [15:0] debug_z_model_v0,
    output  wire [15:0] debug_z_model_v1,
    output  wire [15:0] debug_z_model_v2,
    output  wire [15:0] debug_nx,
    output  wire [15:0] debug_ny,
    output  wire [15:0] debug_nz,
    output  wire [1:0] debug_tri_color,
    output  wire [7:0]  debug_vsfs_fsm_state,

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

  reg dot_start;
  wire dot_done;
  // use in always @(*), not infer registers (that's not what declare the signal of reg type means), 
  // it infers a multiplexer with constant assignment 
  reg signed [15:0] v1_x;			
	reg signed [15:0] v1_y;
	reg signed [15:0] v1_z;
	reg signed [15:0] v1_w;
	reg signed [15:0] v2_x;
	reg signed [15:0] v2_y;
	reg signed [15:0] v2_z;
	reg signed [15:0] v2_w;
  wire signed [15:0] dot_result;
  dot4 dot (.clk (clk), .reset(reset),.start(dot_start)
  			,.v1_x(v1_x),.v1_y(v1_y),.v1_z(v1_z),.v1_w(v1_w)
  			,.v2_x(v2_x),.v2_y(v2_y),.v2_z(v2_z),.v2_w(v2_w)
  			,.done(dot_done),.result(dot_result));

 
 	// main FSM
	reg [7:0] fsm_state; 
	// [M]/[M-1] (use the same reg), [MVP] (row major)
	reg signed [15:0] M_00;					// Q8.8
	reg signed [15:0] M_01;
	reg signed [15:0] M_02;
	reg signed [15:0] M_03;
	reg signed [15:0] M_10;					
	reg signed [15:0] M_11;
	reg signed [15:0] M_12;
	reg signed [15:0] M_13;
	reg signed [15:0] M_20;					
	reg signed [15:0] M_21;
	reg signed [15:0] M_22;
	reg signed [15:0] M_23;
	reg signed [15:0] M_30;					
	reg signed [15:0] M_31;
	reg signed [15:0] M_32;
	reg signed [15:0] M_33;
	reg signed [15:0] MVP_00;					// Q8.8
	reg signed [15:0] MVP_01;
	reg signed [15:0] MVP_02;
	reg signed [15:0] MVP_03;
	reg signed [15:0] MVP_10;					
	reg signed [15:0] MVP_11;
	reg signed [15:0] MVP_12;
	reg signed [15:0] MVP_13;
	reg signed [15:0] MVP_20;					
	reg signed [15:0] MVP_21;
	reg signed [15:0] MVP_22;
	reg signed [15:0] MVP_23;
	reg signed [15:0] MVP_30;					
	reg signed [15:0] MVP_31;
	reg signed [15:0] MVP_32;
	reg signed [15:0] MVP_33;
	// read tri from RAM
	reg [4:0] read_delay;
	reg [17:0] numread;  
	reg [9:0] tri_idx;									// max 1024 tri
	wire [14:0] tri_idx_addr;     			// in byte: numtri * 22 (2 x 3xyz x 3vert + 4 (normal/color));
  assign tri_idx_addr = {1'b0,tri_idx,4'b0000} + {3'b0,tri_idx,2'b0} + {4'b0,tri_idx,1'b0};
	reg signed [15:0] tri_xyz [10:0];		// 
	// model space/NDC (use the same reg)
	reg signed [15:0] x_model_v0;				// Q8.8 from file
	reg signed [15:0] x_model_v1;
	reg signed [15:0] x_model_v2;
	reg signed [15:0] y_model_v0;				
	reg signed [15:0] y_model_v1;
	reg signed [15:0] y_model_v2;
	reg signed [15:0] z_model_v0;				
	reg signed [15:0] z_model_v1;
	reg signed [15:0] z_model_v2;
	reg signed [15:0] x_clip_v0;				// Q8.8
	reg signed [15:0] x_clip_v1;
	reg signed [15:0] x_clip_v2;
	reg signed [15:0] y_clip_v0;
	reg signed [15:0] y_clip_v1;
	reg signed [15:0] y_clip_v2;
	reg signed [15:0] z_clip_v0;
	reg signed [15:0] z_clip_v1;
	reg signed [15:0] z_clip_v2;
	reg signed [15:0] w_clip_v0;
	reg signed [15:0] w_clip_v1;
	reg signed [15:0] w_clip_v2;
	reg signed [15:0] campos_x;					// Q8.8
	reg signed [15:0] campos_y;
	reg signed [15:0] campos_z;
	reg signed [15:0] nx;								// Q8.8 <- Q2.8 from file
	reg signed [15:0] ny;
	reg signed [15:0] nz;	
	reg signed [15:0] light_x;					// Q8.8
	reg signed [15:0] light_y;
	reg signed [15:0] light_z;	
	reg signed [15:0] viewdir_x;				// Q8.8
	reg signed [15:0] viewdir_y;
	reg signed [15:0] viewdir_z;	
	reg [1:0] tri_color;
	// screenspace
	reg signed [19:0] x_screen_v0;			// Q20.0
	reg signed [19:0] x_screen_v1;
	reg signed [19:0] x_screen_v2;
	reg signed [19:0] y_screen_v0;
	reg signed [19:0] y_screen_v1;
	reg signed [19:0] y_screen_v2;
	reg signed [21:0] z_screen_v0;			// Q2.20, to match with bar
	reg signed [21:0] z_screen_v1;
	reg signed [21:0] z_screen_v2;
	reg signed [19:0] bboxMin_X;				// Q20.0
	reg signed [19:0] bboxMin_Y;
	reg signed [19:0] bboxMax_X;
	reg signed [19:0] bboxMax_Y;
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



  //+ for setting wire input to dot4 module
  always @(*)begin
  	case (fsm_state)
  		// backface culling
  		36: begin
  			v1_x = nx;
				v1_y = ny;
				v1_z = nz;
				v1_w = 16'sb0000_0000_0000_0000;
				v2_x = campos_x;
				v2_y = campos_y;
				v2_z = campos_z;
				v2_w = 16'sb0000_0000_0000_0000;
  		end
  		// [MVP]*v
  		38: begin
  			v1_x = x_model_v0;
				v1_y = y_model_v0;
				v1_z = z_model_v0;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_00;
				v2_y = MVP_01;
				v2_z = MVP_02;
				v2_w = MVP_03;
  		end
  		39: begin
  			v1_x = x_model_v0;
				v1_y = y_model_v0;
				v1_z = z_model_v0;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_10;
				v2_y = MVP_11;
				v2_z = MVP_12;
				v2_w = MVP_13;
  		end
  		40: begin
  			v1_x = x_model_v0;
				v1_y = y_model_v0;
				v1_z = z_model_v0;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_20;
				v2_y = MVP_21;
				v2_z = MVP_22;
				v2_w = MVP_23;
  		end
  		41: begin
  			v1_x = x_model_v0;
				v1_y = y_model_v0;
				v1_z = z_model_v0;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_30;
				v2_y = MVP_31;
				v2_z = MVP_32;
				v2_w = MVP_33;
  		end
  		42: begin
  			v1_x = x_model_v1;
				v1_y = y_model_v1;
				v1_z = z_model_v1;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_00;
				v2_y = MVP_01;
				v2_z = MVP_02;
				v2_w = MVP_03;
  		end
  		43: begin
  			v1_x = x_model_v1;
				v1_y = y_model_v1;
				v1_z = z_model_v1;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_10;
				v2_y = MVP_11;
				v2_z = MVP_12;
				v2_w = MVP_13;
  		end
  		44: begin
  			v1_x = x_model_v1;
				v1_y = y_model_v1;
				v1_z = z_model_v1;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_20;
				v2_y = MVP_21;
				v2_z = MVP_22;
				v2_w = MVP_23;
  		end
  		45: begin
  			v1_x = x_model_v1;
				v1_y = y_model_v1;
				v1_z = z_model_v1;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_30;
				v2_y = MVP_31;
				v2_z = MVP_32;
				v2_w = MVP_33;
  		end
  		46: begin
  			v1_x = x_model_v2;
				v1_y = y_model_v2;
				v1_z = z_model_v2;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_00;
				v2_y = MVP_01;
				v2_z = MVP_02;
				v2_w = MVP_03;
  		end
  		47: begin
  			v1_x = x_model_v2;
				v1_y = y_model_v2;
				v1_z = z_model_v2;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_10;
				v2_y = MVP_11;
				v2_z = MVP_12;
				v2_w = MVP_13;
  		end
  		48: begin
  			v1_x = x_model_v2;
				v1_y = y_model_v2;
				v1_z = z_model_v2;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_20;
				v2_y = MVP_21;
				v2_z = MVP_22;
				v2_w = MVP_23;
  		end
  		49: begin
  			v1_x = x_model_v2;
				v1_y = y_model_v2;
				v1_z = z_model_v2;
				v1_w = 16'sb0000_0001_0000_0000;
				v2_x = MVP_30;
				v2_y = MVP_31;
				v2_z = MVP_32;
				v2_w = MVP_33;
  		end
  		// state 191-206 for [M*VP]
  		191: begin
  			v1_x = M_00;
				v1_y = M_10;
				v1_z = M_20;
				v1_w = M_30;
				v2_x = 16'sh020f;
				v2_y = 0;
				v2_z = 0;
				v2_w = 0;
  		end
  		192: begin
  			v1_x = M_01;
				v1_y = M_11;
				v1_z = M_21;
				v1_w = M_31;
				v2_x = 16'sh020f;
				v2_y = 0;
				v2_z = 0;
				v2_w = 0;
  		end
  		193: begin
  			v1_x = M_02;
				v1_y = M_12;
				v1_z = M_22;
				v1_w = M_32;
				v2_x = 16'sh020f;
				v2_y = 0;
				v2_z = 0;
				v2_w = 0;
  		end
  		194: begin
  			v1_x = M_03;
				v1_y = M_13;
				v1_z = M_23;
				v1_w = M_33;
				v2_x = 16'sh020f;
				v2_y = 0;
				v2_z = 0;
				v2_w = 0;
  		end
  		195: begin
  			v1_x = M_00;
				v1_y = M_10;
				v1_z = M_20;
				v1_w = M_30;
				v2_x = 0;
				v2_y = 16'sh02c0;
				v2_z = 0;
				v2_w = 0;
  		end
  		196: begin
  			v1_x = M_01;
				v1_y = M_11;
				v1_z = M_21;
				v1_w = M_31;
				v2_x = 0;
				v2_y = 16'sh02c0;
				v2_z = 0;
				v2_w = 0;
  		end
  		197: begin
  			v1_x = M_02;
				v1_y = M_12;
				v1_z = M_22;
				v1_w = M_32;
				v2_x = 0;
				v2_y = 16'sh02c0;
				v2_z = 0;
				v2_w = 0;
  		end
  		198: begin
  			v1_x = M_03;
				v1_y = M_13;
				v1_z = M_23;
				v1_w = M_33;
				v2_x = 0;
				v2_y = 16'sh02c0;
				v2_z = 0;
				v2_w = 0;
  		end
  		199: begin
  			v1_x = M_00;
				v1_y = M_10;
				v1_z = M_20;
				v1_w = M_30;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shfec9;
				v2_w = 16'sh1a57;
  		end
  		200: begin
  			v1_x = M_01;
				v1_y = M_11;
				v1_z = M_21;
				v1_w = M_31;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shfec9;
				v2_w = 16'sh1a57;
  		end
  		201: begin
  			v1_x = M_02;
				v1_y = M_12;
				v1_z = M_22;
				v1_w = M_32;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shfec9;
				v2_w = 16'sh1a57;
  		end
  		202: begin
  			v1_x = M_03;
				v1_y = M_13;
				v1_z = M_23;
				v1_w = M_33;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shfec9;
				v2_w = 16'sh1a57;
  		end
  		203: begin
  			v1_x = M_00;
				v1_y = M_10;
				v1_z = M_20;
				v1_w = M_30;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shff01;
				v2_w = 16'sh27d8;
  		end
  		204: begin
  			v1_x = M_01;
				v1_y = M_11;
				v1_z = M_21;
				v1_w = M_31;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shff01;
				v2_w = 16'sh27d8;
  		end
  		205: begin
  			v1_x = M_02;
				v1_y = M_12;
				v1_z = M_22;
				v1_w = M_32;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shff01;
				v2_w = 16'sh27d8;
  		end
  		206: begin
  			v1_x = M_03;
				v1_y = M_13;
				v1_z = M_23;
				v1_w = M_33;
				v2_x = 0;
				v2_y = 0;
				v2_z = 16'shff01;
				v2_w = 16'sh27d8;
  		end


  		default: begin
  			v1_x = 0;
				v1_y = 0;
				v1_z = 0;
				v1_w = 0;
				v2_x = 0;
				v2_y = 0;
				v2_z = 0;
				v2_w = 0;
  		end
  	endcase
  end



	always @(posedge clk) begin
    if(reset) begin
    	fsm_state <= 0;
    	//
    	vsfs_addr <= 0;
    	vsfs_data_in <= 0;
    	vsfs_start_read <= 0;
    	vsfs_start_write <= 0;
    	vsfs_stop_txn <= 0;
    	do_swap <= 0;
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
			dot_start <= 0;
    	//
			M_00 <= 0;					// Q8.8
			M_01 <= 0;
			M_02 <= 0;
			M_03 <= 0;
			M_10 <= 0;					
			M_11 <= 0;
			M_12 <= 0;
			M_13 <= 0;
			M_20 <= 0;					
			M_21 <= 0;
			M_22 <= 0;
			M_23 <= 0;
			M_30 <= 0;					
			M_31 <= 0;
			M_32 <= 0;
			M_33 <= 0;
			MVP_00 <= 0;					// Q8.8
			MVP_01 <= 0;
			MVP_02 <= 0;
			MVP_03 <= 0;
			MVP_10 <= 0;					
			MVP_11 <= 0;
			MVP_12 <= 0;
			MVP_13 <= 0;
			MVP_20 <= 0;					
			MVP_21 <= 0;
			MVP_22 <= 0;
			MVP_23 <= 0;
			MVP_30 <= 0;					
			MVP_31 <= 0;
			MVP_32 <= 0;
			MVP_33 <= 0;
			//
    	read_delay <= 0;
    	numread <= 0;
    	tri_idx <= 0;
    	// tri_xyz[10:0]
    	//
    	x_model_v0 <= 0;				// Q8.8 from file
			x_model_v1 <= 0;
			x_model_v2 <= 0;
			y_model_v0 <= 0;				
			y_model_v1 <= 0;
			y_model_v2 <= 0;
			z_model_v0 <= 0;				
			z_model_v1 <= 0;
			z_model_v2 <= 0;
			x_clip_v0 <= 0;
			x_clip_v1 <= 0;
			x_clip_v2 <= 0;
			y_clip_v0 <= 0;
			y_clip_v1 <= 0;
			y_clip_v2 <= 0;
			z_clip_v0 <= 0;
			z_clip_v1 <= 0;
			z_clip_v2 <= 0;
			w_clip_v0 <= 0;
			w_clip_v1 <= 0;
			w_clip_v2 <= 0;
			campos_x <= 0;								
			campos_y <= 0;
			campos_z <= 16'sb0010_1000_0000_0000;	
			nx <= 0;								
			ny <= 0;
			nz <= 0;	
			light_x <= 0;								
			light_y <= 0;
			light_z <= 0;	
			viewdir_x <= 0;								
			viewdir_y <= 0;
			viewdir_z <= 0;	
			tri_color <= 0;
			x_screen_v0 <= 0;			
			x_screen_v1 <= 0;
			x_screen_v2 <= 0;
			y_screen_v0 <= 0;
			y_screen_v1 <= 0;
			y_screen_v2 <= 0;
			z_screen_v0 <= 0;			
			z_screen_v1 <= 0;
			z_screen_v2 <= 0;
			bboxMin_X <= 0;				
			bboxMin_Y <= 0;
			bboxMax_X <= 0;
			bboxMax_Y <= 0;
			//
			e0_init <= 0;					
			e1_init <= 0;
			e2_init <= 0;
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
	    //Z_buffer[3:0], C_buffer[3:0]
	    pixel_y <= 0;
  		pixel_x <= 0;
  		pixel_z <= 0;
  		e0 <= 0;
  		e1 <= 0;
  		e2 <= 0;
    end else begin
			case (fsm_state)
			///////////////////////////////
			// 0. perframe, state [150-250]
				0: begin
					do_swap <= 0;
					vsfs_running <= 0;
					fsm_state <= 150;
				end
				//	- set [M]
				150: begin
						M_00 <= 16'sb0000_0001_0000_0000;					// Q8.8
						M_01 <= 0;
						M_02 <= 0;
						M_03 <= 0;
						M_10 <= 0;					
						M_11 <= 16'sb0000_0001_0000_0000;
						M_12 <= 0;
						M_13 <= 0;
						M_20 <= 0;					
						M_21 <= 0;
						M_22 <= 16'sb0000_0001_0000_0000;
						M_23 <= 0;
						M_30 <= 0;					
						M_31 <= 0;
						M_32 <= 0;
						M_33 <= 16'sb0000_0001_0000_0000;
						fsm_state <= 170;
				end 
				//	- dir light rot 1 axis
				170: begin
						fsm_state <= 190;
				end 

				//	- set [M*VP]
				190: begin
						dot_start <= 1;
						fsm_state <= 191;
				end 
				191: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_00 <= dot_result;
							dot_start <= 1;
							fsm_state <= 192;
						end
				end 
				192: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_01 <= dot_result;
							dot_start <= 1;
							fsm_state <= 193;
						end
				end 
				193: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_02 <= dot_result;
							dot_start <= 1;
							fsm_state <= 194;
						end
				end 
				194: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_03 <= dot_result;
							dot_start <= 1;
							fsm_state <= 195;
						end
				end 
				195: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_10 <= dot_result;
							dot_start <= 1;
							fsm_state <= 196;
						end
				end 
				196: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_11 <= dot_result;
							dot_start <= 1;
							fsm_state <= 197;
						end
				end 
				197: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_12 <= dot_result;
							dot_start <= 1;
							fsm_state <= 198;
						end
				end 
				198: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_13 <= dot_result;
							dot_start <= 1;
							fsm_state <= 199;
						end
				end 
				199: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_20 <= dot_result;
							dot_start <= 1;
							fsm_state <= 200;
						end
				end 
				200: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_21 <= dot_result;
							dot_start <= 1;
							fsm_state <= 201;
						end
				end 
				201: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_22 <= dot_result;
							dot_start <= 1;
							fsm_state <= 202;
						end
				end 
				202: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_23 <= dot_result;
							dot_start <= 1;
							fsm_state <= 203;
						end
				end 
				203: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_30 <= dot_result;
							dot_start <= 1;
							fsm_state <= 204;
						end
				end 
				204: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_31 <= dot_result;
							dot_start <= 1;
							fsm_state <= 205;
						end
				end 
				205: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_32 <= dot_result;
							dot_start <= 1;
							fsm_state <= 206;
						end
				end 
				206: begin
						dot_start <= 0;
						if (dot_done) begin
							MVP_33 <= dot_result;
							fsm_state <= 210;
						end
				end 

				//	- [M-1]
				210: begin
						fsm_state <= 230;
				end 

				//	- [M-1]*campos
				230: begin
						fsm_state <= 240;
				end 

				//	- [M-1]*lightpos 
				240: begin
						fsm_state <= 250;
				end 


			// wait for start_vsfs, after clear z on the 1st subframe 
				250: begin
					if (start_vsfs) begin
						tri_idx <= 0;
						vsfs_running <= 1;
						fsm_state <= 31;
					end
				end 
			///////////////////////////////

			// 1. for each tri
				//	- READ tri
				31: begin
					if(tri_idx == numtri)begin
						// wait a few clk before eof to send do_swap
						if ((y == 524) & (x == 770)) begin
							do_swap <= 1;
							vsfs_running <= 0;
							fsm_state <= 0;
						end
					end else begin
						if (ram_notbusy) begin
							vsfs_stop_txn <= 0;
							vsfs_start_read <= 1;
							vsfs_addr <= 24'd153600 + {9'b0,tri_idx_addr};
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
				//   -- read 43 more 4bit
        33: begin
        	tri_xyz[numread[5:2]][{~numread[1:0],2'b00} +: 4] <= spi_data;
          numread <= numread + 1;
          if(numread == 43) begin
          	numread <= 0;
            vsfs_stop_txn <= 1;
            fsm_state <= 34;
          end
        end
        //   -- chk normal -/+
				34: begin
					vsfs_stop_txn <= 0;
					x_model_v0 <= tri_xyz[0];
					y_model_v0 <= tri_xyz[1];
					z_model_v0 <= tri_xyz[2];
					x_model_v1 <= tri_xyz[3];
					y_model_v1 <= tri_xyz[4];
					z_model_v1 <= tri_xyz[5];
					x_model_v2 <= tri_xyz[6];
					y_model_v2 <= tri_xyz[7];
					z_model_v2 <= tri_xyz[8];
					tri_color <= tri_xyz[10][15:14];
					nz <= (tri_xyz[10][13] == 1'b1)? {6'b1111_11,tri_xyz[10][13:4]} : {6'b0,tri_xyz[10][13:4]};
					ny <= (tri_xyz[10][3] == 1'b1)? {6'b1111_11,tri_xyz[10][3:0],tri_xyz[9][15:10]} : {6'b0,tri_xyz[10][3:0],tri_xyz[9][15:10]};
					nx <= (tri_xyz[9][9] == 1'b1)? {6'b1111_11,tri_xyz[9][9:0]} : {6'b0,tri_xyz[9][9:0]};
					fsm_state <= 35;
				end

			///////////////////////////////
			// 2. VS, 40 states
			//	2.1 backface cullling: viewdir = campos - v1, dot(viewdir,n)
			//	2.2 [MVP]*v, clip->NDC (div w), NDC->screen
			//	2.3 dot(light,n)
			
			// 2.1 backface cullling
				//	- viewdir = campos - v1
				35:begin
					viewdir_x <= campos_x - x_model_v0;
					viewdir_y <= campos_y - y_model_v0;
					viewdir_z <= campos_z - z_model_v0;

					dot_start <= 1;
					fsm_state <= 36;

					//debug, print out model data
					// tri_idx <= tri_idx + 1;
					// fsm_state <= 31;
				end
				//	- dot(viewdir,n)
				36:begin
					dot_start <= 0;
					if (dot_done) begin
						if (dot_result[15] == 1'b1)  begin  	   	  // backfacing
							tri_idx <= tri_idx + 1;
							fsm_state <= 31;
						end else begin
							fsm_state <= 37;
						end
					end
				end
				37: begin
					dot_start <= 1;
					fsm_state <= 38;
				end

			// 2.2 [MVP]*v, clip->NDC (div w), NDC->screen
				// - clip = [MVP] * v
				38:begin
					dot_start <= 0;
					if (dot_done) begin
						x_clip_v0 <= dot_result;
						dot_start <= 1;
						fsm_state <= 39;
					end
				end
				39:begin
					dot_start <= 0;
					if (dot_done) begin
						y_clip_v0 <= dot_result;
						dot_start <= 1;
						fsm_state <= 40;
					end
				end
				40:begin
					dot_start <= 0;
					if (dot_done) begin
						z_clip_v0 <= dot_result;
						dot_start <= 1;
						fsm_state <= 41;
					end
				end
				41:begin
					dot_start <= 0;
					if (dot_done) begin
						w_clip_v0 <= dot_result;
						dot_start <= 1;
						fsm_state <= 42;
					end
				end
				42:begin
					dot_start <= 0;
					if (dot_done) begin
						x_clip_v1 <= dot_result;
						dot_start <= 1;
						fsm_state <= 43;
					end
				end
				43:begin
					dot_start <= 0;
					if (dot_done) begin
						y_clip_v1 <= dot_result;
						dot_start <= 1;
						fsm_state <= 44;
					end
				end
				44:begin
					dot_start <= 0;
					if (dot_done) begin
						z_clip_v1 <= dot_result;
						dot_start <= 1;
						fsm_state <= 45;
					end
				end
				45:begin
					dot_start <= 0;
					if (dot_done) begin
						w_clip_v1 <= dot_result;
						dot_start <= 1;
						fsm_state <= 46;
					end
				end
				46:begin
					dot_start <= 0;
					if (dot_done) begin
						x_clip_v2 <= dot_result;
						dot_start <= 1;
						fsm_state <= 47;
					end
				end
				47:begin
					dot_start <= 0;
					if (dot_done) begin
						y_clip_v2 <= dot_result;
						dot_start <= 1;
						fsm_state <= 48;
					end
				end
				48:begin
					dot_start <= 0;
					if (dot_done) begin
						z_clip_v2 <= dot_result;
						dot_start <= 1;
						fsm_state <= 49;
					end
				end
				// - clip->NDC (div w), clip.xyz / clip.w
				49:begin
					dot_start <= 0;
					if (dot_done) begin
						w_clip_v2 <= dot_result;
						// ndc = clip.xy / clip.w
						// 		Q8.8->Q16.16 -> Q16.16 = Q16.16/Q16.16 -> Q16.16->Q2.14
						// 		signed extended[15:0] <= { {8{extend[7]}}, extend[7:0] };
						div_a <= { {8{x_clip_v0[15]}}, x_clip_v0, 8'b0000_0000};
						div_b <= { {8{w_clip_v0[15]}}, w_clip_v0, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 50;
					end
				end
				50:begin
					div_start <= 0;
					if (div_done) begin
						x_model_v0 <= div_result[17:2];
						div_a <= { {8{y_clip_v0[15]}}, y_clip_v0, 8'b0000_0000};
						div_b <= { {8{w_clip_v0[15]}}, w_clip_v0, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 51;
					end
				end
				51: begin
					div_start <= 0;
					if (div_done) begin
						y_model_v0 <= div_result[17:2];
						div_a <= { {8{z_clip_v0[15]}}, z_clip_v0, 8'b0000_0000};
						div_b <= { {8{w_clip_v0[15]}}, w_clip_v0, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 52;
					end
				end
				52: begin
					div_start <= 0;
					if (div_done) begin
						z_model_v0 <= div_result[17:2];
						div_a <= { {8{x_clip_v1[15]}}, x_clip_v1, 8'b0000_0000};
						div_b <= { {8{w_clip_v1[15]}}, w_clip_v1, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 53;
					end
				end
				53: begin
					div_start <= 0;
					if (div_done) begin
						x_model_v1 <= div_result[17:2];
						div_a <= { {8{y_clip_v1[15]}}, y_clip_v1, 8'b0000_0000};
						div_b <= { {8{w_clip_v1[15]}}, w_clip_v1, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 54;
					end
				end
				54: begin
					div_start <= 0;
					if (div_done) begin
						y_model_v1 <= div_result[17:2];
						div_a <= { {8{z_clip_v1[15]}}, z_clip_v1, 8'b0000_0000};
						div_b <= { {8{w_clip_v1[15]}}, w_clip_v1, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 55;
					end
				end
				55: begin
					div_start <= 0;
					if (div_done) begin
						z_model_v1 <= div_result[17:2];
						div_a <= { {8{x_clip_v2[15]}}, x_clip_v2, 8'b0000_0000};
						div_b <= { {8{w_clip_v2[15]}}, w_clip_v2, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 56;
					end
				end
				56: begin
					div_start <= 0;
					if (div_done) begin
						x_model_v2 <= div_result[17:2];
						div_a <= { {8{y_clip_v2[15]}}, y_clip_v2, 8'b0000_0000};
						div_b <= { {8{w_clip_v2[15]}}, w_clip_v2, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 57;
					end
				end
				57: begin
					div_start <= 0;
					if (div_done) begin
						y_model_v2 <= div_result[17:2];
						div_a <= { {8{z_clip_v2[15]}}, z_clip_v2, 8'b0000_0000};
						div_b <= { {8{w_clip_v2[15]}}, w_clip_v2, 8'b0000_0000};
						div_start <= 1;
						fsm_state <= 58;
					end
				end
				58: begin
					div_start <= 0;
					if (div_done) begin
						z_model_v2 <= div_result[17:2];
						fsm_state <= 59;
					end
				end
				// - NDC->screen, screen = [S] * ndc
				59: begin
					// screen = [S] * ndc
					// 		x_ndc * 160 + 160 = x_ndc << 7 + x_ndc << 5 + 160
					// 				Q2.14 (x_ndc) -> Q9.7 (x_ndc << 7) -> Q11.5
					//				Q2.14 (x_ndc) -> Q7.9 (x_ndc << 5) -> Q11.5
					x_model_v0 <= {{2{x_model_v0[15]}}, x_model_v0[15:2]} + 
												{{4{x_model_v0[15]}}, x_model_v0[15:4]} 
												+ 16'sb000_1010_0000_00000;										// Q11.5 (160)
					x_model_v1 <= {{2{x_model_v1[15]}}, x_model_v1[15:2]} + 
												{{4{x_model_v1[15]}}, x_model_v1[15:4]} 
												+ 16'sb000_1010_0000_00000;
					x_model_v2 <= {{2{x_model_v2[15]}}, x_model_v2[15:2]} + 
												{{4{x_model_v2[15]}}, x_model_v2[15:4]} 
												+ 16'sb000_1010_0000_00000;
					//		120 - y * 120. (128-8)
					//				Q2.14 (y_ndc) -> Q9.7 (y_ndc << 7) -> Q11.5
					//				Q2.14 (y_ndc) -> Q5.11 (y_ndc << 3) -> Q11.5
					y_model_v0 <= {{2{y_model_v0[15]}}, y_model_v0[15:2]} -
												{{6{y_model_v0[15]}}, y_model_v0[15:6]};	
					y_model_v1 <= {{2{y_model_v1[15]}}, y_model_v1[15:2]} -
												{{6{y_model_v1[15]}}, y_model_v1[15:6]};	
					y_model_v2 <= {{2{y_model_v2[15]}}, y_model_v2[15:2]} -
												{{6{y_model_v2[15]}}, y_model_v2[15:6]};	
					//		z/2 + 0.5,   
					//				Q2.14 (z_ndc) -> Q(z_ndc >> 1)
									// [-1,1] -> [-0.5,0.5] -> [0,1]
									// 01.xxxx -> 1.999					00.1111 (0.999)
									// 01.0000 -> 1. 						00.1000 (0.5)
									// 00.xxxx -> 0,0.99.  			00.0111 (0.499)
									// //
									// 11.xxxx -> -0.1,-0.99. 	11.1001 (-0.1)
									// 11.0000 -> -1 						11.1000 (-0.5)
									// 10.xxxx                  11.0xxx (-0.5 - -1)
									// 10.0000 -> -1.999.       11.0000 (-1)
					z_model_v0 <= {z_model_v0[15], z_model_v0[15:1]} +
												+ 16'sb00_1000_0000_0000_00;									// Q2.14 (0.5)
					z_model_v1 <= {z_model_v1[15], z_model_v1[15:1]} +
												+ 16'sb00_1000_0000_0000_00;
					z_model_v2 <= {z_model_v2[15], z_model_v2[15:1]} +
												+ 16'sb00_1000_0000_0000_00;
					fsm_state <= 60;
				end
				60: begin
					// y = 120 - y, flip y (y=0 at the top, invert of opengl)
					y_model_v0 <= 16'sb000_0111_1000_00000 - y_model_v0;				// Q11.5 (120)
					y_model_v1 <= 16'sb000_0111_1000_00000 - y_model_v1;
					y_model_v2 <= 16'sb000_0111_1000_00000 - y_model_v2;
					fsm_state <= 61;
				end
				61: begin
					// z_ndc. Q2.14 -> zscreen Q2.20
					z_screen_v0 <= {2'b00,tri_xyz[2],4'b0000};

					x_screen_v0 <= {9'b0000_0000_0,x_model_v0[15:5]};						// Q20.0 (screen), always positive 
					x_screen_v1 <= {9'b0000_0000_0,x_model_v1[15:5]};
					x_screen_v2 <= {9'b0000_0000_0,x_model_v2[15:5]};
					y_screen_v0 <= {9'b0000_0000_0,y_model_v0[15:5]};
					y_screen_v1 <= {9'b0000_0000_0,y_model_v1[15:5]};
					y_screen_v2 <= {9'b0000_0000_0,y_model_v2[15:5]};
					z_screen_v0 <= {z_model_v0,6'b0};			
					z_screen_v1 <= {z_model_v1,6'b0};		
					z_screen_v2 <= {z_model_v2,6'b0};		
					fsm_state <= 62;
				end
				


			// 2.3 dot(light,n)
				62:begin
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
				end

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
  assign debug_vsfs_fsm_state = fsm_state;
  assign debug_x_model_v0 = dot_result;
  assign debug_x_model_v1 = x_model_v1;
  assign debug_x_model_v2 = x_model_v2;
  assign debug_y_model_v0 = y_model_v0;
  assign debug_y_model_v1 = y_model_v1;
  assign debug_y_model_v2 = y_model_v2;
  assign debug_z_model_v0 = z_model_v0;
  assign debug_z_model_v1 = z_model_v1;
  assign debug_z_model_v2 = z_model_v2;
  assign debug_nx = nx;
  assign debug_ny = ny;
  assign debug_nz = nz;
  assign debug_tri_color = tri_color;


  
endmodule