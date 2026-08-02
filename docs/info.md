<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

##  Result running on TinyTapeout ASIC

 ![flat](https://github.com/pongsagon/tt_um_pongsagon_tinygpu_v2/blob/main/img/asic.gif) 

## How it works

TinyGPU v2.0. A standalone GPU that can display a textured model file from FLASH. \
Render 1K tri at 6.5fps in 320x240, 4-bit color. Tex res 256x256, 4-bit. 

	Spec:
	- GPU can performs transformation & lighting, rasterization
	- 4-bit double buffer, 8-bit depth buffer store on QSPI RAM
	- max tri 1K
	- backface culling
	- 1 dynamic directional light, flat shading
	- affine texture mapping
	- use Gamepad to transform the model and rotate light
	- run at 25Mhz. When fab, it will use around 240k transistor  

## External hardware

- QSPI PMOD
- TinyVGA PMOD
- GamePad PMOD
- SNES controller

## How to test

1. Plug QSPI PMOD, TinyVGA PMOD, GamePad PMOD with SNES controller to the TT demo board
2. Flash a model file .bin from 3Dmodels folder to QSPI Flash PMOD
3. The default QSPI read latency is already set to 0 via ui_in[2:0], if there is a problem of QSPI latency try setting the value to 1-4.
4. The model should be rendered spinning on the VGA monitor.
5. Use D-pad to rotate model around X,Y axis.  A/B button to zoom in/out.  For model without texture, use X/Y button to rotate light.

Current limitation
- The model cannot be rendered beyond the viewport.  The GPU will be freezed and have to be reset.  This limitation is solved in TinyGPU v3.0

## 3D model file converter


How to build/run in mac terminal
 - install GLM library first
 - The code is in OBJconverter folder

Build
- g++ *.cpp -o main -I includes

Run 
- place .obj file + tex.mem/tex.bin(optional) in input_model/
- ./main input.obj true white \
arg[1]: obj filename \
arg[2]: has texture or not \
arg[3]: model color choose from white, pink, cyan, green

Output
- it will output output.bin (binary file) for use with Flash and output.mem (text file) for use with FPGA BRAM
