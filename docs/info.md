<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

TinyGPU v2.0. A standalone GPU that can display a model file from FLASH.  	
	Render 1K tri at 6.5fps in 320x240, 4-bit color.  

	Spec:
	- GPU can performs transformation & lighting, rasterization
	- 4-bit double buffer, 8-bit depth buffer store on QSPI RAM
	- max tri 1K
	- backface culling
	- 1 dynamic directional light, flat shading
	- use Gamepad to transform the model and rotate light
	- run at 25Mhz. When fab, it will use around 200k transistor  

## How to test

Updating document, Please come back again

## External hardware

- QSPI PMOD
- TinyVGA PMOD
- GamePad PMOD
- SNES controller
