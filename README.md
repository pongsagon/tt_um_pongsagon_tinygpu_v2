![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

##  Tested on TinyTapeout ASIC

 ![flat](https://github.com/pongsagon/tt_um_pongsagon_tinygpu_v2/blob/main/img/asic.gif)

##  Tested on Basys3 FPGA

Prototyping results on Basys3 FPGA.

 ![flat](https://github.com/pongsagon/tt_um_pongsagon_tinygpu_v2/blob/main/img/bunny.gif)  ![Texture](https://github.com/pongsagon/tt_um_pongsagon_tinygpu_v2/blob/main/img/ff8.gif) 

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

For model with texture, you have to convert the texture format first using python script img2binhex_tex.py in OBJconverter folder.
- GPU can support the model with only one texture.
- usage: python3 img2binhex_tex.py input_filename.jpeg tex.bin tex.mem
- The output, tex.bin, will be embedded with the model file when calling OBJconverter.
- tex.mem version is used for testing on FPGA BRAM
- you may need to install pillow for python first.  brew install pillow


How to build/run 3D model converter in mac terminal
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

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Set up your Verilog project

1. Add your Verilog files to the `src` folder.
2. Edit the [info.yaml](info.yaml) and update information about your project, paying special attention to the `source_files` and `top_module` properties. If you are upgrading an existing Tiny Tapeout project, check out our [online info.yaml migration tool](https://tinytapeout.github.io/tt-yaml-upgrade-tool/).
3. Edit [docs/info.md](docs/info.md) and add a description of your project.
4. Adapt the testbench to your design. See [test/README.md](test/README.md) for more information.

The GitHub action will automatically build the ASIC files using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

## Enable GitHub actions to build the results page

- [Enabling GitHub Pages](https://tinytapeout.com/faq/#my-github-action-is-failing-on-the-pages-part)

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Edit [this README](README.md) and explain your design, how it works, and how to test it.
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)
