#!/usr/bin/env python3

#install this first in mac
#   brew install pillow

#usage
#   python3 img2binhex_tex.py input_filename.jpeg tex.bin tex.mem

# This script generate 256x256 image, 4-bit RGB 1-2-1 for TinyGPU v2.0/3.0
# It will output .bin format for use with Flash pmod 
# and also output a readable text file in mem/hex format to test with FPGA BRAM. 

import sys
import struct
from PIL import Image


if len(sys.argv) < 4:
    print("Usage: python3 img2binhex_tex.py input_filename.jpeg tex.bin tex.mem")
    sys.exit()
    
input_file = sys.argv[1]
output_file1 = sys.argv[2]  
output_file2 = sys.argv[3]  

width = 256
height = 256

# to output binary file
out_file_bin = open(output_file1, "wb")

# to output readable text file
out_file_mem = open(output_file2, "w")

im = Image.open(input_file);
im = im.resize((width,height))
im = im.convert("RGB")
data = im.load()

# pack 2-pixel as one byte
# The TinyGPU will fetch 4-pixel at a time
for y in range(0,height):
    for x in range(0,width,4):
        # 0-255, 1000_0000 
        r = int(data[x+2, y][0] / 128)
        g = int(data[x+2, y][1] / 64)
        b = int(data[x+2, y][2] / 128)
        lo = int((r << 3) + (g << 1) + b)

        r = int(data[x+3, y][0] / 128)
        g = int(data[x+3, y][1] / 64)
        b = int(data[x+3, y][2] / 128)
        hi = int((r << 3) + (g << 1) + b)

        # to output binary file
        out_file_bin.write(struct.pack('>B', (hi << 4) + lo))

        # to output readable text file, 
        # - '0xff' use [2:] to get rid of 0x 
        out_file_mem.write(hex((hi << 4) + lo)[2:])
        out_file_mem.write("\n")


        r = int(data[x, y][0] / 128)
        g = int(data[x, y][1] / 64)
        b = int(data[x, y][2] / 128)
        lo = int((r << 3) + (g << 1) + b)

        r = int(data[x+1, y][0] / 128)
        g = int(data[x+1, y][1] / 64)
        b = int(data[x+1, y][2] / 128)
        hi = int((r << 3) + (g << 1) + b)

        # to output binary file
        out_file_bin.write(struct.pack('>B', (hi << 4) + lo))

        # to output readable text file, 
        # - '0xff' use [2:] to get rid of 0x 
        out_file_mem.write(hex((hi << 4) + lo)[2:])
        out_file_mem.write("\n")











