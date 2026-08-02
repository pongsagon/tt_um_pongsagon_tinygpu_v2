
// how to build/run in mac terminal
//		- install GLM library first
//		Build
//			g++ *.cpp -o main -I includes
//		Run 
//			- place .obj file + tex.mem/tex.bin(optional) in input_model/
//			/main input.obj true white
//				- arg[1]: obj filename
//				- arg[2]: has texture or not
//				- arg[3]: model color choose from white, pink, cyan, green


// this code output triangle list + texture(optional) to a single file
// 	- it will output output.bin (binary file) for use with Flash
//		and output.mem (text file) for use with FPGA BRAM
//	- the file structure is as followed
//		1. first 2 bytes is #tri
//		2. texture data (optional)
// 		3. pos.xyz x3 Q8.8, face normal Q2.8 x3, color 2bit = 22 byte/tri
// 			little endian (lower byte transfer first)


// Include standard headers
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cstring>

// Include GLM
#include "includes/glm/glm.hpp"
#include "includes/glm/gtc/matrix_transform.hpp"

// 
#include "objloader.h"
#include "fix.h"
#include "graphics.h"


bool isKthBitSet(int n, int k)
{
    if (n & (1 << k))
        return true;
    else
        return false;
}

//array of vec3 to store Geometry 
std::vector<glm::vec3> vertices;
std::vector<glm::vec2> uvs;
std::vector<glm::vec3> normals;
std::vector<glm::vec3> face_normals;
std::vector<int>    idx1;
std::vector<int>    idx2;
std::vector<int>    idx3;
std::vector<int>    uv_idx1;
std::vector<int>    uv_idx2;
std::vector<int>    uv_idx3;
// module[1]
std::vector<glm::vec3>  face_vert;						
std::vector<short>    m1_x;						// Q8.8
std::vector<short>    m1_y;
std::vector<short>    m1_z;
std::vector<unsigned char>    m1_u;				// Q8.0
std::vector<unsigned char>    m1_v;
std::vector<char>    m1_nx;						// Q2.6
std::vector<char>    m1_ny;
std::vector<char>    m1_nz;
std::vector<unsigned short>    m1_idx1;					// Q10.0
std::vector<unsigned short>    m1_idx2;
std::vector<unsigned short>    m1_idx3;
std::vector<unsigned short>    m1_uv_idx1;
std::vector<unsigned short>    m1_uv_idx2;
std::vector<unsigned short>    m1_uv_idx3;
// module[2]
std::vector<unsigned short>    	m2_bboxL;		// Q10.0
std::vector<unsigned short>    	m2_bboxR;		
std::vector<unsigned short>    	m2_bboxT;		
std::vector<unsigned short>    	m2_bboxB;		
std::vector<unsigned char>    	m2_u1;			// Q8.0
std::vector<unsigned char>    	m2_v1;
std::vector<unsigned char>    	m2_u2;			
std::vector<unsigned char>    	m2_v2;
std::vector<unsigned char>    	m2_u3;			
std::vector<unsigned char>    	m2_v3;
std::vector<unsigned char>    	m2_I1;			// Q4.0 
std::vector<unsigned char>    	m2_I2;
std::vector<unsigned char>    	m2_I3;
std::vector<unsigned short>    	m2_z1;			// Q0.16
std::vector<unsigned short>    	m2_z2;
std::vector<unsigned short>    	m2_z3;
std::vector<fix32>    			m2_denom;		// Q2.20 (in C Q4.16)
std::vector<short>    			m2_x1;			// Q11.0, screen
std::vector<short>    			m2_x2;
std::vector<short>    			m2_x3;
std::vector<short>    			m2_y1;
std::vector<short>    			m2_y2;
std::vector<short>    			m2_y3;


int main(int argv, char* args[])
{
	if (argv < 4) {
        printf("No additional command-line arguments provided.\n");
		printf("Usage: ./main input.obj true white\n");
        return 0;
    }else{
		// for (int i = 1; i < argv; i++) {
		// 	printf("%s\n", args[i]);
		// }
	}

	// white, pink, cyan, green
	unsigned char color;
	bool output_tex;

	if (strcmp(args[2], "true") == 0) {
		output_tex = true;
	}else{
		output_tex = false;
	}
	if (strcmp(args[3], "white") == 0) {
		color = 0;
	}else if(strcmp(args[3], "pink") == 0){
		color = 1;
	}else if(strcmp(args[3], "cyan") == 0){
		color = 2;
	}else {
		color = 3;
	}

	char inputPath[50] = "input_model/"; 
    strcat(inputPath, args[1]); 

	bool res = loadOBJ(inputPath, vertices, uvs, normals, 
						idx1, idx2, idx3, uv_idx1, uv_idx2, uv_idx3);

	FILE* file_xyz = fopen("output/output.mem", "w");
	FILE* file_xyz_bin = fopen("output/output.bin", "wb");

	FILE* file_tex = fopen("input_model/tex.mem", "r");
	FILE* file_tex_bin = fopen("input_model/tex.bin", "rb");

	// find min, max
	float minx = 1000000;
	float miny = 1000000;
	float minz = 1000000;
	float maxx = -1000000;
	float maxy = -1000000;
	float maxz = -1000000;
	for (int i = 0; i < vertices.size(); i++) {
		if (vertices[i].x < minx) {
			minx = vertices[i].x;
		}
		if (vertices[i].y < miny) {
			miny = vertices[i].y;
		}
		if (vertices[i].z < minz) {
			minz = vertices[i].z;
		}
		if (vertices[i].x > maxx) {
			maxx = vertices[i].x;
		}
		if (vertices[i].y > maxy) {
			maxy = vertices[i].y;
		}
		if (vertices[i].z > maxz) {
			maxz = vertices[i].z;
		}
	}

	// center model at (0,0,0)
	float centerx = (maxx + minx) / 2;
	float centery = (maxy + miny) / 2;
	float centerz = (maxz + minz) / 2;
	for (int i = 0; i < vertices.size(); i++) {
		vertices[i].x -= centerx;
		vertices[i].y -= centery;
		vertices[i].z -= centerz;
	}


	// scale to [-10,10]
	float scalex = maxx - minx;
	float scaley = maxy - miny;
	float scalez = maxz - minz;
	float scaleMax = std::max(scalez, std::max(scalex, scaley));
	float scaleFactor = 20.0f / scaleMax;
	for (int i = 0; i < vertices.size(); i++) {
		vertices[i].x *= scaleFactor;
		vertices[i].y *= scaleFactor;
		vertices[i].z *= scaleFactor;
	}

	// normalize normal
	for (int i = 0; i < normals.size(); i++){
		normals[i] = glm::normalize(normals[i]);
	}

    // output .mem for module 1
	int i = 0;
	
	// output idx, idx start from 1 not 0
	for (i = 0; i < idx1.size(); i++) {		
		// save to c's module[1]
		m1_idx1.push_back((unsigned short)idx1[i]-1);
		m1_idx2.push_back((unsigned short)idx2[i]-1);
		m1_idx3.push_back((unsigned short)idx3[i]-1);
	}
		
	// output pos
	for (i = 0; i < vertices.size(); i++) {
		// save to c's module[1]
		m1_x.push_back((short)float2fix(vertices[i].x));
		m1_y.push_back((short)float2fix(vertices[i].y));
		m1_z.push_back((short)float2fix(vertices[i].z));

		face_vert.push_back(glm::vec3(vertices[i].x,vertices[i].y,vertices[i].z));
	}

	// uv
	for (i = 0; i < uv_idx1.size(); i++) {		
		// save to c's module[1]
		m1_uv_idx1.push_back((unsigned short)uv_idx1[i]-1);
		m1_uv_idx2.push_back((unsigned short)uv_idx2[i]-1);
		m1_uv_idx3.push_back((unsigned short)uv_idx3[i]-1);
	}
	for (i = 0; i < uvs.size(); i++) {
		// save to c's module[1]
		m1_u.push_back((unsigned char)(uvs[i].x*255.0));
		m1_v.push_back((unsigned char)(uvs[i].y*255.0));
	}

    if (true){

		init3D();

		updateCamEye();
		lookat(eye, center, up);
		VP = mulMatMat(ProjectionMat,ViewMat);

		// print #tri
		char str_tri[10];
		snprintf(str_tri, 10,"%04x",(unsigned short)idx1.size());
		std::string string_tri(str_tri);
		std::string sub_tri;
		sub_tri = string_tri.substr(2,2);
		fprintf(file_xyz, "%s\n", sub_tri.c_str());
		sub_tri = string_tri.substr(0,2);
		fprintf(file_xyz, "%s\n", sub_tri.c_str());
		//
		unsigned short tri_short = (unsigned short)idx1.size();
		fwrite(&tri_short,sizeof(short),1,file_xyz_bin);

		// print tex/no-tex
		if(output_tex){
			unsigned short tmp = 1;
			snprintf(str_tri, 10,"%04x",(unsigned short)tmp);
			string_tri = str_tri;
			sub_tri = string_tri.substr(2,2);
			fprintf(file_xyz, "%s\n", sub_tri.c_str());
			//
			tri_short = 1;
			fwrite(&tri_short,sizeof(char),1,file_xyz_bin);

			// combine tex binary
			char myString[10];
			// Read the content and print it
			while(fgets(myString, 10, file_tex)) {
			  fprintf(file_xyz,"%s", myString);
			}
			//
			char *buffer;
		    long filelen;
		    int i;
     
		    fseek(file_tex_bin, 0, SEEK_END);          
		    filelen = ftell(file_tex_bin);            
		    rewind(file_tex_bin);                      
		    buffer = (char *)malloc((filelen+1)*sizeof(char)); 
		    fread(buffer, filelen, 1, file_tex_bin);
		    fwrite(buffer,sizeof(char),filelen,file_xyz_bin);
		    printf("len %ld\n", filelen);
		    free(buffer);


		}else{
			unsigned short tmp = 0;
			snprintf(str_tri, 10,"%04x",(unsigned short)tmp);
			string_tri = str_tri;
			sub_tri = string_tri.substr(2,2);
			fprintf(file_xyz, "%s\n", sub_tri.c_str());
			//
			tri_short = 0;
			fwrite(&tri_short,sizeof(char),1,file_xyz_bin);
		}


		int count = 0;
		// for each tri
		for (i = 0; i < idx1.size(); i++) {
			// get v1,v2,v3: xyz
			short vert[9];
			unsigned char uv[6];
			glm::vec3 v1, v2, v3;
			
			// Q8.8
			vert[0] = m1_x[m1_idx1[i]];
			vert[1] = m1_y[m1_idx1[i]];
			vert[2] = m1_z[m1_idx1[i]];
			vert[3] = m1_x[m1_idx2[i]];
			vert[4] = m1_y[m1_idx2[i]];
			vert[5] = m1_z[m1_idx2[i]];
			vert[6] = m1_x[m1_idx3[i]];
			vert[7] = m1_y[m1_idx3[i]];
			vert[8] = m1_z[m1_idx3[i]];

			// Q8.0
			uv[0] = m1_u[m1_uv_idx1[i]];
			uv[1] = m1_v[m1_uv_idx1[i]];
			uv[2] = m1_u[m1_uv_idx2[i]];
			uv[3] = m1_v[m1_uv_idx2[i]];
			uv[4] = m1_u[m1_uv_idx3[i]];
			uv[5] = m1_v[m1_uv_idx3[i]];

			char str_x[10];
			std::string string_x;
			std::string sub_x;

			for(int j = 0;j<9;j++){
				snprintf(str_x, 10,"%04x",vert[j]);
				string_x = str_x;
				if (string_x[0] == 'f'){
					string_x = string_x.substr(4);
				}else{
					string_x = string_x.substr(0);
				}
				sub_x = string_x.substr(2,2);
				fprintf(file_xyz, "%s\n", sub_x.c_str());
				sub_x = string_x.substr(0,2);
				fprintf(file_xyz, "%s\n", sub_x.c_str());
				fwrite(&vert[j],sizeof(short),1,file_xyz_bin);
			}


			// write face normal Q2.8 x3
			// float
			v1 = face_vert[m1_idx1[i]];
			v2 = face_vert[m1_idx2[i]];
			v3 = face_vert[m1_idx3[i]];

			// finding face normal
			glm::vec3 edge1 = v2 - v1;
			glm::vec3 edge2 = v3 - v1;
			glm::vec3 f_norm = glm::cross(edge1, edge2);
			f_norm = glm::normalize(f_norm);

			unsigned short mask6bit = 0x03ff;
			unsigned short mask_x, mask_y, mask_z;
			unsigned int pack_CZYX;
			
			// Q 000000_2.8
			mask_x = mask6bit & float2fix(f_norm.x);
			mask_y = mask6bit & float2fix(f_norm.y);
			mask_z = mask6bit & float2fix(f_norm.z);

			// 2 | 2.8 | 2.8 | 2.8
			pack_CZYX = (color << 30) + (mask_z << 20) + (mask_y << 10) + mask_x;
			snprintf(str_x, 10,"%08x",pack_CZYX);
			string_x = str_x;
			sub_x = string_x.substr(6,2);
			fprintf(file_xyz, "%s\n", sub_x.c_str());
			sub_x = string_x.substr(4,2);
			fprintf(file_xyz, "%s\n", sub_x.c_str());
			sub_x = string_x.substr(2,2);
			fprintf(file_xyz, "%s\n", sub_x.c_str());
			sub_x = string_x.substr(0,2);
			fprintf(file_xyz, "%s\n", sub_x.c_str());
			fwrite(&pack_CZYX,sizeof(unsigned int),1,file_xyz_bin);

			// uv
			if(output_tex){
				for(int j = 0;j<6;j++){
					snprintf(str_x, 10,"%04x",uv[j]);
					string_x = str_x;
					sub_x = string_x.substr(2,2);
					fprintf(file_xyz, "%s\n", sub_x.c_str());
					fwrite(&uv[j],sizeof(unsigned char),1,file_xyz_bin);
				}
			}		
		}

		fclose(file_tex);
		fclose(file_tex_bin);
		fclose(file_xyz);
		fclose(file_xyz_bin);
    }

	return 0;
}