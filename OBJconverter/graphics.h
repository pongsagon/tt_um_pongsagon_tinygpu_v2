#pragma once

// standard c
#include <cmath>
#include <limits>
#include <cstdlib>
#include <vector>
#include <string>
#include <stdint.h>

// math
#include "fix.h"

typedef struct Pixel {  // for SDL texture
    uint8_t a;  // transparency
    uint8_t b;  // blue
    uint8_t g;  // green
    uint8_t r;  // red
} Pixel;

// constant
extern fix14    fix14_0;
extern fix      fix_0,fix_1,fix_32,fix_64,fix_neg1, fix_neg10, fix_neg100, fix_nearclip;
extern fix32	fix32_1, fix32_160, fix32_120, fix32_320, fix32_240;

// transform
extern Mat4f ViewMat;
extern Mat4f ModelViewMat;
extern Mat4f ProjectionMat;
extern Mat4f MVP;
extern Mat4f VP;


// cam, light
extern Vec3f14  light_dir;
extern fix      cam_zoom;
extern short    cam_yaw;
extern short    cam_pitch;
extern Vec3f    eye;
extern Vec3f    center;
extern Vec3f    up;

// FOV = 40, near = 10, far = 100
// called by init3D()
void projection();

void init3D();

// orbit camera 
void updateCamEye();
void lookat(Vec3f eye, Vec3f center, Vec3f up);