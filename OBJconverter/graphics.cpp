#include "graphics.h"

// init fix constant
fix fix_0 = int2fix(0);
fix fix_1 = int2fix(1);
fix fix_32 = int2fix(32);
fix fix_64 = int2fix(64);
fix fix_neg1 = float2fix(-1);
fix fix_neg10 = float2fix(-10);
fix fix_neg100 = float2fix(-100);
fix fix_nearclip = float2fix(-3);
//
fix32   fix32_1 = int2fix32(1);
fix32   fix32_160 = int2fix32(160);
fix32   fix32_120 = int2fix32(120);
fix32   fix32_320 = int2fix32(320);
fix32   fix32_240 = int2fix32(240);
//
fix14   fix14_0 = int2fix14(0);


// transform
Mat4f ModelMat;
Mat4f ViewMat;
Mat4f ModelViewMat;
Mat4f ProjectionMat;
Mat4f VP;

// cam, light
Vec3f14  light_dir;
fix      cam_zoom;
short    cam_yaw;
short    cam_pitch;
Vec3f    eye;
Vec3f    center;
Vec3f    up;

// FOV = 40, near = 10, far = 100, 320x240
void projection() {
    ProjectionMat.row0 = (Vec4f){float2fix(2.06), fix_0,fix_0,fix_0};
    ProjectionMat.row1 = (Vec4f){fix_0, float2fix(2.75), fix_0,fix_0};
    ProjectionMat.row2 = (Vec4f){fix_0, fix_0, float2fix(-1.22), float2fix(-22.22)};
    ProjectionMat.row3 = (Vec4f){fix_0, fix_0, float2fix(-1),fix_0};
}


void init3D(){

    //init light, cam
    light_dir = (Vec3f14){int2fix14(0), int2fix14(0), int2fix14(1)};
    cam_yaw = 90;
    cam_pitch = 0;
    cam_zoom = int2fix(40);
    eye = (Vec3f){int2fix(0),int2fix(0),cam_zoom};
    center = (Vec3f){float2fix(0.0),float2fix(0),int2fix(0) };
    up = (Vec3f){fix_0,fix_1,fix_0};

  // set projection matrix
    projection();
}
 

// orbit camera 
void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    // normalize non-near unit vector can easily overflow, use normalize32 just in case
    Vec3f32 viewDir32 = (Vec3f32){eye.x-center.x,eye.y-center.y,eye.z-center.z};
    Vec3f32 z32 = normalize32(viewDir32); 
    Vec3f z;
    z.x = fix322fix(z32.x);
    z.y = fix322fix(z32.y);
    z.z = fix322fix(z32.z);
    Vec3f x = normalize(cross(up,z));
    Vec3f y = normalize(cross(z,x));
    ViewMat.row0 = (Vec4f){x.x, x.y, x.z, dot(eye,x)};
    ViewMat.row1 = (Vec4f){y.x, y.y, y.z, dot(eye,y)};
    ViewMat.row2 = (Vec4f){z.x, z.y, z.z, dot(eye,z)};
    ViewMat.row3 = (Vec4f){fix_0,fix_0,fix_0,fix_1};
    ViewMat.row0.w = multfix(ViewMat.row0.w, fix_neg1);
    ViewMat.row1.w = multfix(ViewMat.row1.w, fix_neg1);
    ViewMat.row2.w = multfix(ViewMat.row2.w, fix_neg1);
}

void updateCamEye(){
  Vec3f dir;
  dir.x = multfix(fixCos(cam_yaw),fixCos(cam_pitch));
  dir.y = fixSin(cam_pitch);
  dir.z = multfix(fixSin(cam_yaw),fixCos(cam_pitch));
  // normalizeScale non-near unit vector can easily overflow, use normalizeScale32 instead
  Vec3f32 dir32;
  dir32.x = fix2fix32(dir.x);
  dir32.y = fix2fix32(dir.y);
  dir32.z = fix2fix32(dir.z);
  Vec3f32 eye32 = normalizeScale32(dir32,fix2fix32(cam_zoom));
  eye.x = fix322fix(eye32.x);
  eye.y = fix322fix(eye32.y);
  eye.z = fix322fix(eye32.z);
}