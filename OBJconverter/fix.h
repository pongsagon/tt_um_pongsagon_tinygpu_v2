// modified from
//https://people.ece.cornell.edu/land/courses/ece4760/PIC32/index_fixed_point.html

// 3 types
// 1. fix32
// Q16.16, [+-32767]
// speedup compare to float: +- 10x, * 3x, / 2x

// 2. fix14
// Q2.14, [-2,1.99] use for light dir, normal dir, uv [-1,1]

// 3. fix
// Q8.8 [+-127] 0.004
// speedup compare to fix32: 2-3x

// range is much more limited than int, Watch out for overflow


#pragma once

#include <math.h>

typedef signed short fix;
typedef signed short fix14;
typedef signed int fix32;

#define LOW(x) (x & 0x00FF)
#define HIG(x) LOW((x>>8))

#define int2fix(a)    ((fix)((a) << 8))
#define fix2int(a)    ((int)((a) >> 8))
#define fix2short(a)  ((short)((a) >> 8))
#define float2fix(a)  ((fix)((a) * 256.0f))
#define fix2float(a)  ((float)(a) / 256.0f)

#define int2fix14(a)    ((fix)((a) << 14))
#define fix2int14(a)    ((int)((a) >> 14))
#define float2fix14(a) ((fix14)((a)*16384.0)) // 2^14
#define fix2float14(a) ((float)(a)/16384.0)

#define int2fix32(a)    ((fix32)((a) << 16))
#define fix2int32(a)    ((int)((a) >> 16))
#define fix2short32(a)   ((short)((a) >> 16))
#define float2fix32(a)  ((fix32)((a) * 65536.0f))
#define fix2float32(a)  ((float)(a) / 65536.0f)

#define fix2fix32(a)    ((fix32)((a) << 8))
#define fix322fix(a)    ((fix)((a) >> 8))
#define fix2fix14(a)    ((fix14)((a) << 6))

// Addition and subtraction, abs just work
// negate (*-1) also work

#define multfix(a,b)  ((fix)(((( long)(a))*(( long)(b)))>>8)) 
#define divfix(a,b)   ((fix)((((long)(a)<<8)/(b)))) 
#define sqrtfix(a)    (float2fix(sqrt(fix2float(a))))

#define multfix14(a,b) ((fix14)((((long)(a))*((long)(b)))>>14)) //multiply two fixed 2.14
#define divfix14(a,b)   ((fix14)(((( long)(a)<<14)/(b)))) 

#define multfix32(a,b)  ((fix32)(((( signed long long)(a))*(( signed long long)(b)))>>16)) 
#define divfix32(a,b)   ((fix32)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix32(a)    (float2fix32(sqrt(fix2float32(a))))

// Q2.14 = I16.0 x Q2.14
#define multfix14_16_14(a,b)  ((fix)(((( long)(a))*(( long)(b))))) 


struct Vec2i
{
  short x,y;
};
struct Vec3i
{
  short x,y,z;
};
struct Vec2i32
{
  int x,y;
};
struct Vec3i32
{
  int x,y,z;
};
struct Vec2f
{
  fix x,y;
};
struct Vec3f
{
  fix x,y,z;
};
struct Vec4f
{
  fix x,y,z,w;
};
struct Vec2f14
{
  fix14 x,y;
};
struct Vec3f14
{
  fix14 x,y,z;
};
struct Vec4f14
{
  fix14 x,y,z,w;
};
struct Vec2f32
{
  fix32 x,y;
};
struct Vec3f32
{
  fix32 x,y,z;
};
struct Vec4f32
{
  fix32 x,y,z,w;
};


struct Mat4f
{
  Vec4f row0;
  Vec4f row1;
  Vec4f row2;
  Vec4f row3;
};
struct Mat4f32
{
  Vec4f32 row0;
  Vec4f32 row1;
  Vec4f32 row2;
  Vec4f32 row3;
};

// 0-90 degree
const fix sinTable[] = {
0,4,8,13,17,22,26,31,35,40,44,48,53,57,61,66,70,74,79,83,87,91,95,100,104,108,112,
116,120,124,128,131,135,139,143,146,150,154,157,161,164,167,171,174,177,181,184,187,
190,193,196,198,201,204,207,209,212,214,217,219,221,223,226,228,230,232,233,235,237,
238,240,242,243,244,246,247,248,249,250,251,252,252,253,254,254,255,255,255,255,255,256};

const fix32 sinTable32[] = {
0,1143,2287,3429,4571,5711,6850,7986,9120,10252,11380,12504,13625,14742,15854,16961,
18064,19160,20251,21336,22414,23486,24550,25606,26655,27696,28729,29752,30767,31772,
32768,33753,34728,35693,36647,37589,38521,39440,40347,41243,42125,42995,43852,44695,
45525,46340,47142,47929,48702,49460,50203,50931,51643,52339,53019,53683,54331,54963,
55577,56175,56755,57319,57864,58393,58903,59395,59870,60326,60763,61183,61583,61965,
62328,62672,62997,63302,63589,63856,64103,64331,64540,64729,64898,65047,65176,65286,
65376,65446,65496,65526,65536};


// performance per x1000
// 240us
fix14 dot14(Vec3f14 v1, Vec3f14 v2);

// fix 16bit
// div 300us, mul 70us
// fix32
// div 300us, mul 200us

// 240us
fix   dot(Vec3f v1, Vec3f v2);
// 720us
fix32   dot32(Vec3f32 v1, Vec3f32 v2);
// 310us
fix   dot4(Vec4f v1, Vec4f v2);
// 880us
fix32   dot432(Vec4f32 v1, Vec4f32 v2);
// 420us
Vec3f cross(Vec3f v1, Vec3f v2);
// 1400us
Vec3f32 cross32(Vec3f32 v1, Vec3f32 v2);
// 1680us
Vec3f normalize(Vec3f v);
// 2800us
Vec3f32 normalize32(Vec3f32 v);
Vec3f32 normalizeScale32(Vec3f32 v, fix32 scale);

// 2000us 
Vec4f mulMatVec(Mat4f mat, Vec4f v);
// 3800us 
Vec4f32 mulMatVec32(Mat4f32 mat, Vec4f32 v);
// 5400us 
Mat4f mulMatMat(Mat4f mat1, Mat4f mat2);
// 14300us 
Mat4f32 mulMatMat32(Mat4f32 mat1, Mat4f32 mat2);
// 180us
fix  fixSin(short i);
fix  fixCos(short i);
// 180us
fix32  fixSin32(short i);
fix32  fixCos32(short i);






