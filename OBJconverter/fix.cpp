#include "fix.h"

fix14 dot14(Vec3f14 v1, Vec3f14 v2){
  return multfix14(v1.x,v2.x)+multfix14(v1.y,v2.y)+multfix14(v1.z,v2.z);
}

fix   dot(Vec3f v1, Vec3f v2){
  return multfix(v1.x,v2.x)+multfix(v1.y,v2.y)+multfix(v1.z,v2.z);
}

fix   dot4(Vec4f v1, Vec4f v2){
  return  multfix(v1.x,v2.x)+multfix(v1.y,v2.y)+multfix(v1.z,v2.z)+multfix(v1.w,v2.w);
}

Vec3f cross(Vec3f v1, Vec3f v2){
  Vec3f result;
  result.x = multfix(v1.y,v2.z)-multfix(v1.z,v2.y);
  result.y = multfix(v1.z,v2.x)-multfix(v1.x,v2.z);
  result.z = multfix(v1.x,v2.y)-multfix(v1.y,v2.x);
  return result;
}


Vec3f normalize(Vec3f v){
  // can easily overflow, ex. 10x10 + 10x10 + 10x10 = 300 > 127
  fix sqrdist = multfix(v.x,v.x)+multfix(v.y,v.y)+multfix(v.z,v.z);
  fix dist = sqrtfix(sqrdist);
  fix inv_dist = divfix(int2fix(1),dist);
  Vec3f result;
  result.x = multfix(v.x,inv_dist);
  result.y = multfix(v.y,inv_dist);
  result.z = multfix(v.z,inv_dist);
  return result;
}




Vec4f mulMatVec(Mat4f mat, Vec4f v){
  Vec4f result;
  result.x = dot4(mat.row0, v);
  result.y = dot4(mat.row1, v);
  result.z = dot4(mat.row2, v);
  result.w = dot4(mat.row3, v);
  return result;
}

Mat4f mulMatMat(Mat4f mat1, Mat4f mat2){
  Mat4f result;
  Vec4f mat2col0 = {mat2.row0.x, mat2.row1.x,mat2.row2.x,mat2.row3.x};
  Vec4f mat2col1 = {mat2.row0.y, mat2.row1.y,mat2.row2.y,mat2.row3.y};
  Vec4f mat2col2 = {mat2.row0.z, mat2.row1.z,mat2.row2.z,mat2.row3.z};
  Vec4f mat2col3 = {mat2.row0.w, mat2.row1.w,mat2.row2.w,mat2.row3.w};
  result.row0.x = dot4(mat1.row0, mat2col0);
  result.row0.y = dot4(mat1.row0, mat2col1);
  result.row0.z = dot4(mat1.row0, mat2col2);
  result.row0.w = dot4(mat1.row0, mat2col3);
  result.row1.x = dot4(mat1.row1, mat2col0);
  result.row1.y = dot4(mat1.row1, mat2col1);
  result.row1.z = dot4(mat1.row1, mat2col2);
  result.row1.w = dot4(mat1.row1, mat2col3);
  result.row2.x = dot4(mat1.row2, mat2col0);
  result.row2.y = dot4(mat1.row2, mat2col1);
  result.row2.z = dot4(mat1.row2, mat2col2);
  result.row2.w = dot4(mat1.row2, mat2col3);
  result.row3.x = dot4(mat1.row3, mat2col0);
  result.row3.y = dot4(mat1.row3, mat2col1);
  result.row3.z = dot4(mat1.row3, mat2col2);
  result.row3.w = dot4(mat1.row3, mat2col3);

  return result;
}

fix  fixSin(short i){
  while(i<0) i+=360;
  while(i>=360) i-=360;
  if(i<90)  return(sinTable[i]); else
  if(i<180) return(sinTable[180-i]); else
  if(i<270) return(-sinTable[i-180]); else
            return(-sinTable[360-i]);
}

fix  fixCos(short i){
  return fixSin(i + 90);
}



fix32   dot32(Vec3f32 v1, Vec3f32 v2){
  return multfix32(v1.x,v2.x)+multfix32(v1.y,v2.y)+multfix32(v1.z,v2.z);
}

fix32   dot432(Vec4f32 v1, Vec4f32 v2){
  return  multfix32(v1.x,v2.x)+multfix32(v1.y,v2.y)+multfix32(v1.z,v2.z)+multfix32(v1.w,v2.w);
}

Vec3f32 cross32(Vec3f32 v1, Vec3f32 v2){
  Vec3f32 result;
  result.x = multfix32(v1.y,v2.z)-multfix32(v1.z,v2.y);
  result.y = multfix32(v1.z,v2.x)-multfix32(v1.x,v2.z);
  result.z = multfix32(v1.x,v2.y)-multfix32(v1.y,v2.x);
  return result;
}

Vec3f32 normalize32(Vec3f32 v){
  fix32 sqrdist = multfix32(v.x,v.x)+multfix32(v.y,v.y)+multfix32(v.z,v.z);
  fix32 dist = sqrtfix32(sqrdist);
  fix32 inv_dist = divfix32(int2fix32(1),dist);
  Vec3f32 result;
  result.x = multfix32(v.x,inv_dist);
  result.y = multfix32(v.y,inv_dist);
  result.z = multfix32(v.z,inv_dist);
  return result;
}
Vec3f32 normalizeScale32(Vec3f32 v, fix32 scale){
  fix32 sqrdist = multfix32(v.x,v.x)+multfix32(v.y,v.y)+multfix32(v.z,v.z);
  fix32 dist = sqrtfix32(sqrdist);
  fix32 inv_dist = divfix32(int2fix32(1),dist);
  fix32 inv_distScale = multfix32(inv_dist, scale);
  Vec3f32 result;
  result.x = multfix32(v.x, inv_distScale);
  result.y = multfix32(v.y,inv_distScale);
  result.z = multfix32(v.z,inv_distScale);
  return result;
}

Vec4f32 mulMatVec32(Mat4f32 mat, Vec4f32 v){
  Vec4f32 result;
  result.x = dot432(mat.row0, v);
  result.y = dot432(mat.row1, v);
  result.z = dot432(mat.row2, v);
  result.w = dot432(mat.row3, v);
  return result;
}

Mat4f32 mulMatMat32(Mat4f32 mat1, Mat4f32 mat2){
  Mat4f32 result;
  Vec4f32 mat2col0 = {mat2.row0.x, mat2.row1.x,mat2.row2.x,mat2.row3.x};
  Vec4f32 mat2col1 = {mat2.row0.y, mat2.row1.y,mat2.row2.y,mat2.row3.y};
  Vec4f32 mat2col2 = {mat2.row0.z, mat2.row1.z,mat2.row2.z,mat2.row3.z};
  Vec4f32 mat2col3 = {mat2.row0.w, mat2.row1.w,mat2.row2.w,mat2.row3.w};
  result.row0.x = dot432(mat1.row0, mat2col0);
  result.row0.y = dot432(mat1.row0, mat2col1);
  result.row0.z = dot432(mat1.row0, mat2col2);
  result.row0.w = dot432(mat1.row0, mat2col3);
  result.row1.x = dot432(mat1.row1, mat2col0);
  result.row1.y = dot432(mat1.row1, mat2col1);
  result.row1.z = dot432(mat1.row1, mat2col2);
  result.row1.w = dot432(mat1.row1, mat2col3);
  result.row2.x = dot432(mat1.row2, mat2col0);
  result.row2.y = dot432(mat1.row2, mat2col1);
  result.row2.z = dot432(mat1.row2, mat2col2);
  result.row2.w = dot432(mat1.row2, mat2col3);
  result.row3.x = dot432(mat1.row3, mat2col0);
  result.row3.y = dot432(mat1.row3, mat2col1);
  result.row3.z = dot432(mat1.row3, mat2col2);
  result.row3.w = dot432(mat1.row3, mat2col3);

  return result;
}

fix32  fixSin32(short i){
  while(i<0) i+=360;
  while(i>=360) i-=360;
  if(i<90)  return(sinTable32[i]); else
  if(i<180) return(sinTable32[180-i]); else
  if(i<270) return(-sinTable32[i-180]); else
            return(-sinTable32[360-i]);
}

fix32  fixCos32(short i){
  return fixSin32(i + 90);
}

