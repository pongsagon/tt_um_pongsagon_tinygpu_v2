

#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>

#include "includes/glm/glm.hpp"

bool loadOBJ(
    const char * path,
    std::vector<glm::vec3> & out_vertices,
    std::vector<glm::vec2> & out_uvs,
    std::vector<glm::vec3> & out_normals,
    std::vector<int>    &idx1,                  // normal must share the same index as pos
    std::vector<int>    &idx2,
    std::vector<int>    &idx3,
    std::vector<int>    &uv_idx1,
    std::vector<int>    &uv_idx2,
    std::vector<int>    &uv_idx3
);

#endif
