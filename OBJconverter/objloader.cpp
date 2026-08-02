

#include "objloader.h"


bool loadOBJ(
    const char * path,
    std::vector<glm::vec3> & out_vertices,
    std::vector<glm::vec2> & out_uvs,
    std::vector<glm::vec3> & out_normals,
    std::vector<int>    &idx1,                  
    std::vector<int>    &idx2,
    std::vector<int>    &idx3,
    std::vector<int>    &uv_idx1,
    std::vector<int>    &uv_idx2,
    std::vector<int>    &uv_idx3
){
    printf("Loading OBJ file %s...\n", path);

    std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    std::vector<glm::vec3> temp_normals;


    FILE * file = fopen(path, "r");
    if( file == NULL ){
        printf("cannot open the file \n");
        return false;
    }

    while( 1 ){

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader

        if ( strcmp( lineHeader, "v" ) == 0 ){
            glm::vec3 vertex;
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z );
            out_vertices.push_back(vertex);
        }else if ( strcmp( lineHeader, "vt" ) == 0 ){
            glm::vec2 uv;
            fscanf(file, "%f %f\n", &uv.x, &uv.y );
            uv.y = uv.y;
            out_uvs.push_back(uv);
        }else if ( strcmp( lineHeader, "vn" ) == 0 ){
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
            temp_normals.push_back(normal);
        }else if ( strcmp( lineHeader, "f" ) == 0 ){
            std::string vertex1, vertex2, vertex3;
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
            if (matches != 9){
                printf("File can't be read by our simple parser :-( Try exporting with other options\n");
                return false;
            }
            // output IB, split into 3 array
            idx1.push_back(vertexIndex[0]);
            idx2.push_back(vertexIndex[1]);
            idx3.push_back(vertexIndex[2]);
            uv_idx1.push_back(uvIndex[0]);
            uv_idx2.push_back(uvIndex[1]);
            uv_idx3.push_back(uvIndex[2]);
            //
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices    .push_back(uvIndex[0]);
            uvIndices    .push_back(uvIndex[1]);
            uvIndices    .push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);
        }else{
            // Probably a comment, eat up the rest of the line
            char stupidBuffer[1000];
            fgets(stupidBuffer, 1000, file);
        }

    }

    // resize output, follow the size of pos
    out_normals.resize(out_vertices.size());

    // For each vertex of each triangle
    for( unsigned int i=0; i<vertexIndices.size(); i++ ){
       
        // Get the indices of its attributes
        unsigned int vertexIndex = vertexIndices[i];
        unsigned int normalIndex = normalIndices[i];

        out_normals[vertexIndex-1] = temp_normals[normalIndex-1];
    }

    return true;
}
