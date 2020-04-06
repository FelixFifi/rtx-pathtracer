//
// Created by felixfifi on 06.04.20.
//

#ifndef RTX_RAYTRACER_OBJ_READER_H
#define RTX_RAYTRACER_OBJ_READER_H

#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include "hittable_list.h"
#include "tri.h"
#include "material.h"
#include "tri_mesh.h"

using std::vector;
using std::shared_ptr;

static bool parseFaceValue(std::istringstream& iss, int& iv, int& it, int& in);

int toArrayIndex(int i, unsigned long count);

shared_ptr<tri_mesh> readObjFile(const std::string& filepath) {
    std::ifstream ifstream(filepath);

    auto vertices = make_shared<vector<vec3>>();
    auto normals = make_shared<vector<vec3>>();
    auto texture_coords = make_shared<vector<vec3>>();
    auto faces = make_shared<vector<vector<index>>>();

    std::string line;
    while (std::getline(ifstream, line)) {
        std::istringstream iss(line);

        std::string type;
        iss >> type;

        if (type == "#") {
            //ignore
        } else if(type == "v") {
            double x, y, z;
            if (!(iss >> x >> y >> z)) {
                std::cerr << "Failed parsing vertex in '" << line << "'" << std::endl;
                break;
            }

            vertices->emplace_back(vec3(x,y,z));
        } else if(type == "vn") {
            double x, y, z;
            if (!(iss >> x >> y >> z)) {
                std::cerr << "Failed parsing normals in '" << line << "'" << std::endl;
                break;
            }

            normals->emplace_back(vec3(x,y,z));
        } else if(type == "vt") {
            double u, v, w;
            if (!(iss >> u >> v >> w)) {
                std::cerr << "Failed parsing texture coordinates in '" << line << "'" << std::endl;
                break;
            }

            texture_coords->emplace_back(vec3(u, v, w));
        } else if(type == "f") {
            vector<index> indices;

            int iv, it, in;

            unsigned long count = vertices->size();
            while (parseFaceValue(iss, iv, it, in)) {
                indices.emplace_back(index(toArrayIndex(iv, count), toArrayIndex(it, count), toArrayIndex(in, count)));
            }

            switch (indices.size()) {
                case 3: {
                    faces->emplace_back(indices);
                    break;
                }
                case 4: {
                    vector<index> indices1;
                    indices1.push_back(indices[0]);
                    indices1.push_back(indices[1]);
                    indices1.push_back(indices[2]);
                    vector<index> indices2;
                    indices2.push_back(indices[0]);
                    indices2.push_back(indices[2]);
                    indices2.push_back(indices[3]);

                    faces->push_back(indices1);
                    faces->push_back(indices2);
                    break;
                }
                default:
                    std::cerr << "Polygon not supported" << std::endl;
            }
        }
    }

     return make_shared<tri_mesh>(vertices, normals, texture_coords, faces, make_shared<lambertian>(vec3(0.3,0.3,0.3)));
}

int toArrayIndex(int i, unsigned long count) {
    if (i == 0) {
        return -1;
    }

    if (i < 0) {
        i = count + i;
    } else {
        i -= 1;
    }
    return i;
}

/**
 * Parses a face value e.g. vi or vi/vt or vi/vt/vn or vi//vn
 * @param iss
 * @param iv vertex_index
 * @param it texture_index
 * @param in normal_index
 * @return
 */
bool parseFaceValue(std::istringstream &iss, int &iv, int &it, int &in) {
    if(!(iss >> iv)) {
        return false;
    }

    it = 0;
    in = 0;

    if (iss.peek() == (int) '/') {
        iss.get();
        if (iss.peek() == (int) '/') {
            iss.get();
            if(!(iss >> in)) {
                return false;
            }
        } else {
            if(!(iss >> it)) {
                return false;
            }

            if (iss.peek() == (int) '/') {
                iss.get();
                if(!(iss >> in)) {
                    return false;
                }
            }
        }
    }

    return true;
}

#endif //RTX_RAYTRACER_OBJ_READER_H
