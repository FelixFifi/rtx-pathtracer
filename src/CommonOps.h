//
// Created by felixfifi on 04.05.20.
//

#ifndef RTX_RAYTRACER_COMMONOPS_H
#define RTX_RAYTRACER_COMMONOPS_H

#include <vector>
#include <fstream>

std::vector<char> readFile(const std::string &filename);

void writeEXR(const char fileName[], const float *pixels, int width, int height);

#endif //RTX_RAYTRACER_COMMONOPS_H
