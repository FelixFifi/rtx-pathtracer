//
// Created by felixfifi on 04.05.20.
//

#ifndef RTX_RAYTRACER_COMMONOPS_H
#define RTX_RAYTRACER_COMMONOPS_H

#include <vector>
#include <fstream>
#include <memory>

std::vector<char> readFile(const std::string &filename);

void writeEXR(const char fileName[], const float *pixels, int width, int height);
std::vector<float> readEXR(const char *filename, int &width, int &height);

#endif //RTX_RAYTRACER_COMMONOPS_H
