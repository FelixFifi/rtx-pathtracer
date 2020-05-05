//
// Created by felixfifi on 04.05.20.
//

#ifndef RTX_RAYTRACER_COMMONOPS_H
#define RTX_RAYTRACER_COMMONOPS_H

#include <vector>
#include <fstream>

static std::vector<char> readFile(const std::string &filename) {
    std::ifstream file(filename, std::ios::ate | std::ios::binary);

    if (!file.is_open()) {
        throw std::runtime_error("failed to open file!");
    }

    size_t fileSize = (size_t) file.tellg();
    std::vector<char> buffer(fileSize);

    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();

    return buffer;
}

#endif //RTX_RAYTRACER_COMMONOPS_H
