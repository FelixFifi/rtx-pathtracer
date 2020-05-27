//
// Created by felixfifi on 27.05.20.
//

#include "CommonOps.h"
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfChannelList.h>

void writeEXR(const char *fileName, const float *pixels, int width, int height) {
    Imf::Header header(width,
                       height);
    header.channels().insert("R", Imf::Channel(Imf::FLOAT));
    header.channels().insert("G", Imf::Channel(Imf::FLOAT));
    header.channels().insert("B", Imf::Channel(Imf::FLOAT));
    header.channels().insert("A", Imf::Channel(Imf::FLOAT));

    Imf::OutputFile file(fileName, header);
    Imf::FrameBuffer frameBuffer;
    frameBuffer.insert("R",                                // name
                       Imf::Slice(Imf::FLOAT,                       // type
                                  (char *) pixels,            // base
                                  sizeof(*pixels) * 4,       // xStride
                                  sizeof(*pixels) * width * 4)); // yStride
    frameBuffer.insert("G",                                // name
                       Imf::Slice(Imf::FLOAT,                       // type
                                  (char *) (pixels + 1),            // base
                                  sizeof(*pixels) * 4,       // xStride
                                  sizeof(*pixels) * width * 4)); // yStride
    frameBuffer.insert("B",                                // name
                       Imf::Slice(Imf::FLOAT,                       // type
                                  (char *) (pixels + 2),            // base
                                  sizeof(*pixels) * 4,       // xStride
                                  sizeof(*pixels) * width * 4)); // yStride
    frameBuffer.insert("A",                                // name
                       Imf::Slice(Imf::FLOAT,                       // type
                                  (char *) (pixels + 3),            // base
                                  sizeof(*pixels) * 4,       // xStride
                                  sizeof(*pixels) * width * 4)); // yStride
    file.setFrameBuffer(frameBuffer);
    file.writePixels(height);
}

std::vector<char> readFile(const std::string &filename) {
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
