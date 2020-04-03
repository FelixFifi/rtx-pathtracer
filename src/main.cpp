#include <iostream>
#include <fstream>

void createSimplePPM(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT);

int main() {
    const int IMAGE_WIDTH = 400;
    const int IMAGE_HEIGHT = 200;

    const std::string filename = "simple.ppm";
    createSimplePPM(filename, IMAGE_WIDTH, IMAGE_HEIGHT);

    return 0;
}

void createSimplePPM(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT) {
    std::fstream fstream = std::fstream();
    fstream.open(filename, std::_S_out);

    fstream << "P3\n" << IMAGE_WIDTH << " " << IMAGE_HEIGHT << "\n" << "255\n";
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            int r = row % 255;
            int g = col % 255;
            int b = (row * col) % 255;
            fstream << r << " " << g << " " << b << " ";
        }
        fstream << "\n";
    }
}
