#include <iostream>
#include "RayTracingApp.h"

void printHelp() {
    std::cout << "Required parameters: WIDTH HEIGHT IC_SIZE GUIDING_SPLITS" << std::endl;
}

int main(int argc, char **argv) {
    if (argc != 5) {
        printHelp();
        return EXIT_FAILURE;
    }

    int WIDTH;
    int HEIGHT;
    int IC_SIZE, GUIDING_SPLITS;

    WIDTH = std::stoi(argv[1]);
    HEIGHT = std::stoi(argv[2]);
    IC_SIZE = std::stoi(argv[3]);
    GUIDING_SPLITS = std::stoi(argv[4]);

    RayTracingApp app(WIDTH, HEIGHT, IC_SIZE, GUIDING_SPLITS);

    app.run();
    app.cleanup();

    return EXIT_SUCCESS;
}
