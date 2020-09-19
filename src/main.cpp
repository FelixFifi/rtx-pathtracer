#include <iostream>
#include "RayTracingApp.h"

void printHelp() {
    std::cout << "Required parameters: WIDTH HEIGHT IC_SIZE" << std::endl;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        printHelp();
        return EXIT_FAILURE;
    }

    int WIDTH;
    int HEIGHT;
    int IC_SIZE;

    WIDTH = std::stoi(argv[1]);
    HEIGHT = std::stoi(argv[2]);
    IC_SIZE = std::stoi(argv[3]);

    RayTracingApp app(WIDTH, HEIGHT, IC_SIZE);

    app.run();
    app.cleanup();

    return EXIT_SUCCESS;
}
