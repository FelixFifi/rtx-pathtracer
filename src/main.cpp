#include <iostream>
#include "RayTracingApp.h"

void printHelp() {
    std::cout << "Required parameters: WIDTH HEIGHT" << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        printHelp();
        return EXIT_FAILURE;
    }

    int WIDTH;
    int HEIGHT;

    WIDTH = std::stoi(argv[1]);
    HEIGHT = std::stoi(argv[2]);

    RayTracingApp app(WIDTH, HEIGHT);

    app.run();
    app.cleanup();

    return EXIT_SUCCESS;
}
