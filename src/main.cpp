#include <iostream>
#include "RayTracingApp.h"


int main() {
    const int WIDTH = 1280;
    const int HEIGHT = 720;
    RayTracingApp app(WIDTH, HEIGHT);

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    app.cleanup();

    return EXIT_SUCCESS;
}
