#include <iostream>
#include "RayTracingApp.h"


int main() {
    const int WIDTH = 800;
    const int HEIGHT = 800;
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
