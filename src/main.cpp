#include <iostream>
#include "RayTracingApp.h"


int main() {
    const int WIDTH = 1920;
    const int HEIGHT = 1080;
    RayTracingApp app(WIDTH, HEIGHT);

    app.run();
    app.cleanup();

    return EXIT_SUCCESS;
}
