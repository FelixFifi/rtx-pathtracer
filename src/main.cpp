#include <iostream>
#include "RayTracingApp.h"


int main() {
    const int WIDTH = 768;
    const int HEIGHT = 512;
    RayTracingApp app(WIDTH, HEIGHT);

    app.run();
    app.cleanup();

    return EXIT_SUCCESS;
}
