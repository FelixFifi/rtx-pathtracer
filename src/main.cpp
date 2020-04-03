#include <iostream>
#include <fstream>
#include "vec3.h"
#include "camera.h"
#include "hittable_list.h"
#include "sphere.h"

void createSimplePPM(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT);
void simpleRaytracing(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT);


vec3 rayColor(const hittable_list& world, const ray& r) {
    hit_record rec;
    if (world.hit(r, 0, std::numeric_limits<double>::infinity(), rec)) {
        return 0.5 * (rec.normal + vec3(1.0, 1.0, 1.0));
    }

    vec3 unitDirection = unit_vector(r.direction());
    auto t = 0.5 * (unitDirection.y() + 1.0);
    const vec3 &sky_low = vec3(1.0, 1.0, 1.0);
    const vec3 &sky_high = vec3(0.5, 0.7, 1.0);
    return (1.0 - t) * sky_low + t * sky_high;
}

int main() {
    const int IMAGE_WIDTH = 400;
    const int IMAGE_HEIGHT = 200;

    const std::string filename = "simple.ppm";
    simpleRaytracing(filename, IMAGE_WIDTH, IMAGE_HEIGHT);

    return 0;
}

void simpleRaytracing(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT) {
    camera cam = camera();

    hittable_list world;
    world.add(make_shared<sphere>(vec3(0,0,-1), 0.5));
    world.add(make_shared<sphere>(vec3(0,-100.5,-1), 100));

    std::fstream fstream = std::fstream();
    fstream.open(filename, std::_S_out);

    fstream << "P3\n" << IMAGE_WIDTH << " " << IMAGE_HEIGHT << "\n" << "255\n";
    for (int row = IMAGE_HEIGHT - 1; row >= 0 ; --row) {
        for (int col = IMAGE_WIDTH - 1; col >= 0; --col) {
            ray r = cam.get_ray((col + 0.5) / IMAGE_WIDTH, (row + 0.5) / IMAGE_HEIGHT);
            vec3 color = rayColor(world, r);
            color.write_color(fstream, 1);
        }
        fstream << "\n";
    }
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
