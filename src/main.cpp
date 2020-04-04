#include <iostream>
#include <fstream>
#include "vec3.h"
#include "camera.h"
#include "hittable_list.h"
#include "sphere.h"
#include "material.h"


void createSimplePPM(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT);
void simpleRaytracing(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT,
                      const int SAMPLES_PER_PIXEL, const int MAX_DEPTH);


hittable_list createWorld();

vec3 rayColor(const hittable_list& world, const ray& r, int depth) {

    // If we exceed the ray bounce limit => no more color contribution
    if (depth <= 0) {
        return vec3(0,0,0);
    }

    hit_record rec;
    if (world.hit(r, 0.001, infinity, rec)) {
        ray scattered;
        vec3 attenuation;

        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered)) {
            return attenuation * rayColor(world, scattered, depth - 1);
        } 
        return vec3(0,0,0);
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
    const int SAMPLES_PER_PIXEL = 100;
    const int MAX_DEPTH = 20;

    simpleRaytracing(filename, IMAGE_WIDTH, IMAGE_HEIGHT, SAMPLES_PER_PIXEL, MAX_DEPTH);

    return 0;
}

void simpleRaytracing(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT,
                      const int SAMPLES_PER_PIXEL, const int MAX_DEPTH) {
    camera cam = camera();

    hittable_list world = createWorld();

    std::fstream fstream = std::fstream();
    fstream.open(filename, std::_S_out);

    fstream << "P3\n" << IMAGE_WIDTH << " " << IMAGE_HEIGHT << "\n" << "255\n";
    for (int row = IMAGE_HEIGHT - 1; row >= 0 ; --row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            vec3 color;

            for (int iSample = 0; iSample < SAMPLES_PER_PIXEL; ++iSample) {
                double u = (col + random_double()) / IMAGE_WIDTH;
                double v = (row + random_double()) / IMAGE_HEIGHT;
                ray r = cam.get_ray(u, v);
                color += rayColor(world, r, MAX_DEPTH);
            }
            color.write_color(fstream, SAMPLES_PER_PIXEL);
        }
        fstream << "\n";
    }
}

hittable_list createWorld() {
    hittable_list world;
    // Diffuse
    auto mat1 = std::make_shared<lambertian>(vec3(0.7, 0.3, 0.3));
    auto mat2 = std::make_shared<lambertian>(vec3(0.8, 0.8, 0));

    world.add(std::make_shared<sphere>(vec3(0, 0, -1), 0.5, mat1));
    world.add(std::make_shared<sphere>(vec3(0, -100.5, -1), 100, mat2));

    // Metal
    const auto &matMetal1 = make_shared<metal>(vec3(0.8, 0.6, 0.2));
    const auto &matMetal2 = make_shared<metal>(vec3(0.8, 0.8, 0.8));
    world.add(make_shared<sphere>(vec3(1, 0, -1), 0.5, matMetal1));
    world.add(make_shared<sphere>(vec3(-1, 0, -1), 0.5, matMetal2));
    return world;
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
