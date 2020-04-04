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
hittable_list random_scene();

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
    const int IMAGE_WIDTH = 600;
    const int IMAGE_HEIGHT = 300;

    const std::string filename = "simple.ppm";
    const int SAMPLES_PER_PIXEL = 100;
    const int MAX_DEPTH = 20;

    simpleRaytracing(filename, IMAGE_WIDTH, IMAGE_HEIGHT, SAMPLES_PER_PIXEL, MAX_DEPTH);

    return 0;
}

void simpleRaytracing(const std::string &filename, const int IMAGE_WIDTH, const int IMAGE_HEIGHT,
                      const int SAMPLES_PER_PIXEL, const int MAX_DEPTH) {
    double aspect_ratio = ((double) IMAGE_WIDTH) / IMAGE_HEIGHT;
    vec3 lookfrom(13,2,3);
    vec3 lookat(0,0,0);
    vec3 vup(0,1,0);

    camera cam(lookfrom, lookat, vup, 20, aspect_ratio);

    hittable_list world = random_scene();

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
        std::cerr << "Row " << IMAGE_HEIGHT - row << " of " << IMAGE_HEIGHT << std::endl;
    }
}

hittable_list createWorld() {
    hittable_list world;
    world.add(make_shared<sphere>(vec3(0,0,-1), 0.5, make_shared<lambertian>(vec3(0.1, 0.2, 0.5))));
    world.add(make_shared<sphere>(
            vec3(0,-100.5,-1), 100, make_shared<lambertian>(vec3(0.8, 0.8, 0.0))));
    world.add(make_shared<sphere>(vec3(1,0,-1), 0.5, make_shared<metal>(vec3(0.8, 0.6, 0.2), 0.3)));
    world.add(make_shared<sphere>(vec3(-1,0,-1), 0.5, make_shared<dielectric>(1.5)));
    world.add(make_shared<sphere>(vec3(-1,0,-1), -0.45, make_shared<dielectric>(1.5)));
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

hittable_list random_scene() {
    hittable_list world;

    world.add(make_shared<sphere>(
            vec3(0,-1000,0), 1000, make_shared<lambertian>(vec3(0.5, 0.5, 0.5))));

    int i = 1;
    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = random_double();
            vec3 center(a + 0.9*random_double(), 0.2, b + 0.9*random_double());
            if ((center - vec3(4, 0.2, 0)).length() > 0.9) {
                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = vec3::random() * vec3::random();
                    world.add(
                            make_shared<sphere>(center, 0.2, make_shared<lambertian>(albedo)));
                } else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = vec3::random(.5, 1);
                    auto fuzz = random_double(0, .5);
                    world.add(
                            make_shared<sphere>(center, 0.2, make_shared<metal>(albedo, fuzz)));
                } else {
                    // glass
                    world.add(make_shared<sphere>(center, 0.2, make_shared<dielectric>(1.5)));
                }
            }
        }
    }

    world.add(make_shared<sphere>(vec3(0, 1, 0), 1.0, make_shared<dielectric>(1.5)));

    world.add(
            make_shared<sphere>(vec3(-4, 1, 0), 1.0, make_shared<lambertian>(vec3(0.4, 0.2, 0.1))));

    world.add(
            make_shared<sphere>(vec3(4, 1, 0), 1.0, make_shared<metal>(vec3(0.7, 0.6, 0.5), 0.0)));

    return world;
}
