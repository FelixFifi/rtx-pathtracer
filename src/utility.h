//
// Created by felixfifi on 03.04.20.
//

#ifndef RTX_RAYTRACER_UTILITY_H
#define RTX_RAYTRACER_UTILITY_H

#include <limits>
#include <functional>
#include <random>


inline double clamp(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

inline double random_double() {
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    static std::mt19937 generator;
    static std::function<double()> rand_generator =
            std::bind(distribution, generator);
    return rand_generator();
}

inline double random_double(double min, double max) {
    // Returns a random real in [min,max).
    return min + (max-min)*random_double();
}

inline double degrees_to_radians(double degree) {
    return degree / 180.0 * pi;
}

#endif //RTX_RAYTRACER_UTILITY_H
