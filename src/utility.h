//
// Created by felixfifi on 03.04.20.
//

#ifndef RTX_RAYTRACER_UTILITY_H
#define RTX_RAYTRACER_UTILITY_H

#include <limits>

inline double clamp(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

inline double infinity() {std::numeric_limits<double>::infinity();}

#endif //RTX_RAYTRACER_UTILITY_H
