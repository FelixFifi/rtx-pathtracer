//
// Created by felixfifi on 08.04.20.
//

#ifndef RTX_RAYTRACER_AABB_H
#define RTX_RAYTRACER_AABB_H

#include "hittable.h"

class AABB {
public:
    AABB() {};

    AABB(std::shared_ptr<std::vector<vec3>> vertices) {

        double x_min = infinity, y_min = infinity, z_min = infinity;
        double x_max = -infinity, y_max = -infinity, z_max = -infinity;
        for(const vec3& v : *vertices) {
            x_min = std::min(x_min, v.x());
            y_min = std::min(y_min, v.y());
            z_min = std::min(z_min, v.z());
            x_max = std::max(x_max, v.x());
            y_max = std::max(y_max, v.y());
            z_max = std::max(z_max, v.z());
        }

        min_v = vec3(x_min, y_min, z_min);
        max_v = vec3(x_max, y_max, z_max);
    }

    virtual bool hit(const ray& r, double t_min, double t_max) const {
        vec3 t_near = (min_v - r.origin()) / r.direction();
        vec3 t_far = (max_v - r.origin()) / r.direction();

        if (r.direction().x() == 0) {
            if (r.origin().x() >= min_v.x() && r.origin().x() <= max_v.x()) {
                t_near.e[0] = -infinity;
                t_far.e[0] = infinity;
            } else {
                return false;
            }
        }

        if (r.direction().y() == 0) {
            if (r.origin().y() >= min_v.y() && r.origin().y() <= max_v.y()) {
                t_near.e[1] = -infinity;
                t_far.e[1] = infinity;
            } else {
                return false;
            }
        }

        if (r.direction().z() == 0) {
            if (r.origin().z() >= min_v.z() && r.origin().z() <= max_v.z()) {
                t_near.e[2] = -infinity;
                t_far.e[2] = infinity;
            } else {
                return false;
            }
        }

        if (r.direction().x() < 0) {
            std::swap(t_near.e[0], t_far.e[0]);
        }

        if (r.direction().y() < 0) {
            std::swap(t_near.e[1], t_far.e[1]);
        }

        if (r.direction().z() < 0) {
            std::swap(t_near.e[2], t_far.e[2]);
        }

        double near_max = *std::max_element(t_near.e, t_near.e + 3);
        double far_min = *std::min_element(t_far.e, t_far.e + 3);

        if (near_max > t_max || far_min < t_min) {
            return false;
        }

        return (near_max <= far_min);

    }
public:
    vec3 min_v, max_v;
};

#endif //RTX_RAYTRACER_AABB_H
