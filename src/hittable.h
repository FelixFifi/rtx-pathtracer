//
// Created by felixfifi on 03.04.20.
//

#ifndef RTX_RAYTRACER_HITTABLE_H
#define RTX_RAYTRACER_HITTABLE_H

#include "vec3.h"
#include "ray.h"

struct hit_record {
    double t;
    vec3 p;
    vec3 normal;
    bool front_face;

    inline void set_face_normal(const ray& r, const vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable {
public:
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const = 0;
};


#endif //RTX_RAYTRACER_HITTABLE_H
