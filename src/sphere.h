//
// Created by felixfifi on 03.04.20.
//

#ifndef RTX_RAYTRACER_SPHERE_H
#define RTX_RAYTRACER_SPHERE_H

#include "hittable.h"

class sphere : public hittable {
public:
    sphere() {}
    sphere(vec3 cen, double r) : center(cen), radius(r) {};

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const;
public:
    vec3 center;
    double radius;
};

bool sphere::hit(const ray &r, double t_min, double t_max, hit_record &rec) const {
    vec3 oc = r.origin() - center;

    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius * radius;
    auto discriminant = half_b * half_b - a*c;

    if (discriminant > 0) {
        auto root = sqrt(discriminant);
        auto temp = (-half_b - root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;
            rec.set_face_normal(r, outward_normal);
            return true;
        }

        temp = (-half_b + root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;
            rec.set_face_normal(r, outward_normal);
            return true;
        }
    }
    return false;
}

#endif //RTX_RAYTRACER_SPHERE_H
