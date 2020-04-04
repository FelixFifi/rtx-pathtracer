//
// Created by felixfifi on 04.04.20.
//

#ifndef RTX_RAYTRACER_MATERIAL_H
#define RTX_RAYTRACER_MATERIAL_H

#include "ray.h"
#include "hittable.h"

class material {
public:
    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered) const = 0;
};

class lambertian : public material {
public:
    lambertian(const vec3& a) : albedo(a) {}

    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered) const {
        if (rec.front_face) {
            attenuation = albedo;
            vec3 direction = rec.normal + random_unit_vector();
            scattered = ray(rec.p, direction);
            return true;
        } else {
            attenuation = vec3(0,0,0);
            std::cerr << "hit backside of lambertian" << std::endl;
            return false;
        }
    }
public:
    vec3 albedo;
};

class metal : public material {
public:
    metal(const vec3& a) : albedo(a) {}

    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered) const {
        if (rec.front_face) {
            attenuation = albedo;
            vec3 direction = reflect(r_in.direction(), rec.normal);
            scattered = ray(rec.p, direction);
            return true;
        } else {
            attenuation = vec3(0,0,0);
            std::cerr << "hit backside of metal" << std::endl;
            return false;
        }
    }
public:
    vec3 albedo;
};

#endif //RTX_RAYTRACER_MATERIAL_H
