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
            vec3 reflected = reflect(r_in.direction(), rec.normal);
            scattered = ray(rec.p, reflected);
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

double schlick(double cosine, double ref_idx);

class dielectric : public material {
public:
    dielectric(double ri) : refraction_index(ri), one_over_ri(1.0/ri) {}

    virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered) const {
        attenuation = vec3(1.0, 1.0, 1.0);

        double etai_over_etao;
        etai_over_etao = rec.front_face ? one_over_ri : refraction_index;

        const vec3 &normed_dir = unit_vector(r_in.direction());
        double cos_theta = fmin(dot(-normed_dir, rec.normal), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta*cos_theta);

        if (etai_over_etao * sin_theta > 1.0) {
            // Must reflect
            vec3 reflected = reflect(r_in.direction(), rec.normal);
            scattered = ray(rec.p, reflected);
            return true;
        }

        double reflect_prob = schlick(cos_theta, refraction_index);

        if (random_double() < reflect_prob) {
            // Chosen to reflect
            vec3 reflected = reflect(r_in.direction(), rec.normal);
            scattered = ray(rec.p, reflected);
            return true;
        }
        // Chosen to refract
        vec3 refracted_direction = refract(normed_dir, rec.normal, etai_over_etao);

        scattered = ray(rec.p, refracted_direction);
        return true;
    }
public:
    double refraction_index;
private:
    double one_over_ri;
};


double schlick(double cosine, double ref_idx) {
    auto r0 = (1-ref_idx) / (1+ref_idx);
    r0 = r0*r0;
    return r0 + (1-r0)*pow((1 - cosine),5);
}

#endif //RTX_RAYTRACER_MATERIAL_H
