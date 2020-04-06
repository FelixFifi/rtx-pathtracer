//
// Created by felixfifi on 05.04.20.
//

#ifndef RTX_RAYTRACER_TRI_H
#define RTX_RAYTRACER_TRI_H

#include "hittable.h"

class tri : public hittable {
public:
    tri(const vec3& a, const vec3& b, const vec3& c, shared_ptr<material> mat_ptr) : a(a), b(b), c(c), mat_ptr(mat_ptr) {
        normal = cross(b - a, c - a);
        u_normal = unit_vector(normal);
        plane_offset = dot(u_normal, a);

        normal_a = u_normal;
        normal_b = u_normal;
        normal_c = u_normal;
    };

    tri(const vec3& a, const vec3& b, const vec3& c, const vec3& na, const vec3& nb, const vec3& nc, shared_ptr<material> mat_ptr) : a(a), b(b), c(c), mat_ptr(mat_ptr) {
        normal = cross(b - a, c - a);
        u_normal = unit_vector(normal);
        plane_offset = dot(u_normal, a);

        normal_a = na;
        normal_b = nb;
        normal_c = nc;
    };

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
        double t = (plane_offset - dot(r.origin(), u_normal)) / dot(r.direction(), u_normal);

        if (!(t >= t_min && t<= t_max)) {
            return false;
        }

        vec3 p = r.at(t);

        vec3 na = cross(c-b, p-b);
        vec3 nb = cross(a-c, p-c);
        vec3 nc = cross(b-a, p-a);

        double la = dot(na, u_normal) / normal.length();
        double lb = dot(nb, u_normal) / normal.length();
        double lc = dot(nc, u_normal) / normal.length();

        if ((la >= 0) && (la <= 1.0) && (lb >= 0) && (lb <= 1.0) && (lc >= 0) && (lc <= 1.0)) {
            vec3 interpolated_normal = unit_vector(la * normal_a + lb * normal_b + lc * normal_c);
            rec.set_face_normal(r, interpolated_normal);
            rec.t = t;
            rec.p = p;
            rec.mat_ptr = mat_ptr;
            return true;
        }

        return false;
    }
public:
    const vec3 a, b, c;
private:
    vec3 normal, u_normal;
    vec3 normal_a, normal_b, normal_c;
    double plane_offset;
    shared_ptr<material> mat_ptr;
};

#endif //RTX_RAYTRACER_TRI_H
