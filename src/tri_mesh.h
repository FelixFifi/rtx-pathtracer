//
// Created by felixfifi on 06.04.20.
//

#ifndef RTX_RAYTRACER_TRI_MESH_H
#define RTX_RAYTRACER_TRI_MESH_H

#include "hittable.h"
#include "AABB.h"
#include <vector>

using std::vector;
using std::shared_ptr;

struct index {
    unsigned int iv;
    unsigned int it;
    unsigned int in;

    index(unsigned int iv, unsigned int it, unsigned int in) : iv(iv), it(it), in(in) {}
};

class tri_mesh : public hittable {
public:
    tri_mesh(shared_ptr<vector<vec3>> vertices, shared_ptr<vector<vec3>> normals,shared_ptr<vector<vec3>> texture_coords,
             shared_ptr<vector<vector<index>>> faces, shared_ptr<material> mat_ptr) :
    vertices(vertices), normals(normals), texture_coords(texture_coords), faces(faces), mat_ptr(mat_ptr) {
        aabb = AABB(vertices);
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
        if (!aabb.hit(r, t_min, t_max)) {
            return false;
        }

        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = t_max;

        for (const auto& face: *faces) {
            vec3 a = vertices->at(face[0].iv);
            vec3 b = vertices->at(face[1].iv);
            vec3 c = vertices->at(face[2].iv);

            vec3 normal = cross(b - a, c - a);
            vec3 u_normal = unit_vector(normal);

            double plane_offset = dot(u_normal, a);

            double t = (plane_offset - dot(r.origin(), u_normal)) / dot(r.direction(), u_normal);

            if (!(t >= t_min && t<= closest_so_far)) {
                continue;
            }

            vec3 p = r.at(t);

            vec3 na = cross(c-b, p-b);
            vec3 nb = cross(a-c, p-c);
            vec3 nc = cross(b-a, p-a);

            double la = dot(na, u_normal) / normal.length();
            double lb = dot(nb, u_normal) / normal.length();
            double lc = dot(nc, u_normal) / normal.length();

            if ((la >= 0) && (la <= 1.0) && (lb >= 0) && (lb <= 1.0) && (lc >= 0) && (lc <= 1.0)) {
                vec3 normal_a = normals->at(face[0].in);
                vec3 normal_b = normals->at(face[1].in);
                vec3 normal_c = normals->at(face[2].in);

                vec3 interpolated_normal = unit_vector(la * normal_a + lb * normal_b + lc * normal_c);
                rec.set_face_normal(r, interpolated_normal);
                rec.t = t;
                rec.p = p;
                rec.mat_ptr = mat_ptr;

                hit_anything = true;
                closest_so_far = t;
            }
        }
        return hit_anything;
    }

public:
    shared_ptr<vector<vec3>> vertices;
    shared_ptr<vector<vec3>> normals;
    shared_ptr<vector<vec3>> texture_coords;
    shared_ptr<vector<vector<index>>> faces;
    shared_ptr<material> mat_ptr;
    AABB aabb;
};

#endif //RTX_RAYTRACER_TRI_MESH_H
