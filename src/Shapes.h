//
// Created by felixfifi on 09.07.20.
//

#ifndef RTX_RAYTRACER_SHAPES_H
#define RTX_RAYTRACER_SHAPES_H


#include "VulkanLoader.h"
#include <glm/vec3.hpp>

#define ALLOC_DEDICATED

#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>

struct Aabb {
    glm::vec3 min;
    glm::vec3 max;

    glm::vec3 getSize() const {
        return max - min;
    }

    /**
     *
     * @param axis -1 for largest, else 0,1,2
     * @return
     */
    std::tuple<Aabb, Aabb> splitAabb(uint axis = -1) const {
        glm::vec3 size = getSize();
        if (axis == -1) {
            // Select largest axis
            axis = size[0] > size[1] ? (size[0] > size[2] ? 0 : 2) : (size[1] > size[2] ? 1 : 2);
        }

        Aabb resL = *this;
        Aabb resR = *this;

        resL.max[axis] -= 0.5f * size[axis];
        resR.min[axis] += 0.5f * size[axis];

        return {resL, resR};
    }

    Aabb addEpsilon() {
        glm::vec3 extent = max - min;
        glm::vec3 center = min + 0.5f * extent;

        return {center - 0.50001f * extent, center + 0.50001f * extent};
    }

    void update(glm::vec3 v) {
        min[0] = fmin(min[0], v[0]);
        min[1] = fmin(min[1], v[1]);
        min[2] = fmin(min[2], v[2]);
        max[0] = fmax(max[0], v[0]);
        max[1] = fmax(max[1], v[1]);
        max[2] = fmax(max[2], v[2]);
    }
};

struct alignas(16) Sphere {
    glm::vec3 center;
    float radius;
    int matertialIndex;
    int iLight = -1;

    Aabb getAabb() const {
        const glm::vec3 &r = glm::vec3(radius, radius, radius);
        return {center - r, center + r};
    }
};


nvvkpp::RaytracingBuilderKHR::Blas
aabbToBlas(vk::Device device, uint32_t aabbCount, vk::Buffer aabbBuffer, vk::GeometryFlagBitsKHR flags);

#endif //RTX_RAYTRACER_SHAPES_H
