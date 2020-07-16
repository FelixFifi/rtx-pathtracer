//
// Created by felixfifi on 09.07.20.
//

#ifndef RTX_RAYTRACER_SPHERE_H
#define RTX_RAYTRACER_SPHERE_H


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


nvvkpp::RaytracingBuilderKHR::Blas spheresToBlas(vk::Device device, uint32_t sphereCount, vk::Buffer aabbBuffer);

#endif //RTX_RAYTRACER_SPHERE_H
