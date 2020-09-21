//
// Created by felixfifi on 19.09.20.
//

#ifndef RTX_RAYTRACER_PATHGUIDING_H
#define RTX_RAYTRACER_PATHGUIDING_H

#include "VulkanLoader.h"

#include "Shapes.h"
#include "VulkanOps.h"
#include <nvvkpp/raytraceKHR_vkpp.hpp>

#define MAX_DISTRIBUTIONS 10

// Taken from lightpmm/VMFKernel.h
// the minumum value of kappa before it gets set to 0.0 for numerical stability
#define VMF_MinKappa 1e-3f

static const int BINDING_AABB = 15;
static const int BINDING_GUIDING = 16;
static const int BINDING_AS = 17;

struct VMF_Theta {
    glm::vec3 mu;
    float k;
    float norm; // = k / (M_2PI * (1 - exp(- 2 * theta.k)))
    float eMin2K; // exp(-2.0f * k)

    void setK(float newK) {
        newK = newK < VMF_MinKappa ? 0.0 : newK;

        k = newK;
        norm = k / (2 * M_PI * (1 - exp(- 2 * k)));
        eMin2K = exp(-2.0 * k);
    }
};

struct VMM_Theta {
    VMF_Theta thetas[MAX_DISTRIBUTIONS];
    float pi[MAX_DISTRIBUTIONS];
    int usedDistributions;
};

class PathGuiding {
private:
    Aabb sceneAabb;

    std::vector<Aabb> aabbs;
    std::vector<VMM_Theta> guidingRegions;
    std::shared_ptr<VulkanOps> vulkanOps;

    nvvkpp::RaytracingBuilderKHR rtBuilder;

    vk::Device device;

    vk::Buffer aabbsBuffer;
    vk::DeviceMemory aabbsBufferMemory;
    vk::Buffer guidingBuffer;
    vk::DeviceMemory guidingBufferMemory;

    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> instances;
    vk::AccelerationStructureKHR accelerationStructure;
public:
    PathGuiding() = default;
    PathGuiding(uint splitCount, Aabb sceneAabb, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex);


    std::array<vk::DescriptorSetLayoutBinding, 3> getDescriptorSetLayouts();

    static std::array<vk::DescriptorPoolSize, 3> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 3> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                 vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                                                                 vk::DescriptorBufferInfo &outAabbsBufferInfo,
                                                                 vk::DescriptorBufferInfo &outGuidingBufferInfo);

    void cleanup();
private:
    void createDummyRegions(uint splitCount, const Aabb &sceneAabb);
    void createBuffers();
    void createAS();

    void cleanupBuffers() const;
};


#endif //RTX_RAYTRACER_PATHGUIDING_H
