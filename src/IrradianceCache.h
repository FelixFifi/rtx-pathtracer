//
// Created by felixfifi on 09.07.20.
//

#ifndef RTX_RAYTRACER_IRRADIANCECACHE_H
#define RTX_RAYTRACER_IRRADIANCECACHE_H

#include "VulkanLoader.h"

#define ALLOC_DEDICATED
#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>
#include "VulkanOps.h"
#include <memory>

struct UpdateCommandsHeader {
    uint32_t maxCommands;
    uint32_t nextCommandSlot = 0;
    uint32_t nextSphereSlot = 0;
    uint32_t maxSpheres;
};

struct UpdateCommad {
    glm::vec3 center;
    float radius;
    int isFilled = 0;
    int isModify;
    uint32_t iSphere;
};

struct CacheData {
    glm::vec3 color;
    glm::vec3 normal;
};

class IrradianceCache {
private:
    uint32_t maxSpheres;
    uint32_t maxCommands;


    std::shared_ptr<VulkanOps> vulkanOps;
    nvvkpp::RaytracingBuilderKHR rtBuilder;

    vk::Device device;

    vk::Buffer spheresBuffer;
    vk::DeviceMemory spheresBufferMemory;
    vk::Buffer aabbsBuffer;
    vk::DeviceMemory aabbsBufferMemory;
    vk::Buffer cacheBuffer;
    vk::DeviceMemory cacheBufferMemory;

    vk::Buffer updateCommandsBuffer;
    vk::DeviceMemory updateCommandsBufferMemory;

    vk::DescriptorSetLayout compSetLayout;
    vk::DescriptorPool compPool;
    vk::DescriptorSet compSet;

    vk::PipelineLayout compPipelineLayout;
    vk::Pipeline compPipeline;

    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> instances;
    vk::AccelerationStructureKHR accelerationStructure;
public:
    IrradianceCache() = default;

    IrradianceCache(uint32_t maxSpheres, uint32_t maxCommands, std::shared_ptr<VulkanOps> vulkanOps,
                    vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex);

    void cleanUp();

    void updateSpheres();

    std::array<vk::DescriptorSetLayoutBinding, 4> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, 4> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 4>
    getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                           vk::DescriptorBufferInfo &outUpdateBufferInfo,
                           vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                           vk::DescriptorBufferInfo &outSpheresBufferInfo,
                           vk::DescriptorBufferInfo &outCacheBufferInfo);

private:
    void createBuffers();

    void createAccelerationStructure();

    void createComputeDescriptorSet();

    void updateComputeDescriptorSet();

    void createComputePipeline();
};


#endif //RTX_RAYTRACER_IRRADIANCECACHE_H
