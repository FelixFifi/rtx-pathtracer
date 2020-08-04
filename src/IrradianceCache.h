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

struct CacheHeader {
    uint32_t nextCacheSlot = 0;
    uint32_t maxCaches;
    uint32_t nextUpdateSlot = 0;
};

struct CacheData {
    glm::vec3 color;
    glm::vec3 normal;
    glm::vec3 rotGrad;
    glm::vec3 transGrad;
    float harmonicR = -1;
    uint numUpdates = 0;
};

class IrradianceCache {
private:
    uint32_t maxCaches;

    std::shared_ptr<VulkanOps> vulkanOps;
    nvvkpp::RaytracingBuilderKHR rtBuilder;

    vk::Device device;

    vk::Buffer spheresBuffer;
    vk::DeviceMemory spheresBufferMemory;
    vk::Buffer aabbsBuffer;
    vk::DeviceMemory aabbsBufferMemory;
    vk::Buffer cacheBuffer;
    vk::DeviceMemory cacheBufferMemory;

    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> instances;
    vk::AccelerationStructureKHR accelerationStructure;
public:
    IrradianceCache() = default;

    IrradianceCache(uint32_t maxCaches, std::shared_ptr<VulkanOps> vulkanOps,
                    vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex);

    void cleanUp();

    void updateSpheres();

    std::array<vk::DescriptorSetLayoutBinding, 4> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, 4> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet ,4>
    getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                           vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                           vk::DescriptorBufferInfo &outSpheresBufferInfo,
                           vk::DescriptorBufferInfo &outCacheBufferInfo,
                           vk::DescriptorBufferInfo &outAabbsBufferInfo);

private:
    void createBuffers();

    void createAccelerationStructure();
};


#endif //RTX_RAYTRACER_IRRADIANCECACHE_H
