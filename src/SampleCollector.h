//
// Created by felixfifi on 22.09.20.
//

#ifndef RTX_RAYTRACER_SAMPLECOLLECTOR_H
#define RTX_RAYTRACER_SAMPLECOLLECTOR_H

#include "pmm/DirectionalData.h"
#include "VulkanLoader.h"
#include "VulkanOps.h"
#include "glm/vec3.hpp"

static const unsigned int INVALID = std::numeric_limits<uint32_t>::max();

static const int BINDING_DIRECTIONAL_DATA = 18;

using lightpmm::DirectionalData;

class SampleCollector {
private:
    uint32_t regionCount;
    vk::Extent2D imageSize;
    uint32_t numPerPixel;

    std::shared_ptr<VulkanOps> vulkanOps;

    vk::Device device;

    vk::DeviceSize bufferSize;
    vk::Buffer directionalDataBuffer;
    vk::DeviceMemory directionalDataBufferMemory;
    vk::Buffer hostDirectionalDataBuffer;
    vk::DeviceMemory hostDirectionalDataBufferMemory;
public:
    SampleCollector() = default;
    SampleCollector(uint32_t regionCount, vk::Extent2D imageSize, uint32_t numPerPixel,
                    std::shared_ptr<VulkanOps> vulkanOps);

    std::shared_ptr<std::vector<DirectionalData>> getSortedData(std::vector<uint32_t>& outRegionIndices);

    static std::array<vk::DescriptorSetLayoutBinding, 1> getDescriptorSetLayouts();

    static std::array<vk::DescriptorPoolSize, 1> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 1> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                 vk::DescriptorBufferInfo &outDirectionalDataInfo);

    void cleanup();

private:
    void createBuffers();
};


#endif //RTX_RAYTRACER_SAMPLECOLLECTOR_H
