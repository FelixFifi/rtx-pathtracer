//
// Created by felixfifi on 22.09.20.
//

#include <iostream>
#include "SampleCollector.h"

SampleCollector::SampleCollector(uint32_t regionCount, vk::Extent2D imageSize, uint32_t numPerPixel,
                                 std::shared_ptr<VulkanOps> vulkanOps)
        : regionCount(regionCount), imageSize(imageSize), numPerPixel(numPerPixel), vulkanOps(vulkanOps),
          device(vulkanOps->getDevice()) {
    createBuffers();
}

void SampleCollector::cleanup() {
    if (!directionalDataBuffer) {
        return;
    }

    device.free(directionalDataBufferMemory);
    device.free(hostDirectionalDataBufferMemory);
    device.destroy(directionalDataBuffer);
    device.destroy(hostDirectionalDataBuffer);
}

void SampleCollector::createBuffers() {
    uint64_t sampleCount = imageSize.height * imageSize.width * numPerPixel;

    std::vector<DirectionalData> data(sampleCount);

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferSrc;
    vk::MemoryPropertyFlags memoryFlags = vk::MemoryPropertyFlagBits::eDeviceLocal;

    vulkanOps->createBufferFromData(data, usage, memoryFlags, directionalDataBuffer, directionalDataBufferMemory);

    // Host buffer to copy the data to
    vk::BufferUsageFlags usageHost = vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst;
    vk::MemoryPropertyFlags memoryFlagsHost =
            vk::MemoryPropertyFlagBits::eHostCoherent | vk::MemoryPropertyFlagBits::eHostVisible;

    bufferSize = sizeof(DirectionalData) * sampleCount;
    vulkanOps->createBuffer(bufferSize, usageHost, memoryFlagsHost, hostDirectionalDataBuffer,
                            hostDirectionalDataBufferMemory);
}

std::array<vk::DescriptorSetLayoutBinding, 1> SampleCollector::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding bufferDirectionalData{BINDING_DIRECTIONAL_DATA, vk::DescriptorType::eStorageBuffer,
                                                         1,
                                                         vk::ShaderStageFlagBits::eRaygenKHR};
    return {bufferDirectionalData};
}

std::array<vk::DescriptorPoolSize, 1> SampleCollector::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1)
    };
}

std::array<vk::WriteDescriptorSet, 1> SampleCollector::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                              vk::DescriptorBufferInfo &outDirectionalDataInfo) {
    outDirectionalDataInfo = vk::DescriptorBufferInfo(directionalDataBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeDirectionalData{descriptorSet, BINDING_DIRECTIONAL_DATA, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                                &outDirectionalDataInfo};

    return {writeDirectionalData};
}

std::shared_ptr<std::vector<DirectionalData>> SampleCollector::getSortedData(std::vector<uint32_t>& outRegionIndices) {
    vulkanOps->copyBuffer(directionalDataBuffer, hostDirectionalDataBuffer, bufferSize);

    void *data;
    data = device.mapMemory(hostDirectionalDataBufferMemory, 0, bufferSize);

    // Reinterpret data
    uint64_t sampleCount = imageSize.height * imageSize.width * numPerPixel;
    std::vector<DirectionalData> directionalData(reinterpret_cast<DirectionalData *>(data),
                                                 reinterpret_cast<DirectionalData *>(data) + sampleCount);

    // Sort per region
    std::sort(directionalData.begin(), directionalData.end(), [](const DirectionalData &a, const DirectionalData &b) {
        return a.flags < b.flags;
    });


    auto sortedData = std::make_shared<std::vector<DirectionalData>>(directionalData);

    outRegionIndices.clear();
    outRegionIndices.reserve(regionCount);

    long lastRegion = -1;
    for (uint32_t i = 0; i < sampleCount; i++) {
        const DirectionalData &datum = (*sortedData)[i];
        uint32_t region = datum.flags;
        
        if (region == INVALID){
            // End of valid data reached
            // => Set all region intervals to size 0
            for (int emptyRegion = lastRegion + 1; emptyRegion < regionCount; emptyRegion++) {
                // Mark empty regions as empty
                outRegionIndices.push_back(i);
            }
            // Mark end
            outRegionIndices.push_back(i);
            break;
        }

        if(region > lastRegion) {
            for (int emptyRegion = lastRegion + 1; emptyRegion < region; emptyRegion++) {
                // Mark empty regions as all beginning here
                // => They can check the next region to see, that they have no data
                outRegionIndices.push_back(i);
            }
            outRegionIndices.push_back(i);
            lastRegion = region;
        }
    }



    device.unmapMemory(hostDirectionalDataBufferMemory);

    return sortedData;
}
