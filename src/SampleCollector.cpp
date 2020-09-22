//
// Created by felixfifi on 22.09.20.
//

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

std::shared_ptr<std::vector<std::vector<DirectionalData>>> SampleCollector::getSortedData() {
    vulkanOps->copyBuffer(directionalDataBuffer, hostDirectionalDataBuffer, bufferSize);

    void *data;
    data = device.mapMemory(hostDirectionalDataBufferMemory, 0, bufferSize);

    // Reinterpret data
    uint64_t sampleCount = imageSize.height * imageSize.width * numPerPixel;
    std::vector<DirectionalData> directionalData(reinterpret_cast<DirectionalData *>(data),
                                                 reinterpret_cast<DirectionalData *>(data) + sampleCount);

    // Sort into per region arrays
    auto sortedData = std::make_shared<std::vector<std::vector<DirectionalData>>>(regionCount);

    for (const DirectionalData &datum : directionalData) {
        uint32_t iRegion = datum.flags;
        if (iRegion != INVALID) {
            (*sortedData)[iRegion].push_back(datum);
        }
    }

    device.unmapMemory(hostDirectionalDataBufferMemory);

    return sortedData;
}
