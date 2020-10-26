//
// Created by felixfifi on 09.07.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "IrradianceCache.h"
#include "Shapes.h"
#include "CommonOps.h"

IrradianceCache::IrradianceCache(uint32_t maxCaches, std::shared_ptr<VulkanOps> vulkanOps,
                                 vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex)
        : maxCaches(maxCaches), vulkanOps(vulkanOps), device(vulkanOps->getDevice()) {
    createBuffers();
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);
    createAccelerationStructure();
}

void IrradianceCache::createAccelerationStructure() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;

    allBlas.emplace_back(
            aabbToBlas(device, maxCaches, aabbsBuffer, vk::GeometryFlagBitsKHR::eNoDuplicateAnyHitInvocation));

    const vk::BuildAccelerationStructureFlagsKHR &asFlags =
            vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate |
            vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace;
    rtBuilder.buildBlas(allBlas, asFlags);

    nvvkpp::RaytracingBuilderKHR::Instance rayInst;
    rayInst.transform = glm::value_ptr(glm::mat4(1.0f));
    rayInst.instanceId = 0;
    rayInst.blasId = 0;
    rayInst.hitGroupId = 2;
    rayInst.mask = 0xFF;
    rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;

    instances.emplace_back(rayInst);

    rtBuilder.buildTlas(instances, asFlags);
    accelerationStructure = rtBuilder.getAccelerationStructure();
}

void IrradianceCache::createBuffers() {
    Sphere tmp{glm::vec3(0.0f, 0.0f, 0.0f), 0.0f};
    Aabb aabb = tmp.getAabb();

    std::vector<Sphere> spheres(maxCaches);
    std::vector<Aabb> aabbs(maxCaches);
    std::vector<CacheData> cacheValues(maxCaches);
    for (int i = 0; i < maxCaches; ++i) {
        spheres[i] = tmp;
        aabbs[i] = aabb;
        cacheValues[i] = {};
    }

    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eShaderDeviceAddress;
    vk::MemoryPropertyFlagBits memoryFlags = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(spheres, usage, memoryFlags, spheresBuffer, spheresBufferMemory);
    vulkanOps->createBufferFromData(aabbs, usage, memoryFlags, aabbsBuffer, aabbsBufferMemory);

    CacheHeader header{};
    header.nextCacheSlot = 0;
    header.maxCaches = maxCaches;

    std::vector<CacheHeader> singleHeader{header};

    vulkanOps->createBufferFrom2Data(singleHeader, cacheValues, usage, memoryFlags, cacheBuffer, cacheBufferMemory);


    vulkanOps->setBufferName(aabbsBuffer, "B: IC AABBs");
    vulkanOps->setBufferName(spheresBuffer, "B: IC Spheres");
    vulkanOps->setBufferName(cacheBuffer, "B: IC Cache");
}

void IrradianceCache::updateSpheres() {
    rtBuilder.updateBlas(0);
    rtBuilder.updateTlasMatrices(instances);
    updated = true;
}

std::array<vk::DescriptorSetLayoutBinding, 4> IrradianceCache::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding bufferAabbs{10, vk::DescriptorType::eStorageBuffer, 1,
                                               vk::ShaderStageFlagBits::eRaygenKHR};
    vk::DescriptorSetLayoutBinding bindingAS{11, vk::DescriptorType::eAccelerationStructureKHR, 1,
                                             vk::ShaderStageFlagBits::eRaygenKHR};
    vk::DescriptorSetLayoutBinding bufferSpheres{12, vk::DescriptorType::eStorageBuffer, 1,
                                                 vk::ShaderStageFlagBits::eRaygenKHR |
                                                 vk::ShaderStageFlagBits::eIntersectionKHR |
                                                 vk::ShaderStageFlagBits::eClosestHitKHR |
                                                 vk::ShaderStageFlagBits::eAnyHitKHR};
    vk::DescriptorSetLayoutBinding bufferCache{13, vk::DescriptorType::eStorageBuffer, 1,
                                               vk::ShaderStageFlagBits::eRaygenKHR |
                                               vk::ShaderStageFlagBits::eIntersectionKHR |
                                               vk::ShaderStageFlagBits::eAnyHitKHR};


    return {bufferAabbs, bindingAS, bufferSpheres, bufferCache};
}

std::array<vk::DescriptorPoolSize, 4> IrradianceCache::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eAccelerationStructureKHR, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1)
    };
}

/**
 *
 * @param descriptorSet
 * @param outBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @return
 */
std::array<vk::WriteDescriptorSet, 4>
IrradianceCache::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                        vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                                        vk::DescriptorBufferInfo &outSpheresBufferInfo,
                                        vk::DescriptorBufferInfo &outCacheBufferInfo,
                                        vk::DescriptorBufferInfo &outAabbsBufferInfo) {

    outDescASInfo.setAccelerationStructureCount(1);
    outDescASInfo.setPAccelerationStructures(&accelerationStructure);

    outAabbsBufferInfo = vk::DescriptorBufferInfo(aabbsBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeAabb{descriptorSet, 10, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                     &outAabbsBufferInfo};

    outSpheresBufferInfo = vk::DescriptorBufferInfo(spheresBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeSpheres{descriptorSet, 12, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &outSpheresBufferInfo};

    outCacheBufferInfo = vk::DescriptorBufferInfo(cacheBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeCache{descriptorSet, 13, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &outCacheBufferInfo};


    std::array<vk::WriteDescriptorSet, 4> writes;
    writes[0] = vk::WriteDescriptorSet(descriptorSet, 11, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&outDescASInfo);

    writes[1] = writeAabb;

    writes[2] = writeSpheres;
    writes[3] = writeCache;

    return writes;
}

void IrradianceCache::cleanUp() {
    if (!aabbsBuffer) {
        return;
    }

    rtBuilder.destroy();

    device.free(aabbsBufferMemory);
    device.free(spheresBufferMemory);
    device.free(cacheBufferMemory);

    device.destroy(spheresBuffer);
    device.destroy(aabbsBuffer);
    device.destroy(cacheBuffer);
}

bool IrradianceCache::wasUpdated() const {
    return updated;
}
