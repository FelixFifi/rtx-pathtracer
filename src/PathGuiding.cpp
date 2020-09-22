//
// Created by felixfifi on 19.09.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "PathGuiding.h"

PathGuiding::PathGuiding(uint splitCount, Aabb sceneAabb, std::shared_ptr<VulkanOps> vulkanOps,
                         vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex) : sceneAabb(sceneAabb),
                                                                                           vulkanOps(vulkanOps),
                                                                                           device(vulkanOps->getDevice()) {
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);

    createDummyRegions(splitCount, sceneAabb);
    createBuffers();
    createAS();
}

float getRandNegPos() { return (rand() / (float) RAND_MAX) * 2 - 1.0f; }

glm::vec3 randomOnUnitSphere() {
    glm::vec3 result;
    do {
        result = {getRandNegPos(), getRandNegPos(), getRandNegPos()};
    } while (glm::length(result) > 1.0f);

    return glm::normalize(result);
}


void PathGuiding::createDummyRegions(uint splitCount, const Aabb &sceneAabb) {
    uint regionCount = 1u << splitCount;

    guidingRegions.clear();
    guidingRegions.reserve(regionCount);

    aabbs = {sceneAabb};

    for (int iSplits; iSplits < splitCount; iSplits++) {
        std::vector<Aabb> currentSplits;
        currentSplits.reserve(aabbs.size() * 2);

        for (Aabb &next : aabbs) {
            Aabb left{}, right{};
            std::tie(left, right) = next.splitAabb();

            currentSplits.push_back(left);
            currentSplits.push_back(right);
        }

        aabbs = currentSplits;
    }

    // Generate random vMF cones
    for (const Aabb &aabb : aabbs) {
        VMM_Theta vmmTheta{};
        const int DIST_COUNT = MAX_DISTRIBUTIONS;
        for (int iDist = 0; iDist < DIST_COUNT; iDist++) {
            VMF_Theta vmf{};

            vmf.mu = randomOnUnitSphere();
            vmf.setK(30.0f);

            vmmTheta.thetas[iDist] = vmf;
            vmmTheta.pi[iDist] = 1.0f / DIST_COUNT;
        }

        vmmTheta.usedDistributions = DIST_COUNT;

        guidingRegions.push_back(vmmTheta);
    }
}

void PathGuiding::createBuffers() {
    if (aabbsBuffer) {
        cleanupBuffers();
    }

    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eShaderDeviceAddress;
    vk::MemoryPropertyFlagBits memoryFlags = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(aabbs, usage, memoryFlags, aabbsBuffer, aabbsBufferMemory);
    vulkanOps->createBufferFromData(guidingRegions, usage, memoryFlags, guidingBuffer, guidingBufferMemory);
}

void PathGuiding::createAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;

    allBlas.emplace_back(
            aabbToBlas(device, aabbs.size(), aabbsBuffer, vk::GeometryFlagBitsKHR::eOpaque));

    const vk::BuildAccelerationStructureFlagsKHR &asFlags =
            vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate |
            vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace;
    rtBuilder.buildBlas(allBlas, asFlags);

    nvvkpp::RaytracingBuilderKHR::Instance rayInst;
    rayInst.transform = glm::value_ptr(glm::mat4(1.0f));
    rayInst.instanceId = 0;
    rayInst.blasId = 0;
    rayInst.hitGroupId = 4;
    rayInst.mask = 0xFF;
    rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;

    instances.emplace_back(rayInst);

    rtBuilder.buildTlas(instances, asFlags);
    accelerationStructure = rtBuilder.getAccelerationStructure();
}


std::array<vk::DescriptorSetLayoutBinding, 3> PathGuiding::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding bufferAabbs{BINDING_AABB, vk::DescriptorType::eStorageBuffer, 1,
                                               vk::ShaderStageFlagBits::eIntersectionKHR};
    vk::DescriptorSetLayoutBinding bufferGuiding{BINDING_GUIDING, vk::DescriptorType::eStorageBuffer, 1,
                                                 vk::ShaderStageFlagBits::eRaygenKHR};
    vk::DescriptorSetLayoutBinding bindingAS{BINDING_AS, vk::DescriptorType::eAccelerationStructureKHR, 1,
                                             vk::ShaderStageFlagBits::eRaygenKHR};


    return {bufferAabbs, bufferGuiding, bindingAS};
}

std::array<vk::DescriptorPoolSize, 3> PathGuiding::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eAccelerationStructureKHR, 1)
    };
}

/**
 *
 * @param descriptorSet
 * @param outBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @return
 */
std::array<vk::WriteDescriptorSet, 3>
PathGuiding::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                    vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                                    vk::DescriptorBufferInfo &outAabbsBufferInfo,
                                    vk::DescriptorBufferInfo &outGuidingBufferInfo) {

    outDescASInfo.setAccelerationStructureCount(1);
    outDescASInfo.setPAccelerationStructures(&accelerationStructure);

    outAabbsBufferInfo = vk::DescriptorBufferInfo(aabbsBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeAabb{descriptorSet, BINDING_AABB, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                     &outAabbsBufferInfo};

    outGuidingBufferInfo = vk::DescriptorBufferInfo(guidingBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeGuiding{descriptorSet, BINDING_GUIDING, 0, 1, vk::DescriptorType::eStorageBuffer,
                                        nullptr,
                                        &outGuidingBufferInfo};


    std::array<vk::WriteDescriptorSet, 3> writes;
    writes[0] = vk::WriteDescriptorSet(descriptorSet, BINDING_AS, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&outDescASInfo);

    writes[1] = writeAabb;
    writes[2] = writeGuiding;

    return writes;
}


void PathGuiding::cleanup() {
    if (!aabbsBuffer) {
        return;
    }

    cleanupBuffers();

    rtBuilder.destroy();
}

void PathGuiding::cleanupBuffers() const {
    device.free(aabbsBufferMemory);
    device.free(guidingBufferMemory);

    device.destroy(aabbsBuffer);
    device.destroy(guidingBuffer);
}
