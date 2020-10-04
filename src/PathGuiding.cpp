//
// Created by felixfifi on 19.09.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <guiding/incrementaldistance.h>
#include <guiding/Range.h>
#include "PathGuiding.h"

PathGuiding::PathGuiding(uint splitCount, Aabb sceneAabb, bool useParrallaxCompensation,
                         std::shared_ptr<VulkanOps> vulkanOps,
                         vk::PhysicalDevice physicalDevice,
                         uint32_t graphicsQueueIndex) : sceneAabb(sceneAabb),
                                                        useParrallaxCompensation(useParrallaxCompensation),
                                                        vulkanOps(vulkanOps),
                                                        device(vulkanOps->getDevice()) {
    regionCount = 1u << splitCount;
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);

    configureVMMFactory();
    createRegions(splitCount);
    createBuffers();
    createAS();
}

float getRandNegPos() { return (rand() / (float) RAND_MAX) * 2 - 1.0f; }

void PathGuiding::configureVMMFactory() {
    lightpmm::VMMFactoryProperties factoryProperties{};
    factoryProperties.numInitialComponents = 8;
    factoryProperties.maxItr = 100;
    factoryProperties.maxKappa = 50000;

    vmmFactory = lightpmm::VMMFactory<PMM>(
            factoryProperties);
}

void PathGuiding::createPMMs() {
    pmms = std::vector<PMM>(regionCount);
    incrementalDistances = std::vector<guiding::IncrementalDistance<PMM>>(regionCount);
    lastParallaxMeans = std::vector<glm::vec3>(regionCount);

    for (int i = 0; i < regionCount; ++i) {
        PMM pmm;
        vmmFactory.initialize(pmm);
        pmms[i] = pmm;
    }
}

VMM_Theta PathGuiding::pmmToVMM_Theta(const PMM &pmm) {
    VMM_Theta vmm{};
    vmm.usedDistributions = pmm.getK();

    for (int iDistribution = 0; iDistribution < vmm.usedDistributions; ++iDistribution) {
        vmm.pi[iDistribution] = pmm.weightK(iDistribution);

        uint iComponent = iDistribution / Scalar::Width::value;
        uint idx = iDistribution % Scalar::Width::value;

        vmm.thetas[iDistribution].mu = pmm.m_comps[iComponent].getMu(idx);
        vmm.thetas[iDistribution].setK(pmm.m_comps[iComponent].getKappa(idx));
    }

//    std::cout << vmm.toString() << std::endl;
    return vmm;
}

glm::vec3 randomOnUnitSphere() {
    glm::vec3 result;
    do {
        result = {getRandNegPos(), getRandNegPos(), getRandNegPos()};
    } while (glm::length(result) > 1.0f);

    return glm::normalize(result);
}


void PathGuiding::createRegions(uint splitCount) {
    guidingRegions = std::vector<VMM_Theta>(regionCount);

    aabbs = {sceneAabb};

    for (int iSplits = 0; iSplits < splitCount; iSplits++) {
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

    // Use lightpmm PMMs
    createPMMs();
    syncPMMsToVMM_Thetas();
}

void PathGuiding::syncPMMsToVMM_Thetas() {
    // Synchronize to GPU structs
    for (int iRegion = 0; iRegion < regionCount; iRegion++) {
        guidingRegions[iRegion] = pmmToVMM_Theta(pmms[iRegion]);
    }
}

void PathGuiding::createBuffers() {
    if (aabbsBuffer) {
        cleanupBuffers();
    }

    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eShaderDeviceAddress |
            vk::BufferUsageFlagBits::eTransferDst;
    vk::MemoryPropertyFlagBits memoryFlags = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(aabbs, usage, memoryFlags, aabbsBuffer, aabbsBufferMemory);
    vulkanOps->createBufferFromData(guidingRegions, usage, memoryFlags, guidingBuffer, guidingBufferMemory);
    vulkanOps->createBufferFromData(guidingRegions, usage | vk::BufferUsageFlagBits::eTransferSrc,
                                    vk::MemoryPropertyFlagBits::eHostVisible |
                                    vk::MemoryPropertyFlagBits::eHostCoherent, guidingUpdateBuffer,
                                    guidingUpdateBufferMemory);
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
                                               vk::ShaderStageFlagBits::eIntersectionKHR |
                                               vk::ShaderStageFlagBits::eClosestHitKHR};
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

uint32_t PathGuiding::getRegionCount() {
    return aabbs.size();
}

void PathGuiding::update(SampleCollector sampleCollector) {
    std::vector<uint32_t> regionIndices;
    std::shared_ptr<std::vector<DirectionalData>> directionalData = sampleCollector.getSortedData(regionIndices);

    for (int iRegion = 0; iRegion < getRegionCount(); ++iRegion) {
        uint32_t regionIndex = regionIndices[iRegion];
        uint32_t nextRegionIndex = regionIndices[iRegion + 1];

        if (nextRegionIndex - regionIndex > 0) {
            updateRegion(directionalData, iRegion, regionIndex, nextRegionIndex);
        }
    }

    firstFit = false;

    syncPMMsToVMM_Thetas();

    void *data;
    vk::DeviceSize bufferSize = guidingRegions.size() * sizeof(VMM_Theta);
    data = device.mapMemory(guidingUpdateBufferMemory, 0, bufferSize);

    memcpy(data, guidingRegions.data(), (size_t) bufferSize);

    device.unmapMemory(guidingUpdateBufferMemory);

    vulkanOps->copyBuffer(guidingUpdateBuffer, guidingBuffer, bufferSize);
}

void PathGuiding::updateRegion(std::shared_ptr<std::vector<DirectionalData>> &directionalData, int iRegion,
                               uint32_t regionBegin, uint32_t nextRegionBegin) {
    guiding::Range<std::vector<DirectionalData>> sampleRange(directionalData->begin() + regionBegin,
                                                             directionalData->begin() + nextRegionBegin);

    if (useParrallaxCompensation) {
        // Find average position
        glm::vec3 posSum;
        for (uint32_t i = regionBegin; i < nextRegionBegin; i++) {
            posSum += (*directionalData)[i].position;
        }
        glm::vec3 parallaxMean = posSum / (float) sampleRange.size();
        guidingRegions[iRegion].meanPosition = parallaxMean;

        // Move all samples to average
        for (uint32_t i = regionBegin; i < nextRegionBegin; i++) {
            glm::vec3 pos = (*directionalData)[i].position;
            float distance = (*directionalData)[i].distance;
            glm::vec3 direction = (*directionalData)[i].direction;

            if (distance > 0) {
                // If not infinite distance
                // => update direction and distance to point to the correct location from the average pos
                glm::vec3 newDirection = pos + distance * direction - parallaxMean;
                (*directionalData)[i].direction = glm::normalize(newDirection);
                (*directionalData)[i].distance = glm::length(newDirection);

            } else {
                (*directionalData)[i].distance = std::numeric_limits<float>::infinity();
            }
            (*directionalData)[i].position = parallaxMean;
        }

        if (!firstFit) {
            // Move distribution to current mean
            incrementalDistances[iRegion].reposition(pmms[iRegion], lastParallaxMeans[iRegion] - parallaxMean);
        }
    }

    if (firstFit) {
        vmmFactory.fit(sampleRange.begin(), sampleRange.end(),
                       pmms[iRegion], false);
    } else {
        vmmFactory.updateFit(sampleRange.begin(), sampleRange.end(),
                             pmms[iRegion]);
    }

    if (useParrallaxCompensation) {
        // Update distances
        incrementalDistances[iRegion].updateDistances(pmms[iRegion], sampleRange);

        // Synchronize to GPU structs
        for (int iRegion = 0; iRegion < regionCount; iRegion++) {
            for (int iDistribution = 0; iDistribution < MAX_DISTRIBUTIONS; iDistribution++) {
                uint iComponent = iDistribution / Scalar::Width::value;
                uint idx = iDistribution % Scalar::Width::value;

                float distance = incrementalDistances[iRegion].distances[iComponent][idx];
                distance = distance == std::numeric_limits<float>::infinity() ? -1.0f : distance;

                guidingRegions[iRegion].thetas[iDistribution].distance = distance;
                guidingRegions[iRegion].thetas[iDistribution].target = guidingRegions[iRegion].meanPosition + distance *
                                                                                                              guidingRegions[iRegion].thetas[iDistribution].mu;
            }
        }
    }
}
