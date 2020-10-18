//
// Created by felixfifi on 19.09.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "guiding/incrementaldistance.h"
#include "PathGuiding.h"

PathGuiding::PathGuiding(uint splitCount, Aabb sceneAabb, bool useParrallaxCompensation,
                         std::shared_ptr<VulkanOps> vulkanOps,
                         vk::PhysicalDevice physicalDevice,
                         uint32_t graphicsQueueIndex) : sceneAabb(sceneAabb.addEpsilon()),
                                                        useParrallaxCompensation(useParrallaxCompensation),
                                                        vulkanOps(vulkanOps),
                                                        device(vulkanOps->getDevice()) {
    regionCount = 1u << splitCount;
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);

    configureVMMFactory();
    createRegions(splitCount);
    createVulkanObjects();
}

void PathGuiding::createVulkanObjects() {
    createBuffers();
    createAS();
}

float getRandNegPos() { return (rand() / (float) RAND_MAX) * 2 - 1.0f; }

void PathGuiding::configureVMMFactory() {
    vmmFactoryProperties.numInitialComponents = 8;
    vmmFactoryProperties.maxItr = 100;
    vmmFactoryProperties.maxKappa = 50000;

    vmmFactory = lightpmm::VMMFactory<PMM>(
            vmmFactoryProperties);
}

void PathGuiding::createPMMs() {
    pmms = std::vector<PMM>(regionCount);
    pmmsExtraData = std::vector<PMM_ExtraData>(regionCount);

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
    if (guidingRegions.size() < regionCount) {
        guidingRegions = std::vector<VMM_Theta>(regionCount);
    }

    // Synchronize to GPU structs
    for (int iRegion = 0; iRegion < regionCount; iRegion++) {
        guidingRegions[iRegion] = pmmToVMM_Theta(pmms[iRegion]);

        if (useParrallaxCompensation) {
            guidingRegions[iRegion].meanPosition = pmmsExtraData[iRegion].parallaxMean;

            // Synchronize distances to GPU structs
            for (int iDistribution = 0; iDistribution < MAX_DISTRIBUTIONS; iDistribution++) {
                uint iComponent = iDistribution / Scalar::Width::value;
                uint idx = iDistribution % Scalar::Width::value;

                float distance = pmmsExtraData[iRegion].incrementalDistance.distances[iComponent][idx];
                distance = distance == std::numeric_limits<float>::infinity() ? -1.0f : distance;

                guidingRegions[iRegion].thetas[iDistribution].distance = distance;
                guidingRegions[iRegion].thetas[iDistribution].target = pmmsExtraData[iRegion].parallaxMean + distance *
                                                                                                             guidingRegions[iRegion].thetas[iDistribution].mu;
            }
        }
    }
}

void PathGuiding::createBuffers() {
    if (aabbsBuffer) {
        cleanupBuffers();
    }

    if (guidingRegions.size() < regionCount) {
        guidingRegions = std::vector<VMM_Theta>(regionCount);
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

void PathGuiding::cleanupBuffers() {
    device.free(aabbsBufferMemory);
    device.free(guidingBufferMemory);
    device.free(guidingUpdateBufferMemory);

    device.destroy(aabbsBuffer);
    device.destroy(guidingBuffer);
    device.destroy(guidingUpdateBuffer);

    aabbsBuffer = nullptr;
    guidingBuffer = nullptr;
    guidingUpdateBuffer = nullptr;
}

uint32_t PathGuiding::getRegionCount() const {
    return regionCount;
}

/**
 *
 * @param sampleCollector
 * @return True - if a Descriptor Set reload is necessary, because a region was split
 */
bool PathGuiding::update(SampleCollector sampleCollector) {
    std::vector<uint32_t> regionIndices;
    std::shared_ptr<std::vector<DirectionalData>> directionalData = sampleCollector.getSortedData(regionIndices);

    // Update regions
    for (int iRegion = 0; iRegion < regionCount; ++iRegion) {
        uint32_t regionIndex = regionIndices[iRegion];
        uint32_t nextRegionIndex = regionIndices[iRegion + 1];

        if (nextRegionIndex - regionIndex > 0) {
            updateRegion(directionalData, iRegion, regionIndex, nextRegionIndex);
        }
    }
    firstFit = false;

    // Check for regions to split
    uint32_t currentRegionCount = getRegionCount();
    bool wasSplit = false;
    for (int iRegion = 0; iRegion < currentRegionCount; ++iRegion) {

        if (splitRegions && pmms[iRegion].m_numSamples > samplesForRegionSplit) {
            splitRegion(iRegion);
            wasSplit = true;
        }
    }

    if (wasSplit) {
        // TODO: Update instead of delete/replace
        cleanup();
        createVulkanObjects();
    }

    // Sync CPU and GPU data
    syncToGPU();

    return wasSplit;
}

void PathGuiding::syncToGPU() {
    syncPMMsToVMM_Thetas();

    void *data;
    vk::DeviceSize bufferSize = guidingRegions.size() * sizeof(VMM_Theta);
    data = device.mapMemory(guidingUpdateBufferMemory, 0, bufferSize);

    memcpy(data, guidingRegions.data(), (size_t) bufferSize);

    device.unmapMemory(guidingUpdateBufferMemory);

    vulkanOps->copyBuffer(guidingUpdateBuffer, guidingBuffer, bufferSize);
}

void PathGuiding::splitRegion(int iRegion) {
    // Split AABB
    Aabb left{}, right{};
    std::tie(left, right) = aabbs[iRegion].splitAabb();

    // Update AABB and add new
    aabbs[iRegion] = left;
    uint32_t newRegion = aabbs.size();
    aabbs.push_back(right);

    // Adjust PMM to give more weight to new samples
    const float decayTerm = 0.25f;
    pmms[iRegion].m_numSamples *= decayTerm;
    pmms[iRegion].m_sampleWeight *= decayTerm;

    // Set PMM as basis for both new PMMs
    pmms.push_back(pmms[iRegion]);
    pmmsExtraData.push_back(pmmsExtraData[iRegion]);

    regionCount++;
}

void PathGuiding::updateRegion(std::shared_ptr<std::vector<DirectionalData>> &directionalData, int iRegion,
                               uint32_t regionBegin, uint32_t nextRegionBegin) {
    guiding::Range<std::vector<DirectionalData>> sampleRange(directionalData->begin() + regionBegin,
                                                             directionalData->begin() + nextRegionBegin);

    preFit(directionalData, iRegion, regionBegin, nextRegionBegin);

    if (firstFit) {
        vmmFactory.fit(sampleRange.begin(), sampleRange.end(),
                       pmms[iRegion], false);
    } else {
        vmmFactory.updateFit(sampleRange.begin(), sampleRange.end(),
                             pmms[iRegion]);
    }

    postFit(iRegion, sampleRange);


}

void PathGuiding::preFit(const std::shared_ptr<std::vector<DirectionalData>> &directionalData, int iRegion,
                         uint32_t regionBegin, uint32_t nextRegionBegin) {
    if (useParrallaxCompensation) {
//        // Find average position
//        glm::vec3 posSum{0.0f,0.0f,0.0f};
//        for (uint32_t i = regionBegin; i < nextRegionBegin; i++) {
//            posSum += (*directionalData)[i].position;
//        }
//        glm::vec3 parallaxMean = posSum / (float) sampleRange.size();
// FIXME: Real mean instead of middle of AABB
        glm::vec3 parallaxMean = aabbs[iRegion].min + 0.5f * (aabbs[iRegion].max - aabbs[iRegion].min);

        pmmsExtraData[iRegion].lastParallaxMean = pmmsExtraData[iRegion].parallaxMean;
        pmmsExtraData[iRegion].parallaxMean = parallaxMean;

        // Move all samples to average
        for (uint32_t i = regionBegin; i < nextRegionBegin; i++) {
            glm::vec3 pos = (*directionalData)[i].position;
            float distance = (*directionalData)[i].distance;
            glm::vec3 direction = (*directionalData)[i].direction;

            if (distance > 0.0f) {
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
            pmmsExtraData[iRegion].incrementalDistance.reposition(pmms[iRegion],
                                                                  pmmsExtraData[iRegion].lastParallaxMean -
                                                                  parallaxMean);
        }
    }
}

void PathGuiding::postFit(int iRegion, const guiding::Range<std::vector<DirectionalData>> &sampleRange) {
    if (splitAndMerge) {
        pmmsExtraData[iRegion].samplesSinceLastMerge += sampleRange.size();

        if (pmmsExtraData[iRegion].samplesSinceLastMerge > minSamplesForMerging) {
            pmms[iRegion].removeWeightPrior(vmmFactoryProperties.vPrior);

            mergeAll(iRegion, sampleRange);

            pmms[iRegion].applyWeightPrior(vmmFactoryProperties.vPrior);

            pmmsExtraData[iRegion].samplesSinceLastMerge = 0;
        }
    }


    if (useParrallaxCompensation) {
        // Update distances
        pmmsExtraData[iRegion].incrementalDistance.updateDistances(pmms[iRegion], sampleRange);
    }


}

/*
The following functions are an adapted part of the implementation of the SIGGRAPH 2020 paper
"Robust Fitting of Parallax-Aware Mixtures for Path Guiding".

Copyright (c) 2020 Lukas Ruppert, Sebastian Herholz.
*/
bool PathGuiding::mergeAll(int iRegion, const guiding::Range<std::vector<DirectionalData>> &sampleRange) {
    uint32_t totalNumMerges = 0;

    PMM &pmm = pmms[iRegion];

    do {
        const uint32_t numComponents = pmm.getK();
        uint32_t numMerges = 0;

        if (EXPECT_NOT_TAKEN(numComponents <= 1))
            return false;

        const uint32_t numSimilarityValues = numComponents * (numComponents - 1) / 2;

        const std::array<float,
                PMM::MaxK::value * (PMM::MaxK::value - 1) / 2> mergeMetric = computePearsonChiSquaredMergeMetric(pmm);

        std::array<std::pair<float, std::pair<uint32_t, uint32_t>>,
                PMM::MaxK::value * (PMM::MaxK::value - 1) / 2> mergeCandidates;
        for (uint32_t componentA = 0, offsetA = 0;
             componentA < numComponents; ++componentA, offsetA += numComponents - componentA) {
            for (uint32_t componentB = componentA + 1, offsetB = 0;
                 componentB < numComponents; ++componentB, ++offsetB) {
                mergeCandidates[offsetA + offsetB] = std::make_pair(mergeMetric[offsetA + offsetB],
                                                                    std::make_pair(componentA, componentB));
            }
        }


        const auto validCandidatesEnd = std::partition(mergeCandidates.begin(),
                                                       mergeCandidates.begin() + numSimilarityValues,
                                                       [this](std::pair<float, std::pair<uint32_t, uint32_t>> candidate) -> bool {
                                                           return candidate.first <= mergeMaxDivergence;
                                                       });

        std::sort(mergeCandidates.begin(), validCandidatesEnd, [](std::pair<float, std::pair<uint32_t, uint32_t>> a,
                                                                  std::pair<float, std::pair<uint32_t, uint32_t>> b) -> bool {
            return a.first < b.first;
        });

        std::array<uint32_t, (PMM::MaxK::value + 31) / 32> bitmask;
        std::fill(bitmask.begin(), bitmask.end(), 0U);

        std::array<std::pair<uint32_t, uint32_t>, PMM::MaxK::value> merges;

        for (auto it = mergeCandidates.begin(); it != validCandidatesEnd; ++it) {
            const uint32_t componentA = std::min(it->second.first, it->second.second);
            const uint32_t componentB = std::max(it->second.first, it->second.second);

            if ((bitmask[componentA / 32] & (1 << (componentA % 32))) != 0 ||
                (bitmask[componentB / 32] & (1 << (componentB % 32))) != 0)
                continue;

            merges[numMerges++] = std::make_pair(componentA, componentB);
            bitmask[componentA / 32] |= (1 << (componentA % 32));
            bitmask[componentB / 32] |= (1 << (componentB % 32));
        }

        if (numMerges == 0)
            break;

        //this is needed to avoid updating indices after individual merges
        std::sort(merges.begin(), merges.begin() + numMerges,
                  [](std::pair<uint32_t, uint32_t> a, std::pair<uint32_t, uint32_t> b) -> bool {
                      return a.second > b.second;
                  });

        for (uint32_t i = 0; i < numMerges; ++i) {
            SAssert(merges[i].second > merges[i].first);
            mergeComponents(iRegion, merges[i].first, merges[i].second);
        }

        totalNumMerges += numMerges;
    } while (true);

    return totalNumMerges;
}

std::array<float, PMM::MaxK::value * (PMM::MaxK::value - 1) / 2>
PathGuiding::computePearsonChiSquaredMergeMetric(const PMM &distribution) {
    const uint32_t numComponents = distribution.getK();
    const uint32_t numActiveKernels = (numComponents + Scalar::Width::value - 1) / Scalar::Width::value;

    std::array<VMF, PMM::NumKernels::value> selfProduct;
    for (uint32_t k = 0; k < numActiveKernels; ++k) {
        selfProduct[k] = distribution.getComponent(k);
        selfProduct[k].product(selfProduct[k]);
    }

    std::array<Scalar, PMM::NumKernels::value * PMM::MaxK::value> componentToKernelSimilarityValues;

    for (uint32_t i = 0; i < numComponents - 1; ++i) {
        const size_t kernelIndexI = i / Scalar::Width::value;
        const size_t inKernelIndexI = i % Scalar::Width::value;

        const VMF componentI = distribution.getComponent(kernelIndexI).extract(inKernelIndexI);
        const VMF componentISqr = selfProduct[kernelIndexI].extract(inKernelIndexI);

        for (uint32_t j = (i + 1) / Scalar::Width::value; j < numActiveKernels; ++j) {
            VMF productIJ{componentI};
            productIJ.product(distribution.getComponent(j));

            const VMF kernelJ{distribution.getComponent(j)};
            VMF merged{componentI};

            {
                VMF kJForMerge{kernelJ};
                for (uint32_t l = 0; l < Scalar::Width::value; ++l)
                    merged.mergeComponent(l, l, kJForMerge);
            }

            const Scalar quotientISqrMerged = VMF{componentISqr}.division(merged);
            const Scalar quotientIJMerged = VMF{productIJ}.division(merged);
            const Scalar quotientJSqrMerged = VMF{selfProduct[j]}.division(merged);

            const Scalar divergence =
                    quotientISqrMerged + 2.0f * quotientIJMerged + quotientJSqrMerged - merged.m_weights;

            componentToKernelSimilarityValues[i * PMM::NumKernels::value + j] = divergence;
        }
    }

    std::array<float, PMM::MaxK::value * (PMM::MaxK::value - 1) / 2> similarityValues;

    uint32_t index = 0;
    for (uint32_t i = 0; i < numComponents - 1; ++i)
        for (size_t j = i + 1; j < numComponents; ++j, ++index)
            similarityValues[index] = componentToKernelSimilarityValues[i * PMM::NumKernels::value +
                                                                        j / Scalar::Width::value][j %
                                                                                                  Scalar::Width::value];

    return similarityValues;
}

void PathGuiding::mergeComponents(int iRegion, const uint32_t componentA, const uint32_t componentB)
{
#ifdef GUIDING_VALIDATE_INTERMEDIATE_MIXTURE_STATES
    const uint32_t numComponents = pmm.getK();

        if (!pmm.valid(pmmFactory.m_vmmFactory.computeMinComponentWeight(numComponents)))
            SLog(EWarn, "invalid mixture state before merging components %u and %u", componentA, componentB);
#endif
    PMM &pmm = pmms[iRegion];

    pmmsExtraData[iRegion].incrementalDistance.merge(pmm, componentA, componentB);
    pmmsExtraData[iRegion].incrementalPearsonChiSquared.merge(pmm, componentA, componentB);
    pmmsExtraData[iRegion].incrementalCovariance2D.merge(pmm, componentA, componentB);

    pmm.mergeComponents(componentA, componentB);

#ifdef GUIDING_VALIDATE_INTERMEDIATE_MIXTURE_STATES
    if (!pmm.valid(pmmFactory.m_vmmFactory.computeMinComponentWeight(numComponents-1)))
            SLog(EWarn, "error occured after merging components %u and %u", componentA, componentB);
#endif
}
