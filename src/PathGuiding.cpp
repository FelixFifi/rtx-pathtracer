//
// Created by felixfifi on 19.09.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <eigen3/Eigen/Eigenvalues>
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

    vulkanOps->setBufferName(aabbsBuffer, "B: Guiding regions AABBs");
    vulkanOps->setBufferName(guidingBuffer, "B: Guiding data device");
    vulkanOps->setBufferName(guidingUpdateBuffer, "B: Guiding update host");
}

void PathGuiding::createAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;

    allBlas.emplace_back(
            aabbToBlas(device, aabbs.size(), aabbsBuffer, vk::GeometryFlagBitsKHR::eNoDuplicateAnyHitInvocation));

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
                                               vk::ShaderStageFlagBits::eClosestHitKHR|
                                               vk::ShaderStageFlagBits::eRaygenKHR};
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

    vulkanOps->copyBuffer(guidingUpdateBuffer, guidingBuffer, bufferSize, nullptr);
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
    PMM &pmm = pmms[iRegion];
    PMM_ExtraData &pmmExtraData = pmmsExtraData[iRegion];

    if (splitAndMerge) {
        pmmExtraData.samplesSinceLastMerge += sampleRange.size();

        if (pmmExtraData.samplesSinceLastMerge > minSamplesForMerging) {
            pmm.removeWeightPrior(vmmFactoryProperties.vPrior);

            mergeAll(iRegion, sampleRange);

            pmm.applyWeightPrior(vmmFactoryProperties.vPrior);

            pmmExtraData.samplesSinceLastMerge = 0;
        }

        //update incremental statistics after merging
        pmmExtraData.incrementalPearsonChiSquared.updateDivergence(pmm, sampleRange);
        pmmExtraData.incrementalCovariance2D.updateStatistics(pmm, sampleRange);

        const bool firstFit = sampleRange.size() == pmm.m_totalNumSamples;
        const bool firstFitOrEnoughSamplesForSplitting = firstFit || sampleRange.size() > minSamplesForPostSplitFitting;

        pmm.removeWeightPrior(vmmFactoryProperties.vPrior);
        //while (split(stats, pmm, samples, pmmFactory, firstFitOrEnoughSamplesForSplitting));
        splitAll(pmmExtraData, pmm, sampleRange, true, firstFitOrEnoughSamplesForSplitting);
        pmm.applyWeightPrior(vmmFactoryProperties.vPrior);

    }


    if (useParrallaxCompensation) {
        // Update distances
        pmmsExtraData[iRegion].incrementalDistance.updateDistances(pmms[iRegion], sampleRange);
    }


}


static std::vector<float> computeEigenValuesVectors(lightpmm::Matrix2x2 covmat, std::vector<glm::vec2> &eigenVectors){
    // https://stackoverflow.com/a/56472351
    Eigen::Matrix2f mat;
    mat << covmat[0][0], covmat[1][0],
           covmat[0][1], covmat[1][1];

    Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
    eigensolver.compute(mat);
    Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
    Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
    std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values;

    for(int i=0; i<eigen_values.size(); i++){
        std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
        eigen_vectors_and_values.push_back(vec_and_val);
    }
    std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
              [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{
                  return std::get<0>(a) <= std::get<0>(b);
              });
    int index = 0;

    std::vector<float> eigenValues;
    for(auto const vect : eigen_vectors_and_values){
        eigenValues.push_back(std::get<0>(vect));
        eigenVectors.emplace_back(std::get<1>(vect)[0],std::get<1>(vect)[1]);
        index++;
    }

    return eigenValues;
}


/*
The following functions are an adapted part of the implementation of the SIGGRAPH 2020 paper
"Robust Fitting of Parallax-Aware Mixtures for Path Guiding".

Copyright (c) 2020 Lukas Ruppert, Sebastian Herholz.
*/

static void splitComponentUsingPCA(PMM_ExtraData& stats, PMM& pmm, const uint32_t component, const float maxKappa)
{
    if (EXPECT_NOT_TAKEN(pmm.getK() == PMM::MaxK::value))
    {
        SLog("Warn", "Skipping split of component, since the mixture has reached its maximum size.");
        return;
    }

#ifdef GUIDING_VALIDATE_INTERMEDIATE_MIXTURE_STATES
    if (!pmm.valid(pmmFactory.m_vmmFactory.computeMinComponentWeight(pmm.getK())))
            SLog(EWarn, "invalid mixture state before splitting component %u", component);
#endif

    const uint32_t sourceIndex = component;
    const uint32_t targetIndex = pmm.getK();

    VMF& sourceComponent = pmm.getComponent(sourceIndex/Scalar::Width::value);
    VMF& targetComponent = pmm.getComponent(targetIndex/Scalar::Width::value);

    //Log(EDebug, "splitting component %zu", index);

    const lightpmm::Frame frame{sourceComponent.getMu(component%Scalar::Width::value)};

    const lightpmm::Matrix2x2 covariance = stats.incrementalCovariance2D.computeCovarianceMatrix(component);

    std::vector<float> eigenValues;
    std::vector<glm::vec2> eigenVectors;
    eigenValues = computeEigenValuesVectors(covariance, eigenVectors);

    const int maxEigValIndex = eigenValues[1] > eigenValues[0];
    const float oneHalfSigmaOffset = std::min(1.0f, 0.5f*sqrt(eigenValues[maxEigValIndex]));

    lightpmm::Vector2 principalComponentDir = eigenVectors[maxEigValIndex];
    principalComponentDir *= oneHalfSigmaOffset;

    const float z = std::sqrt(1.0f-oneHalfSigmaOffset*oneHalfSigmaOffset);

    const lightpmm::Vector3 splitMuA {frame.toWorld({ principalComponentDir.x,  principalComponentDir.y, z})};
    const lightpmm::Vector3 splitMuB {frame.toWorld({-principalComponentDir.x, -principalComponentDir.y, z})};

    pmm.setK(targetIndex+1);

    const float sourceAvgCosine = sourceComponent.getR()[sourceIndex%Scalar::Width::value];
    const float sourceWeight = sourceComponent.getWeight(sourceIndex%Scalar::Width::value);

    const float splitWeight = sourceWeight*0.5f;
    //using this concentration, an immediate merge would recreate the original component
    const float maxAvgCosine = lightpmm::kappaToMeanCosine(maxKappa);
    const float splitAvgCosine = (z > 0.0f) ? std::min(maxAvgCosine, sourceAvgCosine/z) : maxAvgCosine;
    const float splitKappa = lightpmm::meanCosineToKappa(splitAvgCosine);

    sourceComponent.setKappaAndR(sourceIndex%Scalar::Width::value, splitKappa, splitAvgCosine);
    sourceComponent.setMu(sourceIndex%Scalar::Width::value,        splitMuA);
    sourceComponent.setWeight(sourceIndex%Scalar::Width::value,    splitWeight);

    targetComponent.setKappaAndR(targetIndex%Scalar::Width::value, splitKappa, splitAvgCosine);
    targetComponent.setMu(targetIndex%Scalar::Width::value,        splitMuB);
    targetComponent.setWeight(targetIndex%Scalar::Width::value,    splitWeight);

    stats.incrementalDistance.split(pmm, component);
    stats.incrementalPearsonChiSquared.split(pmm, component);
    stats.incrementalCovariance2D.split(pmm, component);

#ifdef GUIDING_VALIDATE_INTERMEDIATE_MIXTURE_STATES
    if (!pmm.valid(pmmFactory.m_vmmFactory.computeMinComponentWeight(pmm.getK())))
            SLog(EWarn, "error occured after splitting component %u", component);
#endif
}

uint32_t PathGuiding::splitAll(PMM_ExtraData& stats, PMM& pmm, const guiding::Range<std::vector<DirectionalData>>& samples, const bool iterative=true, const bool fit=true)
{
    if (EXPECT_NOT_TAKEN(pmm.getK() >= PMM::MaxK::value))
        return false;

    const bool firstFit = samples.size() == pmm.m_totalNumSamples;
    uint32_t totalNumSplits = 0;
    uint32_t numSplits = 0;

    do
    {
        const uint32_t numActiveKernels = (pmm.getK()+Scalar::Width::value-1)/Scalar::Width::value;

        const std::array<Scalar, PMM::NumKernels::value> divergence = stats.incrementalPearsonChiSquared.computeDivergence(pmm);
        std::array<std::pair<float, uint32_t>, PMM::MaxK::value> splitCandidates;
        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            const typename Scalar::BooleanType seenEnoughSamples = stats.incrementalPearsonChiSquared.numSamples[k] > minSamplesForSplitting;
            const Scalar weightedDivergence = lightpmm::ifthen(firstFit || seenEnoughSamples, divergence[k]*pmm.getComponent(k).m_weights, 0.0f);
            for (uint32_t i=0; i<Scalar::Width::value; ++i)
                splitCandidates[k*Scalar::Width::value+i] = std::make_pair(fabs(weightedDivergence[i]), k*Scalar::Width::value+i);
        }

        const auto lastCandidateIterator = std::partition(splitCandidates.begin(), splitCandidates.begin()+pmm.getK(),
                                                          [this](const std::pair<float, uint32_t> divergenceAndIndex) -> bool { return divergenceAndIndex.first >= splitMinDivergence; });

        const uint32_t numSplitCandidates = std::distance(splitCandidates.begin(), lastCandidateIterator);
        const uint32_t maxNumSplits = PMM::MaxK::value-pmm.getK();
        numSplits = std::min(numSplitCandidates, maxNumSplits);

        if (numSplits == 0)
            break;

        if (numSplitCandidates > numSplits)
            std::partial_sort(splitCandidates.begin(), splitCandidates.begin()+numSplits, lastCandidateIterator,
                              [](const std::pair<float, uint32_t> a, const std::pair<float, uint32_t> b) -> bool { return a.first > b.first; });

        std::array<typename Scalar::BooleanType, PMM::NumKernels::value> modifedComponentMask;
        std::fill(modifedComponentMask.begin(), modifedComponentMask.end(), typename Scalar::BooleanType{false});

        for (uint32_t i=0; i<numSplits; ++i)
        {
            modifedComponentMask[splitCandidates[i].second/Scalar::Width::value].insert(splitCandidates[i].second%Scalar::Width::value, true);
            modifedComponentMask[pmm.getK()/Scalar::Width::value].insert(pmm.getK()%Scalar::Width::value, true);

            splitComponentUsingPCA(stats, pmm, splitCandidates[i].second, vmmFactory.m_properties.maxKappa);
        }

        //repeated splitting requires fitting after each split unless all splits are determined beforehand.
        //otherwise, the same component is split over and over again and fitting all the split components becomes increasingly difficult

        //using the masked fit to fit only the splitted components
        if (fit)
        {
            pmm.applyWeightPrior(vmmFactoryProperties.vPrior);
            vmmFactory.maskedFit(samples.begin(), samples.end(), pmm, modifedComponentMask);
            stats.incrementalPearsonChiSquared.updateDivergenceMasked(pmm, samples, modifedComponentMask);
            stats.incrementalCovariance2D.updateStatisticsMasked(pmm, samples, modifedComponentMask);
            pmm.removeWeightPrior(vmmFactoryProperties.vPrior);
        }

        totalNumSplits += numSplits;
    }
    while (iterative);

#ifdef GUIDING_DETAILED_STATISTICS
    avgNumSplits += totalNumSplits;
#endif

    return totalNumSplits;
}

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

void PathGuiding::mergeComponents(int iRegion, const uint32_t componentA, const uint32_t componentB) {
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
