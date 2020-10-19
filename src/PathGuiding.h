//
// Created by felixfifi on 19.09.20.
//

#ifndef RTX_RAYTRACER_PATHGUIDING_H
#define RTX_RAYTRACER_PATHGUIDING_H

#include "VulkanLoader.h"

#include "Shapes.h"
#include "VulkanOps.h"
#include "SampleCollector.h"


#include "pmm/VMMFactory.h"
#include "guiding/incrementaldistance.h"
#include "guiding/incrementalpearsonchisquared.h"
#include "guiding/incrementalcovariance2d.h"
#include "guiding/Range.h"

#include <nvvkpp/raytraceKHR_vkpp.hpp>

#define MAX_DISTRIBUTIONS 32

// Taken from lightpmm/VMFKernel.h
// the minumum value of kappa before it gets set to 0.0 for numerical stability
#define VMF_MinKappa 1e-3f

typedef lightpmm::Scalar4 Scalar;
typedef lightpmm::VMFKernel<Scalar> VMF;
typedef lightpmm::ParametricMixtureModel<VMF, MAX_DISTRIBUTIONS / Scalar::Width::value> PMM;

static const int BINDING_AABB = 15;
static const int BINDING_GUIDING = 16;
static const int BINDING_AS = 17;

struct VMF_Theta {
    glm::vec3 mu;
    float k;
    float norm; // = k / (M_2PI * (1 - exp(- 2 * theta.k)))
    float eMin2K; // exp(-2.0f * k)
    float distance = -1.0f;
    glm::vec3 target;

    void setK(float newK) {
        newK = newK < VMF_MinKappa ? 0.0 : newK;

        k = newK;
        norm = k / (2 * M_PI * (1 - exp(- 2 * k)));
        eMin2K = exp(-2.0 * k);
    }

    std::string toString() const {
        std::ostringstream result;
        result << "k: " << k << " mu: " << mu.x << "|" << mu.y << "|" << mu.z;
        return result.str();
    }
};

struct VMM_Theta {
    VMF_Theta thetas[MAX_DISTRIBUTIONS];
    float pi[MAX_DISTRIBUTIONS];
    glm::vec3 meanPosition;
    int usedDistributions;

    std::string toString() const {
        std::ostringstream result;

        result << "VMM:\n";
        for (const auto &theta : thetas) {
            result << theta.toString() << "\n";
        }
        result << "\n";
        return result.str();
    }
};

struct PMM_ExtraData {
    glm::vec3 parallaxMean;
    glm::vec3 lastParallaxMean;
    guiding::IncrementalDistance<PMM> incrementalDistance;
    guiding::IncrementalCovariance2D<PMM> incrementalCovariance2D;
    guiding::IncrementalPearsonChiSquared<PMM> incrementalPearsonChiSquared;
    uint32_t samplesSinceLastMerge = 0;
};

class PathGuiding {
private:
    uint regionCount;
    bool firstFit = true;

    lightpmm::VMMFactoryProperties vmmFactoryProperties{};

    Aabb sceneAabb;

    std::vector<Aabb> aabbs;
    std::vector<VMM_Theta> guidingRegions;

    lightpmm::VMMFactory<PMM> vmmFactory;
    std::vector<PMM> pmms;
    std::vector<PMM_ExtraData> pmmsExtraData;

    // Parallax Compensation
    bool useParrallaxCompensation;

    std::shared_ptr<VulkanOps> vulkanOps;

    nvvkpp::RaytracingBuilderKHR rtBuilder;

    vk::Device device;

    vk::Buffer aabbsBuffer;
    vk::DeviceMemory aabbsBufferMemory;
    vk::Buffer guidingBuffer;
    vk::DeviceMemory guidingBufferMemory;
    vk::Buffer guidingUpdateBuffer;
    vk::DeviceMemory guidingUpdateBufferMemory;

    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> instances;
    vk::AccelerationStructureKHR accelerationStructure;
public:
    bool splitRegions = false;
    int samplesForRegionSplit = 10000;

    bool splitAndMerge = true;
    size_t minSamplesForMerging = 8192;
    size_t minSamplesForSplitting {4096};
    int minSamplesForPostSplitFitting = 4096;
    float splitMinDivergence {0.5f};
    float mergeMaxDivergence {0.025f};

    PathGuiding() = default;
    PathGuiding(uint splitCount, Aabb sceneAabb, bool useParrallaxCompensation, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex);

    bool update(SampleCollector sampleCollector);
    uint32_t getRegionCount() const;

    std::array<vk::DescriptorSetLayoutBinding, 3> getDescriptorSetLayouts();

    static std::array<vk::DescriptorPoolSize, 3> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 3> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                 vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                                                                 vk::DescriptorBufferInfo &outAabbsBufferInfo,
                                                                 vk::DescriptorBufferInfo &outGuidingBufferInfo);

    void cleanup();
private:
    void createRegions(uint splitCount);
    void createBuffers();
    void createAS();

    void cleanupBuffers();

    void configureVMMFactory();

    void createPMMs();

    static VMM_Theta pmmToVMM_Theta(const PMM &pmm);

    void syncPMMsToVMM_Thetas();

    void updateRegion(std::shared_ptr<std::vector<DirectionalData>> &directionalData, int iRegion, uint32_t regionBegin,
                      uint32_t nextRegionBegin);

    void splitRegion(int iRegion);

    void syncToGPU();

    void createVulkanObjects();

    void preFit(const std::shared_ptr<std::vector<DirectionalData>> &directionalData, int iRegion, uint32_t regionBegin,
                uint32_t nextRegionBegin);

    void postFit(int iRegion, const guiding::Range<std::vector<DirectionalData>> &sampleRange);

    bool mergeAll(int iRegion, const guiding::Range<std::vector<DirectionalData>> &sampleRange);

    static std::array<float, PMM::MaxK::value * (PMM::MaxK::value - 1) / 2> computePearsonChiSquaredMergeMetric(
            const PMM &distribution);

    void mergeComponents(int iRegion, const uint32_t componentA, const uint32_t componentB);

    uint32_t splitAll(PMM_ExtraData &stats, PMM &pmm, const guiding::Range<std::vector<DirectionalData>> &samples,
                      const bool iterative, const bool fit);
};


#endif //RTX_RAYTRACER_PATHGUIDING_H
