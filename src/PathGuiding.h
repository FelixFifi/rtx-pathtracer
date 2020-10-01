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
#include <nvvkpp/raytraceKHR_vkpp.hpp>

#define MAX_DISTRIBUTIONS 8

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

class PathGuiding {
private:
    uint regionCount;
    bool fitted = false;

    Aabb sceneAabb;

    std::vector<Aabb> aabbs;
    std::vector<VMM_Theta> guidingRegions;

    lightpmm::VMMFactory<PMM> vmmFactory;
    std::vector<PMM> pmms;

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
    PathGuiding() = default;
    PathGuiding(uint splitCount, Aabb sceneAabb, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex);

    void update(SampleCollector sampleCollector);
    uint32_t getRegionCount();

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

    void cleanupBuffers() const;

    void configureVMMFactory();

    void createPMMs();

    static VMM_Theta pmmToVMM_Theta(const PMM &pmm);

    void syncPMMsToVMM_Thetas();
};


#endif //RTX_RAYTRACER_PATHGUIDING_H
