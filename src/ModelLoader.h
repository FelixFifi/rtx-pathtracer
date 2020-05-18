//
// Created by felixfifi on 17.05.20.
//

#ifndef RTX_RAYTRACER_MODELLOADER_H
#define RTX_RAYTRACER_MODELLOADER_H

static const int VERTICES_PER_FACE = 3;

#include "VulkanLoader.h"

#include "tiny_obj_loader.h"

// #VKRay
#define ALLOC_DEDICATED
#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>
#include "Model.h"

struct Instance {
    int iModel;
    glm::mat4 transform;
    glm::mat4 normalTransform;
};

struct alignas(16) InstanceInfo {
    glm::mat4 normalTransform;
    int iModel;
};

class ModelLoader {
private:
    std::string objectBaseDir;
    std::string materialBaseDir;

    std::vector<Instance> instances;
    std::vector<Model> models;
    std::vector<Material> materials;

    std::shared_ptr<VulkanOps> vulkanOps;

    vk::Device device;
    vk::Buffer materialBuffer;
    vk::DeviceMemory materialBufferMemory;

    vk::Buffer instanceInfoBuffer;
    vk::DeviceMemory instanceInfoBufferMemory;

    nvvkpp::RaytracingBuilderKHR rtBuilder;
public:
    ModelLoader() = default;
    ModelLoader(const std::vector<std::string> &objPaths, const std::string &materialBaseDir,
                std::shared_ptr<VulkanOps> vulkanOps, vk::PhysicalDevice &physicalDevice,
                uint32_t graphicsQueueIndex);
    ModelLoader(const std::string &filepath, const std::string &objectBaseDir,
                const std::string &materialBaseDir, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex);


    std::array<vk::DescriptorSetLayoutBinding, 4> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, 4> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 4> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                         std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                         std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                         vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                         vk::DescriptorBufferInfo &outInstanceInfoBufferInfo);

    const vk::AccelerationStructureKHR & getAccelerationStructure();

    size_t getModelCount();

    void cleanup();
private:

    void loadModel(const std::string &objFilePath);
    void addModel(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes,
                  int materialIndexOffset);

    void addMaterials(const std::vector<tinyobj::material_t> &tinyMaterials);

    nvvkpp::RaytracingBuilderKHR::Blas modelToBlas(const Model &model);

    void createBottomLevelAS();

    void createMaterialBuffer();
    void createTopLevelAS();

    void parseSceneFile(const std::string &filepath);

    void createInstanceInfoBuffer();

    void createVulkanObjects(vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex);
};


#endif //RTX_RAYTRACER_MODELLOADER_H
