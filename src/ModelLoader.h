//
// Created by felixfifi on 17.05.20.
//

#ifndef RTX_RAYTRACER_MODELLOADER_H
#define RTX_RAYTRACER_MODELLOADER_H

static const int VERTICES_PER_FACE = 3;

#include "VulkanLoader.h"

#include <tiny_obj_loader.h>

// #VKRay
#define ALLOC_DEDICATED
#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>
#include "Model.h"

class ModelLoader {
private:
    std::string materialBaseDir;

    std::vector<Model> models;
    std::vector<Material> materials;

    std::shared_ptr<VulkanOps> vulkanOps;

    vk::Device device;
    vk::Buffer materialBuffer;
    vk::DeviceMemory materialBufferMemory;

    nvvkpp::RaytracingBuilderKHR rtBuilder;
public:
    ModelLoader() = default;
    ModelLoader(const std::vector<std::string> &objPaths, const std::string &materialBaseDir,
                std::shared_ptr<VulkanOps> vulkanOps, vk::PhysicalDevice &physicalDevice,
                uint32_t graphicsQueueIndex);

    void loadModel(const std::string &objFilePath);

    std::array<vk::DescriptorSetLayoutBinding, 3> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, 3> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, 3> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                 std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                                                 std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                                                 vk::DescriptorBufferInfo &outMaterialBufferInfo);

    const vk::AccelerationStructureKHR & getAccelerationStructure();

    void cleanup();
private:
    void addModel(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes,
                  int materialIndexOffset);

    void addMaterials(const std::vector<tinyobj::material_t> &tinyMaterials);

    nvvkpp::RaytracingBuilderKHR::Blas modelToBlas(const Model &model);

    void createBottomLevelAS();

    void createMaterialBuffer();
    void createTopLevelAS();
};


#endif //RTX_RAYTRACER_MODELLOADER_H
