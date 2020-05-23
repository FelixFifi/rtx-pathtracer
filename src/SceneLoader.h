//
// Created by felixfifi on 17.05.20.
//

#ifndef RTX_RAYTRACER_SCENELOADER_H
#define RTX_RAYTRACER_SCENELOADER_H

static const int VERTICES_PER_FACE = 3;
static const int BINDINGS_COUNT = 6;
#define SIZE_LIGHT_RANDOM 10000
#define SIZE_TRI_RANDOM 10000

#include "VulkanLoader.h"

#include "tiny_obj_loader.h"

// #VKRay
#define ALLOC_DEDICATED
#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>
#include <json.hpp>
#include "Model.h"

struct alignas(16) Instance {
    glm::mat4 transform;
    glm::mat4 normalTransform;
    int iModel;
};

struct alignas(16) Light {
    alignas(16) glm::vec3 color; // Does not need to be set for area lights. Is taken from material
    alignas(16) glm::vec3 pos; // Only for point lights
    int isPointLight; // Bool as integer, because glsl uses ints to represent booleans
    uint instanceIndex;
    float sampleProb;
};

struct FaceSample {
    int index;
    float sampleProb;
    float faceArea;
};

class SceneLoader {
private:
    std::string objectBaseDir;
    std::string materialBaseDir;

    std::vector<Instance> instances;
    std::vector<Model> models;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<std::vector<int>> emissiveFacesPerModel;

    std::shared_ptr<VulkanOps> vulkanOps;

    vk::Device device;
    vk::Buffer materialBuffer;
    vk::DeviceMemory materialBufferMemory;

    vk::Buffer instanceInfoBuffer;
    vk::DeviceMemory instanceInfoBufferMemory;
    vk::Buffer lightsBuffer;
    vk::DeviceMemory lightsBufferMemory;
    vk::Buffer lightsSamplersBuffer;
    vk::DeviceMemory lightsSamplerBufferMemory;

    nvvkpp::RaytracingBuilderKHR rtBuilder;
public:
    SceneLoader() = default;
    SceneLoader(const std::string &filepath, const std::string &objectBaseDir,
                const std::string &materialBaseDir, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex);


    std::array<vk::DescriptorSetLayoutBinding, BINDINGS_COUNT> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, BINDINGS_COUNT> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, BINDINGS_COUNT> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                              std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                                                              std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                                                              vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                                                              vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightsBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightSamplersBufferInfo);

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

    void parseModels(nlohmann::basic_json<> &j, std::map<std::string, int> &nameIndexMapping);

    void parseInstances(const nlohmann::basic_json<> &j, std::map<std::string, int> &nameIndexMapping);

    void createLightsBuffers();

    void parsePointLights(const nlohmann::basic_json<> &j);

    void createLightSamplersBuffer();

    std::vector<int> getLightSamplingVector();

    std::vector<FaceSample> getFaceSamplingVector();
};


#endif //RTX_RAYTRACER_SCENELOADER_H
