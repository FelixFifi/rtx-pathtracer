//
// Created by felixfifi on 17.05.20.
//

#ifndef RTX_RAYTRACER_SCENELOADER_H
#define RTX_RAYTRACER_SCENELOADER_H

static const int VERTICES_PER_FACE = 3;
static const int BINDINGS_COUNT = 7;
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
#include <tinyxml2.h>
#include "Model.h"


enum EMatType {
    eDiffuse = 0,
    eSpecular = 1,
    eDielectric = 2,
    eLight = 3,
    ePhong = 4,
    eConductor = 5,
    eRoughConductor = 6
};

struct alignas(16) Material {
    alignas(16) glm::vec3 lightColor;
    alignas(16) glm::vec3 diffuse;
    alignas(16) glm::vec3 specular;
    float specularHighlight;
    float transparency;
    float refractionIndex;
    float refractionIndexInv;
    float eta;
    float k;
    float roughness;
    int textureIdDiffuse = -1;
    int textureIdSpecular = -1;
    EMatType type;
};

struct alignas(16) Instance {
    glm::mat4 transform;
    glm::mat4 normalTransform;
    int iModel;
    int iLight = -1;
};

struct alignas(16) Light {
    alignas(16) glm::vec3 color; // Does not need to be set for area lights. Is taken from material
    alignas(16) glm::vec3 pos; // Only for point lights
    int isPointLight; // Bool as integer, because glsl uses ints to represent booleans
    uint instanceIndex;
    float sampleProb;
    float area;
};

struct FaceSample {
    int index;
    float sampleProb;
    float faceArea;
};

struct Texture {
    vk::Image image;
    vk::DeviceMemory imageMemory;
    vk::ImageView imageView;
    vk::Sampler sampler;

    void cleanup(const vk::Device &device) {
        device.free(imageMemory);
        device.destroy(sampler);
        device.destroy(imageView);
        device.destroy(image);
    }
};

class SceneLoader {
private:
    std::string objectBaseDir;
    std::string materialBaseDir;
    std::string textureBaseDir;

    std::vector<Instance> instances;
    std::vector<Model> models;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<std::vector<int>> emissiveFacesPerModel;
    std::vector<Texture> textures;
    std::map<std::string, int> pathTextureIdMapping;


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
    glm::vec3 origin {0, 2, 15};
    glm::vec3 target {0, -2, 2.5};
    glm::vec3 upDir {0, 1, 0};
    float vfov = 28.0f;
public:
    SceneLoader() = default;
    SceneLoader(const std::string &filepath, const std::string &objectBaseDir,
                const std::string &materialBaseDir, const std::string &textureBaseDir,
                std::shared_ptr<VulkanOps> vulkanOps, vk::PhysicalDevice &physicalDevice,
                uint32_t graphicsQueueIndex);


    std::array<vk::DescriptorSetLayoutBinding, BINDINGS_COUNT> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, BINDINGS_COUNT> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, BINDINGS_COUNT> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                              std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                                                              std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                                                              vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                                                              vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightsBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightSamplersBufferInfo,
                                                                              std::vector<vk::DescriptorImageInfo> &outTexturesInfos);

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
    void parseMitsubaSceneFile(const std::string &filepath);

    void createInstanceInfoBuffer();

    void createVulkanObjects(vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex);

    void parseModels(nlohmann::basic_json<> &j, std::map<std::string, int> &nameIndexMapping);

    void parseInstances(const nlohmann::basic_json<> &j, std::map<std::string, int> &nameIndexMapping);

    void createLightsBuffers();

    void parsePointLights(const nlohmann::basic_json<> &j);

    void createLightSamplersBuffer();

    std::vector<int> getLightSamplingVector();

    std::vector<FaceSample> getFaceSamplingVector();

    int addTexture(const std::string &textureName);

    void parseCameraSettings(tinyxml2::XMLElement *xScene);

    Material parseXmlBSDF(tinyxml2::XMLElement *xBSDF, std::string &outId) const;

    std::map<std::string, int> parseXmlBSDFs(tinyxml2::XMLElement *xScene);

    void
    converteObjData(const std::vector<tinyobj::shape_t> &shapes, const tinyobj::attrib_t &attrib,
                    int materialIndexOffset, int materialIndexOverride, std::vector<Vertex> &outVertices,
                    std::vector<uint32_t> &outIndices, std::vector<int> &outEmissiveFaces) const;

    void readObjFile(const std::string &objFilePath, tinyobj::attrib_t &attrib, std::vector<tinyobj::shape_t> &shapes,
                     std::vector<tinyobj::material_t> &tinyMaterials) const;

    std::string toObjPath(const std::string &path);

    void parseXmlShapes(tinyxml2::XMLElement *xScene, std::map<std::string, int> &definedMaterials);

    Texture generateDefaultTexture() const;
};


#endif //RTX_RAYTRACER_SCENELOADER_H
