//
// Created by felixfifi on 17.05.20.
//

#ifndef RTX_RAYTRACER_SCENELOADER_H
#define RTX_RAYTRACER_SCENELOADER_H

static const int VERTICES_PER_FACE = 3;
static const int BINDINGS_COUNT = 8;

#include "VulkanLoader.h"
#include "../shaders/limits.glsl"
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
#include "Shapes.h"

const std::string SCENE_BASE_DIR = "scenes/";
const std::string MODELS_BASE_DIR = "models/";
const std::string MATERIAL_BASE_DIR = "materials/";
const std::string TEXTURE_BASE_DIR = "textures/";

enum EMatType {
    eDiffuse = 0,
    eSpecular = 1,
    eDielectric = 2,
    eLight = 3,
    ePhong = 4,
    eConductor = 5,
    eRoughConductor = 6
};

enum ELightType {
    eArea = 0,
    ePointLight = 1,
    eSphere = 2,
    eEnvMap = 3,
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
    int type;
};

struct alignas(16) Instance {
    glm::mat4 transform;
    glm::mat4 normalTransform;
    int iModel;
    int iLight = -1;
};

struct Light {
    glm::vec3 color; // Does not need to be set for area lights. Is taken from material
    glm::vec3 pos; // Only for point lights
    uint instanceIndex;
    float sampleProb;
    float area;
    int type;
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
    std::string modelsBaseDir;
    std::string materialBaseDir;
    std::string textureBaseDir;

    std::vector<Instance> instances;
    std::vector<Sphere> spheres;
    std::vector<Model> models;
    std::vector<Aabb> aabbs;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<std::vector<int>> emissiveFacesPerModel;
    std::vector<Texture> textures;
    std::map<std::string, int> pathTextureIdMapping;

    Aabb sceneSize{};
public:
    const Aabb &getSceneSize() const;

private:

    bool hasEnvMap = false;

    uint32_t spheresIndex = -1;
public:
    uint32_t getSpheresIndex() const;

private:

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

    std::vector<Model> defaultModels;
public:
    glm::vec3 origin {0, -10, 4};
    glm::vec3 target {0, 0, 4};
    glm::vec3 upDir {0, 0, 1};
    float vfov = 28.0f;

    vk::Buffer sphereBuffer;
    vk::DeviceMemory sphereBufferMemory;
    vk::Buffer aabbBuffer;
    vk::DeviceMemory aabbBufferMemory;

    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;
    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> tlas;
public:
    SceneLoader() = default;
    SceneLoader(const std::string &filepath, std::shared_ptr<VulkanOps> vulkanOps,
                vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex);


    std::array<vk::DescriptorSetLayoutBinding, BINDINGS_COUNT> getDescriptorSetLayouts();

    std::array<vk::DescriptorPoolSize, BINDINGS_COUNT> getDescriptorPoolSizes();

    std::array<vk::WriteDescriptorSet, BINDINGS_COUNT> getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                              std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                                                              std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                                                              vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                                                              vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightsBufferInfo,
                                                                              vk::DescriptorBufferInfo &outLightSamplersBufferInfo,
                                                                              std::vector<vk::DescriptorImageInfo> &outTexturesInfos,
                                                                              vk::DescriptorBufferInfo &outSpheresBufferInfo);

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
    void createSpheresBuffer();
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

    void parseEnvMap(tinyxml2::XMLElement *xScene);

    void calculateSceneSize();

    void parseJsonCamera(const nlohmann::basic_json<> &j);
};


#endif //RTX_RAYTRACER_SCENELOADER_H
