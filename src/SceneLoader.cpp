//
// Created by felixfifi on 17.05.20.
//

#include "SceneLoader.h"

#define TINYOBJLOADER_IMPLEMENTATION

#include "tiny_obj_loader.h"

#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>

#include "json.hpp"
#include "WeightedSampler.h"

using json = nlohmann::json;

void SceneLoader::createVulkanObjects(vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex) {
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);
    createMaterialBuffer();
    createInstanceInfoBuffer();
    createLightsBuffers();
    createBottomLevelAS();
    createTopLevelAS();
}

SceneLoader::SceneLoader(const std::string &filepath, const std::string &objectBaseDir,
                         const std::string &materialBaseDir, std::shared_ptr<VulkanOps> vulkanOps,
                         vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex)
        : objectBaseDir(objectBaseDir), materialBaseDir(materialBaseDir), vulkanOps(vulkanOps),
          device(vulkanOps->getDevice()) {
    parseSceneFile(filepath);

    createVulkanObjects(physicalDevice, graphicsQueueIndex);
}

void SceneLoader::loadModel(const std::string &objFilePath) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> tinyMaterials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &tinyMaterials, &warn, &err, objFilePath.c_str(),
                          materialBaseDir.c_str())) {
        throw std::runtime_error(warn + err);
    }

    int materialIndexOffset = materials.size();

    addMaterials(tinyMaterials);

    addModel(attrib, shapes, materialIndexOffset);

}

void SceneLoader::addMaterials(const std::vector<tinyobj::material_t> &tinyMaterials) {
    for (const auto &tinyMaterial : tinyMaterials) {
        Material material{};

        material.lightColor = {tinyMaterial.ambient[0], tinyMaterial.ambient[1], tinyMaterial.ambient[2]};
        material.diffuse = {tinyMaterial.diffuse[0], tinyMaterial.diffuse[1], tinyMaterial.diffuse[2]};
        material.specular = {tinyMaterial.specular[0], tinyMaterial.specular[1], tinyMaterial.specular[2]};
        material.specularHighlight = tinyMaterial.shininess;
        material.refractionIndex = tinyMaterial.ior;
        material.refractionIndexInv = 1.0f / tinyMaterial.ior;

        switch (tinyMaterial.illum) {
            case 0:
            case 1:
                material.type = eDiffuse;
                break;
            case 2:
            case 3:
                material.type = eSpecular;
                break;
            case 4:
                material.type = eTransparent;
                break;
            case 11:
                material.type = eLight;
                break;
            default:
                throw std::runtime_error("Unknown illum mode");
        }

        materials.push_back(material);
    }
}

void SceneLoader::addModel(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes,
                           int materialIndexOffset) {
    std::unordered_map<Vertex, uint32_t> uniqueVertices = {};

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<int> emissiveFaces;


    for (const auto &shape : shapes) { // TODO: separate obj by shape instead of joing all
        unsigned long numFaces = shape.mesh.indices.size() / VERTICES_PER_FACE;
        for (int iFace = 0; iFace < numFaces; ++iFace) {

            // Material offsets are per file, but we accumulate materials over multiple files
            int materialIndex = materialIndexOffset + shape.mesh.material_ids[iFace];
            for (int i = 0; i < VERTICES_PER_FACE; i++) {
                const auto &index = shape.mesh.indices[VERTICES_PER_FACE * iFace + i];
                Vertex vertex = {};

                vertex.pos = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2]
                };

                vertex.normal = {
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2]
                };

                vertex.texCoord = {
                        attrib.texcoords[2 * index.texcoord_index + 0],
                        1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                };

                vertex.materialIndex = materialIndex;

                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }

                indices.push_back(uniqueVertices[vertex]);
            }

            if (materials[materialIndex].type == eLight) {
                emissiveFaces.push_back(iFace);
            }

        }
    }

    models.emplace_back(vertices, indices, vulkanOps);
    emissiveFacesPerModel.push_back(emissiveFaces);
}

void SceneLoader::parseSceneFile(const std::string &filepath) {// read a JSON file
    std::ifstream inFile(filepath);
    json j;
    inFile >> j;

    std::map<std::string, int> nameIndexMapping;

    parseModels(j, nameIndexMapping);

    parseInstances(j, nameIndexMapping);

    parsePointLights(j);

}

/**
 * Has to come after instances, because the area lights have to be added first
 * @param j
 */
void SceneLoader::parsePointLights(const json &j) {
    if (j.contains("lights")) {
        std::vector<json> jLights = j["lights"];

        for (auto jLight : jLights) {
            Light light{};
            light.isPointLight = true;
            std::vector<float> color = jLight["color"];
            light.color = {color[0], color[1], color[2]};

            std::vector<float> pos = jLight["position"];
            light.pos = {pos[0], pos[1], pos[2]};

            lights.push_back(light);
        }
    }
}

void SceneLoader::parseInstances(const json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jInstances = j["instances"];

    for (auto jInstance : jInstances) {
        std::string name = jInstance.items().begin().key();

        Instance instance{};
        instance.iModel = nameIndexMapping[name];

        // if any of the model faces are emissive -> add light struct
        if (!emissiveFacesPerModel[instance.iModel].empty()) {
            Light light{};
            light.isPointLight = false;
            light.instanceIndex = instances.size();
            lights.push_back(light);
        }

        glm::mat4 transform(1.0f);
        // Normals have to be transformed differently
        // Scale needs to be inverted
        glm::mat4 normalTransform(1.0f);

        std::unordered_map<std::string, json> properties = jInstance[name];


        if (properties.contains("translate")) {
            std::vector<float> translate = properties["translate"];
            transform = glm::translate(transform, {translate[0], translate[1], translate[2]});
        }

        if (properties.contains("rotate")) {
            std::vector<float> rotate = properties["rotate"];
            transform = glm::rotate(transform, glm::radians(rotate[0]), {1, 0, 0});
            transform = glm::rotate(transform, glm::radians(rotate[1]), {0, 1, 0});
            transform = glm::rotate(transform, glm::radians(rotate[2]), {0, 0, 1});

            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[0]), {1, 0, 0});
            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[1]), {0, 1, 0});
            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[2]), {0, 0, 1});
        }

        if (properties.contains("scale")) {
            std::vector<float> scale = properties["scale"];
            transform = glm::scale(transform, {scale[0], scale[1], scale[2]});
            normalTransform = glm::scale(normalTransform, {1.0f / scale[0], 1.0f / scale[1], 1.0f / scale[2]});
        }

        instance.transform = transform;
        instance.normalTransform = normalTransform;

        instances.push_back(instance);
    }
}

void SceneLoader::parseModels(json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jModels = j["models"];

    for (auto jModel : jModels) {
        std::string name = jModel.items().begin().key();
        auto path = jModel[name].get<std::string>();

        loadModel(std::filesystem::path(objectBaseDir) / path);
        nameIndexMapping[name] = models.size() - 1;
    }
}

void SceneLoader::createMaterialBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(materials, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, materialBuffer, materialBufferMemory);
}

void SceneLoader::createInstanceInfoBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(instances, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, instanceInfoBuffer,
                                    instanceInfoBufferMemory);
}

void SceneLoader::createLightsBuffers() {
    createLightSamplersBuffer();

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;

    // Light count is padded with 0s to satisfy struct alignment
    std::vector<int> lightCount{static_cast<int>(lights.size()), 0, 0, 0};
    vulkanOps->createBufferFrom2Data(lightCount, lights, usage,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal, lightsBuffer, lightsBufferMemory);
}

/**
 * Creates buffer that hold samplers to choose a random light, and if it is an area light, to choose a random face.
 * These random numbers should be weighted by the total power and face area.
 */
void SceneLoader::createLightSamplersBuffer() {
    std::vector<int> randomLightIndex = getLightSamplingVector();

    std::vector<FaceSample> randomTriIndicesPerLight = getFaceSamplingVector();

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;

    vulkanOps->createBufferFrom2Data(randomLightIndex, randomTriIndicesPerLight, usage,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal, lightsSamplersBuffer,
                                     lightsSamplerBufferMemory);
}

std::vector<FaceSample> SceneLoader::getFaceSamplingVector() {
    std::vector<FaceSample> randomTriIndicesPerLight;
    for (const auto &light : lights) {
        // Area lights are before all point lights
        if (light.isPointLight) {
            break;
        }

        Instance &instance = instances[light.instanceIndex];
        int iModel = instance.iModel;
        Model model = models[iModel];
        std::vector<float> areas(emissiveFacesPerModel[iModel].size());

        for (int iEmissiveFace = 0; iEmissiveFace < emissiveFacesPerModel[iModel].size(); ++iEmissiveFace) {
            areas[iEmissiveFace] = model.getFaceArea(emissiveFacesPerModel[iModel][iEmissiveFace], instance.transform);
        }

        WeightedSampler faceSampler(areas);

        std::vector<float> probSampleFace = faceSampler.getProbabilities();

        std::vector<FaceSample> randomTriIndices(SIZE_TRI_RANDOM);
        for (int i = 0; i < SIZE_TRI_RANDOM; ++i) {
            int sample = faceSampler.sample();

            FaceSample faceSample{};
            faceSample.index = emissiveFacesPerModel[iModel][sample];
            faceSample.sampleProb = probSampleFace[sample];
            faceSample.faceArea = areas[sample];
            randomTriIndices[i] = faceSample;
        }

        randomTriIndicesPerLight.insert(randomTriIndicesPerLight.end(), randomTriIndices.begin(), randomTriIndices.end());
    }
    return randomTriIndicesPerLight;
}

std::vector<int> SceneLoader::getLightSamplingVector() {
    std::vector<float> powers;
    powers.reserve(lights.size());
    for (const auto &light : lights) {
        powers.push_back(1.0f); // TODO: Calculate per light power for weighting
    }

    WeightedSampler lightSampler(powers);

    // Set probabilities to sample a light
    std::vector<float> probSampleLight = lightSampler.getProbabilities();

    for (int iLight = 0; iLight < lights.size(); ++iLight) {
        lights[iLight].sampleProb = probSampleLight[iLight];
    }

    std::vector<int> randomLightIndex(SIZE_LIGHT_RANDOM);
    for (int i = 0; i < SIZE_LIGHT_RANDOM; ++i) {
        randomLightIndex[i] = lightSampler.sample();
    }
    return randomLightIndex;
}

std::array<vk::DescriptorSetLayoutBinding, BINDINGS_COUNT> SceneLoader::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding vertexBufferBinding(1, vk::DescriptorType::eStorageBuffer, models.size(),
                                                       vk::ShaderStageFlagBits::eClosestHitKHR |
                                                       vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding indexBufferBinding(2, vk::DescriptorType::eStorageBuffer, models.size(),
                                                      vk::ShaderStageFlagBits::eClosestHitKHR |
                                                      vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding materialBufferBinding(3, vk::DescriptorType::eStorageBuffer, 1,
                                                         vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding instanceInfoBufferBinding(4, vk::DescriptorType::eStorageBuffer, 1,
                                                             vk::ShaderStageFlagBits::eClosestHitKHR |
                                                             vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding lightsBufferBinding(5, vk::DescriptorType::eStorageBuffer, 1,
                                                       vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding lightSamplersBufferBinding(6, vk::DescriptorType::eStorageBuffer, 1,
                                                              vk::ShaderStageFlagBits::eRaygenKHR);

    return {vertexBufferBinding, indexBufferBinding, materialBufferBinding, instanceInfoBufferBinding,
            lightsBufferBinding, lightSamplersBufferBinding};
}

std::array<vk::DescriptorPoolSize, BINDINGS_COUNT> SceneLoader::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1)
    };
}

/**
 *
 * @param descriptorSet
 * @param outVertexBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @param outIndexBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @param outMaterialBufferInfo Necessary as their memory location is used in the write descriptor sets
 * @return
 */
std::array<vk::WriteDescriptorSet, BINDINGS_COUNT>
SceneLoader::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                    std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                    std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                    vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                    vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                    vk::DescriptorBufferInfo &outLightsBufferInfo,
                                    vk::DescriptorBufferInfo &outLightSamplersBufferInfo) {
    unsigned long modelCount = models.size();
    outVertexBufferInfos.reserve(modelCount);
    outIndexBufferInfos.reserve(modelCount);

    for (const auto &model: models) {
        vk::DescriptorBufferInfo vertexBufferInfo(model.vertexBuffer, 0, VK_WHOLE_SIZE);
        vk::DescriptorBufferInfo indexBufferInfo(model.indexBuffer, 0, VK_WHOLE_SIZE);

        outVertexBufferInfos.push_back(vertexBufferInfo);
        outIndexBufferInfos.push_back(indexBufferInfo);
    }

    outMaterialBufferInfo = vk::DescriptorBufferInfo(materialBuffer, 0, VK_WHOLE_SIZE);
    outInstanceInfoBufferInfo = vk::DescriptorBufferInfo(instanceInfoBuffer, 0, VK_WHOLE_SIZE);
    outLightsBufferInfo = vk::DescriptorBufferInfo(lightsBuffer, 0, VK_WHOLE_SIZE);
    outLightSamplersBufferInfo = vk::DescriptorBufferInfo(lightsSamplersBuffer, 0, VK_WHOLE_SIZE);

    return {
            vk::WriteDescriptorSet(descriptorSet, 1, 0, modelCount,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   outVertexBufferInfos.data(), nullptr),
            vk::WriteDescriptorSet(descriptorSet, 2, 0, modelCount,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   outIndexBufferInfos.data(), nullptr),
            vk::WriteDescriptorSet(descriptorSet, 3, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outMaterialBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 4, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outInstanceInfoBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 5, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outLightsBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 6, 0, 1,
            vk::DescriptorType::eStorageBuffer, nullptr,
            &outLightSamplersBufferInfo, nullptr)
    };
}

nvvkpp::RaytracingBuilderKHR::Blas SceneLoader::modelToBlas(const Model &model) {
    // Setting up the creation info of acceleration structure
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    asCreate.setIndexType(vk::IndexType::eUint32);
    asCreate.setVertexFormat(vk::Format::eR32G32B32Sfloat);
    asCreate.setMaxPrimitiveCount(model.indices.size() / 3);  // Nb triangles
    asCreate.setMaxVertexCount(model.vertices.size());
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

    // Building part
    auto infoVB = vk::BufferDeviceAddressInfo(model.vertexBuffer);
    auto infoIB = vk::BufferDeviceAddressInfo(model.indexBuffer);

    vk::DeviceAddress vertexAddress = device.getBufferAddressKHR(&infoVB);
    vk::DeviceAddress indexAddress = device.getBufferAddressKHR(&infoIB);

    vk::AccelerationStructureGeometryTrianglesDataKHR triangles;
    triangles.setVertexFormat(asCreate.vertexFormat);
    triangles.setVertexData(vertexAddress);
    triangles.setVertexStride(sizeof(Vertex));
    triangles.setIndexType(asCreate.indexType);
    triangles.setIndexData(indexAddress);
    triangles.setTransformData({});

    // Setting up the build info of the acceleration
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(asCreate.geometryType);
    asGeom.setFlags(vk::GeometryFlagBitsKHR::eOpaque);
    asGeom.geometry.setTriangles(triangles);

    // The primitive itself
    vk::AccelerationStructureBuildOffsetInfoKHR offset;
    offset.setFirstVertex(0);
    offset.setPrimitiveCount(asCreate.maxPrimitiveCount);
    offset.setPrimitiveOffset(0);
    offset.setTransformOffset(0);

    // Our blas is only one geometry, but could be made of many geometries
    nvvkpp::RaytracingBuilderKHR::Blas blas{};
    blas.asGeometry.emplace_back(asGeom);
    blas.asCreateGeometryInfo.emplace_back(asCreate);
    blas.asBuildOffsetInfo.emplace_back(offset);

    return blas;
}

void SceneLoader::createBottomLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;
    allBlas.reserve(models.size());

    for (const auto &model : models) {
        allBlas.push_back(modelToBlas(model));
    }

    rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void SceneLoader::createTopLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> tlas;
    tlas.reserve(models.size());

    for (int i = 0; i < static_cast<int>(instances.size()); ++i) {
        Instance instance = instances[i];

        nvvkpp::RaytracingBuilderKHR::Instance rayInst;
        rayInst.transform = glm::value_ptr(instance.transform);
        rayInst.instanceId = i;
        rayInst.blasId = instance.iModel;
        rayInst.hitGroupId = 0; // Same hit group for all
        rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        rayInst.mask = 0xFF;
        tlas.emplace_back(rayInst);
    }

    rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

const vk::AccelerationStructureKHR &SceneLoader::getAccelerationStructure() {
    return rtBuilder.getAccelerationStructure();
}

void SceneLoader::cleanup() {

    device.free(materialBufferMemory);
    device.destroy(materialBuffer);
    device.free(instanceInfoBufferMemory);
    device.destroy(instanceInfoBuffer);
    device.free(lightsBufferMemory);
    device.destroy(lightsBuffer);
    device.free(lightsSamplerBufferMemory);
    device.destroy(lightsSamplersBuffer);

    for (auto &model: models) {
        model.cleanup();
    }

    rtBuilder.destroy();
}

size_t SceneLoader::getModelCount() {
    return models.size();
}
