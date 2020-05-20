//
// Created by felixfifi on 17.05.20.
//

#include "ModelLoader.h"

#define TINYOBJLOADER_IMPLEMENTATION

#include "tiny_obj_loader.h"

#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>

#include "json.hpp"

using json = nlohmann::json;

ModelLoader::ModelLoader(const std::vector<std::string> &objPaths, const std::string &materialBaseDir,
                         std::shared_ptr<VulkanOps> vulkanOps, vk::PhysicalDevice &physicalDevice,
                         uint32_t graphicsQueueIndex)
        : materialBaseDir(materialBaseDir), vulkanOps(vulkanOps), device(vulkanOps->getDevice()) {

    for (const auto &objPath : objPaths) {
        loadModel(objPath);
    }

    createVulkanObjects(physicalDevice, graphicsQueueIndex);
}

void ModelLoader::createVulkanObjects(vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex) {
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);
    createMaterialBuffer();
    createInstanceInfoBuffer();
    createLightsBuffer();
    createBottomLevelAS();
    createTopLevelAS();
}

ModelLoader::ModelLoader(const std::string &filepath, const std::string &objectBaseDir,
                         const std::string &materialBaseDir, std::shared_ptr<VulkanOps> vulkanOps,
                         vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex)
        : objectBaseDir(objectBaseDir), materialBaseDir(materialBaseDir), vulkanOps(vulkanOps),
          device(vulkanOps->getDevice()) {
    parseSceneFile(filepath);

    createVulkanObjects(physicalDevice, graphicsQueueIndex);
}

void ModelLoader::loadModel(const std::string &objFilePath) {
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

void ModelLoader::addMaterials(const std::vector<tinyobj::material_t> &tinyMaterials) {
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

void ModelLoader::addModel(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes,
                           int materialIndexOffset) {
    std::unordered_map<Vertex, uint32_t> uniqueVertices = {};

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    for (const auto &shape : shapes) {
        unsigned long numFaces = shape.mesh.indices.size() / VERTICES_PER_FACE;
        for (int iFace = 0; iFace < numFaces; ++iFace) {
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

                // Material offsets are per file, but we accumulate materials over multiple files
                vertex.materialIndex = materialIndexOffset + shape.mesh.material_ids[iFace];

                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }

                indices.push_back(uniqueVertices[vertex]);
            }
        }
    }

    models.emplace_back(vertices, indices, vulkanOps);
}

void ModelLoader::parseSceneFile(const std::string &filepath) {// read a JSON file
    std::ifstream inFile(filepath);
    json j;
    inFile >> j;

    std::map<std::string, int> nameIndexMapping;

    parseModels(j, nameIndexMapping);

    parseInstances(j, nameIndexMapping);

    parseLights(j);

}

void ModelLoader::parseLights(const json &j) {
    if (j.contains("lights")) {
        std::vector<json> jLights = j["lights"];

        for (auto jLight : jLights) {
            Light light{};
            std::vector<float> color = jLight["color"];
            light.color = {color[0], color[1], color[2]};

            std::vector<float> pos = jLight["position"];
            light.pos = {pos[0], pos[1], pos[2]};

            lights.push_back(light);
        }
    }
}

void ModelLoader::parseInstances(const json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jInstances = j["instances"];

    for (auto jInstance : jInstances) {
        std::string name = jInstance.items().begin().key();

        Instance instance{};
        instance.iModel = nameIndexMapping[name];

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

void ModelLoader::parseModels(json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jModels = j["models"];

    for (auto jModel : jModels) {
        std::string name = jModel.items().begin().key();
        auto path = jModel[name].get<std::string>();

        loadModel(std::filesystem::path(objectBaseDir) / path);
        nameIndexMapping[name] = models.size() - 1;
    }
}

void ModelLoader::createMaterialBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(materials, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, materialBuffer, materialBufferMemory);
}

void ModelLoader::createInstanceInfoBuffer() {
    std::vector<InstanceInfo> instanceInfos;
    instanceInfos.reserve(instances.size());

    for (const auto &instance : instances) {
        InstanceInfo info{instance.normalTransform, instance.iModel};
        instanceInfos.emplace_back(info);
    }

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(instanceInfos, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, instanceInfoBuffer,
                                    instanceInfoBufferMemory);
}

void ModelLoader::createLightsBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;

    // Light count is padded with 0s to satisfy struct alignment
    std::vector<int> lightCount{static_cast<int>(lights.size()), 0, 0, 0};
    vulkanOps->createBufferFrom2Data(lightCount, lights, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, lightsBuffer, lightsBufferMemory);
}

std::array<vk::DescriptorSetLayoutBinding, 5> ModelLoader::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding vertexBufferBinding(1, vk::DescriptorType::eStorageBuffer, models.size(),
                                                       vk::ShaderStageFlagBits::eClosestHitKHR);

    vk::DescriptorSetLayoutBinding indexBufferBinding(2, vk::DescriptorType::eStorageBuffer, models.size(),
                                                      vk::ShaderStageFlagBits::eClosestHitKHR);

    vk::DescriptorSetLayoutBinding materialBufferBinding(3, vk::DescriptorType::eStorageBuffer, 1,
                                                         vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding instanceInfoBufferBinding(4, vk::DescriptorType::eStorageBuffer, 1,
                                                             vk::ShaderStageFlagBits::eClosestHitKHR);
    vk::DescriptorSetLayoutBinding lightsBufferBinding(5, vk::DescriptorType::eStorageBuffer, 1,
                                                             vk::ShaderStageFlagBits::eRaygenKHR);

    return {vertexBufferBinding, indexBufferBinding, materialBufferBinding, instanceInfoBufferBinding, lightsBufferBinding};
}

std::array<vk::DescriptorPoolSize, 5> ModelLoader::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
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
std::array<vk::WriteDescriptorSet, 5> ModelLoader::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                                                          std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                                                          std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                                                          vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                                                          vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                                                          vk::DescriptorBufferInfo &outLightsBufferInfo) {
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
                                   &outLightsBufferInfo, nullptr)
    };
}

nvvkpp::RaytracingBuilderKHR::Blas ModelLoader::modelToBlas(const Model &model) {
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

void ModelLoader::createBottomLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;
    allBlas.reserve(models.size());

    for (const auto &model : models) {
        allBlas.push_back(modelToBlas(model));
    }

    rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void ModelLoader::createTopLevelAS() {
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

const vk::AccelerationStructureKHR &ModelLoader::getAccelerationStructure() {
    return rtBuilder.getAccelerationStructure();
}

void ModelLoader::cleanup() {

    device.free(materialBufferMemory);
    device.destroy(materialBuffer);
    device.free(instanceInfoBufferMemory);
    device.destroy(instanceInfoBuffer);
    device.free(lightsBufferMemory);
    device.destroy(lightsBuffer);

    for (auto &model: models) {
        model.cleanup();
    }

    rtBuilder.destroy();
}

size_t ModelLoader::getModelCount() {
    return models.size();
}
