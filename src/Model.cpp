//
// Created by felixfifi on 06.05.20.
//

#include "Model.h"
#include "VulkanOps.h"

#define STB_IMAGE_IMPLEMENTATION

#include <stb_image.h>

#define TINYOBJLOADER_IMPLEMENTATION

#include <tiny_obj_loader.h>

Model::Model(const std::string &objFilePath, const std::string &materialBaseDir,
             const std::shared_ptr<VulkanOps> &vulkanOps) : device(
        vulkanOps->getDevice()) {
    loadModel(objFilePath, materialBaseDir);
    createBuffers(vulkanOps);
}

void Model::createBuffers(const std::shared_ptr<VulkanOps> &vulkanOps) {
    vk::Flags<vk::BufferUsageFlagBits> usage =
            vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eStorageBuffer |
            vk::BufferUsageFlagBits::eShaderDeviceAddressKHR | vk::BufferUsageFlagBits::eTransferDst;
    vk::MemoryPropertyFlagBits memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(vertices, usage, memoryProperties, vertexBuffer, vertexBufferMemory);


    usage = vk::BufferUsageFlagBits::eIndexBuffer | vk::BufferUsageFlagBits::eStorageBuffer |
            vk::BufferUsageFlagBits::eShaderDeviceAddressKHR | vk::BufferUsageFlagBits::eTransferDst;
    memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(indices, usage, memoryProperties, indexBuffer, indexBufferMemory);
}

void Model::loadModel(const std::string &objFilePath, const std::string &materialBaseDir) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, objFilePath.c_str(), materialBaseDir.c_str())) {
        throw std::runtime_error(warn + err);
    }

    if (materials.size() == 1) {
        hasMaterial = true;
        tinyobj::material_t &tiny_material = materials[0];
        material.diffuse = {tiny_material.diffuse[0], tiny_material.diffuse[1], tiny_material.diffuse[2]};
        material.specular = {tiny_material.specular[0], tiny_material.specular[1], tiny_material.specular[2]};
        material.specularHighlight = tiny_material.shininess;

        switch (tiny_material.illum) {
            case 0:
            case 1:
                material.type = EMatType::eDiffuse;
                break;
            case 2:
            case 3:
                material.type = EMatType::eSpecular;
                break;
            case 4:
                material.type = EMatType::eTransparent;
                break;
            default:
                throw std::runtime_error("Unknown illum mode");
        }

    } else if(materials.size() > 0) {
        throw std::runtime_error("Multiple materials given in one obj file");
    }

    std::unordered_map<Vertex, uint32_t> uniqueVertices = {};

    for (const auto &shape : shapes) {
        for (const auto &index : shape.mesh.indices) {
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

            if (uniqueVertices.count(vertex) == 0) {
                uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                vertices.push_back(vertex);
            }

            indices.push_back(uniqueVertices[vertex]);
        }
    }
}

void Model::cleanup() {
    device.free(vertexBufferMemory);
    device.destroy(vertexBuffer);
    device.free(indexBufferMemory);
    device.destroy(indexBuffer);
}


