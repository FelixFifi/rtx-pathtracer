//
// Created by felixfifi on 06.05.20.
//

#include "Model.h"
#include "VulkanOps.h"

#define STB_IMAGE_IMPLEMENTATION

#include <stb_image.h>

#define TINYOBJLOADER_IMPLEMENTATION

#include <tiny_obj_loader.h>

Model::Model(const std::string &objFilePath, const std::shared_ptr<VulkanOps> &vulkanOps) : device(
        vulkanOps->getDevice()) {
    loadModel(objFilePath);
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

void Model::loadModel(const std::string &objFilePath) {
//    tinyobj::attrib_t attrib;
//    std::vector<tinyobj::shape_t> shapes;
//    std::vector<tinyobj::material_t> materials;
//    std::string warn, err;
//
//    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, objFilePath.c_str())) {
//        throw std::runtime_error(warn + err);
//    }
    vertices.clear();
    indices.clear();

    Vertex v1 = {};
    v1.pos = {100.0f,0.0f,0.0f};
    v1.color = {1.0f,1.0f,1.0f};
    v1.texCoord = {1.0f,1.0f};
    Vertex v2 = {};
    v2.pos = {0.0f,0.0f,100.0f};
    v2.color = {1.0f,1.0f,1.0f};
    v2.texCoord = {1.0f,1.0f};
    Vertex v3 = {};
    v3.pos = {0.0f,100.0f,0.0f};
    v3.color = {1.0f,1.0f,1.0f};
    v3.texCoord = {1.0f,1.0f};

    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);

    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);

//    std::unordered_map<Vertex, uint32_t> uniqueVertices = {};
//
//    for (const auto &shape : shapes) {
//        for (const auto &index : shape.mesh.indices) {
//            Vertex vertex = {};
//
//            vertex.pos = {
//                    attrib.vertices[3 * index.vertex_index + 0],
//                    attrib.vertices[3 * index.vertex_index + 1],
//                    attrib.vertices[3 * index.vertex_index + 2]
//            };
//
//            vertex.texCoord = {
//                    attrib.texcoords[2 * index.texcoord_index + 0],
//                    1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
//            };
//
//            vertex.color = {1.0f, 1.0f, 1.0f};
//
//            if (uniqueVertices.count(vertex) == 0) {
//                uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
//                vertices.push_back(vertex);
//            }
//
//            indices.push_back(uniqueVertices[vertex]);
//        }
//    }
}

void Model::cleanup() {
    device.free(vertexBufferMemory);
    device.destroy(vertexBuffer);
    device.free(indexBufferMemory);
    device.destroy(indexBuffer);
}
