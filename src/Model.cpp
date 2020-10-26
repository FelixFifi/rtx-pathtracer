//
// Created by felixfifi on 06.05.20.
//

#include "Model.h"
#include "VulkanOps.h"

#include <utility>
#include <limits>

Model::Model(std::vector<Vertex> vertices, std::vector<uint32_t> indices,
             const std::shared_ptr<VulkanOps> &vulkanOps) : vertices(std::move(vertices)), indices(std::move(indices)), device(vulkanOps->getDevice()) {
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


    vulkanOps->setBufferName(vertexBuffer, "B: Model Vertex");
    vulkanOps->setBufferName(indexBuffer, "B: Model Index");
}

float Model::getFaceArea(int iFace, glm::mat4 transform) {
    glm::vec3 v0{}, v1{}, v2{};
    v0 = vertices[indices[3 * iFace + 0]].pos;
    v1 = vertices[indices[3 * iFace + 1]].pos;
    v2 = vertices[indices[3 * iFace + 2]].pos;

    // To account for scaling, transform vertices
    v0 = transform * glm::vec4(v0, 1.0f);
    v1 = transform * glm::vec4(v1, 1.0f);
    v2 = transform * glm::vec4(v2, 1.0f);

    glm::vec3 d01 = v1 - v0;
    glm::vec3 d02 = v2 - v0;

    // https://math.stackexchange.com/a/128995
    float l01 = glm::length(d01);
    float l02 = glm::length(d02);

    float cosAlpha = glm::dot(d01, d02) / (l01 * l02);
    float sinAlpha = sqrt(1 - cosAlpha * cosAlpha);

    return l01 * l02 * sinAlpha / 2.0f;
}

void Model::cleanup() {
    device.free(vertexBufferMemory);
    device.destroy(vertexBuffer);
    device.free(indexBufferMemory);
    device.destroy(indexBuffer);
}

Aabb Model::getAabb(glm::mat4 transform) {
    Aabb aabb;

    float infinity = std::numeric_limits<float>::infinity();
    aabb.min = {infinity, infinity, infinity};
    aabb.max = {-infinity, -infinity, -infinity};

    for (Vertex &v : vertices) {
        aabb.update(transform * glm::vec4(v.pos, 1));
    }

    return aabb;
}


