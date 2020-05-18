//
// Created by felixfifi on 06.05.20.
//

#include "Model.h"
#include "VulkanOps.h"

#define STB_IMAGE_IMPLEMENTATION

#include <stb_image.h>

#include <utility>

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
}

void Model::cleanup() {
    device.free(vertexBufferMemory);
    device.destroy(vertexBuffer);
    device.free(indexBufferMemory);
    device.destroy(indexBuffer);
}


