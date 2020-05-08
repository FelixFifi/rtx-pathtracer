//
// Created by felixfifi on 06.05.20.
//

#ifndef RTX_RAYTRACER_MODEL_H
#define RTX_RAYTRACER_MODEL_H

#include "VulkanLoader.h"
#include "VulkanOps.h"


#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

#include <string>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec3 color;
    glm::vec2 texCoord;

    static vk::VertexInputBindingDescription getBindingDescription() {
        vk::VertexInputBindingDescription bindingDescription(0, sizeof(Vertex), vk::VertexInputRate::eVertex);

        return bindingDescription;
    }

    static std::array<vk::VertexInputAttributeDescription, 3> getAttributeDescriptions() {
        std::array<vk::VertexInputAttributeDescription, 3> attributeDescriptions = {};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = vk::Format::eR32G32B32Sfloat;
        attributeDescriptions[0].offset = offsetof(Vertex, pos);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = vk::Format::eR32G32B32Sfloat;
        attributeDescriptions[1].offset = offsetof(Vertex, color);

        attributeDescriptions[2].binding = 0;
        attributeDescriptions[2].location = 2;
        attributeDescriptions[2].format = vk::Format::eR32G32Sfloat;
        attributeDescriptions[2].offset = offsetof(Vertex, texCoord);


        return attributeDescriptions;
    }

    bool operator==(const Vertex &other) const {
        return pos == other.pos && color == other.color && texCoord == other.texCoord;
    }
};

namespace std {
    template<>
    struct hash<Vertex> {
        size_t operator()(Vertex const &vertex) const {
            size_t res = 17;
            res = res * 31 + hash<glm::vec3>()(vertex.pos);
            res = res * 31 + hash<glm::vec3>()(vertex.normal);
            res = res * 31 + hash<glm::vec3>()(vertex.color);
            res = res * 31 + hash<glm::vec2>()(vertex.texCoord);
            return res;
        }
    };
}

class Model {
public:
    Model(const std::string &objFilePath, const std::shared_ptr<VulkanOps> &vulkanOps);

    void cleanup();
private:
    vk::Device device;

public:
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    vk::Buffer vertexBuffer;
    vk::DeviceMemory vertexBufferMemory;
    vk::Buffer indexBuffer;
    vk::DeviceMemory indexBufferMemory;


private:
    void loadModel(const std::string &objFilePath);
    void createBuffers(const std::shared_ptr<VulkanOps> &vulkanOps);
};


#endif //RTX_RAYTRACER_MODEL_H
