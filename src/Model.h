//
// Created by felixfifi on 06.05.20.
//

#ifndef RTX_RAYTRACER_MODEL_H
#define RTX_RAYTRACER_MODEL_H

#include "VulkanLoader.h"
#include "VulkanOps.h"
#include "Shapes.h"


#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

#include <string>


struct Vertex {
    alignas(16) glm::vec3 pos;
    alignas(16) glm::vec3 normal;
    alignas(8) glm::vec2 texCoord;
    int materialIndex;

    bool operator==(const Vertex &other) const {
        return pos == other.pos && normal == other.normal && texCoord == other.texCoord && materialIndex == other.materialIndex;
    }
};

namespace std {
    template<>
    struct hash<Vertex> {
        size_t operator()(Vertex const &vertex) const {
            size_t res = 17;
            res = res * 31 + hash<glm::vec3>()(vertex.pos);
            res = res * 31 + hash<glm::vec3>()(vertex.normal);
            res = res * 31 + hash<glm::vec2>()(vertex.texCoord);
            res = res * 31 + hash<int>()(vertex.materialIndex);
            return res;
        }
    };
}

class Model {
public:
    Model(std::vector<Vertex> vertices, std::vector<uint32_t> indices,
          const std::shared_ptr<VulkanOps> &vulkanOps);

    float getFaceArea(int iFace, glm::mat4 transform = glm::mat4(1.0f));

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

    Aabb getAabb(glm::mat4 transform);
private:
    void createBuffers(const std::shared_ptr<VulkanOps> &vulkanOps);
};


#endif //RTX_RAYTRACER_MODEL_H
