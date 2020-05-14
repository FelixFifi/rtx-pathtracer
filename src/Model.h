//
// Created by felixfifi on 06.05.20.
//

#ifndef RTX_RAYTRACER_MODEL_H
#define RTX_RAYTRACER_MODEL_H

#include "VulkanLoader.h"
#include "VulkanOps.h"


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
    glm::vec2 texCoord;

    bool operator==(const Vertex &other) const {
        return pos == other.pos && normal == other.normal && texCoord == other.texCoord;
    }
};

enum EMatType {
    eDiffuse = 0,
    eSpecular = 1,
    eTransparent = 2
};

struct alignas(16) Material {
    alignas(16) glm::vec3 diffuse;
    alignas(16) glm::vec3 specular;
    float specularHighlight;
    float transparency;
    float refractionIndex;
    int type;
};

namespace std {
    template<>
    struct hash<Vertex> {
        size_t operator()(Vertex const &vertex) const {
            size_t res = 17;
            res = res * 31 + hash<glm::vec3>()(vertex.pos);
            res = res * 31 + hash<glm::vec3>()(vertex.normal);
            res = res * 31 + hash<glm::vec2>()(vertex.texCoord);
            return res;
        }
    };
}

class Model {
public:
    Model(const std::string &objFilePath, const std::string &materialBaseDir,
          const std::shared_ptr<VulkanOps> &vulkanOps);

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

    bool hasMaterial = false;
    Material material;



private:
    void loadModel(const std::string &objFilePath, const std::string &materialBaseDir);
    void createBuffers(const std::shared_ptr<VulkanOps> &vulkanOps);

};


#endif //RTX_RAYTRACER_MODEL_H
