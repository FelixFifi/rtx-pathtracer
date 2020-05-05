//
// Created by felixfifi on 03.05.20.
//

#ifndef RTX_RAYTRACER_POSTPROCESSING_H
#define RTX_RAYTRACER_POSTPROCESSING_H


#include "VulkanOps.h"
#include "VulkanWindow.h"
#include <glm/glm.hpp>


struct VertexTex {
    glm::vec3 pos;
    glm::vec2 texCoord;

    static vk::VertexInputBindingDescription getBindingDescription() {
        vk::VertexInputBindingDescription bindingDescription(0, sizeof(VertexTex), vk::VertexInputRate::eVertex);

        return bindingDescription;
    }

    static std::array<vk::VertexInputAttributeDescription, 3> getAttributeDescriptions() {
        std::array<vk::VertexInputAttributeDescription, 3> attributeDescriptions = {};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = vk::Format::eR32G32B32Sfloat;
        attributeDescriptions[0].offset = offsetof(VertexTex, pos);

        attributeDescriptions[2].binding = 0;
        attributeDescriptions[2].location = 1;
        attributeDescriptions[2].format = vk::Format::eR32G32Sfloat;
        attributeDescriptions[2].offset = offsetof(VertexTex, texCoord);

        return attributeDescriptions;
    }

    bool operator==(const VertexTex &other) const {
        return pos == other.pos && texCoord == other.texCoord;
    }
};

const std::vector<VertexTex> fullscreenQuadVertices = {
        {{-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f}},
        {{-1.0f, 1.0f,  0.0f}, {0.0f, 1.0f}},
        {{1.0f,  -1.0f, 0.0f}, {1.0f, 0.0f}},
        {{1.0f,  1.0f,  0.0f}, {1.0f, 1.0f}}
};

const std::vector<uint32_t> fullscreenQuadInd = {
        0, 1, 2,
        0, 3, 1
};

class PostProcessing {
private:
    VulkanWindow vulkanWindow;


    vk::Extent2D extentOffscreen;

    // From window
    VulkanOps vulkanOps;
    vk::PhysicalDevice physicalDevice;
    vk::Device device;

    vk::RenderPass renderPass;

    vk::Extent2D swapChainExtent;

    std::vector<vk::CommandBuffer> commandBuffers;
    std::vector<vk::Framebuffer> swapChainFramebuffers;


    //

    vk::Buffer vertexBuffer;
    vk::DeviceMemory vertexBufferMemory;
    vk::Buffer indexBuffer;
    vk::DeviceMemory indexBufferMemory;

    vk::Image offscreenImage;
    vk::DeviceMemory offscreenImageMemory;
    vk::ImageView offscreenImageView;
    vk::Sampler offscreenImageSampler;

    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    std::vector<vk::DescriptorSet> descriptorSets;

    vk::PipelineLayout pipelineLayout;
    vk::Pipeline graphicsPipeline;


public:
    PostProcessing(vk::Extent2D extentOffscreen, VulkanWindow vulkanWindow);

    PostProcessing() = default;

    const vk::ImageView &getOffscreenImageView() const;

    const vk::Sampler &getOffscreenImageSampler() const;


    void recreateSwapChainCallback();

    void drawCallback(uint32_t imageIndex);

private:
    void init();

    void createVertexBuffer();

    void createIndexBuffer();

    void createDecriptorSetLayout();

    void createDescriptorPool();


    void createDescriptorSets();

    void createOffscreenImage();

    void createOffscreenImageView();

    void createOffscreenSampler();

    void createGraphicsPipeline();

    void configureCommandBuffers();

    void cleanupSwapChainDependant();

    void getSwapChainObjects();

    void cleanup();

    void createSwapChainDependant();
};

#endif //RTX_RAYTRACER_POSTPROCESSING_H
