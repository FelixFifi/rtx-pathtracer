//
// Created by felixfifi on 09.04.20.
//

#ifndef RTX_RAYTRACER_RAYTRACINGAPP_H
#define RTX_RAYTRACER_RAYTRACINGAPP_H


#include "VulkanOps.h"

#include <SDL.h>
#include <SDL_vulkan.h>

// #VKRay
#define ALLOC_DEDICATED

#include <nvmath/nvmath.h>
#include <nvvkpp/utilities_vkpp.hpp>
#include <nvvkpp/descriptorsets_vkpp.hpp>
#include <nvvkpp/raytraceKHR_vkpp.hpp>

#include <iostream>
#include <exception>
#include <string>
#include <vector>
#include <optional>
#include <set>
#include <chrono>

#include "CommonOps.h"
#include "PostProcessing.h"
#include "VulkanWindow.h"
#include "Model.h"
#include "CameraController.h"
#include "ModelLoader.h"

const std::string MATERIAL_BASE_DIR = "materials/";
const std::string MODEL_FLOOR = "objs/floor.obj";
const std::string MODEL_TEAPOT = "objs/teapot.obj";
const std::string MODEL_GLASS_CUBE = "objs/cube.obj";
const std::string MODEL_LIGHT_BOX = "objs/lightBox.obj";
const std::string TEXTURE_PATH = "textures/chalet.jpg";

const int RANDOM_SIZE = 2048;


static const int MAX_RECURSION = 1;

struct CameraMatrices {
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 viewInverse;
    glm::mat4 projInverse;
};

class RayTracingApp {
public:
    RayTracingApp(uint32_t width, uint32_t height);

    void run();

    void cleanup();

    struct alignas(16) RtPushConstant {
        glm::vec4 skyColor1 =  {0.3, 0.3, 0.4, 0 };
        glm::vec4 skyColor2 =  {0.2, 0.2, 0.2, 0 };
        glm::vec3 lightPosition = {20,20,20};
        float lightIntensity = 500;
        glm::vec2 noiseUVOffset;
        int lightType = 0;
        uint previousFrames = -1;
        int maxDepth = 10;
        int samplesPerPixel = 10;
    } rtPushConstants;

private:
    PostProcessing postProcessing;
    VulkanWindow vulkanWindow;

    ModelLoader modelLoader;

    bool autoRotate = false;
    bool accumulateResults = true;
    bool hasInputChanged = false;

    CameraController cameraController;

    // Vulkan
    vk::Buffer uniformBuffer;
    vk::DeviceMemory uniformBufferMemory;

    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    vk::DescriptorSet descriptorSet;

    vk::Image noiseImage;
    vk::DeviceMemory noiseImageMemory;
    vk::ImageView noiseImageView;
    vk::Sampler noiseImageSampler;

    // From window
    std::shared_ptr<VulkanOps> vulkanOps;

    vk::Device device;
    vk::PhysicalDevice physicalDevice;

    // From PostProcesing
    vk::Extent2D offscreenExtent;

    // Ray tracing
    vk::PhysicalDeviceRayTracingPropertiesKHR rtProperties;

    std::vector<vk::DescriptorSetLayoutBinding> rtDescSetLayoutBind;
    vk::DescriptorPool rtDescPool;
    vk::DescriptorSetLayout rtDescSetLayout;
    vk::DescriptorSet rtDescSet;

    std::vector<vk::RayTracingShaderGroupCreateInfoKHR> rtShaderGroups;
    vk::PipelineLayout rtPipelineLayout;
    vk::Pipeline rtPipeline;
    vk::Buffer rtSBTBuffer;
    vk::DeviceMemory rtSBTBufferMemory;

private:
    void drawCallback(uint32_t imageIndex);

    void recreateSwapchainCallback();

    void loadModels();
    void cleanupDescriptorSets();
    void recreateDescriptorSets();

    /*
    void initVulkan() {
        setupDispatchLoader();
        createInstance();

        VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);

        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();

        VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

        createCommandPool();
        VulkanOps::init(physicalDevice, device, commandPool, graphicsQueue);
        postProcessing = PostProcessing(device, swapChainExtent);

        setupDebugMessenger();

        createSwapChain();
        createImageViews();
        createRenderPass();
        createDecriptorSetLayout();
        createGraphicsPipeline();



        createDepthResources();
        createFramebuffers();
        createTextureImage();
        createTextureImageView();
        createTextureSampler();
        loadModel();
        createVertexBuffer();
        createIndexBuffer();
        createUniformBuffers();
        createDescriptorPool();
        createDescriptorSets();
        initRayTracing();
        createBottomLevelAS();
        createTopLevelAS();
        createRtDescriptorSet();
        createRtPipeline();
        createRtShaderBindingTable();
        createCommandBuffers();
        createSyncObjects();
    }


    void updateUniformBuffer(uint32_t currentImage) {
        static auto startTime = std::chrono::high_resolution_clock::now();

        auto currentTime = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

        CameraMatrices ubo = {};
        ubo.model = glm::rotate(glm::mat4(1.0f), time * glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        ubo.view = glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float) swapChainExtent.height, 0.1f,
                                    10.0f);
        ubo.proj[1][1] *= -1;

        void *data;
        device.mapMemory(uniformBuffersMemory[currentImage], 0, sizeof(ubo), {}, &data);
        memcpy(data, &ubo, sizeof(ubo));
        device.unmapMemory(uniformBuffersMemory[currentImage]);
    }





    void createRenderPass() {
        vk::AttachmentDescription colorAttachment({}, swapChainImageFormat, vk::SampleCountFlagBits::e1,
                                                  vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore,
                                                  vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
                                                  vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR);

        vk::AttachmentReference colorAttachmentRef(0, vk::ImageLayout::eColorAttachmentOptimal);

        vk::AttachmentDescription depthAttachment({}, findDepthFormat(), vk::SampleCountFlagBits::e1,
                                                  vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eDontCare,
                                                  vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
                                                  vk::ImageLayout::eUndefined,
                                                  vk::ImageLayout::eDepthStencilAttachmentOptimal);

        vk::AttachmentReference depthAttachmentRef(1, vk::ImageLayout::eDepthStencilAttachmentOptimal);

        vk::SubpassDescription subpass({}, vk::PipelineBindPoint::eGraphics,
                                       {}, {}, 1, &colorAttachmentRef,
                                       {}, &depthAttachmentRef, {}, {});

        vk::SubpassDependency dependency(VK_SUBPASS_EXTERNAL, 0, vk::PipelineStageFlagBits::eColorAttachmentOutput,
                                         vk::PipelineStageFlagBits::eColorAttachmentOutput, {},
                                         vk::AccessFlagBits::eColorAttachmentWrite, {});

        std::array<vk::AttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};

        vk::RenderPassCreateInfo renderPassInfo({}, static_cast<uint32_t>(attachments.size()), attachments.data(),
                                                1, &subpass, 1, &dependency);

        renderPass = device.createRenderPass(renderPassInfo);
    }


    void createGraphicsPipeline() {
        auto vertShaderCode = readFile("shaders/vert.spv");
        auto fragShaderCode = readFile("shaders/frag.spv");

        vk::ShaderModule vertShaderModule = VulkanOps::createShaderModule(vertShaderCode);
        vk::ShaderModule fragShaderModule = VulkanOps::createShaderModule(fragShaderCode);

        vk::PipelineShaderStageCreateInfo vertShaderStageInfo({}, vk::ShaderStageFlagBits::eVertex,
                                                              vertShaderModule, "main", {});

        vk::PipelineShaderStageCreateInfo fragShaderStageInfo({}, vk::ShaderStageFlagBits::eFragment,
                                                              fragShaderModule, "main", {});

        vk::PipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};

        auto bindingDescription = Vertex::getBindingDescription();
        auto attributeDescriptions = Vertex::getAttributeDescriptions();
        vk::PipelineVertexInputStateCreateInfo vertexInputInfo({}, 1, &bindingDescription,
                                                               static_cast<uint32_t>(attributeDescriptions.size()),
                                                               attributeDescriptions.data());

        vk::PipelineInputAssemblyStateCreateInfo inputAssembly({}, vk::PrimitiveTopology::eTriangleList, false);

        vk::Viewport viewport(0.0f, 0.0f,
                              (float) swapChainExtent.width, (float) swapChainExtent.height,
                              0.0f, 1.0f);

        vk::Rect2D scissor({0, 0}, swapChainExtent);

        vk::PipelineViewportStateCreateInfo viewportState({}, 1, &viewport, 1, &scissor);

        vk::PipelineRasterizationStateCreateInfo rasterizer({}, false, false, vk::PolygonMode::eFill,
                                                            vk::CullModeFlagBits::eBack,
                                                            vk::FrontFace::eCounterClockwise,
                                                            false, {}, {}, {},
                                                            1.0f);

        vk::PipelineMultisampleStateCreateInfo multisampling({}, vk::SampleCountFlagBits::e1, false);

        vk::PipelineColorBlendAttachmentState colorBlendAttachment(false, {}, {},
                                                                   {}, {}, {}, {},
                                                                   vk::ColorComponentFlagBits::eR |
                                                                   vk::ColorComponentFlagBits::eG |
                                                                   vk::ColorComponentFlagBits::eB |
                                                                   vk::ColorComponentFlagBits::eA);

        vk::PipelineColorBlendStateCreateInfo colorBlending({}, false, vk::LogicOp::eCopy,
                                                            1, &colorBlendAttachment, {0.0f, 0.0f, 0.0f, 0.0f});

        vk::PipelineLayoutCreateInfo pipelineLayoutInfo({}, 1, &descriptorSetLayout,
                                                        0, {});

        pipelineLayout = device.createPipelineLayout(pipelineLayoutInfo);

        vk::PipelineDepthStencilStateCreateInfo depthStencil({}, true, true, vk::CompareOp::eLess,
                                                             false, false);

        vk::GraphicsPipelineCreateInfo pipelineInfo({}, 2, shaderStages, &vertexInputInfo, &inputAssembly, nullptr,
                                                    &viewportState, &rasterizer, &multisampling, &depthStencil,
                                                    &colorBlending,
                                                    nullptr, pipelineLayout, renderPass, 0);

        graphicsPipeline = device.createGraphicsPipeline(nullptr, pipelineInfo);

        device.destroy(fragShaderModule);
        device.destroy(vertShaderModule);
    }

    void createVertexBuffer() {
        vk::DeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, vertices.data(), (size_t) bufferSize);
        device.unmapMemory(stagingBufferMemory);

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer |
                                            vk::BufferUsageFlagBits::eStorageBuffer |
                                            vk::BufferUsageFlagBits::eShaderDeviceAddress,
                                vk::MemoryPropertyFlagBits::eDeviceLocal, vertexBuffer, vertexBufferMemory);

        VulkanOps::copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }


    void createIndexBuffer() {
        vk::DeviceSize bufferSize = sizeof(indices[0]) * indices.size();

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, indices.data(), (size_t) bufferSize);
        device.unmapMemory(stagingBufferMemory);

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer |
                                            vk::BufferUsageFlagBits::eStorageBuffer,
                                vk::MemoryPropertyFlagBits::eDeviceLocal, indexBuffer, indexBufferMemory);

        VulkanOps::copyBuffer(stagingBuffer, indexBuffer, bufferSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }


    void createUniformBuffers() {
        vk::DeviceSize bufferSize = sizeof(CameraMatrices);

        uniformBuffers.resize(swapChainImages.size());
        uniformBuffersMemory.resize(swapChainImages.size());

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eUniformBuffer,
                         vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                    uniformBuffers[i], uniformBuffersMemory[i]);
        }
    }

    void createTextureImage() {
        int texWidth, texHeight, texChannels;
        stbi_uc *pixels = stbi_load(TEXTURE_PATH.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
        vk::DeviceSize imageSize = texWidth * texHeight * 4;

        if (!pixels) {
            throw std::runtime_error("failed to load texture image!");
        }

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;
        VulkanOps::createBuffer(imageSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer,
                                stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, imageSize);
        memcpy(data, pixels, static_cast<size_t>(imageSize));
        device.unmapMemory(stagingBufferMemory);

        stbi_image_free(pixels);

        VulkanOps::createImage(texWidth, texHeight, vk::Format::eR8G8B8A8Srgb, vk::ImageTiling::eOptimal,
                    vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
                               vk::MemoryPropertyFlagBits::eDeviceLocal,
                               textureImage, textureImageMemory);

        VulkanOps::transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageLayout::eUndefined,
                                         vk::ImageLayout::eTransferDstOptimal);
        VulkanOps::copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(texWidth),
                                     static_cast<uint32_t>(texHeight));
        VulkanOps::transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageLayout::eTransferDstOptimal,
                                         vk::ImageLayout::eShaderReadOnlyOptimal);


        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    void createTextureImageView() {
        textureImageView = VulkanOps::createImageView(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageAspectFlagBits::eColor);
    }

    void createTextureSampler() {
        vk::SamplerCreateInfo samplerInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear,
                                          vk::SamplerAddressMode::eRepeat, vk::SamplerAddressMode::eRepeat,
                                          vk::SamplerAddressMode::eRepeat, 0.0f, true, 16, false,
                                          vk::CompareOp::eAlways, 0.0f, 0.0f, vk::BorderColor::eIntOpaqueBlack, false);
        textureSampler = device.createSampler(samplerInfo);
    }

    vk::Format findSupportedFormat(const std::vector<vk::Format> &candidates, vk::ImageTiling tiling,
                                   const vk::FormatFeatureFlags &features) {
        for (vk::Format format : candidates) {
            vk::FormatProperties props;
            props = physicalDevice.getFormatProperties(format);

            if (tiling == vk::ImageTiling::eLinear && (props.linearTilingFeatures & features) == features) {
                return format;
            } else if (tiling == vk::ImageTiling::eOptimal && (props.optimalTilingFeatures & features) == features) {
                return format;
            }
        }

        throw std::runtime_error("failed to find supported format!");
    }

    void loadModel() {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, MODEL_PATH.c_str())) {
            throw std::runtime_error(warn + err);
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

                vertex.texCoord = {
                        attrib.texcoords[2 * index.texcoord_index + 0],
                        1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                };

                vertex.color = {1.0f, 1.0f, 1.0f};

                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }

                indices.push_back(uniqueVertices[vertex]);
            }
        }
    }




*/
    void initRayTracing();

    nvvkpp::RaytracingBuilderKHR::Blas modelToBlas(const std::unique_ptr<Model> &model);

    void createBottomLevelAS();

    void createTopLevelAS();

    void createRtDescriptorSet();

    void updateRtDescriptorSet(uint32_t currentImage);

    void createRtPipeline();

    void createRtShaderBindingTable();

    void raytrace(const vk::CommandBuffer &cmdBuf);

    void createUniformBuffers();

    void createDecriptorSetLayout();
    void createDescriptorPool();
    void createDescriptorSets();

    void updateUniformBuffer(uint32_t currentImage);

    void imGuiWindowSetup();

    void createNoiseTexture();

    void sceneSwitcher(int num);
};

#endif //RTX_RAYTRACER_RAYTRACINGAPP_H
