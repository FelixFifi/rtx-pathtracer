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

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/hash.hpp>


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


const std::string MODEL_PATH = "models/chalet.obj";
const std::string TEXTURE_PATH = "textures/chalet.jpg";



struct Vertex {
    glm::vec3 pos;
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
            return ((hash<glm::vec3>()(vertex.pos) ^
                     (hash<glm::vec3>()(vertex.color) << 1)) >> 1) ^
                   (hash<glm::vec2>()(vertex.texCoord) << 1);
        }
    };
}

struct UniformBufferObject {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
};

class RayTracingApp {
public:
    RayTracingApp(uint32_t width, uint32_t height);

    void run();
    void cleanup();
private:
    PostProcessing postProcessing;
    VulkanWindow vulkanWindow;


    void drawCallback(uint32_t imageIndex);
    void recreateSwapchainCallback();


/*

    vk::Instance instance;
    vk::PhysicalDevice physicalDevice;
    vk::Device device;

    vk::Queue graphicsQueue;
    vk::Queue presentQueue;

    vk::RenderPass renderPass;
    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    std::vector<vk::DescriptorSet> descriptorSets;
    vk::PipelineLayout pipelineLayout;
    vk::Pipeline graphicsPipeline;

    vk::CommandPool commandPool;
    std::vector<vk::CommandBuffer> commandBuffers;

    std::vector<vk::Semaphore> imageAvailableSemaphores;
    std::vector<vk::Semaphore> renderFinishedSemaphores;
    std::vector<vk::Fence> inFlightFences;
    std::vector<vk::Fence> imagesInFlight;
    size_t currentFrame = 0;

    bool framebufferResized = false;

    vk::Buffer vertexBuffer;
    vk::DeviceMemory vertexBufferMemory;
    vk::Buffer indexBuffer;
    vk::DeviceMemory indexBufferMemory;

    std::vector<vk::Buffer> uniformBuffers;
    std::vector<vk::DeviceMemory> uniformBuffersMemory;

    vk::Image textureImage;
    vk::DeviceMemory textureImageMemory;
    vk::ImageView textureImageView;
    vk::Sampler textureSampler;

    vk::Image depthImage;
    vk::DeviceMemory depthImageMemory;
    vk::ImageView depthImageView;

    // Ray tracing
    vk::PhysicalDeviceRayTracingPropertiesKHR rtProperties;
    nvvkpp::RaytracingBuilderKHR rtBuilder;

    std::vector<vk::DescriptorSetLayoutBinding> rtDescSetLayoutBind;
    vk::DescriptorPool rtDescPool;
    vk::DescriptorSetLayout rtDescSetLayout;
    vk::DescriptorSet rtDescSet;

    std::vector<vk::RayTracingShaderGroupCreateInfoKHR> rtShaderGroups;
    vk::PipelineLayout rtPipelineLayout;
    vk::Pipeline rtPipeline;
    vk::Buffer rtSBTBuffer;
    vk::DeviceMemory rtSBTBufferMemory;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    struct RtPushConstant {
        nvmath::vec4f clearColor;
        nvmath::vec3f lightPosition;
        float lightIntensity;
        int lightType;
    } rtPushConstants;
*/


/*
    void cleanup() {
        device.destroy(rtSBTBuffer);
        device.free(rtSBTBufferMemory);
        device.destroy(rtPipeline);
        device.destroy(rtPipelineLayout);

        rtBuilder.destroy();
        device.destroy(rtDescPool);
        device.destroy(rtDescSetLayout);

    }


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

        UniformBufferObject ubo = {};
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
        vk::DeviceSize bufferSize = sizeof(UniformBufferObject);

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

    void initRayTracing() {
        auto properties = physicalDevice.getProperties2<vk::PhysicalDeviceProperties2, vk::PhysicalDeviceRayTracingPropertiesKHR>();
        rtProperties = properties.get<vk::PhysicalDeviceRayTracingPropertiesKHR>();

        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);
        rtBuilder.setup(device, physicalDevice, queueFamilyIndices.graphicsFamily.value());
    }

    nvvkpp::RaytracingBuilderKHR::Blas toBlas() {
        // Setting up the creation info of acceleration structure
        vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
        asCreate.setGeometryType(vk::GeometryTypeKHR::eTriangles);
        asCreate.setIndexType(vk::IndexType::eUint32);
        asCreate.setVertexFormat(vk::Format::eR32G32B32Sfloat);
        asCreate.setMaxPrimitiveCount(indices.size() / 3);  // Nb triangles
        asCreate.setMaxVertexCount(vertices.size());
        asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

        // Building part
        vk::BufferDeviceAddressInfo infoVB = vk::BufferDeviceAddressInfo(vertexBuffer);
        vk::BufferDeviceAddressInfo infoIB = vk::BufferDeviceAddressInfo(indexBuffer);

        std::cout << VULKAN_HPP_DEFAULT_DISPATCHER.vkGetBufferDeviceAddressKHR << std::endl;

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

    void createBottomLevelAS() {
        std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;

        allBlas.push_back(toBlas());

        std::cout << VULKAN_HPP_DEFAULT_DISPATCHER.vkGetBufferMemoryRequirements2KHR << VULKAN_HPP_DEFAULT_DISPATCHER.vkGetBufferMemoryRequirements2 << std::endl;

        rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
    }

    void createTopLevelAS() {
        std::vector<nvvkpp::RaytracingBuilderKHR::Instance> tlas;
        tlas.reserve(1);

        for (int i = 0; i < static_cast<int>(1); ++i) {
            nvvkpp::RaytracingBuilderKHR::Instance rayInst;
            rayInst.transform = {};
            rayInst.instanceId = i;
            rayInst.blasId = 0;
            rayInst.hitGroupId = 0; // Same hit group for all
            rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleCullDisable; // TODO
            tlas.emplace_back(rayInst);
        }

        rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
    }

    void createRtDescriptorSet() {
        using vkDT = vk::DescriptorType;
        using vkSS = vk::ShaderStageFlagBits;
        using vkDSLB = vk::DescriptorSetLayoutBinding;

        rtDescSetLayoutBind.emplace_back(vkDSLB(0, vkDT::eAccelerationStructureKHR, 1, vkSS::eRaygenKHR)); // TLAS
        rtDescSetLayoutBind.emplace_back(vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR)); // Output image

        rtDescPool = nvvkpp::util::createDescriptorPool(device, rtDescSetLayoutBind);
        rtDescSetLayout = nvvkpp::util::createDescriptorSetLayout(device, rtDescSetLayoutBind);
        rtDescSet = device.allocateDescriptorSets({rtDescPool, 1, &rtDescSetLayout})[0];

        vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
        descASInfo.setAccelerationStructureCount(1);
        descASInfo.setPAccelerationStructures(&rtBuilder.getAccelerationStructure());
        vk::DescriptorImageInfo imageInfo{
                {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral}; // TODO: Update each frame

        std::vector<vk::WriteDescriptorSet> writes;
        writes.emplace_back(
                nvvkpp::util::createWrite(rtDescSet, rtDescSetLayoutBind[0], &descASInfo));
        writes.emplace_back(nvvkpp::util::createWrite(rtDescSet, rtDescSetLayoutBind[1], &imageInfo));
        device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }

    void updateRtDescriptorSet(uint32_t currentImage) {
        using vkDT = vk::DescriptorType;

        // (1) Output buffer
        vk::DescriptorImageInfo imageInfo{
                {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral};
        vk::WriteDescriptorSet wds{rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
        device.updateDescriptorSets(wds, nullptr);
    }

    void createRtPipeline() {
        auto raygenCode = readFile("shaders/raytrace.rgen.spv");
        auto missCode = readFile("shaders/raytrace.rmiss.spv");
        auto chitCode = readFile("shaders/raytrace.rchit.spv");

        vk::ShaderModule raygenShaderModule = VulkanOps::createShaderModule(raygenCode);
        vk::ShaderModule missShaderModule = VulkanOps::createShaderModule(missCode);
        vk::ShaderModule chitShaderModule = VulkanOps::createShaderModule(chitCode);

        std::vector<vk::PipelineShaderStageCreateInfo> stages;

        // Raygen
        vk::RayTracingShaderGroupCreateInfoKHR rg{vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};

        stages.push_back({{}, vk::ShaderStageFlagBits::eRaygenKHR, raygenShaderModule, "main"});
        rg.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
        rtShaderGroups.push_back(rg);
        // Miss
        vk::RayTracingShaderGroupCreateInfoKHR mg{vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
        stages.push_back({{}, vk::ShaderStageFlagBits::eMissKHR, missShaderModule, "main"});
        mg.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
        rtShaderGroups.push_back(mg);

        // Hit Group - Closest Hit + AnyHit

        vk::RayTracingShaderGroupCreateInfoKHR hg{vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                                  VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
        stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitShaderModule, "main"});
        hg.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));
        rtShaderGroups.push_back(hg);

        vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;

        // Push constant: we want to be able to update constants used by the shaders
        vk::PushConstantRange pushConstant{vk::ShaderStageFlagBits::eRaygenKHR
                                           | vk::ShaderStageFlagBits::eClosestHitKHR
                                           | vk::ShaderStageFlagBits::eMissKHR,
                                           0, sizeof(RtPushConstant)};
        pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
        pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstant);

        // Descriptor sets: one specific to ray tracing, and one shared with the rasterization pipeline
        std::vector<vk::DescriptorSetLayout> rtDescSetLayouts = {rtDescSetLayout};
        pipelineLayoutCreateInfo.setSetLayoutCount(static_cast<uint32_t>(rtDescSetLayouts.size()));
        pipelineLayoutCreateInfo.setPSetLayouts(rtDescSetLayouts.data());

        rtPipelineLayout = device.createPipelineLayout(pipelineLayoutCreateInfo);

        // Assemble the shader stages and recursion depth info into the ray tracing pipeline
        vk::RayTracingPipelineCreateInfoKHR rayPipelineInfo;
        rayPipelineInfo.setStageCount(static_cast<uint32_t>(stages.size()));  // Stages are shaders
        rayPipelineInfo.setPStages(stages.data());

        rayPipelineInfo.setGroupCount(
                static_cast<uint32_t>(rtShaderGroups.size()));  // 1-raygen, n-miss, n-(hit[+anyhit+intersect])
        rayPipelineInfo.setPGroups(rtShaderGroups.data());

        rayPipelineInfo.setMaxRecursionDepth(1);  // Ray depth
        rayPipelineInfo.setLayout(rtPipelineLayout);
        rtPipeline = device.createRayTracingPipelineKHR({}, rayPipelineInfo).value;

        device.destroy(raygenShaderModule);
        device.destroy(missShaderModule);
        device.destroy(chitShaderModule);
    }

    void createRtShaderBindingTable() {
        auto groupCount = static_cast<uint64_t>(rtShaderGroups.size()); // 3 shaders: raygen, miss, chit
        uint64_t groupHandleSize = static_cast<uint64_t>(rtProperties.shaderGroupHandleSize); // Size of a program identifier

        // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
        uint64_t sbtSize = groupCount * groupHandleSize;

        std::vector<uint8_t> shaderHandleStorage(sbtSize);
        device.getRayTracingShaderGroupHandlesKHR(rtPipeline, 0, groupCount, sbtSize,
                                                  shaderHandleStorage.data());
        // Write the handles in the SBT
        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        VulkanOps::createBuffer(sbtSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, sbtSize);

        memcpy(data, shaderHandleStorage.data(), (size_t) sbtSize);

        device.unmapMemory(stagingBufferMemory);

        VulkanOps::createBuffer(sbtSize, vk::BufferUsageFlagBits::eRayTracingKHR, vk::MemoryPropertyFlagBits::eDeviceLocal,
                                rtSBTBuffer, rtSBTBufferMemory);
        VulkanOps::copyBuffer(stagingBuffer, vertexBuffer, sbtSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    void raytrace(const vk::CommandBuffer &cmdBuf, const nvmath::vec4f &clearColor) {
        // Initializing push constant values
        rtPushConstants.clearColor = clearColor;
        rtPushConstants.lightPosition = nvmath::vec3f(20, 20, 20);
        rtPushConstants.lightIntensity = 20.0f;
        rtPushConstants.lightType = 0;

        cmdBuf.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, rtPipeline);
        cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, rtPipelineLayout, 0,
                                  {rtDescSet}, {});
        cmdBuf.pushConstants<RtPushConstant>(rtPipelineLayout,
                                             vk::ShaderStageFlagBits::eRaygenKHR
                                             | vk::ShaderStageFlagBits::eClosestHitKHR
                                             | vk::ShaderStageFlagBits::eMissKHR,
                                             0, rtPushConstants);

        vk::DeviceSize progSize = rtProperties.shaderGroupHandleSize;  // Size of a program identifier
        vk::DeviceSize rayGenOffset = 0u * progSize;  // Start at the beginning of sbtBuffer
        vk::DeviceSize missOffset = 1u * progSize;  // Jump over raygen
        vk::DeviceSize missStride = progSize;
        vk::DeviceSize hitGroupOffset = 2u * progSize;  // Jump over the previous shaders
        vk::DeviceSize hitGroupStride = progSize;

        vk::DeviceSize sbtSize = progSize * (vk::DeviceSize) rtShaderGroups.size();

        const vk::StridedBufferRegionKHR raygenShaderBindingTable = {rtSBTBuffer, rayGenOffset,
                                                                     progSize, sbtSize};
        const vk::StridedBufferRegionKHR missShaderBindingTable = {rtSBTBuffer, missOffset,
                                                                   progSize, sbtSize};
        const vk::StridedBufferRegionKHR hitShaderBindingTable = {rtSBTBuffer, hitGroupOffset,
                                                                  progSize, sbtSize};
        const vk::StridedBufferRegionKHR callableShaderBindingTable;

        cmdBuf.traceRaysKHR(&raygenShaderBindingTable, &missShaderBindingTable, &hitShaderBindingTable,
                            &callableShaderBindingTable,      //
                            swapChainExtent.width, swapChainExtent.height, 1);  //
    }


*/

};

#endif //RTX_RAYTRACER_RAYTRACINGAPP_H
