//
// Created by felixfifi on 04.05.20.
//

#define IMGUI_UNLIMITED_FRAME_RATE
#include <imgui/imgui.h>
#include <imgui/imgui_impl_vulkan.h>
#include <imgui/imgui_impl_sdl.h>
#include "PostProcessing.h"
#include "CommonOps.h"


PostProcessing::PostProcessing(vk::Extent2D extentOffscreen, VulkanWindow vulkanWindow) : extentOffscreen(
        extentOffscreen),
                                                                                          vulkanWindow(vulkanWindow) {
    vulkanOps = vulkanWindow.getVulkanOps();
    physicalDevice = vulkanWindow.getPhysicalDevice();
    device = vulkanWindow.getDevice();

    getSwapChainObjects();

    init();
}


void PostProcessing::init() {
    createOffscreenImage();
    createOffscreenImageView();
    createOffscreenSampler();

    createVertexBuffer();
    createIndexBuffer();

    createDecriptorSetLayout();

    createSwapChainDependant();
}

void PostProcessing::drawCallback(uint32_t imageIndex) {
    configureCommandBuffer(imageIndex); // TODO: extra command buffers and renderpass for Imgui

    // ImGUI
    ImGui::NewFrame();

    for (const auto &callback: imGuiCallbacks) {
        callback();
    }

    ImGui::Render();

    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffers[imageIndex]);

    commandBuffers[imageIndex].endRenderPass();
    commandBuffers[imageIndex].end();

}

void PostProcessing::addImGuiCallback(const fImGuiCallback &callback) {
    imGuiCallbacks.push_back(callback);
}

void PostProcessing::createVertexBuffer() {
    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer;

    vk::MemoryPropertyFlags memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;

    vulkanOps->createBufferFromData(fullscreenQuadVertices, usage, memoryProperties, vertexBuffer, vertexBufferMemory);
}

void PostProcessing::createIndexBuffer() {
    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer;

    vk::MemoryPropertyFlags memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;

    vulkanOps->createBufferFromData(fullscreenQuadInd, usage, memoryProperties, indexBuffer, indexBufferMemory);
}


void PostProcessing::recreateSwapChainCallback() {
    cleanupSwapChainDependant();

    getSwapChainObjects();

    createSwapChainDependant();
}

void PostProcessing::createSwapChainDependant() {
    createDescriptorPool();
    createDescriptorSets();
    createGraphicsPipeline();

    commandBuffers = vulkanWindow.getCommandBuffers();
    //configureCommandBuffers();
}

void PostProcessing::getSwapChainObjects() {
    swapChainFramebuffers = vulkanWindow.getSwapChainFramebuffers();
    swapChainExtent = vulkanWindow.getSwapChainExtent();
    renderPass = vulkanWindow.getRenderPass();
    commandBuffers = vulkanWindow.getCommandBuffers();
}

void PostProcessing::cleanupSwapChainDependant() {
    device.destroy(descriptorPool);

    device.destroy(graphicsPipeline);
    device.destroy(pipelineLayout);


}

void PostProcessing::cleanup() {
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    cleanupSwapChainDependant();
    device.destroy(descriptorSetLayout);

    device.free(vertexBufferMemory);
    device.destroy(vertexBuffer);

    device.free(indexBufferMemory);
    device.destroy(indexBuffer);

    device.destroy(offscreenImageSampler);
    device.destroy(offscreenImageView);
    device.free(offscreenImageMemory);
    device.destroy(offscreenImage);


}

void PostProcessing::createDecriptorSetLayout() {
    vk::DescriptorSetLayoutBinding samplerLayoutBinding(0, vk::DescriptorType::eCombinedImageSampler,
                                                        1, vk::ShaderStageFlagBits::eFragment,
                                                        nullptr);

    std::array<vk::DescriptorSetLayoutBinding, 1> bindings = {samplerLayoutBinding};
    vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());


    descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);
}

void PostProcessing::createDescriptorPool() {
    std::array<vk::DescriptorPoolSize, 1> poolSizes = {
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler,
                                   static_cast<uint32_t>(swapChainFramebuffers.size()))};

    vk::DescriptorPoolCreateInfo poolInfo({}, static_cast<uint32_t>(swapChainFramebuffers.size()),
                                          static_cast<uint32_t>(poolSizes.size()), poolSizes.data());

    descriptorPool = device.createDescriptorPool(poolInfo);
}

void PostProcessing::createDescriptorSets() {
    std::vector<vk::DescriptorSetLayout> layouts(static_cast<uint32_t>(swapChainFramebuffers.size()),
                                                 descriptorSetLayout);
    vk::DescriptorSetAllocateInfo allocInfo(descriptorPool, static_cast<uint32_t>(swapChainFramebuffers.size()),
                                            layouts.data());

    descriptorSets = device.allocateDescriptorSets(allocInfo);

    for (size_t i = 0; i < swapChainFramebuffers.size(); i++) {
        vk::DescriptorImageInfo imageInfo(offscreenImageSampler, offscreenImageView,
                                          vk::ImageLayout::eGeneral);


        std::array<vk::WriteDescriptorSet, 1> descriptorWrites = {};

        descriptorWrites[0] = vk::WriteDescriptorSet(descriptorSets[i], 0, 0, 1,
                                                     vk::DescriptorType::eCombinedImageSampler, &imageInfo,
                                                     nullptr, nullptr);

        device.updateDescriptorSets(descriptorWrites, nullptr);
    }
}

void PostProcessing::createOffscreenImage() {
    vulkanOps->createImage(extentOffscreen.width, extentOffscreen.height, offscreenImageFormat, vk::ImageTiling::eOptimal,
                           vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage,
                           vk::MemoryPropertyFlagBits::eDeviceLocal,
                           offscreenImage, offscreenImageMemory);

    vulkanOps->transitionImageLayout(offscreenImage, offscreenImageFormat, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eGeneral);
}

void PostProcessing::createOffscreenImageView() {
    offscreenImageView = vulkanOps->createImageView(offscreenImage, offscreenImageFormat,
                                                    vk::ImageAspectFlagBits::eColor);
}

void PostProcessing::createOffscreenSampler() {
    vk::SamplerCreateInfo samplerInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear,
                                      vk::SamplerAddressMode::eRepeat, vk::SamplerAddressMode::eRepeat,
                                      vk::SamplerAddressMode::eRepeat, 0.0f, false, 0, false,
                                      vk::CompareOp::eAlways, 0.0f, 0.0f, vk::BorderColor::eIntOpaqueBlack, false);
    offscreenImageSampler = device.createSampler(samplerInfo);
}

void PostProcessing::createGraphicsPipeline() {
    auto vertShaderCode = readFile("shaders/post.vert.spv");
    auto fragShaderCode = readFile("shaders/post.frag.spv");

    vk::ShaderModule vertShaderModule = vulkanOps->createShaderModule(vertShaderCode);
    vk::ShaderModule fragShaderModule = vulkanOps->createShaderModule(fragShaderCode);

    vk::PipelineShaderStageCreateInfo vertShaderStageInfo({}, vk::ShaderStageFlagBits::eVertex,
                                                          vertShaderModule, "main", {});

    vk::PipelineShaderStageCreateInfo fragShaderStageInfo({}, vk::ShaderStageFlagBits::eFragment,
                                                          fragShaderModule, "main", {});

    vk::PipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};

    auto bindingDescription = VertexTex::getBindingDescription();
    auto attributeDescriptions = VertexTex::getAttributeDescriptions();
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

    vk::GraphicsPipelineCreateInfo pipelineInfo({}, 2, shaderStages, &vertexInputInfo, &inputAssembly, nullptr,
                                                &viewportState, &rasterizer, &multisampling, nullptr,
                                                &colorBlending,
                                                nullptr, pipelineLayout, renderPass, 0);

    graphicsPipeline = device.createGraphicsPipeline(nullptr, pipelineInfo);

    device.destroy(fragShaderModule);
    device.destroy(vertShaderModule);
}

void PostProcessing::configureCommandBuffers() {
    commandBuffers = vulkanWindow.getCommandBuffers();

    for (size_t i = 0; i < commandBuffers.size(); i++) {
        configureCommandBuffer(i);
    }
}

void PostProcessing::configureCommandBuffer(size_t imageIndex) const {
    vk::CommandBufferBeginInfo beginInfo({}, nullptr);

    commandBuffers[imageIndex].reset({});

    commandBuffers[imageIndex].begin(beginInfo);


    std::array<vk::ClearValue, 2> clearValues = {};
    clearValues[0].color = vk::ClearColorValue(std::array<float, 4>({0.0f, 0.0f, 0.0f, 1.0f}));
    clearValues[1].depthStencil = vk::ClearDepthStencilValue(1.0f, 0);

    vk::RenderPassBeginInfo renderPassInfo(renderPass, swapChainFramebuffers[imageIndex],
                                           {{0, 0}, swapChainExtent},
                                           static_cast<uint32_t>(clearValues.size()), clearValues.data());


    commandBuffers[imageIndex].beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);
    commandBuffers[imageIndex].bindPipeline(vk::PipelineBindPoint::eGraphics, graphicsPipeline);

    vk::DeviceSize offset = 0;
    commandBuffers[imageIndex].bindVertexBuffers(0, vertexBuffer, offset);
    commandBuffers[imageIndex].bindIndexBuffer(indexBuffer, 0, vk::IndexType::eUint32);
    commandBuffers[imageIndex].bindDescriptorSets(vk::PipelineBindPoint::eGraphics, pipelineLayout,
                                                  0, descriptorSets[imageIndex], nullptr);

    commandBuffers[imageIndex].drawIndexed(static_cast<uint32_t>(fullscreenQuadInd.size()), 1, 0,
                                           0, 0);
}

const vk::ImageView &PostProcessing::getOffscreenImageView() const {
    return offscreenImageView;
}

const vk::Sampler &PostProcessing::getOffscreenImageSampler() const {
    return offscreenImageSampler;
}

const vk::Extent2D &PostProcessing::getExtentOffscreen() const {
    return extentOffscreen;
}


