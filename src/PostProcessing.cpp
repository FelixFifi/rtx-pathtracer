//
// Created by felixfifi on 04.05.20.
//

#include "PostProcessing.h"
#include "CommonOps.h"


PostProcessing::PostProcessing(vk::Extent2D extentOffscreen, VulkanWindow vulkanWindow) : extentOffscreen(extentOffscreen),
                                                                                          vulkanWindow(vulkanWindow) {
    vulkanOps = vulkanWindow.getVulkanOps();

    vk::Image image;
    vk::DeviceMemory memory;

    vulkanOps.createImage(512, 256, vk::Format::eR8G8B8A8Srgb, vk::ImageTiling::eOptimal,
                          vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
                          vk::MemoryPropertyFlagBits::eDeviceLocal,
                          image, memory);

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
    createDescriptorPool();
    createDescriptorSets();
}

void PostProcessing::drawCallback(uint32_t imageIndex) {

}


void PostProcessing::createVertexBuffer() {
    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer;

    vk::MemoryPropertyFlags memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;

    vulkanOps.createBufferFromData(fullscreenQuadVertices, usage, memoryProperties, vertexBuffer, vertexBufferMemory);
}

void PostProcessing::createIndexBuffer() {
    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer;

    vk::MemoryPropertyFlags memoryProperties = vk::MemoryPropertyFlagBits::eDeviceLocal;

    vulkanOps.createBufferFromData(fullscreenQuadInd, usage, memoryProperties, indexBuffer, indexBufferMemory);
}



void PostProcessing::recreateSwapChainCallback() {
    cleanupSwapChainDependant();

    getSwapChainObjects();

    createSwapChainDependant();
}

void PostProcessing::createSwapChainDependant() {
    createDescriptorPool();
    createDescriptorSets();
    configureCommandBuffers();

    createGraphicsPipeline();
}

void PostProcessing::getSwapChainObjects() {
    swapChainFramebuffers = vulkanWindow.getSwapChainFramebuffers();
    swapChainExtent = vulkanWindow.getSwapChainExtent();
    renderPass = vulkanWindow.getRenderPass();
    commandBuffers = vulkanWindow.getCommandBuffers();
}

void PostProcessing::cleanupSwapChainDependant() {
    device.destroy(graphicsPipeline);
    device.destroy(pipelineLayout);

    device.freeDescriptorSets(descriptorPool, descriptorSets);
    device.destroy(descriptorPool);

}

void PostProcessing::cleanup() {
    cleanupSwapChainDependant();
    device.destroy(descriptorSetLayout);


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

    vk::DescriptorPoolCreateInfo poolInfo({}, static_cast<uint32_t>(swapChainFramebuffers.size()), static_cast<uint32_t>(poolSizes.size()), poolSizes.data());

    descriptorPool = device.createDescriptorPool(poolInfo);
}

void PostProcessing::createDescriptorSets() {
    std::vector<vk::DescriptorSetLayout> layouts(static_cast<uint32_t>(swapChainFramebuffers.size()), descriptorSetLayout);
    vk::DescriptorSetAllocateInfo allocInfo(descriptorPool, static_cast<uint32_t>(swapChainFramebuffers.size()),
                                            layouts.data());

    descriptorSets = device.allocateDescriptorSets(allocInfo);

    for (size_t i = 0; i < swapChainFramebuffers.size(); i++) {
        vk::DescriptorImageInfo imageInfo(offscreenImageSampler, offscreenImageView,
                                          vk::ImageLayout::eGeneral);


        std::array<vk::WriteDescriptorSet, 1> descriptorWrites = {};

        descriptorWrites[1] = vk::WriteDescriptorSet(descriptorSets[i], 0, 0, 1,
                                                     vk::DescriptorType::eCombinedImageSampler, &imageInfo,
                                                     nullptr, nullptr);

        device.updateDescriptorSets(descriptorWrites, nullptr);
    }
}


void PostProcessing::createOffscreenImage() {
    vulkanOps.createImage(512, 256, vk::Format::eR8G8B8A8Srgb, vk::ImageTiling::eOptimal,
                           vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
                           vk::MemoryPropertyFlagBits::eDeviceLocal,
                           offscreenImage, offscreenImageMemory);

    vulkanOps.createImage(extentOffscreen.width, extentOffscreen.height, vk::Format::eR32G32B32A32Sfloat, vk::ImageTiling::eOptimal,
                           vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eColorAttachment,
                           vk::MemoryPropertyFlagBits::eDeviceLocal,
                           offscreenImage, offscreenImageMemory);

    vulkanOps.transitionImageLayout(offscreenImage, vk::Format::eR32G32B32A32Sfloat, vk::ImageLayout::eUndefined,
                                     vk::ImageLayout::eGeneral);
}

void PostProcessing::createOffscreenImageView() {
    offscreenImageView = vulkanOps.createImageView(offscreenImage, vk::Format::eR32G32B32A32Sfloat, vk::ImageAspectFlagBits::eColor);
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

    vk::ShaderModule vertShaderModule = vulkanOps.createShaderModule(vertShaderCode);
    vk::ShaderModule fragShaderModule = vulkanOps.createShaderModule(fragShaderCode);

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
        vk::CommandBufferBeginInfo beginInfo({}, nullptr);

        commandBuffers[i].begin(beginInfo);


        std::array<vk::ClearValue, 2> clearValues = {};
        clearValues[0].color = vk::ClearColorValue(std::array<float, 4>({0.0f, 0.0f, 0.0f, 1.0f}));
        clearValues[1].depthStencil = vk::ClearDepthStencilValue(1.0f, 0);

        vk::RenderPassBeginInfo renderPassInfo(renderPass, swapChainFramebuffers[i],
                                               {{0, 0}, swapChainExtent},
                                               static_cast<uint32_t>(clearValues.size()), clearValues.data());


        commandBuffers[i].beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);
        commandBuffers[i].bindPipeline(vk::PipelineBindPoint::eGraphics, graphicsPipeline);

        vk::DeviceSize offset = 0;
        commandBuffers[i].bindVertexBuffers(0, vertexBuffer, offset);
        commandBuffers[i].bindIndexBuffer(indexBuffer, 0, vk::IndexType::eUint32);
        commandBuffers[i].bindDescriptorSets(vk::PipelineBindPoint::eGraphics, pipelineLayout,
                                             0, descriptorSets[i], nullptr);

        commandBuffers[i].drawIndexed(static_cast<uint32_t>(fullscreenQuadInd.size()), 1, 0,
                                      0, 0);

        commandBuffers[i].endRenderPass();
        commandBuffers[i].end();
    }
}

const vk::ImageView &PostProcessing::getOffscreenImageView() const {
    return offscreenImageView;
}

const vk::Sampler &PostProcessing::getOffscreenImageSampler() const {
    return offscreenImageSampler;
}


