//
// Created by felixfifi on 03.05.20.
//

#include <glm/vec4.hpp>
#include "VulkanOps.h"

void VulkanOps::createBuffer(uint64_t size, const vk::BufferUsageFlags &usage,
                             const vk::MemoryPropertyFlags &properties, vk::Buffer &buffer,
                             vk::DeviceMemory &bufferMemory) {
    vk::BufferCreateInfo bufferInfo({}, size, usage, vk::SharingMode::eExclusive,
                                    0, nullptr);

    buffer = device.createBuffer(bufferInfo);

    vk::MemoryRequirements memoryRequirements = device.getBufferMemoryRequirements(buffer);

    vk::MemoryAllocateInfo allocateInfo(memoryRequirements.size,
                                        findMemoryType(memoryRequirements.memoryTypeBits, properties));
    vk::MemoryAllocateFlagsInfo allocateFlagsInfo;
    allocateFlagsInfo.flags = vk::MemoryAllocateFlagBits::eDeviceAddress;
    allocateInfo.pNext = &allocateFlagsInfo;

    bufferMemory = device.allocateMemory(allocateInfo);

    device.bindBufferMemory(buffer, bufferMemory, 0);
}

void VulkanOps::copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, uint64_t size, const char *label) {
    vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

    if (label) {
        vk::DebugUtilsLabelEXT vkLabel;
        vkLabel.pLabelName = label;
        commandBuffer.beginDebugUtilsLabelEXT(vkLabel);
    }

    vk::BufferCopy copyRegion(0, 0, size);
    commandBuffer.copyBuffer(srcBuffer, dstBuffer, copyRegion);

    if (label) {
        commandBuffer.endDebugUtilsLabelEXT();
    }

    endSingleTimeCommands(commandBuffer, label);
}

void VulkanOps::createImage(uint32_t width, uint32_t height, vk::Format format, vk::ImageTiling tiling,
                            const vk::ImageUsageFlags &usage,
                            const vk::MemoryPropertyFlags &properties, vk::Image &image,
                            vk::DeviceMemory &imageMemory) {
    vk::ImageCreateInfo imageInfo({}, vk::ImageType::e2D, format, {width, height, 1}, 1, 1,
                                  vk::SampleCountFlagBits::e1, tiling, usage, vk::SharingMode::eExclusive, 0,
                                  nullptr, vk::ImageLayout::eUndefined);

    image = device.createImage(imageInfo);

    vk::MemoryRequirements memRequirements;
    memRequirements = device.getImageMemoryRequirements(image);

    vk::MemoryAllocateInfo allocInfo(memRequirements.size,
                                     findMemoryType(memRequirements.memoryTypeBits, properties));

    imageMemory = device.allocateMemory(allocInfo);
    device.bindImageMemory(image, imageMemory, 0);
}

vk::ImageView
VulkanOps::createImageView(vk::Image image, vk::Format format, const vk::ImageAspectFlags &aspectFlags) {
    vk::ImageSubresourceRange subresourceRange(aspectFlags, 0, 1, 0, 1);

    vk::ImageViewCreateInfo viewInfo({}, image, vk::ImageViewType::e2D, format, {}, subresourceRange);

    return device.createImageView(viewInfo);
}

void
VulkanOps::transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout,
                                 vk::ImageLayout newLayout) {
    vk::CommandBuffer commandBuffer = VulkanOps::beginSingleTimeCommands();

    vk::ImageAspectFlags aspectMask;

    if (newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
        aspectMask = vk::ImageAspectFlagBits::eDepth;

        if (hasStencilComponent(format)) {
            aspectMask |= vk::ImageAspectFlagBits::eStencil;
        }
    } else {
        aspectMask = vk::ImageAspectFlagBits::eColor;
    }

    vk::ImageMemoryBarrier barrier({}, {}, oldLayout, newLayout, {}, {}, image, {aspectMask, 0, 1, 0, 1});

    vk::PipelineStageFlags sourceStage;
    vk::PipelineStageFlags destinationStage;

    vk::AccessFlags srcAccessMask, dstAccessMask;
    if (oldLayout == vk::ImageLayout::eUndefined && newLayout == vk::ImageLayout::eTransferDstOptimal) {
        srcAccessMask = {};
        dstAccessMask = vk::AccessFlagBits::eTransferWrite;

        sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
        destinationStage = vk::PipelineStageFlagBits::eTransfer;
    } else if (oldLayout == vk::ImageLayout::eTransferDstOptimal &&
               newLayout == vk::ImageLayout::eShaderReadOnlyOptimal) {
        srcAccessMask = vk::AccessFlagBits::eTransferWrite;
        dstAccessMask = vk::AccessFlagBits::eShaderRead;

        sourceStage = vk::PipelineStageFlagBits::eTransfer;
        destinationStage = vk::PipelineStageFlagBits::eFragmentShader;
    } else if (oldLayout == vk::ImageLayout::eUndefined &&
               newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
        srcAccessMask = {};
        dstAccessMask =
                vk::AccessFlagBits::eDepthStencilAttachmentRead | vk::AccessFlagBits::eDepthStencilAttachmentWrite;

        sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
        destinationStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
    } else if (oldLayout == vk::ImageLayout::eUndefined &&
               newLayout == vk::ImageLayout::eGeneral) {
        srcAccessMask = {};
        dstAccessMask =
                vk::AccessFlagBits::eMemoryWrite;

        sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
        destinationStage = vk::PipelineStageFlagBits::eTopOfPipe; // TODO: optimize
    } else {
        throw std::invalid_argument("unsupported layout transition!");
    }

    commandBuffer.pipelineBarrier(sourceStage, destinationStage, {}, 0, nullptr, 0, nullptr, 1, &barrier);

    VulkanOps::endSingleTimeCommands(commandBuffer, nullptr);
}

bool VulkanOps::hasStencilComponent(vk::Format format) {
    return format == vk::Format::eD32SfloatS8Uint || format == vk::Format::eD24UnormS8Uint;
}

void
VulkanOps::createNoiseTextureFromData(const std::vector<glm::vec4> &noise, int texWidth, int texHeight,
                                      vk::Image &outImage,
                                      vk::DeviceMemory &outMemory, vk::ImageView &outImageView,
                                      vk::Sampler &outSampler) {
    assert(noise.size() == texWidth * texHeight);
    vk::Format format = vk::Format::eR32G32B32A32Sfloat;

    createImageFromData(noise, texWidth, texHeight, format, outImage, outMemory, outImageView);


    vk::SamplerCreateInfo samplerInfo({}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
                                      vk::SamplerAddressMode::eMirroredRepeat, vk::SamplerAddressMode::eMirroredRepeat,
                                      vk::SamplerAddressMode::eRepeat, 0.0f, false, 0, false,
                                      vk::CompareOp::eAlways, 0.0f, 0.0f, vk::BorderColor::eIntOpaqueBlack, false);
    outSampler = device.createSampler(samplerInfo);
}

void VulkanOps::copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height) {
    vk::CommandBuffer commandBuffer = VulkanOps::beginSingleTimeCommands();

    vk::BufferImageCopy region(0, 0, 0,
                               {vk::ImageAspectFlagBits::eColor, 0, 0, 1},
                               {0, 0, 0}, {width, height, 1});

    commandBuffer.copyBufferToImage(buffer, image, vk::ImageLayout::eTransferDstOptimal, region);

    VulkanOps::endSingleTimeCommands(commandBuffer, nullptr);
}

vk::CommandBuffer VulkanOps::beginSingleTimeCommands() {
    vk::CommandBufferAllocateInfo allocInfo(commandPool, vk::CommandBufferLevel::ePrimary, 1);

    vk::CommandBuffer commandBuffer;
    commandBuffer = device.allocateCommandBuffers(allocInfo)[0];

    vk::CommandBufferBeginInfo beginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit, {});
    commandBuffer.begin(beginInfo);

    return commandBuffer;
}

void VulkanOps::endSingleTimeCommands(vk::CommandBuffer commandBuffer, const char *label) {
    commandBuffer.end();

    if (label) {
        vk::DebugUtilsLabelEXT queueLabel;
        queueLabel.pLabelName = label;

        graphicsQueue.beginDebugUtilsLabelEXT(queueLabel);
    }

    vk::SubmitInfo submitInfo(0, nullptr, {},
                              1, &commandBuffer, 0, nullptr);

    graphicsQueue.submit(submitInfo, nullptr);
    graphicsQueue.waitIdle();

    if (label) {
        graphicsQueue.endDebugUtilsLabelEXT();
    }

    device.freeCommandBuffers(commandPool, commandBuffer);
}

uint32_t VulkanOps::findMemoryType(uint32_t typeFilter, const vk::MemoryPropertyFlags &properties) {
    vk::PhysicalDeviceMemoryProperties memProperties;
    memProperties = physicalDevice.getMemoryProperties();

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }

    throw std::runtime_error("failed to find suitable memory type!");
}

vk::ShaderModule VulkanOps::createShaderModule(const std::vector<char> &code) {
    vk::ShaderModuleCreateInfo createInfo({}, code.size(), reinterpret_cast<const uint32_t *>(code.data()));

    return device.createShaderModule(createInfo);
}

VulkanOps::VulkanOps(const vk::SurfaceKHR &surface, const vk::PhysicalDevice &physicalDevice, const vk::Device &device,
                     const vk::CommandPool &commandPool, const vk::Queue &graphicsQueue)
        : surface(surface), physicalDevice(physicalDevice), device(device), commandPool(commandPool),
          graphicsQueue(graphicsQueue) {}

const vk::Device &VulkanOps::getDevice() const {
    return device;
}

void VulkanOps::setObjectName(uint64_t object, vk::ObjectType objectType, const char *name) {
    vk::DebugUtilsObjectNameInfoEXT nameInfo = {};
    nameInfo.objectType = objectType;
    nameInfo.pObjectName = name;
    nameInfo.objectHandle = object;
    device.setDebugUtilsObjectNameEXT(nameInfo);
}

void VulkanOps::setBufferName(const vk::Buffer &buffer, const char *name) {
    setObjectName((uint64_t)(VkBuffer)buffer, vk::Buffer::objectType, name);
}

vk::PhysicalDevice &VulkanOps::getPhysicalDevice() {
    return physicalDevice;
}
