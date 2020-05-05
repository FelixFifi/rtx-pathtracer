//
// Created by felixfifi on 03.05.20.
//

#ifndef RTX_RAYTRACER_VULKANOPS_H
#define RTX_RAYTRACER_VULKANOPS_H

#include "VulkanLoader.h"

struct QueueFamilyIndices {
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete() {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails {
    vk::SurfaceCapabilitiesKHR capabilities;
    std::vector<vk::SurfaceFormatKHR> formats;
    std::vector<vk::PresentModeKHR> presentModes;
};

class VulkanOps {
private:
    vk::SurfaceKHR surface;
    vk::PhysicalDevice physicalDevice;
    vk::Device device;
    vk::CommandPool commandPool;
    vk::Queue graphicsQueue;

public:
    VulkanOps(const vk::SurfaceKHR &surface, const vk::PhysicalDevice &physicalDevice, const vk::Device &device, const vk::CommandPool &commandPool,
              const vk::Queue &graphicsQueue) : surface(surface), physicalDevice(physicalDevice), device(device), commandPool(commandPool), graphicsQueue(graphicsQueue) {};

    VulkanOps() = default;

    void
    init(const vk::PhysicalDevice &physicalDevice_, const vk::Device &device_, const vk::CommandPool &commandPool_,
         const vk::Queue &graphicsQueue_) {
        physicalDevice = physicalDevice_;
        device = device_;
        commandPool = commandPool_;
        graphicsQueue = graphicsQueue_;
    }

    void createBuffer(vk::DeviceSize size, const vk::BufferUsageFlags &usage,
                      const vk::MemoryPropertyFlags &properties,
                      vk::Buffer &buffer, vk::DeviceMemory &bufferMemory);

    template<class T>
    void createBufferFromData(const std::vector<T> &dataToCopy, const vk::BufferUsageFlags &usage,
                                         const vk::MemoryPropertyFlags &memoryProperties, vk::Buffer &outBuffer,
                                         vk::DeviceMemory &outMemory) {

        vk::DeviceSize bufferSize = sizeof(dataToCopy[0]) * dataToCopy.size();

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                                vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, dataToCopy.data(), (size_t) bufferSize);
        device.unmapMemory(stagingBufferMemory);


        VulkanOps::createBuffer(bufferSize, usage,
                                memoryProperties, outBuffer, outMemory);

        VulkanOps::copyBuffer(stagingBuffer, outBuffer, bufferSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    void copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, vk::DeviceSize size);

    void createImage(uint32_t width, uint32_t height, vk::Format format, vk::ImageTiling tiling,
                     const vk::ImageUsageFlags &usage,
                     const vk::MemoryPropertyFlags &properties, vk::Image &image,
                     vk::DeviceMemory &imageMemory);

    vk::ImageView createImageView(vk::Image image, vk::Format format, const vk::ImageAspectFlags &aspectFlags);

    vk::CommandBuffer beginSingleTimeCommands();

    void endSingleTimeCommands(vk::CommandBuffer commandBuffer);

    uint32_t findMemoryType(uint32_t typeFilter, const vk::MemoryPropertyFlags &properties);

    void
    transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout, vk::ImageLayout newLayout);

    void copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height);

    bool hasStencilComponent(vk::Format format);

    vk::ShaderModule createShaderModule(const std::vector<char> &code);
};

#endif //RTX_RAYTRACER_VULKANOPS_H
