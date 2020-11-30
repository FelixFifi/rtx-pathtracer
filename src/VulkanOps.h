//
// Created by felixfifi on 03.05.20.
//

#ifndef RTX_RAYTRACER_VULKANOPS_H
#define RTX_RAYTRACER_VULKANOPS_H

#include "VulkanLoader.h"
#include <glm/vec4.hpp>

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
              const vk::Queue &graphicsQueue);

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
        vk::BufferUsageFlags _usage = usage | vk::BufferUsageFlagBits::eTransferDst;

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


        VulkanOps::createBuffer(bufferSize, _usage,
                                memoryProperties, outBuffer, outMemory);

        VulkanOps::copyBuffer(stagingBuffer, outBuffer, bufferSize, nullptr);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    template<class T, class U>
    void createBufferFrom2Data(const std::vector<T> &dataToCopy1, const std::vector<U> &dataToCopy2, vk::BufferUsageFlags &usage,
                               const vk::MemoryPropertyFlags &memoryProperties, vk::Buffer &outBuffer,
                               vk::DeviceMemory &outMemory) {
        usage = usage | vk::BufferUsageFlagBits::eTransferDst;

        size_t data1Size = sizeof(dataToCopy1[0]) * dataToCopy1.size();
        size_t data2Size = sizeof(dataToCopy2[0]) * dataToCopy2.size();
        vk::DeviceSize bufferSize = data1Size + data2Size;

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        VulkanOps::createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                                vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                                stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, dataToCopy1.data(), data1Size);
        memcpy((static_cast<char*>(data) + data1Size), dataToCopy2.data(), data2Size);
        device.unmapMemory(stagingBufferMemory);


        VulkanOps::createBuffer(bufferSize, usage,
                                memoryProperties, outBuffer, outMemory);

        VulkanOps::copyBuffer(stagingBuffer, outBuffer, bufferSize, nullptr);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    void copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, uint64_t size, const char *label);

    void createImage(uint32_t width, uint32_t height, vk::Format format, vk::ImageTiling tiling,
                     const vk::ImageUsageFlags &usage,
                     const vk::MemoryPropertyFlags &properties, vk::Image &image,
                     vk::DeviceMemory &imageMemory);

    vk::ImageView createImageView(vk::Image image, vk::Format format, const vk::ImageAspectFlags &aspectFlags);

    vk::CommandBuffer beginSingleTimeCommands();

    void endSingleTimeCommands(vk::CommandBuffer commandBuffer, const char *label);

    uint32_t findMemoryType(uint32_t typeFilter, const vk::MemoryPropertyFlags &properties);

    void
    transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout, vk::ImageLayout newLayout);

    void copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height);

    bool hasStencilComponent(vk::Format format);

    vk::ShaderModule createShaderModule(const std::vector<char> &code);

    const vk::Device &getDevice() const;

    void
    createNoiseTextureFromData(const std::vector<glm::vec4> &noise, int texWidth, int texHeight, vk::Image &outImage,
                               vk::DeviceMemory &outMemory, vk::ImageView &outImageView, vk::Sampler &outSampler);

    template<class T>
    void createImageFromData(const std::vector<T> &dataToCopy, int texWidth, int texHeight, const vk::Format &format,
                                        vk::Image &outImage, vk::DeviceMemory &outMemory, vk::ImageView &outImageView) {
        vk::DeviceSize imageSize = dataToCopy.size() * sizeof(T);


        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;
        createBuffer(imageSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                     stagingBuffer,
                     stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, imageSize);
        memcpy(data, dataToCopy.data(), static_cast<size_t>(imageSize));
        device.unmapMemory(stagingBufferMemory);


        createImage(texWidth, texHeight, format, vk::ImageTiling::eOptimal,
                    vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    outImage, outMemory);

        transitionImageLayout(outImage, format, vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eTransferDstOptimal);
        copyBufferToImage(stagingBuffer, outImage, static_cast<uint32_t>(texWidth),
                          static_cast<uint32_t>(texHeight));
        transitionImageLayout(outImage, format, vk::ImageLayout::eTransferDstOptimal,
                              vk::ImageLayout::eShaderReadOnlyOptimal);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);

        outImageView = createImageView(outImage, format, vk::ImageAspectFlagBits::eColor);
    }

    void setBufferName(const vk::Buffer &buffer, const char *name);

    void setObjectName(uint64_t object, vk::ObjectType objectType, const char *name);
    vk::PhysicalDevice &getPhysicalDevice();
};

#endif //RTX_RAYTRACER_VULKANOPS_H
