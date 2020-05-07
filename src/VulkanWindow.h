//
// Created by felixfifi on 04.05.20.
//

#ifndef RTX_RAYTRACER_VULKANWINDOW_H
#define RTX_RAYTRACER_VULKANWINDOW_H

#include "VulkanLoader.h"
#include "VulkanOps.h"

#include <SDL.h>
#include <SDL_vulkan.h>

#include <memory>


const std::vector<const char *> validationLayers = {
        "VK_LAYER_KHRONOS_validation"
        , "VK_LAYER_LUNARG_api_dump"
};

const std::vector<const char *> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_KHR_RAY_TRACING_EXTENSION_NAME,
        VK_KHR_MAINTENANCE3_EXTENSION_NAME,
        VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
        VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME,
        VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME,
        VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
        VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
        VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME
};

const std::vector<const char *> instanceExtensions = {
        VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME
};

const int MAX_FRAMES_IN_FLIGHT = 2;

#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif




typedef std::function<void(uint32_t imageIndex)> fDrawCallback;
typedef std::function<void()> fRecreateSwapchainCallback;

class VulkanWindow {
private:
    fDrawCallback drawCallback;
    fRecreateSwapchainCallback recreateSwapchainCallback;

    SDL_Window *window = nullptr;

    vk::Instance instance;
    vk::DebugUtilsMessengerEXT debugMessenger;
    vk::SurfaceKHR surface;

    vk::PhysicalDevice physicalDevice;
    vk::Device device;

    QueueFamilyIndices queueFamilyIndices;
public:
    const QueueFamilyIndices &getQueueFamilyIndices() const;

private:

    vk::Queue graphicsQueue;
    vk::Queue presentQueue;


    vk::SwapchainKHR swapChain;
    std::vector<vk::Image> swapChainImages;
    vk::Format swapChainImageFormat{};
    vk::Extent2D swapChainExtent;
    std::vector<vk::ImageView> swapChainImageViews;
    std::vector<vk::Framebuffer> swapChainFramebuffers;

    vk::RenderPass renderPass;

    vk::CommandPool commandPool;
    std::vector<vk::CommandBuffer> commandBuffers;

    std::vector<vk::Semaphore> imageAvailableSemaphores;
    std::vector<vk::Semaphore> renderFinishedSemaphores;
    std::vector<vk::Fence> inFlightFences;
    std::vector<vk::Fence> imagesInFlight;
    size_t currentFrame = 0;

    bool framebufferResized = false;

    std::shared_ptr<VulkanOps> vulkanOps;
public:
    const vk::PhysicalDevice &getPhysicalDevice() const;

    const vk::Device &getDevice() const;

    vk::Format getSwapChainImageFormat() const;

    const std::shared_ptr<VulkanOps> & getVulkanOps() const;

    const std::vector<vk::Framebuffer> &getSwapChainFramebuffers() const;

    const vk::RenderPass &getRenderPass() const;

    const vk::Extent2D &getSwapChainExtent() const;

    const std::vector<vk::ImageView> &getSwapChainImageViews() const;

    const vk::CommandPool &getCommandPool() const;

    const std::vector<vk::CommandBuffer> &getCommandBuffers() const;

private:
    void initWindow(int width, int height);
    void initVulkan();

    void mainLoop();

    void createSwapChain();
    void createFramebuffers();

    void createImageViews();

    vk::Extent2D chooseSwapExtent(const vk::SurfaceCapabilitiesKHR &capabilities);

    static vk::PresentModeKHR chooseSwapPresentMode(const std::vector<vk::PresentModeKHR> &availablePresentModes);

    static vk::SurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<vk::SurfaceFormatKHR> &availableFormats);

    void createRenderPass();

    QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice physicalDevice);

    SwapChainSupportDetails querySwapChainSupport(vk::PhysicalDevice physicalDevice);

public:
    VulkanWindow() = default;
    VulkanWindow(int width, int height, fDrawCallback drawCallback, fRecreateSwapchainCallback recreateSwapchainCallback);

    void run();
    void cleanup();

    bool sdlEventHandler();

    void setupDispatchLoader() const;

    static bool checkValidationLayerSupport();

    std::vector<const char *> getRequiredInstanceExtensions();

    static bool checkDeviceExtensionSupport(vk::PhysicalDevice physicalDevice);

    bool isDeviceSuitable(vk::PhysicalDevice physicalDevice);

    void createLogicalDevice();

    void pickPhysicalDevice();

    void createSurface();

    void setupDebugMessenger();

    void populateDebugMessengerCreateInfo(vk::DebugUtilsMessengerCreateInfoEXT &createInfo);

    void createInstance();

    void createCommandPool();

    void createCommandBuffers();

    void createSyncObjects();

    void cleanupSwapChain();

    void recreateSwapChain();

    void drawFrame();

    void createSwapChainDependant();
};


#endif //RTX_RAYTRACER_VULKANWINDOW_H
