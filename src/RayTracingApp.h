//
// Created by felixfifi on 09.04.20.
//

#ifndef RTX_RAYTRACER_RAYTRACINGAPP_H
#define RTX_RAYTRACER_RAYTRACINGAPP_H

#define VK_ENABLE_BETA_EXTENSIONS

#include <SDL.h>
#include <SDL_vulkan.h>

#define VULKAN_HPP_DISPATCH_LOADER_DYNAMIC 1

#include <vulkan/vulkan.hpp>

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE;

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

#define STB_IMAGE_IMPLEMENTATION

#include <stb_image.h>

#define TINYOBJLOADER_IMPLEMENTATION

#include <tiny_obj_loader.h>

#include <iostream>
#include <exception>
#include <string>
#include <vector>
#include <optional>
#include <set>
#include <chrono>

const std::string MODEL_PATH = "models/chalet.obj";
const std::string TEXTURE_PATH = "textures/chalet.jpg";

const std::vector<const char *> validationLayers = {
        "VK_LAYER_KHRONOS_validation"
        //, "VK_LAYER_LUNARG_api_dump"
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
    RayTracingApp(uint32_t width, uint32_t height) : WIDTH(width), HEIGHT(height) {}

    void run() {
        initWindow();
        initVulkan();
        mainLoop();
        cleanup();
    }

public:
    const uint32_t WIDTH, HEIGHT; // TODO: Disable resize or remove these

private:
    SDL_Window *window = nullptr;

    vk::Instance instance;
    vk::DebugUtilsMessengerEXT debugMessenger;
    vk::SurfaceKHR surface;

    vk::DynamicLoader dl;

    vk::PhysicalDevice physicalDevice;
    vk::Device device;

    vk::Queue graphicsQueue;
    vk::Queue presentQueue;

    vk::SwapchainKHR swapChain;
    std::vector<vk::Image> swapChainImages;
    vk::Format swapChainImageFormat{};
    vk::Extent2D swapChainExtent;

    std::vector<vk::ImageView> swapChainImageViews;
    vk::RenderPass renderPass;
    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    std::vector<vk::DescriptorSet> descriptorSets;
    vk::PipelineLayout pipelineLayout;
    vk::Pipeline graphicsPipeline;

    std::vector<vk::Framebuffer> swapChainFramebuffers;

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


    void initWindow() {
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            std::string msg = "SDL init failed with error: ";
            msg += SDL_GetError();
            throw std::runtime_error(msg);
        }

        window = SDL_CreateWindow(
                "RTX Raytracer",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                WIDTH,
                HEIGHT,
                SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE
        );
        if (window == nullptr) {
            SDL_Quit();
            throw std::runtime_error("SDL window creation failed!");
        }
    }

    void initVulkan() {
        setupDispatchLoader();
        createInstance();

        VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);

        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();

        VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

        setupDebugMessenger();

        createSwapChain();
        createImageViews();
        createRenderPass();
        createDecriptorSetLayout();
        createGraphicsPipeline();
        createCommandPool();
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

    void setupDispatchLoader() const {
        PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr =
                dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
        VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);
    }

    void mainLoop() {
        bool quit = false;
        while (!quit) {
            drawFrame();

            quit = sdlEventHandler();
        }

        vkDeviceWaitIdle(device);
    }

    void drawFrame() {
        device.waitForFences(1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

        uint32_t imageIndex;
        vk::ResultValue<uint32_t> result = device.acquireNextImageKHR(swapChain, UINT64_MAX,
                                                                      imageAvailableSemaphores[currentFrame],
                                                                      nullptr);

        switch (result.result) {
            case vk::Result::eErrorOutOfDateKHR:
                recreateSwapChain();
                return;
            case vk::Result::eSuboptimalKHR:
            case vk::Result::eSuccess:
                imageIndex = result.value;
                break;
            default:
                throw std::runtime_error("failed to acquire swap chain image!");
        }

        if (imagesInFlight[imageIndex]) {
            device.waitForFences(1, &imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
        }
        imagesInFlight[imageIndex] = inFlightFences[currentFrame];

        updateUniformBuffer(imageIndex);
        updateRtDescriptorSet(imageIndex);


        vk::Semaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};
        vk::PipelineStageFlags waitStages[] = {vk::PipelineStageFlagBits::eColorAttachmentOutput};
        vk::Semaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};

        vk::SubmitInfo submitInfo(1, waitSemaphores, waitStages, 1,
                                  &commandBuffers[imageIndex], 1, signalSemaphores);

        device.resetFences(inFlightFences[currentFrame]);

        graphicsQueue.submit(submitInfo, inFlightFences[currentFrame]);

        vk::PresentInfoKHR presentInfo(1, signalSemaphores, 1, &swapChain, &imageIndex);
        vk::Result resultPresent = presentQueue.presentKHR(presentInfo);

        switch (resultPresent) {
            case vk::Result::eErrorOutOfDateKHR:
            case vk::Result::eSuboptimalKHR:
                framebufferResized = false;
                recreateSwapChain();
                break;
            case vk::Result::eSuccess:
                break;
            default:
                throw std::runtime_error("Unknown present result");
        }

        if (framebufferResized) {
            framebufferResized = false;
            recreateSwapChain();
        }

        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
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

    bool sdlEventHandler() {
        bool quit = false;
        static SDL_Event sdlEvent;
        while (SDL_PollEvent(&sdlEvent)) {
            //If user closes the window
            if (sdlEvent.type == SDL_QUIT) {
                quit = true;
            }

            if (sdlEvent.type == SDL_WINDOWEVENT) {
                if (sdlEvent.window.event == SDL_WINDOWEVENT_RESIZED) {
                    framebufferResized = true;
                }
            }

//            //If user presses any key
//            if (sdlEvent.type == SDL_KEYDOWN){
//                quit = true;
//            }
//            //If user clicks the mouse
//            if (sdlEvent.type == SDL_MOUSEBUTTONDOWN){
//                quit = true;
//            }
        }
        return quit;
    }

    void cleanup() {
        cleanupSwapChain();

        device.destroy(textureSampler, nullptr);
        device.destroy(textureImageView, nullptr);

        device.destroy(textureImage, nullptr);
        vkFreeMemory(device, textureImageMemory, nullptr);

        device.destroy(descriptorSetLayout, nullptr);

        device.destroy(vertexBuffer, nullptr);
        vkFreeMemory(device, vertexBufferMemory, nullptr);

        device.destroy(indexBuffer, nullptr);
        vkFreeMemory(device, indexBufferMemory, nullptr);

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            device.destroy(renderFinishedSemaphores[i], nullptr);
            device.destroy(imageAvailableSemaphores[i], nullptr);
            device.destroy(inFlightFences[i], nullptr);
        }

        device.destroy(commandPool, nullptr);

        device.destroy(rtSBTBuffer);
        device.free(rtSBTBufferMemory);
        device.destroy(rtPipeline);
        device.destroy(rtPipelineLayout);

        rtBuilder.destroy();
        device.destroy(rtDescPool);
        device.destroy(rtDescSetLayout);

        device.destroy(nullptr);

        if (enableValidationLayers) {
            instance.destroyDebugUtilsMessengerEXT(debugMessenger);
        }

        instance.destroy(surface, nullptr);
        instance.destroy(nullptr);


        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    void createInstance() {
        if (enableValidationLayers && !checkValidationLayerSupport()) {
            throw std::runtime_error("validation layers requested, but not available!");
        }

        const char *name = "Hello Triangle";
        const char *engineName = "No Engine";
        vk::ApplicationInfo appInfo(name, VK_MAKE_VERSION(1, 0, 0), engineName,
                                    VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_0);


        auto extensions = getRequiredExtensions();
        auto enabledExtensionCount = static_cast<uint32_t>(extensions.size());

        vk::InstanceCreateInfo createInfo;
        if (enableValidationLayers) {
            auto enabledLayerCount = static_cast<uint32_t>(validationLayers.size());

            createInfo = vk::InstanceCreateInfo({}, &appInfo, enabledLayerCount, validationLayers.data(),
                                                enabledExtensionCount, extensions.data());

            vk::DebugUtilsMessengerCreateInfoEXT debugCreateInfo;
            populateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext = &debugCreateInfo;
        } else {
            uint32_t enabledLayerCount = 0;
            createInfo = vk::InstanceCreateInfo({}, &appInfo, enabledLayerCount, nullptr, enabledExtensionCount,
                                                extensions.data());
        }

        instance = vk::createInstance(createInfo);
    }

    static void populateDebugMessengerCreateInfo(vk::DebugUtilsMessengerCreateInfoEXT &createInfo) {
        vk::DebugUtilsMessageSeverityFlagsEXT messageSeverity = vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose |
                                                                vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
                                                                vk::DebugUtilsMessageSeverityFlagBitsEXT::eError;
        vk::DebugUtilsMessageTypeFlagsEXT messageType = vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
                                                        vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
                                                        vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance;

        createInfo = vk::DebugUtilsMessengerCreateInfoEXT({}, messageSeverity, messageType, debugCallback, {});
    }

    void setupDebugMessenger() {
        vk::DebugUtilsMessengerCreateInfoEXT createInfo;
        populateDebugMessengerCreateInfo(createInfo);

        debugMessenger = instance.createDebugUtilsMessengerEXT(createInfo);
    }

    void createSurface() {
        VkSurfaceKHR c_surface;

        if (SDL_Vulkan_CreateSurface(window, static_cast<VkInstance>(instance), &c_surface) != SDL_TRUE) {
            std::string msg = "SDL create surface failed with error: ";
            msg += SDL_GetError();
            throw std::runtime_error(msg);
        }

        surface = c_surface;
    }

    void pickPhysicalDevice() {
        vector<vk::PhysicalDevice> devices = instance.enumeratePhysicalDevices();

        if (devices.empty()) {
            throw std::runtime_error("failed to find GPUs with Vulkan support!");
        }

        for (const auto &device : devices) {
            if (isDeviceSuitable(device)) {
                physicalDevice = device;
                break;
            }
        }

        if (!physicalDevice) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    void createLogicalDevice() {
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

        std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
        std::set<uint32_t> uniqueQueueFamilies = {queueFamilyIndices.graphicsFamily.value(),
                                                  queueFamilyIndices.presentFamily.value()};

        float queuePriority = 1.0f;
        for (uint32_t queueFamily : uniqueQueueFamilies) {
            vk::DeviceQueueCreateInfo queueCreateInfo({}, queueFamily, 1, &queuePriority);
            queueCreateInfos.push_back(queueCreateInfo);
        }


        vk::PhysicalDeviceBufferDeviceAddressFeatures addresFeatures;
        addresFeatures.bufferDeviceAddress = true;

        vk::PhysicalDeviceFeatures deviceFeatures = {};
        deviceFeatures.samplerAnisotropy = VK_TRUE;
        deviceFeatures.robustBufferAccess = false;

        vk::PhysicalDeviceFeatures2 deviceFeatures2(deviceFeatures);
        deviceFeatures2.setPNext(&addresFeatures);

        auto queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
        auto enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());


        uint32_t enabledLayerCount = 0;
        const char *const *enabledLayerNames = nullptr;
        if (enableValidationLayers) {
            enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            enabledLayerNames = validationLayers.data();
        }

        vk::DeviceCreateInfo createInfo({}, queueCreateInfoCount, queueCreateInfos.data(), enabledLayerCount,
                                        enabledLayerNames, enabledExtensionCount, deviceExtensions.data(),
                                        nullptr);
        createInfo.setPNext(&deviceFeatures2);

        device = physicalDevice.createDevice(createInfo);

        graphicsQueue = device.getQueue(queueFamilyIndices.graphicsFamily.value(), 0);
        presentQueue = device.getQueue(queueFamilyIndices.presentFamily.value(), 0);
    }

    void createSwapChain() {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

        vk::SurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
        vk::PresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
        vk::Extent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

        uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
        if (swapChainSupport.capabilities.maxImageCount > 0 &&
            imageCount > swapChainSupport.capabilities.maxImageCount) {
            imageCount = swapChainSupport.capabilities.maxImageCount;
        }

        vk::SwapchainCreateInfoKHR createInfo = {};
        createInfo.surface = surface;

        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = vk::ImageUsageFlagBits::eColorAttachment;

        QueueFamilyIndices queueFamilies = findQueueFamilies(physicalDevice);
        uint32_t queueFamilyIndices[] = {queueFamilies.graphicsFamily.value(), queueFamilies.presentFamily.value()};

        if (queueFamilies.graphicsFamily != queueFamilies.presentFamily) {
            createInfo.imageSharingMode = vk::SharingMode::eConcurrent;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = queueFamilyIndices;
        } else {
            createInfo.imageSharingMode = vk::SharingMode::eExclusive;
        }

        createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
        createInfo.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;

        createInfo.oldSwapchain = nullptr;
        createInfo.pNext = nullptr;

        swapChain = device.createSwapchainKHR(createInfo);
        swapChainImages = device.getSwapchainImagesKHR(swapChain);

        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;
    }

    static vk::SurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<vk::SurfaceFormatKHR> &availableFormats) {
        for (const auto &availableFormat : availableFormats) {
            if (availableFormat.format == vk::Format::eB8G8R8A8Srgb &&
                availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
                return availableFormat;
            }
        }

        return availableFormats[0];
    }

    static vk::PresentModeKHR chooseSwapPresentMode(const std::vector<vk::PresentModeKHR> &availablePresentModes) {
        for (const auto &availablePresentMode : availablePresentModes) {
            if (availablePresentMode == vk::PresentModeKHR::eMailbox) {
                return availablePresentMode;
            }
        }

        return vk::PresentModeKHR::eFifo;
    }

    vk::Extent2D chooseSwapExtent(const vk::SurfaceCapabilitiesKHR &capabilities) {
        if (capabilities.currentExtent.width != UINT32_MAX) {
            return capabilities.currentExtent;
        } else {
            int width;
            int height;
            SDL_Vulkan_GetDrawableSize(window, &width, &height);

            auto u_width = static_cast<uint32_t>(width);
            auto u_height = static_cast<uint32_t>(height);

            width = std::clamp(u_width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            height = std::clamp(u_height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

            return vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
        }
    }

    SwapChainSupportDetails querySwapChainSupport(vk::PhysicalDevice physicalDevice) {
        SwapChainSupportDetails details;

        details.capabilities = physicalDevice.getSurfaceCapabilitiesKHR(surface);
        details.formats = physicalDevice.getSurfaceFormatsKHR(surface);
        details.presentModes = physicalDevice.getSurfacePresentModesKHR(surface);

        return details;
    }

    bool isDeviceSuitable(vk::PhysicalDevice physicalDevice) {
        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);

        bool extensionsSupported = checkDeviceExtensionSupport(physicalDevice);

        bool swapChainAdequate = false;
        if (extensionsSupported) {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);
            swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
        }

        vk::PhysicalDeviceFeatures supportedFeatures;
        supportedFeatures = physicalDevice.getFeatures();

        return indices.isComplete() && extensionsSupported && swapChainAdequate && supportedFeatures.samplerAnisotropy;
    }

    static bool checkDeviceExtensionSupport(vk::PhysicalDevice physicalDevice) {

        std::vector<vk::ExtensionProperties> availableExtensions = physicalDevice.enumerateDeviceExtensionProperties();

        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

        for (const auto &extension : availableExtensions) {
            requiredExtensions.erase(extension.extensionName);
        }

        return requiredExtensions.empty();
    }

    QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice physicalDevice) {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        std::vector<vk::QueueFamilyProperties> queueFamilies = physicalDevice.getQueueFamilyProperties();

        int i = 0;
        for (const auto &queueFamily : queueFamilies) {
            if (queueFamily.queueFlags & vk::QueueFlagBits::eGraphics) {
                indices.graphicsFamily = i;
            }

            vk::Bool32 presentSupport = physicalDevice.getSurfaceSupportKHR(i, surface);

            if (presentSupport) {
                indices.presentFamily = i;
            }

            if (indices.isComplete()) {
                break;
            }

            i++;
        }

        return indices;
    }

    std::vector<const char *> getRequiredExtensions() {
        unsigned int sdlExtensionCount = 0;
        SDL_Vulkan_GetInstanceExtensions(window, &sdlExtensionCount, nullptr);

        std::vector<const char *> extensions;
        extensions.resize(sdlExtensionCount);

        if (SDL_Vulkan_GetInstanceExtensions(window, &sdlExtensionCount, extensions.data()) != SDL_TRUE) {
            std::string msg = "SDL Vulkan_GetInstanceExtensions failed with error: ";
            msg += SDL_GetError();
            throw std::runtime_error(msg);
        }

        if (enableValidationLayers) {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }

        for (auto &extension: instanceExtensions) {
            extensions.push_back(extension);
        }

        return extensions;
    }

    static bool checkValidationLayerSupport() {
        std::vector<vk::LayerProperties> availableLayers = vk::enumerateInstanceLayerProperties();

        for (const char *layerName : validationLayers) {
            bool layerFound = false;

            for (const auto &layerProperties : availableLayers) {
                if (strcmp(layerName, layerProperties.layerName) == 0) {
                    layerFound = true;
                    break;
                }
            }

            if (!layerFound) {
                return false;
            }
        }

        return true;
    }


    void createImageViews() {
        swapChainImageViews.resize(swapChainImages.size());

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            swapChainImageViews[i] = createImageView(swapChainImages[i], swapChainImageFormat,
                                                     vk::ImageAspectFlagBits::eColor);
        }
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

        vk::ShaderModule vertShaderModule = createShaderModule(vertShaderCode);
        vk::ShaderModule fragShaderModule = createShaderModule(fragShaderCode);

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

    vk::ShaderModule createShaderModule(const std::vector<char> &code) {
        vk::ShaderModuleCreateInfo createInfo({}, code.size(), reinterpret_cast<const uint32_t *>(code.data()));

        return device.createShaderModule(createInfo);
    }

    void createVertexBuffer() {
        vk::DeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                     stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, vertices.data(), (size_t) bufferSize);
        device.unmapMemory(stagingBufferMemory);

        createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer |
                                 vk::BufferUsageFlagBits::eStorageBuffer |
                                 vk::BufferUsageFlagBits::eShaderDeviceAddress,
                     vk::MemoryPropertyFlagBits::eDeviceLocal, vertexBuffer, vertexBufferMemory);

        copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }


    void createIndexBuffer() {
        vk::DeviceSize bufferSize = sizeof(indices[0]) * indices.size();

        vk::Buffer stagingBuffer;
        vk::DeviceMemory stagingBufferMemory;

        createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                     stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, bufferSize);

        memcpy(data, indices.data(), (size_t) bufferSize);
        device.unmapMemory(stagingBufferMemory);

        createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eIndexBuffer |
                                 vk::BufferUsageFlagBits::eStorageBuffer,
                     vk::MemoryPropertyFlagBits::eDeviceLocal, indexBuffer, indexBufferMemory);

        copyBuffer(stagingBuffer, indexBuffer, bufferSize);

        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }


    void copyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer, vk::DeviceSize size) {
        vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

        vk::BufferCopy copyRegion(0, 0, size);
        commandBuffer.copyBuffer(srcBuffer, dstBuffer, copyRegion);

        endSingleTimeCommands(commandBuffer);
    }

    uint32_t findMemoryType(uint32_t typeFilter, const vk::MemoryPropertyFlags &properties) {
        vk::PhysicalDeviceMemoryProperties memProperties;
        memProperties = physicalDevice.getMemoryProperties();

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }

        throw std::runtime_error("failed to find suitable memory type!");
    }

    void createBuffer(vk::DeviceSize size, const vk::BufferUsageFlags &usage, const vk::MemoryPropertyFlags &properties,
                      vk::Buffer &buffer, vk::DeviceMemory &bufferMemory) {
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

    void createFramebuffers() {
        swapChainFramebuffers.resize(swapChainImageViews.size());

        for (size_t i = 0; i < swapChainImageViews.size(); i++) {
            std::array<vk::ImageView, 2> attachments = {
                    swapChainImageViews[i],
                    depthImageView
            };

            vk::FramebufferCreateInfo framebufferInfo({}, renderPass,
                                                      static_cast<uint32_t>(attachments.size()), attachments.data(),
                                                      swapChainExtent.width, swapChainExtent.height, 1);

            swapChainFramebuffers[i] = device.createFramebuffer(framebufferInfo);
        }
    }

    void createCommandPool() {
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

        vk::CommandPoolCreateInfo poolInfo({}, queueFamilyIndices.graphicsFamily.value());

        commandPool = device.createCommandPool(poolInfo);
    }

    void createCommandBuffers() {
        commandBuffers.resize(swapChainFramebuffers.size());

        vk::CommandBufferAllocateInfo allocInfo(commandPool, vk::CommandBufferLevel::ePrimary,
                                                static_cast<uint32_t>(commandBuffers.size()));

        commandBuffers = device.allocateCommandBuffers(allocInfo);

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

            commandBuffers[i].drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0,
                                          0, 0);

            commandBuffers[i].endRenderPass();
            commandBuffers[i].end();
        }
    }

    void createSyncObjects() {
        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
        imagesInFlight.resize(swapChainImages.size(), nullptr);

        vk::SemaphoreCreateInfo semaphoreInfo;

        vk::FenceCreateInfo fenceInfo(vk::FenceCreateFlagBits::eSignaled);

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            imageAvailableSemaphores[i] = device.createSemaphore(semaphoreInfo);
            renderFinishedSemaphores[i] = device.createSemaphore(semaphoreInfo);
            inFlightFences[i] = device.createFence(fenceInfo);
        }
    }

    void cleanupSwapChain() {
        device.destroy(depthImageView);
        device.destroy(depthImage);
        device.freeMemory(depthImageMemory);

        for (auto swapChainFramebuffer : swapChainFramebuffers) {
            device.destroy(swapChainFramebuffer);
        }

        device.freeCommandBuffers(commandPool, commandBuffers);

        device.destroy(graphicsPipeline);
        device.destroy(pipelineLayout);
        device.destroy(renderPass);

        for (auto swapChainImageView : swapChainImageViews) {
            device.destroy(swapChainImageView);
        }

        device.destroy(swapChain);

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            device.destroy(uniformBuffers[i]);
            device.freeMemory(uniformBuffersMemory[i]);
        }

        device.destroy(descriptorPool);
    }

    void recreateSwapChain() {

        // Doesn't seem to be necessary, as SDL seems to keep drawable size constant even when minimized
        int width = 0, height = 0;
        SDL_Vulkan_GetDrawableSize(window, &width, &height);
        while (width == 0 || height == 0) {
            SDL_Vulkan_GetDrawableSize(window, &width, &height);
            SDL_WaitEvent(nullptr);
        }

        device.waitIdle();

        cleanupSwapChain();

        createSwapChain();
        createImageViews();
        createRenderPass();
        createGraphicsPipeline();
        createDepthResources();
        createFramebuffers();
        createUniformBuffers();
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
    }

    void createDecriptorSetLayout() {
        vk::DescriptorSetLayoutBinding uboLayoutBinding(0, vk::DescriptorType::eUniformBuffer,
                                                        1, vk::ShaderStageFlagBits::eVertex |
                                                           vk::ShaderStageFlagBits::eRaygenKHR,
                                                        nullptr);

        vk::DescriptorSetLayoutBinding samplerLayoutBinding(1, vk::DescriptorType::eCombinedImageSampler,
                                                            1, vk::ShaderStageFlagBits::eFragment,
                                                            nullptr);

        std::array<vk::DescriptorSetLayoutBinding, 2> bindings = {uboLayoutBinding, samplerLayoutBinding};
        vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());


        descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);
    }

    void createDescriptorPool() {
        std::array<vk::DescriptorPoolSize, 2> poolSizes = {
                vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer,
                                       static_cast<uint32_t>(swapChainImages.size())),
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler,
                                       static_cast<uint32_t>(swapChainImages.size()))};

        vk::DescriptorPoolCreateInfo poolInfo({}, static_cast<uint32_t>(swapChainImages.size()),
                                              static_cast<uint32_t>(poolSizes.size()), poolSizes.data());

        descriptorPool = device.createDescriptorPool(poolInfo);
    }

    void createDescriptorSets() {
        std::vector<vk::DescriptorSetLayout> layouts(swapChainImages.size(), descriptorSetLayout);
        vk::DescriptorSetAllocateInfo allocInfo(descriptorPool, static_cast<uint32_t>(swapChainImages.size()),
                                                layouts.data());

        descriptorSets = device.allocateDescriptorSets(allocInfo);

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            vk::DescriptorBufferInfo bufferInfo(uniformBuffers[i], 0, sizeof(UniformBufferObject));

            vk::DescriptorImageInfo imageInfo(textureSampler, textureImageView,
                                              vk::ImageLayout::eShaderReadOnlyOptimal);

            std::array<vk::WriteDescriptorSet, 2> descriptorWrites = {};

            descriptorWrites[0] = vk::WriteDescriptorSet(descriptorSets[i], 0, 0, 1, vk::DescriptorType::eUniformBuffer,
                                                         nullptr, &bufferInfo, nullptr);

            descriptorWrites[1] = vk::WriteDescriptorSet(descriptorSets[i], 1, 0, 1,
                                                         vk::DescriptorType::eCombinedImageSampler, &imageInfo,
                                                         nullptr, nullptr);

            device.updateDescriptorSets(descriptorWrites, nullptr);
        }
    }

    void createUniformBuffers() {
        vk::DeviceSize bufferSize = sizeof(UniformBufferObject);

        uniformBuffers.resize(swapChainImages.size());
        uniformBuffersMemory.resize(swapChainImages.size());

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            createBuffer(bufferSize, vk::BufferUsageFlagBits::eUniformBuffer,
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
        createBuffer(imageSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                     stagingBuffer,
                     stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, imageSize);
        memcpy(data, pixels, static_cast<size_t>(imageSize));
        device.unmapMemory(stagingBufferMemory);

        stbi_image_free(pixels);

        createImage(texWidth, texHeight, vk::Format::eR8G8B8A8Srgb, vk::ImageTiling::eOptimal,
                    vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    textureImage, textureImageMemory);

        transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eTransferDstOptimal);
        copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(texWidth),
                          static_cast<uint32_t>(texHeight));
        transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageLayout::eTransferDstOptimal,
                              vk::ImageLayout::eShaderReadOnlyOptimal);


        device.destroy(stagingBuffer);
        device.freeMemory(stagingBufferMemory);
    }

    void
    transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout, vk::ImageLayout newLayout) {
        vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

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
        } else {
            throw std::invalid_argument("unsupported layout transition!");
        }

        commandBuffer.pipelineBarrier(sourceStage, destinationStage, {}, 0, nullptr, 0, nullptr, 1, &barrier);

        endSingleTimeCommands(commandBuffer);
    }

    void copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height) {
        vk::CommandBuffer commandBuffer = beginSingleTimeCommands();

        vk::BufferImageCopy region(0, 0, 0,
                                   {vk::ImageAspectFlagBits::eColor, 0, 0, 1},
                                   {0, 0, 0}, {width, height, 1});

        commandBuffer.copyBufferToImage(buffer, image, vk::ImageLayout::eTransferDstOptimal, region);

        endSingleTimeCommands(commandBuffer);
    }

    void
    createImage(uint32_t width, uint32_t height, vk::Format format, vk::ImageTiling tiling,
                const vk::ImageUsageFlags &usage,
                const vk::MemoryPropertyFlags &properties, vk::Image &image, vk::DeviceMemory &imageMemory) {
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

    vk::CommandBuffer beginSingleTimeCommands() {
        vk::CommandBufferAllocateInfo allocInfo(commandPool, vk::CommandBufferLevel::ePrimary, 1);

        vk::CommandBuffer commandBuffer;
        commandBuffer = device.allocateCommandBuffers(allocInfo)[0];

        vk::CommandBufferBeginInfo beginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit, {});
        commandBuffer.begin(beginInfo);

        return commandBuffer;
    }

    void endSingleTimeCommands(vk::CommandBuffer commandBuffer) {
        commandBuffer.end();

        vk::SubmitInfo submitInfo(0, nullptr, {},
                                  1, &commandBuffer, 0, nullptr);

        graphicsQueue.submit(submitInfo, nullptr);
        graphicsQueue.waitIdle();

        device.freeCommandBuffers(commandPool, commandBuffer);
    }


    void createTextureImageView() {
        textureImageView = createImageView(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageAspectFlagBits::eColor);
    }

    vk::ImageView createImageView(vk::Image image, vk::Format format, const vk::ImageAspectFlags &aspectFlags) {
        vk::ImageSubresourceRange subresourceRange(aspectFlags, 0, 1, 0, 1);

        vk::ImageViewCreateInfo viewInfo({}, image, vk::ImageViewType::e2D, format, {}, subresourceRange);

        return device.createImageView(viewInfo);
    }

    void createTextureSampler() {
        vk::SamplerCreateInfo samplerInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear,
                                          vk::SamplerAddressMode::eRepeat, vk::SamplerAddressMode::eRepeat,
                                          vk::SamplerAddressMode::eRepeat, 0.0f, true, 16, false,
                                          vk::CompareOp::eAlways, 0.0f, 0.0f, vk::BorderColor::eIntOpaqueBlack, false);
        textureSampler = device.createSampler(samplerInfo);
    }

    void createDepthResources() {
        vk::Format depthFormat = findDepthFormat();

        createImage(swapChainExtent.width, swapChainExtent.height, depthFormat, vk::ImageTiling::eOptimal,
                    vk::ImageUsageFlagBits::eDepthStencilAttachment, vk::MemoryPropertyFlagBits::eDeviceLocal,
                    depthImage, depthImageMemory);
        depthImageView = createImageView(depthImage, depthFormat, vk::ImageAspectFlagBits::eDepth);

        transitionImageLayout(depthImage, depthFormat, vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eDepthStencilAttachmentOptimal);
    }

    vk::Format findDepthFormat() {
        return findSupportedFormat(
                {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint, vk::Format::eD24UnormS8Uint},
                vk::ImageTiling::eOptimal,
                vk::FormatFeatureFlagBits::eDepthStencilAttachment
        );
    }

    static bool hasStencilComponent(vk::Format format) {
        return format == vk::Format::eD32SfloatS8Uint || format == vk::Format::eD24UnormS8Uint;
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
                {}, swapChainImageViews[0], vk::ImageLayout::eGeneral}; // TODO: Update each frame

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
                {}, swapChainImageViews[currentImage], vk::ImageLayout::eGeneral};
        vk::WriteDescriptorSet wds{rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
        device.updateDescriptorSets(wds, nullptr);
    }

    void createRtPipeline() {
        auto raygenCode = readFile("shaders/raytrace.rgen.spv");
        auto missCode = readFile("shaders/raytrace.rmiss.spv");
        auto chitCode = readFile("shaders/raytrace.rchit.spv");

        vk::ShaderModule raygenShaderModule = createShaderModule(raygenCode);
        vk::ShaderModule missShaderModule = createShaderModule(missCode);
        vk::ShaderModule chitShaderModule = createShaderModule(chitCode);

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

        createBuffer(sbtSize, vk::BufferUsageFlagBits::eTransferSrc,
                     vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                     stagingBuffer, stagingBufferMemory);

        void *data;
        data = device.mapMemory(stagingBufferMemory, 0, sbtSize);

        memcpy(data, shaderHandleStorage.data(), (size_t) sbtSize);

        device.unmapMemory(stagingBufferMemory);

        createBuffer(sbtSize, vk::BufferUsageFlagBits::eRayTracingKHR, vk::MemoryPropertyFlagBits::eDeviceLocal,
                     rtSBTBuffer, rtSBTBufferMemory);
        copyBuffer(stagingBuffer, vertexBuffer, sbtSize);

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

    static std::vector<char> readFile(const std::string &filename) {
        std::ifstream file(filename, std::ios::ate | std::ios::binary);

        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }

        size_t fileSize = (size_t) file.tellg();
        std::vector<char> buffer(fileSize);

        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();

        return buffer;
    }


    static VKAPI_ATTR VkBool32 VKAPI_CALL
    debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType,
                  const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData) {
        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

        return VK_FALSE;
    }
};

#endif //RTX_RAYTRACER_RAYTRACINGAPP_H
