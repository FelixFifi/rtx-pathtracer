//
// Created by felixfifi on 04.05.20.
//

#include <set>
#include <iostream>
#include <err.h>
#include "VulkanWindow.h"
#include "VulkanOps.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_vulkan.h"

static VKAPI_ATTR VkBool32 VKAPI_CALL
debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType,
              const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData) {

    // TODO:https://www.khronos.org/registry/vulkan/specs/1.2-extensions/man/html/VkDebugUtilsObjectNameInfoEXT.html https://www.khronos.org/registry/vulkan/specs/1.2-extensions/man/html/VK_EXT_debug_utils.html
    std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

    return VK_FALSE;
}


VulkanWindow::VulkanWindow(int width, int height, fDrawCallback drawCallback,
                           fRecreateSwapchainCallback recreateSwapchainCallback)
        : drawCallback(drawCallback), recreateSwapchainCallback(recreateSwapchainCallback) {
    initWindow(width, height);
    initVulkan();
}

void VulkanWindow::run() {
    mainLoop();
}

void VulkanWindow::drawFrame() {
    // TODO: Pre draw callback

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

    // Start the Dear ImGui frame
    ImGui_ImplSDL2_NewFrame(window);
    ImGui_ImplVulkan_NewFrame();

    if (imagesInFlight[imageIndex]) {
        device.waitForFences(1, &imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
    }
    imagesInFlight[imageIndex] = inFlightFences[currentFrame];

    // Custom function
    drawCallback(imageIndex);

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

void VulkanWindow::initWindow(int width, int height) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::string msg = "SDL init failed with error: ";
        msg += SDL_GetError();
        throw std::runtime_error(msg);
    }

    window = SDL_CreateWindow(
            "RTX Raytracer",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            width,
            height,
            SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE
    );
    if (window == nullptr) {
        SDL_Quit();
        throw std::runtime_error("SDL window creation failed!");
    }
}


void VulkanWindow::initVulkan() {

    setupDispatchLoader();
    createInstance();

    VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);

    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();

    VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

    createCommandPool();

    vulkanOps = std::make_shared<VulkanOps>(surface, physicalDevice, device, commandPool, graphicsQueue);

    if(enableValidationLayers) {
        setupDebugMessenger();
    }

    createSwapChainDependant();

    createSyncObjects();

    createImGuiDescriptorPool();
    setupImGui();
}

void VulkanWindow::createImGuiDescriptorPool() {

    vk::DescriptorPoolSize pool_sizes[] =
            {
                    { vk::DescriptorType::eCombinedImageSampler, 2 }
//                    ,{ vk::DescriptorType::eSampler, 1000 },
//                    { vk::DescriptorType::eSampledImage, 1000 },
//                    { vk::DescriptorType::eStorageImage, 1000 },
//                    { vk::DescriptorType::eUniformTexelBuffer, 1000 },
//                    { vk::DescriptorType::eStorageTexelBuffer, 1000 },
//                    { vk::DescriptorType::eUniformBuffer, 1000 },
//                    { vk::DescriptorType::eStorageBuffer, 1000 },
//                    { vk::DescriptorType::eUniformBufferDynamic, 1000 },
//                    { vk::DescriptorType::eStorageBufferDynamic, 1000 },
//                    { vk::DescriptorType::eInputAttachment, 1000 }
            };
    vk::DescriptorPoolCreateInfo pool_info = {};
    pool_info.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
    pool_info.maxSets = 2;
    pool_info.poolSizeCount = (uint32_t)IM_ARRAYSIZE(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;
    imGuiDescriptorPool = device.createDescriptorPool(pool_info);
}

static void check_vk_result(VkResult err)
{
    if (err == 0) return;
    printf("VkResult %d\n", err);
    if (err < 0)
        abort();
}

void VulkanWindow::setupImGui() {
    IMGUI_CHECKVERSION();

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForVulkan(window);
    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = instance;
    init_info.PhysicalDevice = physicalDevice;
    init_info.Device = device;
    init_info.QueueFamily = queueFamilyIndices.graphicsFamily.value();
    init_info.Queue = graphicsQueue;
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = imGuiDescriptorPool;
    init_info.Allocator = nullptr;
    init_info.MinImageCount = 2;
    init_info.ImageCount = swapChainFramebuffers.size();
    init_info.CheckVkResultFn = check_vk_result;
    ImGui_ImplVulkan_Init(&init_info, renderPass);

    vk::CommandBuffer cmdBuf = vulkanOps->beginSingleTimeCommands();

    ImGui_ImplVulkan_CreateFontsTexture(cmdBuf);

    vulkanOps->endSingleTimeCommands(cmdBuf);

    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void VulkanWindow::mainLoop() {
    bool quit = false;
    while (!quit) {
        drawFrame();

        quit = sdlEventHandler();
    }

    vkDeviceWaitIdle(device);
}

bool VulkanWindow::sdlEventHandler() {
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

        ImGui_ImplSDL2_ProcessEvent(&sdlEvent);

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


void VulkanWindow::setupDispatchLoader() const {
    vk::DynamicLoader dl;

    PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr =
            dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
    VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);
}


void VulkanWindow::createInstance() {
    if (enableValidationLayers && !checkValidationLayerSupport()) {
        throw std::runtime_error("validation layers requested, but not available!");
    }

    const char *name = "Hello Triangle";
    const char *engineName = "No Engine";
    vk::ApplicationInfo appInfo(name, VK_MAKE_VERSION(1, 0, 0), engineName,
                                VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_0);


    auto extensions = getRequiredInstanceExtensions();
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

void VulkanWindow::populateDebugMessengerCreateInfo(vk::DebugUtilsMessengerCreateInfoEXT &createInfo) {
    vk::DebugUtilsMessageSeverityFlagsEXT messageSeverity = vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose |
                                                            vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
                                                            vk::DebugUtilsMessageSeverityFlagBitsEXT::eError;
    vk::DebugUtilsMessageTypeFlagsEXT messageType = vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
                                                    vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
                                                    vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance;

    createInfo = vk::DebugUtilsMessengerCreateInfoEXT({}, messageSeverity, messageType, debugCallback, {});
}

void VulkanWindow::setupDebugMessenger() {
    vk::DebugUtilsMessengerCreateInfoEXT createInfo;
    populateDebugMessengerCreateInfo(createInfo);

    debugMessenger = instance.createDebugUtilsMessengerEXT(createInfo);
}

void VulkanWindow::createSurface() {
    VkSurfaceKHR c_surface;

    if (SDL_Vulkan_CreateSurface(window, static_cast<VkInstance>(instance), &c_surface) != SDL_TRUE) {
        std::string msg = "SDL create surface failed with error: ";
        msg += SDL_GetError();
        throw std::runtime_error(msg);
    }

    surface = c_surface;
}

void VulkanWindow::pickPhysicalDevice() {
    std::vector<vk::PhysicalDevice> devices = instance.enumeratePhysicalDevices();

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

    queueFamilyIndices = findQueueFamilies(physicalDevice);
}

void VulkanWindow::createLogicalDevice() {
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


    vk::PhysicalDeviceRayTracingFeaturesKHR rtFeatures;
    rtFeatures.rayTracing = true;

    vk::PhysicalDeviceDescriptorIndexingFeatures diFeatures;
    diFeatures.runtimeDescriptorArray = true;

    vk::PhysicalDeviceFeatures deviceFeatures = {};
    deviceFeatures.samplerAnisotropy = VK_TRUE;
    deviceFeatures.robustBufferAccess = false;

    vk::PhysicalDeviceFeatures2 deviceFeatures2(deviceFeatures);
    deviceFeatures2.setPNext(&addresFeatures);
    addresFeatures.setPNext(&rtFeatures);
    rtFeatures.setPNext(&diFeatures);

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

bool VulkanWindow::isDeviceSuitable(vk::PhysicalDevice physicalDevice) {
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

bool VulkanWindow::checkDeviceExtensionSupport(vk::PhysicalDevice physicalDevice) {

    std::vector<vk::ExtensionProperties> availableExtensions = physicalDevice.enumerateDeviceExtensionProperties();

    std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

    for (const auto &extension : availableExtensions) {
        requiredExtensions.erase(extension.extensionName);
    }

    return requiredExtensions.empty();
}

std::vector<const char *> VulkanWindow::getRequiredInstanceExtensions() {
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

bool VulkanWindow::checkValidationLayerSupport() {
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

void VulkanWindow::createSwapChain() {
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


vk::SurfaceFormatKHR
VulkanWindow::chooseSwapSurfaceFormat(const std::vector<vk::SurfaceFormatKHR> &availableFormats) {
    for (const auto &availableFormat : availableFormats) {
        if (availableFormat.format == vk::Format::eB8G8R8A8Srgb &&
            availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
            return availableFormat;
        }
    }

    return availableFormats[0];
}

vk::PresentModeKHR
VulkanWindow::chooseSwapPresentMode(const std::vector<vk::PresentModeKHR> &availablePresentModes) {
    for (const auto &availablePresentMode : availablePresentModes) {
        if (availablePresentMode == vk::PresentModeKHR::eImmediate) {
            return availablePresentMode;
        }
    }

    return vk::PresentModeKHR::eFifo;
}

vk::Extent2D VulkanWindow::chooseSwapExtent(const vk::SurfaceCapabilitiesKHR &capabilities) {
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

void VulkanWindow::createImageViews() {
    swapChainImageViews.resize(swapChainImages.size());

    for (size_t i = 0; i < swapChainImages.size(); i++) {
        swapChainImageViews[i] = vulkanOps->createImageView(swapChainImages[i], swapChainImageFormat,
                                                           vk::ImageAspectFlagBits::eColor);
    }
}

void VulkanWindow::createRenderPass() {
    vk::AttachmentDescription colorAttachment({}, swapChainImageFormat, vk::SampleCountFlagBits::e1,
                                              vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore,
                                              vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare,
                                              vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorAttachmentRef(0, vk::ImageLayout::eColorAttachmentOptimal);

    vk::SubpassDescription subpass({}, vk::PipelineBindPoint::eGraphics,
                                   {}, {}, 1, &colorAttachmentRef,
                                   {}, nullptr, {}, {});

    vk::SubpassDependency dependency(VK_SUBPASS_EXTERNAL, 0, vk::PipelineStageFlagBits::eColorAttachmentOutput,
                                     vk::PipelineStageFlagBits::eColorAttachmentOutput, {},
                                     vk::AccessFlagBits::eColorAttachmentWrite, {});

    std::array<vk::AttachmentDescription, 1> attachments = {colorAttachment};

    vk::RenderPassCreateInfo renderPassInfo({}, static_cast<uint32_t>(attachments.size()), attachments.data(),
                                            1, &subpass, 1, &dependency);

    renderPass = device.createRenderPass(renderPassInfo);
}

void VulkanWindow::createFramebuffers() {
    swapChainFramebuffers.resize(swapChainImageViews.size());

    for (size_t i = 0; i < swapChainImageViews.size(); i++) {
        std::array<vk::ImageView, 1> attachments = {
                swapChainImageViews[i]
        };

        vk::FramebufferCreateInfo framebufferInfo({}, renderPass,
                                                  static_cast<uint32_t>(attachments.size()), attachments.data(),
                                                  swapChainExtent.width, swapChainExtent.height, 1);

        swapChainFramebuffers[i] = device.createFramebuffer(framebufferInfo);
    }
}

void VulkanWindow::createCommandPool() {
    QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

    vk::CommandPoolCreateInfo poolInfo(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, queueFamilyIndices.graphicsFamily.value());

    commandPool = device.createCommandPool(poolInfo);
}

void VulkanWindow::createCommandBuffers() {
    commandBuffers.resize(swapChainFramebuffers.size());

    vk::CommandBufferAllocateInfo allocInfo(commandPool, vk::CommandBufferLevel::ePrimary,
                                            static_cast<uint32_t>(swapChainFramebuffers.size()));

    commandBuffers = device.allocateCommandBuffers(allocInfo);


}

void VulkanWindow::createSyncObjects() {
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

QueueFamilyIndices VulkanWindow::findQueueFamilies(vk::PhysicalDevice physicalDevice) {
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

SwapChainSupportDetails
VulkanWindow::querySwapChainSupport(vk::PhysicalDevice physicalDevice) {
    SwapChainSupportDetails details;

    details.capabilities = physicalDevice.getSurfaceCapabilitiesKHR(surface);
    details.formats = physicalDevice.getSurfaceFormatsKHR(surface);
    details.presentModes = physicalDevice.getSurfacePresentModesKHR(surface);

    return details;
}

void VulkanWindow::recreateSwapChain() {
    // Doesn't seem to be necessary, as SDL seems to keep drawable size constant even when minimized
    int width = 0, height = 0;
    SDL_Vulkan_GetDrawableSize(window, &width, &height);
    while (width == 0 || height == 0) {
        SDL_Vulkan_GetDrawableSize(window, &width, &height);
        SDL_WaitEvent(nullptr);
    }

    device.waitIdle();

    cleanupSwapChain();

    createSwapChainDependant();

    recreateSwapchainCallback();
}

void VulkanWindow::createSwapChainDependant() {
    createSwapChain();
    createImageViews();
    createRenderPass();
    createFramebuffers();
    createCommandBuffers();
}

void VulkanWindow::cleanupSwapChain() {
    for (auto swapChainFramebuffer : swapChainFramebuffers) {
        device.destroy(swapChainFramebuffer);
    }

    device.freeCommandBuffers(commandPool, commandBuffers);

    device.destroy(renderPass);

    for (auto swapChainImageView : swapChainImageViews) {
        device.destroy(swapChainImageView);
    }

    device.destroy(swapChain);
}

void VulkanWindow::cleanup() {
    cleanupSwapChain();

    device.destroy(imGuiDescriptorPool);

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        device.destroy(renderFinishedSemaphores[i]);
        device.destroy(imageAvailableSemaphores[i]);
        device.destroy(inFlightFences[i]);
    }

    device.destroy(commandPool);

    device.destroy();

    if (enableValidationLayers) {
        instance.destroyDebugUtilsMessengerEXT(debugMessenger);
    }

    instance.destroy(surface);
    instance.destroy();


    SDL_DestroyWindow(window);
    SDL_Quit();
}

const std::vector<vk::Framebuffer> &VulkanWindow::getSwapChainFramebuffers() const {
    return swapChainFramebuffers;
}

const vk::RenderPass &VulkanWindow::getRenderPass() const {
    return renderPass;
}

const vk::PhysicalDevice &VulkanWindow::getPhysicalDevice() const {
    return physicalDevice;
}

const vk::Device &VulkanWindow::getDevice() const {
    return device;
}

vk::Format VulkanWindow::getSwapChainImageFormat() const {
    return swapChainImageFormat;
}

const vk::Extent2D &VulkanWindow::getSwapChainExtent() const {
    return swapChainExtent;
}

const std::vector<vk::ImageView> &VulkanWindow::getSwapChainImageViews() const {
    return swapChainImageViews;
}

const vk::CommandPool &VulkanWindow::getCommandPool() const {
    return commandPool;
}

const std::vector<vk::CommandBuffer> &VulkanWindow::getCommandBuffers() const {
    return commandBuffers;
}

const std::shared_ptr<VulkanOps> &VulkanWindow::getVulkanOps() const {
    return vulkanOps;
}

const QueueFamilyIndices &VulkanWindow::getQueueFamilyIndices() const {
    return queueFamilyIndices;
}

