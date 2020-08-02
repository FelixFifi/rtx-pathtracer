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
#include "SceneLoader.h"
#include "IrradianceCache.h"

const std::string MATERIAL_BASE_DIR = "materials/";
const std::string TEXTURE_BASE_DIR = "textures/";
const std::vector<std::string> SCENES{
        "scenes/irradianceCache.xml",
        "scenes/matpreview.xml",
        "scenes/materialTest.xml",
        "scenes/testSpheres.xml",
        "scenes/mi.xml",
        "scenes/veach-mis.json",
        "scenes/cornell-dielectric-path.json",
        "scenes/test.json",
        "scenes/cornell.json",
        "scenes/large.json",
        "scenes/fireplace.json"
};

static const int MAX_RECURSION = 2;

static const int ACCUMULATE_IMAGE_BINDING = 9;

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

    struct RtPushConstant {
        glm::vec4 skyColor1 = {0, 0, 0, 0};
        glm::vec4 skyColor2 = {0, 0, 0, 0};
        uint randomUInt;
        int lightType = 0;
        uint previousFrames = -1;
        int maxDepth = 6;
        int samplesPerPixel = 1;
        int enableRR = 0; // GLSL has 4 byte bool
        int enableNEE = 1; // GLSL has 4 byte bool
        int enableAverageInsteadOfMix = 0;
        int enableMIS = 0;
        int showIrradianceCache = 0;
        int showIrradianceCacheOnly = 0;
        int useIrradianceCache = 0;
        int highlightIrradianceCacheColor = 0;
        float irradianceA = 0.5;
        float irradianceUpdateProb = 0.00001;
        float irradianceCreateProb = 0.0001;
        float irradianceVisualizationScale = 10;
        int useVisibleSphereSampling = 1;
    } rtPushConstants;

private:
    PostProcessing postProcessing;
    VulkanWindow vulkanWindow;

    SceneLoader sceneLoader;

    bool autoRotate = false;
    bool accumulateResults = false;
    bool hasInputChanged = false;

    bool takePicture = false;

    CameraController cameraController;
    IrradianceCache irradianceCache;

    // Vulkan
    vk::Buffer uniformBuffer;
    vk::DeviceMemory uniformBufferMemory;

    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    vk::DescriptorSet descriptorSet;


    vk::Image accumulateImage;
    vk::DeviceMemory accumulateImageMemory;
    vk::ImageView accumulateImageView;
    vk::Sampler accumulateImageSampler;

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

    void loadScene();

    void cleanupDescriptorSets();

    void recreateDescriptorSets();

    void initRayTracing();

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

    void createAccumulateImage();

    void sceneSwitcher(int num);

    void cleanupRtPipeline();
};

#endif //RTX_RAYTRACER_RAYTRACINGAPP_H
