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
#include "PathGuiding.h"
#include "SampleCollector.h"

static const int MAX_RECURSION = 2;

static const int ACCUMULATE_IMAGE_BINDING = 9;
static const int ESTIMATE_IMAGE_BINDING = 14;

static const vk::Flags<vk::ShaderStageFlagBits> PUSH_CONSTANT_STAGES = vk::ShaderStageFlagBits::eRaygenKHR
                                                                       | vk::ShaderStageFlagBits::eClosestHitKHR
                                                                       | vk::ShaderStageFlagBits::eMissKHR
                                                                       | vk::ShaderStageFlagBits::eAnyHitKHR
                                                                       | vk::ShaderStageFlagBits::eIntersectionKHR;
enum VisualizationMode {
    ERayTrace = 0,
    EDepthMax = 1,
    EDepthAverage = 2,
    ESplits = 3,
    EEstimate = 4,
    EGuidingRegions = 5,
    EGuidingOverlay = 6,
    EGuidingActiveDistributions = 7,
    EGuidingPiP = 8
};

struct CameraMatrices {
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 viewInverse;
    glm::mat4 projInverse;
};

class RayTracingApp {
public:
    RayTracingApp(uint32_t width, uint32_t height, uint32_t icSize, uint32_t guidingSplits,
                  const std::vector<std::string> &scenes);

    void run();

    void cleanup();

    struct RtPushConstant {
        uint randomUInt;
        uint previousFrames = -1;
        int maxDepth = 6;
        int maxFollowDiscrete = 3;  // TODO: Option to estimate surface after > maxFollow as diffuse
        int samplesPerPixel = 1;
        int enableRR = 0; // GLSL has 4 byte bool
        int enableNEE = 1; // GLSL has 4 byte bool
        int numNEE = 1;
        int enableAverageInsteadOfMix = 0;
        int enableMIS = 0;
        int usePowerHeuristic = 1;
        int storeEstimate = 0;
        int visualizeMode = ERayTrace;
        int showIrradianceCacheOnly = 0;
        int showIrradianceGradients = 0;
        int useIrradianceCache = 0;
        int highlightIrradianceCacheColor = 0;
        float irradianceA = 0.2;
        float irradianceUpdateProb = 0.00001;
        float irradianceCreateProb = 0.0001;
        float irradianceVisualizationScale = 1.0;
        int useIrradianceGradients = 0;
        int useIrradianceCacheOnGlossy = 0;
        float irradianceGradientsMaxLength = 5;
        int isIrradiancePrepareFrame = 0;
        int irradianceNumNEE = 1;
        float irradianceCacheMinRadius = 0.1;
        int irradianceCachePerformVisibilityCheck = 1;
        int useVisibleSphereSampling = 0; // WIP/never finished
        int useADRRS = 0;
        float adrrsS = 5;
        int adrrsSplit = 1;
        int splitOnFirst = 0;
        int useGuiding = 0;
        float guidingProb = 0.5;
        float guidingVisuScale = 0.5;
        float guidingVisuMax = 1.0;
        int guidingVisuIgnoreOcclusioon = 0;
        int updateGuiding = 0;
        int useParallaxCompensation = 1;
        float time;
        int guidingVisuMove = 0;
        float guidingVisuPhiScale = 0.004f;
        float guidingVisuThetaScale = 0.01f;
        int numGuidingRegions;
        int guidingPiPHighlightRegion = 0;
        int guidingPiPShowSpheres = 1;
        float guidingPiPSize = 0.3f;
    } rtPushConstants;

private:
    std::vector<std::string> scenes;

    PostProcessing postProcessing;
    VulkanWindow vulkanWindow;

    SceneLoader sceneLoader;

    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

    uint32_t icSize;

    std::string currentScene;

    bool autoRotate = false;
    bool accumulateResults = false;
    bool hasInputChanged = false;
    bool needSceneReload = false;
    int sceneIndex = 0;
    bool showOtherVisualizations = false;
    int currentVisualizeMode = EGuidingOverlay;

    bool takePicture = false;

    CameraController cameraController;
    IrradianceCache irradianceCache;
    int irradianceCachePrepareFrames = 10;
    int currentPrepareFrames = 0;

    PathGuiding guiding;
    bool enableParallaxCompensationForOptimization = true;
    int numGuidingOptimizations = 6;
    int currentGuidingOptimizations = -1;
    uint guidingSplits;
    SampleCollector sampleCollector;

    RtPushConstant backupPushConstant;
    bool loadBackupNextIteration = false;

    // Vulkan
    vk::Buffer uniformBuffer;
    vk::DeviceMemory uniformBufferMemory;

    vk::DescriptorSetLayout descriptorSetLayout;
    vk::DescriptorPool descriptorPool;
    vk::DescriptorSet descriptorSet;


    vk::Image accumulateImage;
    vk::DeviceMemory accumulateImageMemory;
    vk::ImageView accumulateImageView;

    vk::Image estimateImage;
    vk::DeviceMemory estimateImageMemory;
    vk::ImageView estimateImageView;

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

    void createVulkanImages();

    void sceneSwitcher(int index);

    void cleanupRtPipeline();

    void takePictureCurrentTime();

    void setEstimateRTSettings();

    void updateGuidingDescriptorSets();

    std::string getModeString() const;

    void imGuiGuiding();

    void imGuiADRRS();

    void imGuiIC();

    void imGuiGeneral();

    void sceneSwitcherFromKey(int num);

    void imGuiSceneSelection();
};

#endif //RTX_RAYTRACER_RAYTRACINGAPP_H
