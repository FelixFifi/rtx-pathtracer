//
// Created by felixfifi on 06.05.20.
//

#include <imgui/imgui.h>
#include <glm/gtc/random.hpp>
#include <random>
#include <filesystem>
#include "RayTracingApp.h"

RayTracingApp::RayTracingApp(uint32_t width, uint32_t height, uint32_t icSize, uint32_t guidingSplits,
                             const std::vector<std::string> &scenes) : icSize(icSize),
                                                                       guidingSplits(guidingSplits),
                                                                       scenes(scenes){
    startTime = std::chrono::high_resolution_clock::now();
    fDrawCallback drawFunc = [this](uint32_t imageIndex) { drawCallback(imageIndex); };
    fRecreateSwapchainCallback recreateSwapchainFunc = [this] { recreateSwapchainCallback(); };

    vulkanWindow = VulkanWindow(width, height, drawFunc, recreateSwapchainFunc);
    postProcessing = PostProcessing({width, height}, vulkanWindow);

    float aspectRatio = width / (float) height;
    cameraController = CameraController(glm::vec3(0, 2, 15), 360.0f / width, glm::vec3(0, 0.952424, -0.304805),
                                        glm::vec3(0, -0.30478, -0.9524), aspectRatio);
    fEventCallback eventCallback = [this](const SDL_Event &event) { cameraController.eventCallbackSDL(event); };
    vulkanWindow.setEventCallback(eventCallback);
    fNumberKeyEventCallback numberKeyCallback = [this](int key) { sceneSwitcherFromKey(key); };
    vulkanWindow.setNumberKeyEventCallback(numberKeyCallback);

    fImGuiCallback callbackImGui = [this] { imGuiWindowSetup(); };
    postProcessing.addImGuiCallback(callbackImGui);

    device = vulkanWindow.getDevice();
    physicalDevice = vulkanWindow.getPhysicalDevice();
    vulkanOps = vulkanWindow.getVulkanOps();

    offscreenExtent = postProcessing.getExtentOffscreen();

    uint32_t regionCount = 1 << guidingSplits;
    sampleCollector = SampleCollector(regionCount, offscreenExtent, vulkanOps);

    createVulkanImages();
    createUniformBuffers();
    sceneSwitcher(0);

    initRayTracing();
}

void RayTracingApp::createVulkanImages() {
    // Accumulate image
    vk::Format format = vk::Format::eR32G32B32A32Sfloat;
    vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage;
    vulkanOps->createImage(offscreenExtent.width, offscreenExtent.height, format, vk::ImageTiling::eOptimal, usage,
                           vk::MemoryPropertyFlagBits::eDeviceLocal, accumulateImage, accumulateImageMemory);

    accumulateImageView = vulkanOps->createImageView(accumulateImage, format, vk::ImageAspectFlagBits::eColor);

    vulkanOps->transitionImageLayout(accumulateImage, format, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral);

    // Estimate image
    vulkanOps->createImage(offscreenExtent.width, offscreenExtent.height, format, vk::ImageTiling::eOptimal, usage,
                           vk::MemoryPropertyFlagBits::eDeviceLocal, estimateImage, estimateImageMemory);

    estimateImageView = vulkanOps->createImageView(estimateImage, format, vk::ImageAspectFlagBits::eColor);

    vulkanOps->transitionImageLayout(estimateImage, format, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral);
}

void RayTracingApp::run() {
    vulkanWindow.run();
}

void RayTracingApp::drawCallback(uint32_t imageIndex) {
    if (needSceneReload) {
        sceneSwitcher(sceneIndex);
    }

    updateUniformBuffer(imageIndex);

    vk::CommandBuffer cmdBuf = vulkanOps->beginSingleTimeCommands();

    raytrace(cmdBuf);
    vulkanOps->endSingleTimeCommands(cmdBuf, nullptr);

    device.waitIdle();

    if (rtPushConstants.useIrradianceCache && (rtPushConstants.irradianceUpdateProb > 0  || rtPushConstants.irradianceCreateProb > 0)) {
        if (irradianceCache.updateSpheres(false)) {
            updateIrradianceCacheASDescriptorSets();
        }
    }

    cameraController.resetStatus();

    // Limit number of guiding optimizations
    if (rtPushConstants.updateGuiding) {
        if (currentGuidingOptimizations == -1) {
            currentGuidingOptimizations = 0;
        } else {
            currentGuidingOptimizations++;
        }

        if (numGuidingOptimizations != -1 && currentGuidingOptimizations >= numGuidingOptimizations) {
            rtPushConstants.updateGuiding = 0;
            currentGuidingOptimizations = -1;
        }
    } else {
        currentGuidingOptimizations = -1;
    }


    if (rtPushConstants.updateGuiding > 0) {
        if (guiding.update(sampleCollector)) {
            // Region split occurred
            // => update sampleCollector and Descriptor sets
            sampleCollector.cleanup();
            sampleCollector = SampleCollector(guiding.getRegionCount(), offscreenExtent, vulkanOps);


            updateGuidingDescriptorSets();
        }
    }

    // TODO: Fences
    device.waitIdle();

    if (takePicture) {

        takePictureCurrentTime();

        takePicture = false;
    }

    postProcessing.drawCallback(imageIndex);
}

void RayTracingApp::updateGuidingDescriptorSets() {
    vk::DescriptorBufferInfo guidingBufferInfo;
    vk::DescriptorBufferInfo guidingAabbsBufferInfo;
    vk::WriteDescriptorSetAccelerationStructureKHR guidingAsInfo;
    vk::DescriptorBufferInfo directionalDataBufferInfo;


    auto guidingWrites = guiding.getWriteDescriptorSets(descriptorSet, guidingAsInfo,
                                                        guidingAabbsBufferInfo,
                                                        guidingBufferInfo);

    auto sampleCollectorWrites = sampleCollector.getWriteDescriptorSets(descriptorSet, directionalDataBufferInfo);

    std::array<vk::WriteDescriptorSet, 4> writes{
            guidingWrites[0],
            guidingWrites[1],
            guidingWrites[2],
            sampleCollectorWrites[0]
    };

    device.updateDescriptorSets(writes, nullptr);
}

void RayTracingApp::updateIrradianceCacheASDescriptorSets() {
    vk::WriteDescriptorSetAccelerationStructureKHR icAsInfo;

    auto icASWrite = irradianceCache.getASWriteDescriptorSet(descriptorSet, icAsInfo);

    std::array<vk::WriteDescriptorSet, 1> writes{
            icASWrite
    };

    device.updateDescriptorSets(writes, nullptr);
}

void RayTracingApp::takePictureCurrentTime() {
    time_t t = time(nullptr);   // get time now
    tm *now = localtime(&t);

    std::filesystem::create_directories("images/");

    char *timestring = new char[128];
    strftime(timestring, 128, "images/%F--%H-%M-%S", now);

    std::string mode = getModeString();
    std::string scene = std::filesystem::path(currentScene).filename().replace_extension("");
    std::string separator = "_";

    const std::string filepath =
            timestring + separator + scene + mode + ".exr";

    postProcessing.saveOffscreenImage(filepath);
    std::cout << "Wrote file " << filepath << std::endl;
}

std::string RayTracingApp::getModeString() const {
    std::string mode = "";

    if (rtPushConstants.enableNEE) {
        mode += "_NEE";

        if (rtPushConstants.enableMIS) {
            mode += "_MIS";
        }
    }


    if (rtPushConstants.useIrradianceCache) {
        mode += "_IC";
    }

    if (rtPushConstants.useADRRS) {
        mode += "_ADRRS";
    }

    if (rtPushConstants.useGuiding) {
        mode += "_Guiding";

        if (rtPushConstants.useParallaxCompensation) {
            mode += "_Parallax";
        }
    }
    return mode;
}

void RayTracingApp::sceneSwitcherFromKey(int num) {
    // 0 should load the 10th scene
    if (num == 0) {
        sceneSwitcher(9);
    } else {
        sceneSwitcher(num - 1);
    }
}

void RayTracingApp::sceneSwitcher(int index) {
    needSceneReload = false;

    // Don't load out of bounds
    int sceneCount = scenes.size();
    sceneIndex = std::min(index, sceneCount - 1);

    sceneLoader.cleanup();

    uint32_t graphicsQueueIndex = vulkanWindow.getQueueFamilyIndices().graphicsFamily.value();
    sceneLoader = SceneLoader(scenes[sceneIndex], vulkanOps,
                              physicalDevice,
                              graphicsQueueIndex);

    currentScene = scenes[sceneIndex];

    cameraController.vfov = sceneLoader.vfov;
    cameraController.lookAt(sceneLoader.origin, sceneLoader.target, sceneLoader.upDir);

    irradianceCache.cleanUp();
    irradianceCache = IrradianceCache(icSize, vulkanOps, physicalDevice, graphicsQueueIndex);
    currentPrepareFrames = 0;

    guiding.cleanup();
    guiding = PathGuiding(guidingSplits, sceneLoader.getSceneSize(), enableParallaxCompensationForOptimization,
                          vulkanOps, physicalDevice,
                          graphicsQueueIndex);

    recreateDescriptorSets();

    if (rtDescSet) {
        updateRtDescriptorSet(0);
    }

    if (rtPipeline) {
        cleanupRtPipeline();
        initRayTracing();
    }

    hasInputChanged = true;
}

void RayTracingApp::recreateDescriptorSets() {
    if (descriptorPool) {
        cleanupDescriptorSets();
    }

    createDecriptorSetLayout();
    createDescriptorPool();
    createDescriptorSets();
}

void RayTracingApp::cleanupDescriptorSets() {
    device.destroy(descriptorPool);
    device.destroy(descriptorSetLayout);
}

void RayTracingApp::recreateSwapchainCallback() {
    postProcessing.recreateSwapChainCallback();
}

void RayTracingApp::createUniformBuffers() {
    vk::DeviceSize bufferSize = sizeof(CameraMatrices);

    vulkanOps->createBuffer(bufferSize, vk::BufferUsageFlagBits::eUniformBuffer,
                            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                            uniformBuffer, uniformBufferMemory);
}

void RayTracingApp::createDecriptorSetLayout() {
    vk::DescriptorSetLayoutBinding uniformBufferLayoutBinding(0, vk::DescriptorType::eUniformBuffer,
                                                              1, vk::ShaderStageFlagBits::eRaygenKHR,
                                                              nullptr);

    vk::DescriptorSetLayoutBinding accumulateImageLayoutBinding(ACCUMULATE_IMAGE_BINDING,
                                                                vk::DescriptorType::eStorageImage, 1,
                                                                vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding estimateImageLayoutBinding(ESTIMATE_IMAGE_BINDING,
                                                              vk::DescriptorType::eStorageImage, 1,
                                                              vk::ShaderStageFlagBits::eRaygenKHR);


    auto sceneBindings = sceneLoader.getDescriptorSetLayouts();
    auto irradianceBindings = irradianceCache.getDescriptorSetLayouts();
    auto guidingBindings = guiding.getDescriptorSetLayouts();
    auto sampleCollectorBindings = sampleCollector.getDescriptorSetLayouts();
    std::array<vk::DescriptorSetLayoutBinding, ESTIMATE_IMAGE_BINDING + 5> bindings = {uniformBufferLayoutBinding,
                                                                                       sceneBindings[0],
                                                                                       sceneBindings[1],
                                                                                       sceneBindings[2],
                                                                                       sceneBindings[3],
                                                                                       sceneBindings[4],
                                                                                       sceneBindings[5],
                                                                                       sceneBindings[6],
                                                                                       sceneBindings[7],
                                                                                       accumulateImageLayoutBinding,
                                                                                       irradianceBindings[0],
                                                                                       irradianceBindings[1],
                                                                                       irradianceBindings[2],
                                                                                       irradianceBindings[3],
                                                                                       estimateImageLayoutBinding,
                                                                                       guidingBindings[0],
                                                                                       guidingBindings[1],
                                                                                       guidingBindings[2],
                                                                                       sampleCollectorBindings[0]};
    vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());


    descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);
}

void RayTracingApp::createDescriptorPool() {
    auto vertexIndexMaterialPoolSizes = sceneLoader.getDescriptorPoolSizes();
    auto irradiancePoolSizes = irradianceCache.getDescriptorPoolSizes();
    auto guidingPoolSizes = PathGuiding::getDescriptorPoolSizes();
    auto sampleCollectorPoolSizes = SampleCollector::getDescriptorPoolSizes();

    std::array<vk::DescriptorPoolSize, ESTIMATE_IMAGE_BINDING + 5> poolSizes = {
            vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer,
                                   static_cast<uint32_t>(1)),
            vertexIndexMaterialPoolSizes[0],
            vertexIndexMaterialPoolSizes[1],
            vertexIndexMaterialPoolSizes[2],
            vertexIndexMaterialPoolSizes[3],
            vertexIndexMaterialPoolSizes[4],
            vertexIndexMaterialPoolSizes[5],
            vertexIndexMaterialPoolSizes[6],
            vertexIndexMaterialPoolSizes[7],
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1),
            irradiancePoolSizes[0],
            irradiancePoolSizes[1],
            irradiancePoolSizes[2],
            irradiancePoolSizes[3],
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1),
            guidingPoolSizes[0],
            guidingPoolSizes[1],
            guidingPoolSizes[2],
            sampleCollectorPoolSizes[0]
    }; // TODO: One per frame

    vk::DescriptorPoolCreateInfo poolInfo({}, static_cast<uint32_t>(1),
                                          static_cast<uint32_t>(poolSizes.size()), poolSizes.data());

    descriptorPool = device.createDescriptorPool(poolInfo);
}

void RayTracingApp::createDescriptorSets() {
    std::vector<vk::DescriptorSetLayout> layouts(static_cast<uint32_t>(1), // TODO: One per frame
                                                 descriptorSetLayout);
    vk::DescriptorSetAllocateInfo allocInfo(descriptorPool, static_cast<uint32_t>(1),
                                            layouts.data());

    descriptorSet = device.allocateDescriptorSets(allocInfo)[0];

    vk::DescriptorBufferInfo bufferInfo(uniformBuffer, 0, sizeof(CameraMatrices));
    vk::DescriptorImageInfo accumulateImageInfo({}, accumulateImageView,
                                                vk::ImageLayout::eGeneral);
    vk::DescriptorImageInfo estimateImageInfo({}, estimateImageView,
                                              vk::ImageLayout::eGeneral);


    std::vector<vk::DescriptorBufferInfo> vertexBufferInfos;
    std::vector<vk::DescriptorBufferInfo> indexBufferInfos;
    vk::DescriptorBufferInfo materialBufferInfo;
    vk::DescriptorBufferInfo instanceInfoBufferInfo;
    vk::DescriptorBufferInfo lightBufferInfo;
    vk::DescriptorBufferInfo lightSamplersBufferInfo;
    std::vector<vk::DescriptorImageInfo> textureInfos;
    vk::DescriptorBufferInfo spheresBufferInfo;
    vk::DescriptorBufferInfo irradianceSpheresBufferInfo;
    vk::DescriptorBufferInfo irradianceCacheBufferInfo;
    vk::DescriptorBufferInfo irradianceAabbsBufferInfo;
    vk::WriteDescriptorSetAccelerationStructureKHR asInfo;
    vk::DescriptorBufferInfo guidingBufferInfo;
    vk::DescriptorBufferInfo guidingAabbsBufferInfo;
    vk::WriteDescriptorSetAccelerationStructureKHR guidingAsInfo;
    vk::DescriptorBufferInfo directionalDataBufferInfo;

    auto sceneWrites = sceneLoader.getWriteDescriptorSets(descriptorSet, vertexBufferInfos,
                                                          indexBufferInfos, materialBufferInfo,
                                                          instanceInfoBufferInfo, lightBufferInfo,
                                                          lightSamplersBufferInfo, textureInfos,
                                                          spheresBufferInfo);
    auto irradianceWrites = irradianceCache.getWriteDescriptorSets(descriptorSet, asInfo,
                                                                   irradianceSpheresBufferInfo,
                                                                   irradianceCacheBufferInfo,
                                                                   irradianceAabbsBufferInfo);
    auto guidingWrites = guiding.getWriteDescriptorSets(descriptorSet, guidingAsInfo,
                                                        guidingAabbsBufferInfo,
                                                        guidingBufferInfo);
    auto sampleCollectorWrites = sampleCollector.getWriteDescriptorSets(descriptorSet, directionalDataBufferInfo);

    const vk::WriteDescriptorSet accumulateImageWrite = vk::WriteDescriptorSet(descriptorSet, ACCUMULATE_IMAGE_BINDING,
                                                                               0,
                                                                               1,
                                                                               vk::DescriptorType::eStorageImage,
                                                                               &accumulateImageInfo,
                                                                               nullptr, nullptr);

    const vk::WriteDescriptorSet estimateImageWrite = vk::WriteDescriptorSet(descriptorSet, ESTIMATE_IMAGE_BINDING,
                                                                             0,
                                                                             1,
                                                                             vk::DescriptorType::eStorageImage,
                                                                             &estimateImageInfo,
                                                                             nullptr, nullptr);
    std::array<vk::WriteDescriptorSet, ESTIMATE_IMAGE_BINDING + 5> descriptorWrites = {
            vk::WriteDescriptorSet(descriptorSet, 0, 0, 1,
                                   vk::DescriptorType::eUniformBuffer, nullptr,
                                   &bufferInfo, nullptr),
            sceneWrites[0],
            sceneWrites[1],
            sceneWrites[2],
            sceneWrites[3],
            sceneWrites[4],
            sceneWrites[5],
            sceneWrites[6],
            sceneWrites[7],
            accumulateImageWrite,
            irradianceWrites[0],
            irradianceWrites[1],
            irradianceWrites[2],
            irradianceWrites[3],
            estimateImageWrite,
            guidingWrites[0],
            guidingWrites[1],
            guidingWrites[2],
            sampleCollectorWrites[0]
    };

    device.updateDescriptorSets(descriptorWrites, nullptr);
}

void RayTracingApp::updateUniformBuffer(uint32_t currentImage) {
    static auto startTime = std::chrono::high_resolution_clock::now();

    CameraMatrices ubo = {};

    if (autoRotate) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

        float alpha = time;
        float z = 10.0f + 10.0f * glm::sin(alpha / 3.0f);

        ubo.view = glm::lookAt(glm::vec3(30 * glm::sin(alpha), 30 * glm::cos(alpha), z),
                               glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    } else {
        ubo.view = cameraController.getViewMatrix();
    }
    ubo.proj = cameraController.getProjMatrix();

    ubo.projInverse = glm::inverse(ubo.proj);
    ubo.viewInverse = glm::inverse(ubo.view);

    void *data;
    device.mapMemory(uniformBufferMemory, 0, sizeof(ubo), {}, &data);
    memcpy(data, &ubo, sizeof(ubo));
    device.unmapMemory(uniformBufferMemory);
}


void RayTracingApp::initRayTracing() {
    auto properties = physicalDevice.getProperties2<vk::PhysicalDeviceProperties2, vk::PhysicalDeviceRayTracingPropertiesKHR>();
    rtProperties = properties.get<vk::PhysicalDeviceRayTracingPropertiesKHR>();

    createRtDescriptorSet();
    updateRtDescriptorSet(0);
    createRtPipeline();
    createRtShaderBindingTable();
}

void RayTracingApp::createRtDescriptorSet() {
    using vkDT = vk::DescriptorType;
    using vkSS = vk::ShaderStageFlagBits;
    using vkDSLB = vk::DescriptorSetLayoutBinding;

    rtDescSetLayoutBind.emplace_back(
            vkDSLB(0, vkDT::eAccelerationStructureKHR, 1, vkSS::eRaygenKHR | vkSS::eClosestHitKHR)); // TLAS
    rtDescSetLayoutBind.emplace_back(vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR)); // Output image

    rtDescPool = nvvkpp::util::createDescriptorPool(device, rtDescSetLayoutBind);
    rtDescSetLayout = nvvkpp::util::createDescriptorSetLayout(device, rtDescSetLayoutBind);
    rtDescSet = device.allocateDescriptorSets({rtDescPool, 1, &rtDescSetLayout})[0];

    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&sceneLoader.getAccelerationStructure());
    vk::DescriptorImageInfo imageInfo{
            {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral}; // TODO: Update each frame

    std::array<vk::WriteDescriptorSet, 2> writes;
    writes[0] = vk::WriteDescriptorSet(rtDescSet, 0, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&descASInfo);

    writes[1] = vk::WriteDescriptorSet(rtDescSet, 1, 0, 1, vk::DescriptorType::eStorageImage, &imageInfo);

    device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void RayTracingApp::updateRtDescriptorSet(uint32_t currentImage) {
    using vkDT = vk::DescriptorType;

    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&sceneLoader.getAccelerationStructure());

    // (1) Output buffer
    vk::DescriptorImageInfo imageInfo{
            {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet storageImageWrite{rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};


    std::array<vk::WriteDescriptorSet, 2> writes;
    writes[0] = vk::WriteDescriptorSet(rtDescSet, 0, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&descASInfo);

    writes[1] = storageImageWrite;
    device.updateDescriptorSets(writes, nullptr);
}

void RayTracingApp::createRtPipeline() {
    auto raygenCode = readFile("shaders/raytrace.rgen.spv");
    auto missCode = readFile("shaders/raytrace.rmiss.spv");
    auto chitCode = readFile("shaders/raytrace.rchit.spv");
    auto ahitCode = readFile("shaders/raytrace.rahit.spv");
    auto shadowMissCode = readFile("shaders/raytrace.shadow.rmiss.spv");
    auto sphereIntCode = readFile("shaders/raytrace.sphere.rint.spv");
    auto sphereChitCode = readFile("shaders/raytrace.sphere.rchit.spv");
    auto irradianceIntCode = readFile("shaders/raytrace.irradiance.rint.spv");
    auto irradianceAhitCode = readFile("shaders/raytrace.irradiance.rahit.spv");
    auto irradianceVisualizeAhitCode = readFile("shaders/raytrace.irradiance.visualize.rahit.spv");
    auto guidingChitCode = readFile("shaders/raytrace.guiding.rchit.spv");
    auto guidingIntCode = readFile("shaders/raytrace.guiding.rint.spv");
    auto guidingVisualizeChitCode = readFile("shaders/raytrace.guiding.visualize.rchit.spv");
    auto guidingVisualizeIntCode = readFile("shaders/raytrace.guiding.visualize.rint.spv");
    auto guidingVisualizeAhitCode = readFile("shaders/raytrace.guiding.visualize.rahit.spv");

    vk::ShaderModule raygenShaderModule = vulkanOps->createShaderModule(raygenCode);
    vk::ShaderModule missShaderModule = vulkanOps->createShaderModule(missCode);
    vk::ShaderModule chitShaderModule = vulkanOps->createShaderModule(chitCode);
    vk::ShaderModule ahitShaderModule = vulkanOps->createShaderModule(ahitCode);
    vk::ShaderModule shadowMissShaderModule = vulkanOps->createShaderModule(shadowMissCode);
    vk::ShaderModule sphereIntShaderModule = vulkanOps->createShaderModule(sphereIntCode);
    vk::ShaderModule sphereChitShaderModule = vulkanOps->createShaderModule(sphereChitCode);
    vk::ShaderModule irradianceIntShaderModule = vulkanOps->createShaderModule(irradianceIntCode);
    vk::ShaderModule irradianceAhitShaderModule = vulkanOps->createShaderModule(irradianceAhitCode);
    vk::ShaderModule irradianceVisualizeAhitShaderModule = vulkanOps->createShaderModule(irradianceVisualizeAhitCode);
    vk::ShaderModule guidingChitShaderModule = vulkanOps->createShaderModule(guidingChitCode);
    vk::ShaderModule guidingIntShaderModule = vulkanOps->createShaderModule(guidingIntCode);
    vk::ShaderModule guidingVisualizeChitShaderModule = vulkanOps->createShaderModule(guidingVisualizeChitCode);
    vk::ShaderModule guidingVisualizeIntShaderModule = vulkanOps->createShaderModule(guidingVisualizeIntCode);
    vk::ShaderModule guidingVisualizeAhitShaderModule = vulkanOps->createShaderModule(guidingVisualizeAhitCode);

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


    // Miss shadow
    vk::RayTracingShaderGroupCreateInfoKHR ms{vk::RayTracingShaderGroupTypeKHR::eGeneral,
                                              VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                              VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eMissKHR, shadowMissShaderModule, "main"});
    ms.setGeneralShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(ms);

    // Hit Group 0 - Closest Hit + AnyHit

    vk::RayTracingShaderGroupCreateInfoKHR hg0{vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, chitShaderModule, "main"});
    hg0.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eAnyHitKHR, ahitShaderModule, "main"});
    hg0.setAnyHitShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg0);

    // Hit Group 1 - Intersection + Closest Hit
    vk::RayTracingShaderGroupCreateInfoKHR hg1{vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, sphereChitShaderModule, "main"});
    hg1.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, sphereIntShaderModule, "main"});
    hg1.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg1);

    // Hit Group 2 - Irradiance Intersection + Closest Hit
    vk::RayTracingShaderGroupCreateInfoKHR hg2{vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eAnyHitKHR, irradianceAhitShaderModule, "main"});
    hg2.setAnyHitShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, irradianceIntShaderModule, "main"});
    hg2.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg2);

    // Hit Group 3 - Irradiance Any hit
    vk::RayTracingShaderGroupCreateInfoKHR hg3{vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eAnyHitKHR, irradianceVisualizeAhitShaderModule, "main"});
    hg3.setAnyHitShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, irradianceIntShaderModule, "main"});
    hg3.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg3);

    // Hit Group 4 - Guiding
    vk::RayTracingShaderGroupCreateInfoKHR hg4{vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, guidingIntShaderModule, "main"});
    hg4.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, guidingChitShaderModule, "main"});
    hg4.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg4);

    // Hit Group 5 - Guiding Visualize
    vk::RayTracingShaderGroupCreateInfoKHR hg5{vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR,
                                               VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    stages.push_back({{}, vk::ShaderStageFlagBits::eIntersectionKHR, guidingVisualizeIntShaderModule, "main"});
    hg5.setIntersectionShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eClosestHitKHR, guidingVisualizeChitShaderModule, "main"});
    hg5.setClosestHitShader(static_cast<uint32_t>(stages.size() - 1));

    stages.push_back({{}, vk::ShaderStageFlagBits::eAnyHitKHR, guidingVisualizeAhitShaderModule, "main"});
    hg5.setAnyHitShader(static_cast<uint32_t>(stages.size() - 1));
    rtShaderGroups.push_back(hg5);


    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;

    // Push constant: we want to be able to update constants used by the shaders
    vk::PushConstantRange pushConstant{PUSH_CONSTANT_STAGES,
                                       0, sizeof(RtPushConstant)};
    pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
    pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstant);

    // Descriptor sets
    std::vector<vk::DescriptorSetLayout> rtDescSetLayouts = {rtDescSetLayout, descriptorSetLayout};
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

    rayPipelineInfo.setMaxRecursionDepth(MAX_RECURSION);  // Ray depth
    rayPipelineInfo.setLayout(rtPipelineLayout);
    rtPipeline = device.createRayTracingPipelineKHR({}, rayPipelineInfo).value;

    device.destroy(raygenShaderModule);
    device.destroy(missShaderModule);
    device.destroy(chitShaderModule);
    device.destroy(ahitShaderModule);
    device.destroy(shadowMissShaderModule);
    device.destroy(sphereIntShaderModule);
    device.destroy(sphereChitShaderModule);
    device.destroy(irradianceIntShaderModule);
    device.destroy(irradianceAhitShaderModule);
    device.destroy(irradianceVisualizeAhitShaderModule);
    device.destroy(guidingIntShaderModule);
    device.destroy(guidingChitShaderModule);
    device.destroy(guidingVisualizeIntShaderModule);
    device.destroy(guidingVisualizeChitShaderModule);
    device.destroy(guidingVisualizeAhitShaderModule);
}

void RayTracingApp::createRtShaderBindingTable() {
    auto groupCount =
            static_cast<uint32_t>(rtShaderGroups.size());
    uint32_t groupHandleSize = rtProperties.shaderGroupHandleSize;  // Size of a program identifier
    uint32_t baseAlignment = rtProperties.shaderGroupBaseAlignment;  // Size of shader alignment

    // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
    uint32_t sbtSize = groupCount * baseAlignment;

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    device.getRayTracingShaderGroupHandlesKHR(rtPipeline, 0, groupCount, sbtSize,
                                              shaderHandleStorage.data());


    // Write the handles in the SBT
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eRayTracingKHR | vk::BufferUsageFlagBits::eTransferDst;

    vk::Buffer stagingBuffer;
    vk::DeviceMemory stagingBufferMemory;

    vulkanOps->createBuffer(sbtSize, vk::BufferUsageFlagBits::eTransferSrc,
                            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                            stagingBuffer, stagingBufferMemory);

    uint8_t *data;
    data = reinterpret_cast<uint8_t *>(device.mapMemory(stagingBufferMemory, 0, sbtSize));

    // Write the handles in the SBT
    for (uint32_t g = 0; g < groupCount; g++) {
        memcpy(data, shaderHandleStorage.data() + g * groupHandleSize, groupHandleSize);  // raygen
        data += baseAlignment;
    }

    device.unmapMemory(stagingBufferMemory);

    vulkanOps->createBuffer(sbtSize, usage,
                            vk::MemoryPropertyFlagBits::eDeviceLocal, rtSBTBuffer, rtSBTBufferMemory);

    vulkanOps->copyBuffer(stagingBuffer, rtSBTBuffer, sbtSize, nullptr);

    device.destroy(stagingBuffer);
    device.freeMemory(stagingBufferMemory);
}


void RayTracingApp::imGuiWindowSetup() {
    imGuiGeneral();
    imGuiIC();
    imGuiADRRS();
    imGuiGuiding();
}

void RayTracingApp::imGuiSceneSelection() {
    ImGui::Text("Scene");
    if (ImGui::BeginCombo(" ", scenes[sceneIndex].c_str())) {

        for (int iScene = 0; iScene < scenes.size(); iScene++) {
            const char *scene = scenes[iScene].c_str();
            if (ImGui::Selectable(scene, iScene == sceneIndex)) {
                sceneIndex = iScene;
                needSceneReload = true;
            }
        }
        ImGui::EndCombo();
    }
}

void RayTracingApp::imGuiGeneral() {
    ImGui::Begin("Raytrace Window");

    imGuiSceneSelection();

    ImGui::PushItemWidth(100.0f);
    hasInputChanged |= ImGui::InputInt("Samples per pixel", &rtPushConstants.samplesPerPixel, 1, 5);
    hasInputChanged |= ImGui::InputInt("Max depth", &rtPushConstants.maxDepth, 1, 5);
    hasInputChanged |= ImGui::InputInt("Max Follow Discrete after max depth", &rtPushConstants.maxFollowDiscrete, 1, 5);
    ImGui::PopItemWidth();

    ImGui::Checkbox("Accumulate results", &accumulateResults);
    ImGui::Spacing();
    hasInputChanged |= ImGui::Checkbox("Russian Roulette", reinterpret_cast<bool *>(&rtPushConstants.enableRR));
    hasInputChanged |= ImGui::Checkbox("Next Event Estimation", reinterpret_cast<bool *>(&rtPushConstants.enableNEE));
    ImGui::PushItemWidth(100.0f);
    hasInputChanged |= ImGui::InputInt("Num NEE", &rtPushConstants.numNEE, 1, 5);
    ImGui::PopItemWidth();
    hasInputChanged |= ImGui::Checkbox("Multiple Importance Sampling (for NEE)",
                                       reinterpret_cast<bool *>(&rtPushConstants.enableMIS));
    hasInputChanged |= ImGui::Checkbox("Use power heuristic, else balance heuristic",
                                       reinterpret_cast<bool *>(&rtPushConstants.usePowerHeuristic));
    ImGui::Spacing();

    ImGui::Checkbox("Auto rotate", &autoRotate);
    ImGui::Spacing();
    hasInputChanged |= ImGui::Checkbox("Enable average instead of mix",
                                       reinterpret_cast<bool *>(&rtPushConstants.enableAverageInsteadOfMix));
    hasInputChanged |= ImGui::Checkbox("Split on first surface",
                                       reinterpret_cast<bool *>(&rtPushConstants.splitOnFirst));

    hasInputChanged |= ImGui::Checkbox("Show:", &showOtherVisualizations);


    bool hasRadioButtonChanged = false;
    hasRadioButtonChanged |= ImGui::RadioButton("Max Depth", &currentVisualizeMode, EDepthMax);
    ImGui::SameLine();
    hasRadioButtonChanged |= ImGui::RadioButton("Average Depth", &currentVisualizeMode, EDepthAverage);
    ImGui::SameLine();
    hasRadioButtonChanged |= ImGui::RadioButton("Splits", &currentVisualizeMode, ESplits);

    hasRadioButtonChanged |= ImGui::RadioButton("Estimate", &currentVisualizeMode, EEstimate);
    ImGui::SameLine();
    hasRadioButtonChanged |= ImGui::RadioButton("Guiding Regions", &currentVisualizeMode, EGuidingRegions);
    ImGui::SameLine();
    hasRadioButtonChanged |= ImGui::RadioButton("Guiding Overlay", &currentVisualizeMode, EGuidingOverlay);

    hasRadioButtonChanged |= ImGui::RadioButton("Guiding Active Components", &currentVisualizeMode,
                                                EGuidingActiveDistributions);

    hasRadioButtonChanged |= ImGui::RadioButton("Guiding PiP of region", &currentVisualizeMode, EGuidingPiP);
    ImGui::SameLine();
    ImGui::PushItemWidth(100.0f);
    ImGui::InputInt("",
                    &rtPushConstants.guidingPiPHighlightRegion, 1, 10);
    ImGui::PopItemWidth();

    rtPushConstants.visualizeMode = ERayTrace;

    if (showOtherVisualizations) {
        rtPushConstants.visualizeMode = currentVisualizeMode;
        hasInputChanged |= hasRadioButtonChanged;
    }

    ImGui::Spacing();

    ImGui::Checkbox("Take picture", &takePicture);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);

    ImGui::End();
}

void RayTracingApp::imGuiIC() {
    ImGui::Begin("Irradiance Cache");
    hasInputChanged |= ImGui::Checkbox("Use Irradiance Cache",
                                       reinterpret_cast<bool *>(&rtPushConstants.useIrradianceCache));
    hasInputChanged |= ImGui::Checkbox("Use Irradiance Cache Gradients",
                                       reinterpret_cast<bool *>(&rtPushConstants.useIrradianceGradients));
    hasInputChanged |= ImGui::Checkbox("Use Irradiance Cache On Glossy",
                                       reinterpret_cast<bool *>(&rtPushConstants.useIrradianceCacheOnGlossy));
    hasInputChanged |= ImGui::SliderFloat("Irradiance a", &rtPushConstants.irradianceA, 0.0f, 2.0f);
    hasInputChanged |= ImGui::Checkbox("Perform Visibility Check",
                                       reinterpret_cast<bool *>(&rtPushConstants.irradianceCachePerformVisibilityCheck));

    ImGui::Spacing();
    bool probUpdated = ImGui::InputFloat("Update prob", &rtPushConstants.irradianceUpdateProb, 0.0001, 0.001,
                      "%.6f");
    probUpdated |= ImGui::InputFloat("Create prob", &rtPushConstants.irradianceCreateProb, 0.0001, 0.001,
                      "%.6f");

    // After updating or creation is disabled, update one last time
    if (probUpdated && rtPushConstants.irradianceUpdateProb <= 0 && rtPushConstants.irradianceCreateProb <= 0) {
        irradianceCache.updateSpheres(true);
        updateIrradianceCacheASDescriptorSets();
    }

    ImGui::Spacing();
    ImGui::InputInt("Prepare frames", &irradianceCachePrepareFrames, 1, 10);
    bool irNumNeeChanged = ImGui::InputInt("Num NEE", &rtPushConstants.irradianceNumNEE, 1, 10);
    needSceneReload |= irradianceCache.wasUpdated() && irNumNeeChanged;
    hasInputChanged |= ImGui::SliderFloat("Gradients max length", &rtPushConstants.irradianceGradientsMaxLength, 0.0f,
                                          50.0f);

    ImGui::Spacing();
    hasInputChanged |= ImGui::Checkbox("Show only IC",
                                       reinterpret_cast<bool *>(&rtPushConstants.showIrradianceCacheOnly));
    hasInputChanged |= ImGui::Checkbox("Show IC Gradients",
                                       reinterpret_cast<bool *>(&rtPushConstants.showIrradianceGradients));
    hasInputChanged |= ImGui::Checkbox("Highlight IC Color",
                                       reinterpret_cast<bool *>(&rtPushConstants.highlightIrradianceCacheColor));
    hasInputChanged |= ImGui::SliderFloat("Visualization scale",
                                          &rtPushConstants.irradianceVisualizationScale, 0.0f, 20.0f);
    hasInputChanged |= ImGui::SliderFloat("IC min sphere radius",
                                          &rtPushConstants.irradianceCacheMinRadius, 0.0f, 1);

    ImGui::End();
}

void RayTracingApp::imGuiADRRS() {
    ImGui::Begin("ADRRS");
    hasInputChanged |= ImGui::Checkbox("Use ADRRS",
                                       reinterpret_cast<bool *>(&rtPushConstants.useADRRS));
    hasInputChanged |= ImGui::InputFloat("Window width ratio",
                                         &rtPushConstants.adrrsS, 1.0f, 5.0f);
    hasInputChanged |= ImGui::Checkbox("Split", reinterpret_cast<bool *>(&rtPushConstants.adrrsSplit));
    hasInputChanged |= ImGui::Checkbox("Estimate image",
                                       reinterpret_cast<bool *>(&rtPushConstants.storeEstimate));
    ImGui::End();
}

void RayTracingApp::imGuiGuiding() {
    ImGui::Begin("Guiding");
    hasInputChanged |= ImGui::Checkbox("Use Guiding",
                                       reinterpret_cast<bool *>(&rtPushConstants.useGuiding));
    hasInputChanged |= ImGui::SliderFloat("Guiding Prob",
                                          &rtPushConstants.guidingProb, 0.0f, 1.0f);
    hasInputChanged |= ImGui::Checkbox("Use Parallax Compensation",
                                       reinterpret_cast<bool *>(&rtPushConstants.useParallaxCompensation));

    ImGui::Checkbox("Enable", &guiding.splitRegions);
    ImGui::SameLine();
    ImGui::PushItemWidth(100.0f);
    ImGui::InputInt("Samples for region split",
                    &guiding.samplesForRegionSplit, 10000.0f, 100000.0f);
    ImGui::PopItemWidth();

    hasInputChanged |= ImGui::Checkbox("Update Guiding",
                                       reinterpret_cast<bool *>(&rtPushConstants.updateGuiding));
    ImGui::SameLine();
    ImGui::PushItemWidth(100.0f);

    ImGui::InputInt("times", &numGuidingOptimizations, 1, 10);

    ImGui::PopItemWidth();

    needSceneReload |= ImGui::Checkbox("Use Parallax Compensation for Optimization",
                                       &enableParallaxCompensationForOptimization);


    ImGui::Spacing();

    hasInputChanged |= ImGui::SliderFloat("Guiding Visu Scale",
                                          &rtPushConstants.guidingVisuScale, 0.0f, 1.0f);

    // Automatic movement/animation of guiding visu position
    hasInputChanged |= ImGui::Checkbox("Guiding Visu Move",
                                       reinterpret_cast<bool *>(&rtPushConstants.guidingVisuMove));
    hasInputChanged |= ImGui::InputFloat("Phi Scale",
                                         &rtPushConstants.guidingVisuPhiScale, 0.01f, 0.1f);
    hasInputChanged |= ImGui::InputFloat("Theta Scale",
                                         &rtPushConstants.guidingVisuThetaScale, 0.01f, 0.1f);

    hasInputChanged |= ImGui::SliderFloat("Guiding Visu Max",
                                          &rtPushConstants.guidingVisuMax, 0.0f, 50.0f);
    hasInputChanged |= ImGui::Checkbox("Guiding Visu Ignore Occlusion",
                                       reinterpret_cast<bool *>(&rtPushConstants.guidingVisuIgnoreOcclusioon));
    hasInputChanged |= ImGui::Checkbox("Guiding PiP show spheres",
                                       reinterpret_cast<bool *>(&rtPushConstants.guidingPiPShowSpheres));
    hasInputChanged |= ImGui::SliderFloat("Guiding PiP size",
                                          &rtPushConstants.guidingPiPSize, 0.0f, 1.0f);
    ImGui::End();
}

void RayTracingApp::raytrace(const vk::CommandBuffer &cmdBuf) {
    rtPushConstants.randomUInt = static_cast<uint>(glm::linearRand(0.0f, 1.0f) * std::numeric_limits<uint>::max());

    if (accumulateResults && !hasInputChanged && !cameraController.hasCameraChanged() && !autoRotate) {
        rtPushConstants.previousFrames += 1;
    } else {
        rtPushConstants.previousFrames = 0;
    }

    // Irradiance cache needs a few iterations to create the initial irradiance caches
    if (rtPushConstants.useIrradianceCache && currentPrepareFrames < irradianceCachePrepareFrames) {
        rtPushConstants.isIrradiancePrepareFrame = 1;
        rtPushConstants.previousFrames = -1;
        currentPrepareFrames++;
    } else {
        rtPushConstants.isIrradiancePrepareFrame = 0;
    }

    if (rtPushConstants.storeEstimate) {
        if (loadBackupNextIteration) {
            rtPushConstants = backupPushConstant;
            rtPushConstants.storeEstimate = 0;
            loadBackupNextIteration = false;
        } else {
            backupPushConstant = rtPushConstants;

            setEstimateRTSettings();

            loadBackupNextIteration = true;
        }
    }

    rtPushConstants.time = (std::chrono::high_resolution_clock::now() - startTime).count() / 10000000.f;
    rtPushConstants.numGuidingRegions = guiding.getRegionCount();

    cmdBuf.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, rtPipeline);
    cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, rtPipelineLayout, 0,
                              {rtDescSet, descriptorSet}, {});
    cmdBuf.pushConstants<RtPushConstant>(rtPipelineLayout,
                                         PUSH_CONSTANT_STAGES,
                                         0, rtPushConstants);

    vk::DeviceSize progSize = rtProperties.shaderGroupBaseAlignment;  // Alignment of a program identifier
    vk::DeviceSize rayGenOffset = 0u * progSize;  // Start at the beginning of sbtBuffer
    vk::DeviceSize missOffset = 1u * progSize;  // Jump over raygen
    vk::DeviceSize missStride = progSize;
    vk::DeviceSize hitGroupOffset = 3u * progSize;  // Jump over the previous shaders
    vk::DeviceSize hitGroupStride = progSize;

    vk::DeviceSize sbtSize = progSize * (vk::DeviceSize) rtShaderGroups.size();

    const vk::StridedBufferRegionKHR raygenShaderBindingTable = {rtSBTBuffer, rayGenOffset,
                                                                 progSize, sbtSize};
    const vk::StridedBufferRegionKHR missShaderBindingTable = {rtSBTBuffer, missOffset,
                                                               missStride, sbtSize};
    const vk::StridedBufferRegionKHR hitShaderBindingTable = {rtSBTBuffer, hitGroupOffset,
                                                              hitGroupStride, sbtSize};
    const vk::StridedBufferRegionKHR callableShaderBindingTable;

    cmdBuf.traceRaysKHR(&raygenShaderBindingTable, &missShaderBindingTable, &hitShaderBindingTable,
                        &callableShaderBindingTable,      //
                        offscreenExtent.width, offscreenExtent.height, 1);  //

    hasInputChanged = false;
}

void RayTracingApp::setEstimateRTSettings() {
    rtPushConstants.samplesPerPixel = 4;
    rtPushConstants.useIrradianceCache = true;
    rtPushConstants.useIrradianceCacheOnGlossy = true;
    rtPushConstants.enableNEE = true;
    rtPushConstants.visualizeMode = ERayTrace;
    rtPushConstants.maxDepth = 1;
    rtPushConstants.maxFollowDiscrete = 10;
    rtPushConstants.showIrradianceCacheOnly = false;
}

void RayTracingApp::cleanup() {
    irradianceCache.cleanUp();

    device.destroy(accumulateImageView);
    device.destroyImage(accumulateImage);
    device.free(accumulateImageMemory);

    device.destroy(estimateImageView);
    device.destroyImage(estimateImage);
    device.free(estimateImageMemory);

    device.free(uniformBufferMemory);
    device.destroy(uniformBuffer);

    cleanupDescriptorSets();

    cleanupRtPipeline();

    sceneLoader.cleanup();
    guiding.cleanup();
    sampleCollector.cleanup();
    postProcessing.cleanup();
    vulkanWindow.cleanup();
}

void RayTracingApp::cleanupRtPipeline() {
    device.destroy(rtSBTBuffer);
    device.free(rtSBTBufferMemory);
    device.destroy(rtPipeline);
    device.destroy(rtPipelineLayout);

    device.destroy(rtDescPool);
    device.destroy(rtDescSetLayout);

    rtDescSetLayoutBind.clear();
}
