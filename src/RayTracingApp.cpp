//
// Created by felixfifi on 06.05.20.
//

#include <imgui/imgui.h>
#include <glm/gtc/random.hpp>
#include <random>
#include "RayTracingApp.h"

RayTracingApp::RayTracingApp(uint32_t width, uint32_t height) {
    fDrawCallback drawFunc = [this](uint32_t imageIndex) { drawCallback(imageIndex); };
    fRecreateSwapchainCallback recreateSwapchainFunc = [this] { recreateSwapchainCallback(); };

    vulkanWindow = VulkanWindow(width, height, drawFunc, recreateSwapchainFunc);
    postProcessing = PostProcessing({width, height}, vulkanWindow);

    cameraController = CameraController(glm::vec3(-20, 0, 15), 360.0f / width);
    fEventCallback eventCallback = [this](const SDL_Event &event) { cameraController.eventCallbackSDL(event); };
    vulkanWindow.setEventCallback(eventCallback);
    fNumberKeyEventCallback numberKeyCallback = [this](int key) { sceneSwitcher(key); };
    vulkanWindow.setNumberKeyEventCallback(numberKeyCallback);

    fImGuiCallback callbackImGui = [this] { imGuiWindowSetup(); };;
    postProcessing.addImGuiCallback(callbackImGui);

    device = vulkanWindow.getDevice();
    physicalDevice = vulkanWindow.getPhysicalDevice();
    vulkanOps = vulkanWindow.getVulkanOps();

    offscreenExtent = postProcessing.getExtentOffscreen();

    createNoiseTexture();
    createUniformBuffers();
    loadModels();


    initRayTracing();
}

void RayTracingApp::createNoiseTexture() {
    std::vector<glm::vec4> noise(RANDOM_SIZE * RANDOM_SIZE);

    float noiseMin = -1.0f;
    float noiseMax = 1.0f;

    std::mt19937 generator;
    std::uniform_real_distribution<float> dis(noiseMin, noiseMax);

    for (int i = 0; i < RANDOM_SIZE * RANDOM_SIZE; ++i) {
        noise[i] = {dis(generator),
                    dis(generator),
                    dis(generator),
                    dis(generator)};
    }

    vulkanOps->createNoiseTextureFromData(noise, RANDOM_SIZE, RANDOM_SIZE, noiseImage, noiseImageMemory, noiseImageView,
                                          noiseImageSampler);
}

void RayTracingApp::loadModels() {
    sceneSwitcher(1);
}

void RayTracingApp::run() {
    vulkanWindow.run();
}

void RayTracingApp::drawCallback(uint32_t imageIndex) {
    updateUniformBuffer(imageIndex);

    vk::CommandBuffer cmdBuf = vulkanOps->beginSingleTimeCommands();

    raytrace(cmdBuf);
    vulkanOps->endSingleTimeCommands(cmdBuf);

    cameraController.resetStatus();

    // TODO: Fences
    device.waitIdle();

    postProcessing.drawCallback(imageIndex);
}

void RayTracingApp::sceneSwitcher(int num) {
    // 0 should load the 10th scene
    if (num == 0) {
        num = 10;
    }

    // Don't load out of bounds
    int sceneCount = SCENES.size();
    int sceneIndex = std::min(num - 1, sceneCount - 1);

    if (modelLoader.getModelCount() > 0) {
        modelLoader.cleanup();
    }

    uint32_t graphicsQueueIndex = vulkanWindow.getQueueFamilyIndices().graphicsFamily.value();
    modelLoader = ModelLoader(SCENES[sceneIndex], "objs", MATERIAL_BASE_DIR, vulkanOps, physicalDevice,
                              vulkanWindow.getQueueFamilyIndices().graphicsFamily.value());

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

    vk::DescriptorSetLayoutBinding noiseSamplerBinding(5, vk::DescriptorType::eCombinedImageSampler, 1,
                                                       vk::ShaderStageFlagBits::eRaygenKHR);


    auto vertexIndexMaterialBindings = modelLoader.getDescriptorSetLayouts();
    std::array<vk::DescriptorSetLayoutBinding, 6> bindings = {uniformBufferLayoutBinding,
                                                              vertexIndexMaterialBindings[0],
                                                              vertexIndexMaterialBindings[1],
                                                              vertexIndexMaterialBindings[2],
                                                              vertexIndexMaterialBindings[3],
                                                              noiseSamplerBinding};
    vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());


    descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);
}

void RayTracingApp::createDescriptorPool() {
    auto vertexIndexMaterialPoolSizes = modelLoader.getDescriptorPoolSizes();

    std::array<vk::DescriptorPoolSize, 6> poolSizes = {
            vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer,
                                   static_cast<uint32_t>(1)),
            vertexIndexMaterialPoolSizes[0],
            vertexIndexMaterialPoolSizes[1],
            vertexIndexMaterialPoolSizes[2],
            vertexIndexMaterialPoolSizes[3],
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 1)
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

    for (size_t i = 0; i < 1; i++) {
        vk::DescriptorBufferInfo bufferInfo(uniformBuffer, 0, sizeof(CameraMatrices));
        vk::DescriptorImageInfo noiseImageInfo(noiseImageSampler, noiseImageView,
                                               vk::ImageLayout::eShaderReadOnlyOptimal);


        std::vector<vk::DescriptorBufferInfo> vertexBufferInfos;
        std::vector<vk::DescriptorBufferInfo> indexBufferInfos;
        vk::DescriptorBufferInfo materialBufferInfo;
        vk::DescriptorBufferInfo instanceInfoBufferInfo;

        auto vertexIndexBufferWrites = modelLoader.getWriteDescriptorSets(descriptorSet, vertexBufferInfos,
                                                                          indexBufferInfos, materialBufferInfo,
                                                                          instanceInfoBufferInfo);

        std::array<vk::WriteDescriptorSet, 6> descriptorWrites = {};

        descriptorWrites[0] = vk::WriteDescriptorSet(descriptorSet, 0, 0, 1,
                                                     vk::DescriptorType::eUniformBuffer, nullptr,
                                                     &bufferInfo, nullptr);
        descriptorWrites[1] = vertexIndexBufferWrites[0];
        descriptorWrites[2] = vertexIndexBufferWrites[1];
        descriptorWrites[3] = vertexIndexBufferWrites[2];
        descriptorWrites[4] = vertexIndexBufferWrites[3];
        descriptorWrites[5] = vk::WriteDescriptorSet(descriptorSet, 5, 0, 1,
                                                     vk::DescriptorType::eCombinedImageSampler, &noiseImageInfo,
                                                     nullptr, nullptr);

        device.updateDescriptorSets(descriptorWrites, nullptr);
    }
}

void RayTracingApp::updateUniformBuffer(uint32_t currentImage) {
    static auto startTime = std::chrono::high_resolution_clock::now();

    CameraMatrices ubo = {};
    ubo.proj = glm::perspective(glm::radians(45.0f), offscreenExtent.width / (float) offscreenExtent.height, 0.1f,
                                1000.0f);
    ubo.proj[1][1] *= -1;


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
    descASInfo.setPAccelerationStructures(&modelLoader.getAccelerationStructure());
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
    descASInfo.setPAccelerationStructures(&modelLoader.getAccelerationStructure());

    // (1) Output buffer
    vk::DescriptorImageInfo imageInfo{
            {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet wds{rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};


    std::array<vk::WriteDescriptorSet, 2> writes;
    writes[0] = vk::WriteDescriptorSet(rtDescSet, 0, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&descASInfo);

    writes[1] = vk::WriteDescriptorSet(rtDescSet, 1, 0, 1, vk::DescriptorType::eStorageImage, &imageInfo);
    device.updateDescriptorSets(writes, nullptr);
}

void RayTracingApp::createRtPipeline() {
    auto raygenCode = readFile("shaders/raytrace.rgen.spv");
    auto missCode = readFile("shaders/raytrace.rmiss.spv");
    auto chitCode = readFile("shaders/raytrace.rchit.spv");
    auto shadowMissCode = readFile("shaders/raytrace.shadow.rmiss.spv");

    vk::ShaderModule raygenShaderModule = vulkanOps->createShaderModule(raygenCode);
    vk::ShaderModule missShaderModule = vulkanOps->createShaderModule(missCode);
    vk::ShaderModule chitShaderModule = vulkanOps->createShaderModule(chitCode);
    vk::ShaderModule shadowMissShaderModule = vulkanOps->createShaderModule(shadowMissCode);

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
    device.destroy(shadowMissShaderModule);
}

void RayTracingApp::createRtShaderBindingTable() {
    auto groupCount = static_cast<uint64_t>(rtShaderGroups.size()); // 4 shaders: raygen, miss, chit, shadow miss
    auto groupHandleSize = static_cast<uint64_t>(rtProperties.shaderGroupHandleSize); // Size of a program identifier

    // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
    uint64_t sbtSize = groupCount * groupHandleSize;

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    device.getRayTracingShaderGroupHandlesKHR(rtPipeline, 0, groupCount, sbtSize,
                                              shaderHandleStorage.data());


    // Write the handles in the SBT
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eRayTracingKHR;
    vulkanOps->createBufferFromData(shaderHandleStorage, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, rtSBTBuffer, rtSBTBufferMemory);
}

void RayTracingApp::imGuiWindowSetup() {
    ImGui::Begin("Raytrace Window");

    hasInputChanged = false;

    hasInputChanged |= ImGui::InputInt("LightType", &rtPushConstants.lightType);
    hasInputChanged |= ImGui::InputFloat3("LightPosition", &rtPushConstants.lightPosition.x, "%.2f");
    hasInputChanged |= ImGui::InputFloat("LightIntensity", &rtPushConstants.lightIntensity);
    hasInputChanged |= ImGui::InputFloat4("SkyColor1", &rtPushConstants.skyColor1.x, "%.2f");
    hasInputChanged |= ImGui::InputFloat4("SkyColor2", &rtPushConstants.skyColor2.x, "%.2f");
    ImGui::Checkbox("Auto rotate", &autoRotate);
    ImGui::Spacing();
    ImGui::Checkbox("Accumulate results", &accumulateResults);
    hasInputChanged |= ImGui::InputInt("Samples per pixel", &rtPushConstants.samplesPerPixel, 20, 100);
    hasInputChanged |= ImGui::InputInt("Max depth", &rtPushConstants.maxDepth, 1, 5);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);


    ImGui::End();
}

void RayTracingApp::raytrace(const vk::CommandBuffer &cmdBuf) {
    rtPushConstants.uvOffset = {glm::linearRand(0.0f, 1.0f), glm::linearRand(0.0f, 1.0f)};

    if (accumulateResults && !hasInputChanged && !cameraController.hasCameraChanged() && !autoRotate) {
        rtPushConstants.previousFrames += 1;
    } else {
        rtPushConstants.previousFrames = 0;
    }

    cmdBuf.bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, rtPipeline);
    cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, rtPipelineLayout, 0,
                              {rtDescSet, descriptorSet}, {});
    cmdBuf.pushConstants<RtPushConstant>(rtPipelineLayout,
                                         vk::ShaderStageFlagBits::eRaygenKHR
                                         | vk::ShaderStageFlagBits::eClosestHitKHR
                                         | vk::ShaderStageFlagBits::eMissKHR,
                                         0, rtPushConstants);

    vk::DeviceSize progSize = rtProperties.shaderGroupHandleSize;  // Size of a program identifier
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
}

void RayTracingApp::cleanup() {

    device.destroy(noiseImageSampler);
    device.destroy(noiseImageView);
    device.destroyImage(noiseImage);
    device.free(noiseImageMemory);

    device.free(uniformBufferMemory);
    device.destroy(uniformBuffer);

    cleanupDescriptorSets();

    cleanupRtPipeline();

    modelLoader.cleanup();

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
