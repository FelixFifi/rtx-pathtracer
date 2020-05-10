//
// Created by felixfifi on 06.05.20.
//

#include <imgui/imgui.h>
#include "RayTracingApp.h"
#include "Model.h"

RayTracingApp::RayTracingApp(uint32_t width, uint32_t height) {
    fDrawCallback drawFunc = [this](uint32_t imageIndex) { drawCallback(imageIndex); };
    fRecreateSwapchainCallback recreateSwapchainFunc = [this] { recreateSwapchainCallback(); };

    vulkanWindow = VulkanWindow(width, height, drawFunc, recreateSwapchainFunc);
    postProcessing = PostProcessing({width, height}, vulkanWindow);

    fImGuiCallback callbackImGui = [this] { imGuiWindowSetup(); };;
    postProcessing.addImGuiCallback(callbackImGui);

    device = vulkanWindow.getDevice();
    physicalDevice = vulkanWindow.getPhysicalDevice();
    vulkanOps = vulkanWindow.getVulkanOps();

    offscreenExtent = postProcessing.getExtentOffscreen();

    loadModels();

    createUniformBuffers();
    createDecriptorSetLayout();
    createDescriptorPool();
    createDescriptorSets();

    initRayTracing();
}

void RayTracingApp::loadModels() {
    // Currently only one hardcoded
    models.emplace_back(std::make_unique<Model>(MODEL_PATH1, vulkanOps));
    models.emplace_back(std::make_unique<Model>(MODEL_PATH2, vulkanOps));
}

void RayTracingApp::run() {
    vulkanWindow.run();
}

void RayTracingApp::drawCallback(uint32_t imageIndex) {
    updateUniformBuffer(imageIndex);

    vk::CommandBuffer cmdBuf = vulkanOps->beginSingleTimeCommands();

    raytrace(cmdBuf, {0, 0, 0, 0});
    vulkanOps->endSingleTimeCommands(cmdBuf);

    // TODO: Fences
    device.waitIdle();

    postProcessing.drawCallback(imageIndex);
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

    vk::DescriptorSetLayoutBinding vertexBufferBinding(1, vk::DescriptorType::eStorageBuffer, models.size(),
                                                       vk::ShaderStageFlagBits::eClosestHitKHR);

    vk::DescriptorSetLayoutBinding indexBufferBinding(2, vk::DescriptorType::eStorageBuffer, models.size(),
                                                      vk::ShaderStageFlagBits::eClosestHitKHR);


    std::array<vk::DescriptorSetLayoutBinding, 3> bindings = {uniformBufferLayoutBinding, vertexBufferBinding,
                                                              indexBufferBinding};
    vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());


    descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);
}

void RayTracingApp::createDescriptorPool() {
    std::array<vk::DescriptorPoolSize, 3> poolSizes = {
            vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer,
                                   static_cast<uint32_t>(1)),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size())
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


        unsigned long modelCount = models.size();
        std::vector<vk::DescriptorBufferInfo> vertexBufferInfos;
        std::vector<vk::DescriptorBufferInfo> indexBufferInfos;
        vertexBufferInfos.reserve(modelCount);
        indexBufferInfos.reserve(modelCount);

        for (const auto &model: models) {
            vk::DescriptorBufferInfo vertexBufferInfo(model->vertexBuffer, 0, VK_WHOLE_SIZE);
            vk::DescriptorBufferInfo indexBufferInfo(model->indexBuffer, 0, VK_WHOLE_SIZE);

            vertexBufferInfos.push_back(vertexBufferInfo);
            indexBufferInfos.push_back(indexBufferInfo);
        }

        std::array<vk::WriteDescriptorSet, 3> descriptorWrites = {};

        descriptorWrites[0] = vk::WriteDescriptorSet(descriptorSet, 0, 0, 1,
                                                     vk::DescriptorType::eUniformBuffer, nullptr,
                                                     &bufferInfo, nullptr);
        descriptorWrites[1] = vk::WriteDescriptorSet(descriptorSet, 1, 0, modelCount,
                                                     vk::DescriptorType::eStorageBuffer, nullptr,
                                                     vertexBufferInfos.data(), nullptr);
        descriptorWrites[2] = vk::WriteDescriptorSet(descriptorSet, 2, 0, modelCount,
                                                     vk::DescriptorType::eStorageBuffer, nullptr,
                                                     indexBufferInfos.data(), nullptr);

        device.updateDescriptorSets(descriptorWrites, nullptr);
    }
}

void RayTracingApp::updateUniformBuffer(uint32_t currentImage) {
    static auto startTime = std::chrono::high_resolution_clock::now();

    auto currentTime = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

    CameraMatrices ubo = {};
    ubo.view = glm::lookAt(glm::vec3(30 * glm::sin(time), 30 * glm::cos(time), 10.0f + 20.0f * glm::sin(time / 3.0f)),
                           glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    ubo.proj = glm::perspective(glm::radians(45.0f), offscreenExtent.width / (float) offscreenExtent.height, 0.1f,
                                1000.0f);
    ubo.proj[1][1] *= -1;

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

    QueueFamilyIndices queueFamilyIndices = vulkanWindow.getQueueFamilyIndices();
    rtBuilder.setup(device, physicalDevice, queueFamilyIndices.graphicsFamily.value());

    createBottomLevelAS();
    createTopLevelAS();
    createRtDescriptorSet();
    updateRtDescriptorSet(0);
    createRtPipeline();
    createRtShaderBindingTable();
}

nvvkpp::RaytracingBuilderKHR::Blas RayTracingApp::modelToBlas(const std::unique_ptr<Model> &model) {
    // Setting up the creation info of acceleration structure
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    asCreate.setIndexType(vk::IndexType::eUint32);
    asCreate.setVertexFormat(vk::Format::eR32G32B32Sfloat);
    asCreate.setMaxPrimitiveCount(model->indices.size() / 3);  // Nb triangles
    asCreate.setMaxVertexCount(model->vertices.size());
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

    // Building part
    auto infoVB = vk::BufferDeviceAddressInfo(model->vertexBuffer);
    auto infoIB = vk::BufferDeviceAddressInfo(model->indexBuffer);

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

void RayTracingApp::createBottomLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;
    allBlas.reserve(models.size());

    for (const auto &model : models) {
        allBlas.push_back(modelToBlas(model));
    }

    rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void RayTracingApp::createTopLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> tlas;
    tlas.reserve(models.size());

    for (int i = 0; i < static_cast<int>(models.size()); ++i) {
        nvvkpp::RaytracingBuilderKHR::Instance rayInst;
        rayInst.transform = i == 0 ? nvmath::mat4f_id : nvmath::translation_mat4<nvmath::nv_scalar>(30.0f, 0.0f, 0.0f).scale(10.0f) ;
        rayInst.instanceId = i;
        rayInst.blasId = i;
        rayInst.hitGroupId = 0; // Same hit group for all
        rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable; // TODO
        tlas.emplace_back(rayInst);
    }

    rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void RayTracingApp::createRtDescriptorSet() {
    using vkDT = vk::DescriptorType;
    using vkSS = vk::ShaderStageFlagBits;
    using vkDSLB = vk::DescriptorSetLayoutBinding;

    rtDescSetLayoutBind.emplace_back(vkDSLB(0, vkDT::eAccelerationStructureKHR, 1, vkSS::eRaygenKHR | vkSS::eClosestHitKHR)); // TLAS
    rtDescSetLayoutBind.emplace_back(vkDSLB(1, vkDT::eStorageImage, 1, vkSS::eRaygenKHR)); // Output image

    rtDescPool = nvvkpp::util::createDescriptorPool(device, rtDescSetLayoutBind);
    rtDescSetLayout = nvvkpp::util::createDescriptorSetLayout(device, rtDescSetLayoutBind);
    rtDescSet = device.allocateDescriptorSets({rtDescPool, 1, &rtDescSetLayout})[0];

    vk::WriteDescriptorSetAccelerationStructureKHR descASInfo;
    descASInfo.setAccelerationStructureCount(1);
    descASInfo.setPAccelerationStructures(&rtBuilder.getAccelerationStructure());
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

    // (1) Output buffer
    vk::DescriptorImageInfo imageInfo{
            {}, postProcessing.getOffscreenImageView(), vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet wds{rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
    device.updateDescriptorSets(wds, nullptr);
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

    rayPipelineInfo.setMaxRecursionDepth(2);  // Ray depth
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

    ImGui::InputInt("LightType", &rtPushConstants.lightType);
    ImGui::InputFloat3("LightPosition", &rtPushConstants.lightPosition.x, "%.2f");
    ImGui::InputFloat("LightIntensity", &rtPushConstants.lightIntensity);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::End();
}

void RayTracingApp::raytrace(const vk::CommandBuffer &cmdBuf, const nvmath::vec4f &clearColor) {
    // Initializing push constant values
    rtPushConstants.clearColor = clearColor;

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
    device.free(uniformBufferMemory);
    device.destroy(uniformBuffer);

    device.destroy(descriptorPool);
    device.destroy(descriptorSetLayout);

    for (auto &model: models) {
        model->cleanup();
    }

    device.destroy(rtSBTBuffer);
    device.free(rtSBTBufferMemory);
    device.destroy(rtPipeline);
    device.destroy(rtPipelineLayout);

    rtBuilder.destroy();
    device.destroy(rtDescPool);
    device.destroy(rtDescSetLayout);

    postProcessing.cleanup();
    vulkanWindow.cleanup();
}