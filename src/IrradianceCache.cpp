//
// Created by felixfifi on 09.07.20.
//

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "IrradianceCache.h"
#include "Sphere.h"
#include "CommonOps.h"

IrradianceCache::IrradianceCache(uint32_t maxSpheres, uint32_t maxCommands, std::shared_ptr<VulkanOps> vulkanOps,
                                 vk::PhysicalDevice physicalDevice, uint32_t graphicsQueueIndex)
        : maxSpheres(maxSpheres), maxCommands(maxCommands), vulkanOps(vulkanOps), device(vulkanOps->getDevice()) {
    createBuffers();
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);
    createAccelerationStructure();

    createComputeDescriptorSet();
    updateComputeDescriptorSet();
    createComputePipeline();

}

void IrradianceCache::createAccelerationStructure() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;

    allBlas.emplace_back(spheresToBlas(device, maxSpheres, aabbsBuffer));

    const vk::BuildAccelerationStructureFlagsKHR &asFlags =
            vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate |
            vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace;
    rtBuilder.buildBlas(allBlas, asFlags);

    nvvkpp::RaytracingBuilderKHR::Instance rayInst;
    rayInst.transform = glm::value_ptr(glm::mat4(1.0f));
    rayInst.instanceId = 0;
    rayInst.blasId = 0;
    rayInst.hitGroupId = 2;
    rayInst.mask = 0xFF;
    rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;

    instances.emplace_back(rayInst);

    rtBuilder.buildTlas(instances, asFlags);
    accelerationStructure = rtBuilder.getAccelerationStructure();
}

void IrradianceCache::createBuffers() {
    Sphere tmp{glm::vec3(0.0f, 0.0f, 0.0f), 0.0f};
    Aabb aabb = tmp.getAabb();

    std::vector<Sphere> spheres(maxSpheres);
    std::vector<Aabb> aabbs(maxSpheres);
    std::vector<CacheData> cacheValues(maxSpheres);
    for (int i = 0; i < maxSpheres; ++i) {
        spheres[i] = tmp;
        aabbs[i] = aabb;
        cacheValues[i] = {};
    }

    vk::BufferUsageFlags usage =
            vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eShaderDeviceAddress;
    vk::MemoryPropertyFlagBits memoryFlags = vk::MemoryPropertyFlagBits::eDeviceLocal;
    vulkanOps->createBufferFromData(spheres, usage, memoryFlags, spheresBuffer, spheresBufferMemory);
    vulkanOps->createBufferFromData(aabbs, usage, memoryFlags, aabbsBuffer, aabbsBufferMemory);
    vulkanOps->createBufferFromData(cacheValues, usage, memoryFlags, cacheBuffer, cacheBufferMemory);

    UpdateCommandsHeader header{};
    header.maxCommands = maxCommands;
    header.nextCommandSlot = 0;
    header.nextSphereSlot = 0;
    header.maxSpheres = maxSpheres;

    std::vector<UpdateCommandsHeader> singleHeader{header};
    std::vector<UpdateCommad> updateCommands(maxCommands);
    for (int i = 0; i < maxCommands; ++i) {
        updateCommands[i] = {};
    }

    vulkanOps->createBufferFrom2Data(singleHeader, updateCommands, usage, memoryFlags, updateCommandsBuffer,
                                     updateCommandsBufferMemory);
}

void IrradianceCache::createComputeDescriptorSet() {
    vk::DescriptorType type = vk::DescriptorType::eStorageBuffer;
    vk::DescriptorPoolSize poolSizeSpheres{type, 1};
    vk::DescriptorPoolSize poolSizeAabbs{type, 1};
    vk::DescriptorPoolSize poolSizeUpdate{type, 1};

    std::array<vk::DescriptorPoolSize, 3> poolSizes{poolSizeSpheres, poolSizeAabbs, poolSizeUpdate};

    vk::DescriptorPoolCreateInfo poolInfo{{}, 1, poolSizes.size(), poolSizes.data()};
    compPool = device.createDescriptorPool(poolInfo);

    uint32_t bindingSpheres = 0;
    uint32_t bindingAabbs = 1;
    uint32_t bindingUpdate = 2;
    vk::DescriptorSetLayoutBinding bufferBindingSphere{bindingSpheres, type, 1, vk::ShaderStageFlagBits::eCompute};
    vk::DescriptorSetLayoutBinding bufferBindingAabbs{bindingAabbs, type, 1, vk::ShaderStageFlagBits::eCompute};
    vk::DescriptorSetLayoutBinding bufferBindingUpdate{bindingUpdate, type, 1, vk::ShaderStageFlagBits::eCompute};
    std::array<vk::DescriptorSetLayoutBinding, 3> bindings{bufferBindingSphere, bufferBindingAabbs,
                                                           bufferBindingUpdate};

    vk::DescriptorSetLayoutCreateInfo layoutInfo{{}, bindings.size(), bindings.data()};
    compSetLayout = device.createDescriptorSetLayout(layoutInfo);

    vk::DescriptorSetAllocateInfo allocateInfo{compPool, 1, &compSetLayout};
    compSet = device.allocateDescriptorSets(allocateInfo)[0];
}

void IrradianceCache::updateComputeDescriptorSet() {
    vk::DescriptorBufferInfo sphereBufferInfo{spheresBuffer, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo aabbsBufferInfo{aabbsBuffer, 0, VK_WHOLE_SIZE};
    vk::DescriptorBufferInfo updateBufferInfo{updateCommandsBuffer, 0, VK_WHOLE_SIZE};

    vk::WriteDescriptorSet writeSpheres{compSet, 0, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &sphereBufferInfo};
    vk::WriteDescriptorSet writeAabbs{compSet, 1, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr, &aabbsBufferInfo};
    vk::WriteDescriptorSet writeUpdates{compSet, 2, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &updateBufferInfo};

    std::array<vk::WriteDescriptorSet, 3> writes{writeSpheres, writeAabbs, writeUpdates};

    device.updateDescriptorSets(writes, nullptr);
}

void IrradianceCache::createComputePipeline() {
    vk::PipelineLayoutCreateInfo layoutCreateInfo{{}, 1, &compSetLayout, 0, nullptr};
    compPipelineLayout = device.createPipelineLayout(layoutCreateInfo);

    auto compCode = readFile("shaders/sphere_mod.comp.spv");
    vk::ShaderModule compShaderModule = vulkanOps->createShaderModule(compCode);

    vk::PipelineShaderStageCreateInfo shaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eCompute, compShaderModule,
                                                            "main"};

    vk::ComputePipelineCreateInfo computePipelineCreateInfo{{}, shaderStageCreateInfo, compPipelineLayout};
    compPipeline = device.createComputePipeline({}, computePipelineCreateInfo);

    device.destroy(compShaderModule);
}

void IrradianceCache::updateSpheres() {
    vk::CommandBuffer cmdBuf = vulkanOps->beginSingleTimeCommands();

    cmdBuf.bindPipeline(vk::PipelineBindPoint::eCompute, compPipeline);
    cmdBuf.bindDescriptorSets(vk::PipelineBindPoint::eCompute, compPipelineLayout, 0, compSet, {});

    cmdBuf.dispatch(maxCommands, 1, 1);

    vulkanOps->endSingleTimeCommands(cmdBuf);

    rtBuilder.updateBlas(0);
    rtBuilder.updateTlasMatrices(instances);
}

std::array<vk::DescriptorSetLayoutBinding, 4> IrradianceCache::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding bufferBindingUpdate{10, vk::DescriptorType::eStorageBuffer, 1,
                                                       vk::ShaderStageFlagBits::eRaygenKHR};
    vk::DescriptorSetLayoutBinding bindingAS{11, vk::DescriptorType::eAccelerationStructureKHR, 1,
                                             vk::ShaderStageFlagBits::eRaygenKHR};
    vk::DescriptorSetLayoutBinding bufferSpheres{12, vk::DescriptorType::eStorageBuffer, 1,
                                                 vk::ShaderStageFlagBits::eIntersectionKHR |
                                                 vk::ShaderStageFlagBits::eClosestHitKHR};
    vk::DescriptorSetLayoutBinding bufferCache{13, vk::DescriptorType::eStorageBuffer, 1,
                                                 vk::ShaderStageFlagBits::eRaygenKHR |
                                                 vk::ShaderStageFlagBits::eIntersectionKHR};


    return {bufferBindingUpdate, bindingAS, bufferSpheres, bufferCache};
}

std::array<vk::DescriptorPoolSize, 4> IrradianceCache::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eAccelerationStructureKHR, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1)
    };
}

/**
 *
 * @param descriptorSet
 * @param outBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @return
 */
std::array<vk::WriteDescriptorSet, 4>
IrradianceCache::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                        vk::DescriptorBufferInfo &outUpdateBufferInfo,
                                        vk::WriteDescriptorSetAccelerationStructureKHR &outDescASInfo,
                                        vk::DescriptorBufferInfo &outSpheresBufferInfo,
                                        vk::DescriptorBufferInfo &outCacheBufferInfo) {

    outDescASInfo.setAccelerationStructureCount(1);
    outDescASInfo.setPAccelerationStructures(&accelerationStructure);

    // (10) Update commands buffer
    outUpdateBufferInfo = vk::DescriptorBufferInfo(updateCommandsBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeUpdates{descriptorSet, 10, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &outUpdateBufferInfo};

    outSpheresBufferInfo = vk::DescriptorBufferInfo(spheresBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeSpheres{descriptorSet, 12, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &outSpheresBufferInfo};

    outCacheBufferInfo = vk::DescriptorBufferInfo(cacheBuffer, 0, VK_WHOLE_SIZE);
    vk::WriteDescriptorSet writeCache{descriptorSet, 13, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr,
                                        &outCacheBufferInfo};


    std::array<vk::WriteDescriptorSet, 4> writes;
    writes[0] = vk::WriteDescriptorSet(descriptorSet, 11, 0, 1, vk::DescriptorType::eAccelerationStructureKHR);
    writes[0].setPNext(&outDescASInfo);

    writes[1] = writeUpdates;

    writes[2] = writeSpheres;
    writes[3] = writeCache;

    return writes;
}

void IrradianceCache::cleanUp() {
    if (!aabbsBuffer) {
        return;
    }

    rtBuilder.destroy();

    device.free(aabbsBufferMemory);
    device.free(spheresBufferMemory);
    device.free(cacheBufferMemory);
    device.free(updateCommandsBufferMemory);

    device.destroy(spheresBuffer);
    device.destroy(aabbsBuffer);
    device.destroy(cacheBuffer);
    device.destroy(updateCommandsBuffer);

    device.destroy(compPool);
    device.destroy(compSetLayout);
    device.destroy(compPipeline);
    device.destroy(compPipelineLayout);
}
