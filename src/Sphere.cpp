//
// Created by felixfifi on 11.07.20.
//

#include "Sphere.h"

nvvkpp::RaytracingBuilderKHR::Blas
spheresToBlas(vk::Device device, uint32_t sphereCount, vk::Buffer aabbBuffer, vk::GeometryFlagBitsKHR flags) {
    // Setting up the creation info of acceleration structure
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eAabbs);
    asCreate.setIndexType(vk::IndexType::eNoneKHR);
    asCreate.setVertexFormat(vk::Format::eUndefined);
    asCreate.setMaxPrimitiveCount(sphereCount);
    asCreate.setMaxVertexCount(0);
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

    // Building part
    vk::DeviceAddress aabbAddress = device.getBufferAddressKHR({aabbBuffer});

    vk::AccelerationStructureGeometryAabbsDataKHR aabbsData;
    aabbsData.setData(aabbAddress);
    aabbsData.setStride(sizeof(Aabb));

    // Setting up the build info of the acceleration
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(asCreate.geometryType);
    asGeom.setFlags(flags);
    asGeom.geometry.setAabbs(aabbsData);

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
