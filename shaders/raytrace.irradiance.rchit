#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "wavefront.glsl"
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitInfo info;

layout(binding = 12, set = 1, std430) buffer Spheres { sphere spheres[]; };

void main()
{
    sphere s = spheres[gl_PrimitiveID];

    vec3 pos = gl_WorldRayOriginEXT + gl_HitTEXT * gl_WorldRayDirectionEXT;
    vec3 normal = normalize(pos - s.center);

    info.worldPos = pos;
    info.normal = normal;
    info.t = gl_HitTEXT;
    info.isMiss = false;
    info.instanceIndex = gl_PrimitiveID;
}