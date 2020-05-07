#version 460
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInNV hitPayload prd;
hitAttributeNV vec3 attribs;

void main()
{
    prd.hitValue = vec3(1.0, 0.0, 0.0);
}