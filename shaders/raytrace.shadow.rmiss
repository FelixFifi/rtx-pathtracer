#version 460
#extension GL_EXT_ray_tracing : require
#include "raycommon.glsl"

layout(location = 2) rayPayloadInEXT shadowCheck isShadowed;

void main()
{
    isShadowed.isShadowed = 0;
}