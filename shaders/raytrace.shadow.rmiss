#version 460
#extension GL_EXT_ray_tracing : require
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT shadowCheck shadowInfo;

void main()
{
    shadowInfo.isShadowed = false;
}