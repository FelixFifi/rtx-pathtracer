#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitInfo info;

layout(push_constant) uniform PushConstant{ pushConstant pushC; };

void main()
{
    info.normal = -gl_WorldRayDirectionEXT;
    info.t = -1.0;
    info.isLight = true;
    info.lightColor = mix(pushC.clearColor.xyz, vec3(1.0, 1.0, 1.0), gl_WorldRayDirectionEXT.z);
}