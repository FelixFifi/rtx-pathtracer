#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

#define M_PI 3.1415926535897932384626433832795
#define M_INV_2PI 1.0 / (2 * M_PI)

//@formater:off
layout(location = 0) rayPayloadInEXT hitInfo info;
layout(binding = 7, set = 1) uniform sampler2D textureSamplers[];
layout(push_constant) uniform PushConstant{ pushConstant pushC; };
//@formater:on

void main()
{
    info.normal = -gl_WorldRayDirectionEXT;
    info.t = gl_RayTmaxEXT;
    info.isMiss = true;

    vec3 udir = normalize(gl_WorldRayDirectionEXT);

    float atan = atan(udir.x, -udir.z);
    float u = atan * M_INV_2PI;
    float v = acos(udir.y) / M_PI;

    vec2 envMapUV = vec2(u, v);
    info.missColor = texture(textureSamplers[0], envMapUV).xyz;
    info.matIndex = -1;
}