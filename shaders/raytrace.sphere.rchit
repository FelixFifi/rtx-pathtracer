#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "wavefront.glsl"
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitInfo info;

layout(binding = 8, set = 1, scalar) buffer Spheres { sphere spheres[]; };

void main()
{
    sphere s = spheres[gl_PrimitiveID];

    vec3 pos = gl_WorldRayOriginEXT + gl_HitTEXT * gl_WorldRayDirectionEXT;
    vec3 normal = normalize(pos - s.center);

    if (dot(gl_WorldRayDirectionEXT, normal) < 0) {
        info.isFrontFace = true;
        info.normal = normal;
    } else {
        info.isFrontFace = false;
        info.normal = -normal;
    }

    info.worldPos = pos;
    info.textureUV = vec2(0, 0);
    info.matIndex = s.materialIndex;
    info.t = gl_HitTEXT;
    info.isMiss = false;
    info.isSphere = true;
    info.instanceIndex = gl_PrimitiveID;
}