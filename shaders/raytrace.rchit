#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "wavefront.glsl"
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitInfo info;
hitAttributeEXT vec3 attribs;

layout(binding = 1, set = 1, std140) buffer Vertices { Vertex v[]; } vertices[];
layout(binding = 2, set = 1) buffer Indices { uint i[]; } indices[];
layout(binding = 4, set = 1, std140) buffer Instances { InstanceInfo instanceInfos[]; };

void main()
{
    int iModel = instanceInfos[gl_InstanceID].modelIndex;

    ivec3 ind = ivec3(indices[iModel].i[3 * gl_PrimitiveID + 0],   //
                      indices[iModel].i[3 * gl_PrimitiveID + 1],   //
                      indices[iModel].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[iModel].v[ind.x];
    Vertex v1 = vertices[iModel].v[ind.y];
    Vertex v2 = vertices[iModel].v[ind.z];

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;

    // Transforming the normal to world space
    normal = normalize((instanceInfos[gl_InstanceID].normalTransform * vec4(normal, 0.0)).xyz);

    // Computing the coordinates of the hit position
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = (instanceInfos[gl_InstanceID].transform * vec4(worldPos, 1.0)).xyz;

    vec2 textureUV = v0.texCoord * barycentrics.x + v1.texCoord * barycentrics.y + v2.texCoord * barycentrics.z;

    if (dot(gl_WorldRayDirectionEXT, normal) < 0) {
        info.isFrontFace = true;
        info.normal = normal;
    } else {
        info.isFrontFace = false;
        info.normal = -normal;
    }

    info.worldPos = worldPos;
    info.textureUV = textureUV;
    info.matIndex = v0.materialIndex;
    info.t = gl_HitTEXT;
    info.isMiss = false;
    info.isSphere = false;
    info.instanceIndex = gl_InstanceID;
}