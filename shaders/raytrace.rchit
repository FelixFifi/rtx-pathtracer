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
    int objId = instanceInfos[gl_InstanceID].modelIndex;

    ivec3 ind = ivec3(indices[objId].i[3 * gl_PrimitiveID + 0],   //
                      indices[objId].i[3 * gl_PrimitiveID + 1],   //
                      indices[objId].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[objId].v[ind.x];
    Vertex v1 = vertices[objId].v[ind.y];
    Vertex v2 = vertices[objId].v[ind.z];

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;

    // Transforming the normal to world space
    normal = normalize((instanceInfos[gl_InstanceID].normalTransform * vec4(normal, 0.0)).xyz);

    // Computing the coordinates of the hit position
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = (gl_ObjectToWorldEXT * vec4(worldPos, 1.0));

    info.worldPos = worldPos;
    info.normal = normal;
    info.matIndex = v0.materialIndex;
    info.t = gl_HitTEXT;
    info.isLight = false;
}