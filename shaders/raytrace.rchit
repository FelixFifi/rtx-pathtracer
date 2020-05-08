#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

layout(location = 0) rayPayloadInEXT hitPayload prd;
hitAttributeEXT vec3 attribs;

layout(binding = 1, set = 1, scalar) buffer Vertices { Vertex v[]; } vertices[];
layout(binding = 2, set = 1) buffer Indices { uint i[]; } indices[];

void main()
{
    int objId = gl_InstanceID;

    ivec3 ind = ivec3(indices[objId].i[3 * gl_PrimitiveID + 0],   //
                      indices[objId].i[3 * gl_PrimitiveID + 1],   //
                      indices[objId].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[objId].v[ind.x];
    Vertex v1 = vertices[objId].v[ind.y];
    Vertex v2 = vertices[objId].v[ind.z];
    prd.hitValue = vec3(1.0, 0.0, 0.0);

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
    normal = (normal + vec3(1.0, 1.0, 1.0)) / 2.0;
    // Transforming the normal to world space
    normal = normalize(normal);

    prd.hitValue = normal;
}