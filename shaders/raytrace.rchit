#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

layout(location = 0) rayPayloadInEXT hitPayload prd;
hitAttributeEXT vec3 attribs;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout(binding = 1, set = 1, scalar) buffer Vertices { Vertex v[]; } vertices[];
layout(binding = 2, set = 1) buffer Indices { uint i[]; } indices[];

layout(location = 1) rayPayloadEXT bool isShadowed;

layout(push_constant) uniform Constants
        {
        vec4  clearColor;
        vec3  lightPosition;
        float lightIntensity;
        int   lightType;
        }
pushC;

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
    // Transforming the normal to world space
    normal = normalize(normal);

    // Computing the coordinates of the hit position
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = (gl_ObjectToWorldEXT * vec4(worldPos, 1.0));

    // Vector toward the light
    vec3  L;
    float lightIntensity = pushC.lightIntensity;
    float lightDistance  = 100000.0;
    // Point light
    if(pushC.lightType == 0)
    {
        vec3 lDir      = pushC.lightPosition - worldPos;
        lightDistance  = length(lDir);
        lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
        L              = normalize(lDir);
    }
    else // Directional light
    {
        L = normalize(pushC.lightPosition - vec3(0));
    }

    float cosTheta = dot(normal, L);

    // Tracing shadow ray only if the light is visible from the surface
    if(cosTheta > 0) {
        float tMin = 0.001;
        float tMax = lightDistance;
        vec3 origin = worldPos;
        vec3 rayDir = L;
        uint flags =
                gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT;
        isShadowed = true;
        traceRayEXT(topLevelAS,  // acceleration structure
                    flags,       // rayFlags
                    0xFF,        // cullMask
                    0,           // sbtRecordOffset
                    0,           // sbtRecordStride
                    1,           // missIndex
                    origin,      // ray origin
                    tMin,        // ray min range
                    rayDir,      // ray direction
                    tMax,        // ray max range
                    1            // payload (location = 1)
        );
    }

    if (isShadowed || cosTheta <= 0) {
        prd.hitValue = vec3(0.0, 0.0, 0.0);
    } else {
        prd.hitValue = cosTheta * lightIntensity.xxx;
    }
}