#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable
#include "wavefront.glsl"
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitPayload prd;
layout(location = 1) rayPayloadEXT shadowCheck isShadowed;
layout(location = 2) rayPayloadEXT hitPayload recursion;
hitAttributeEXT vec3 attribs;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout(binding = 1, set = 1, std430) buffer Vertices { Vertex v[]; } vertices[];
layout(binding = 2, set = 1) buffer Indices { uint i[]; } indices[];
layout(binding = 3, set = 1, std430) buffer Materials { Material mats[]; } materials;


layout(push_constant) uniform PushConstant { pushConstant pushC; };

// https://thebookofshaders.com/10/
float random (vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))*
                 43758.5453123);
}

// Rendering Vorlesung - Nori src/common.cpp
float fresnel(in float eta, in float cosThetaI) {
    float sinThetaSqr = eta * eta * (1- cosThetaI * cosThetaI);

    if (sinThetaSqr > 1.0f) {
        return 1.0f; // Total internal reflection
    }

    float cosThetaT = sqrt(1.0f - sinThetaSqr);

    float Rs = (eta * cosThetaI - cosThetaT)
               / (eta * cosThetaI + cosThetaT);
    float Rp = (cosThetaI - eta * cosThetaT)
               / (cosThetaI + eta * cosThetaT);

    return (Rs * Rs + Rp * Rp) / 2.0f;
}


void main()
{
    if (prd.recursionDepth >= pushC.maxRecursion) {
        prd.hitValue = vec3(1.0, 0.0, 0.0);
        return;
    }
    int objId = gl_InstanceID;

    ivec3 ind = ivec3(indices[objId].i[3 * gl_PrimitiveID + 0],   //
                      indices[objId].i[3 * gl_PrimitiveID + 1],   //
                      indices[objId].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[objId].v[ind.x];
    Vertex v1 = vertices[objId].v[ind.y];
    Vertex v2 = vertices[objId].v[ind.z];

    Material mat = materials.mats[objId];

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
    // Transforming the normal to world space
    normal = normalize((gl_ObjectToWorldEXT * vec4(normal, 0.0)).xyz);

    // Computing the coordinates of the hit position
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = (gl_ObjectToWorldEXT * vec4(worldPos, 1.0));

    switch (mat.type) {
        case 0: // Diffuse
            // Vector toward the light
            vec3  L;
            float lightIntensity = pushC.lightIntensity;
            float lightDistance  = 100000.0;

            // Point light
            if(pushC.lightType == 0)
            {
                vec3 lDir      = pushC.lightPosition - worldPos;
                lightDistance  = length(pushC.lightPosition - worldPos);
                lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
                L              = normalize(lDir);
            }
            else // Directional light
            {
                L = normalize(pushC.lightPosition - vec3(0));
            }

            float cosTheta = dot(normal, L);

            isShadowed.isShadowed = 1;

            // Tracing shadow ray only if the light is visible from the surface
            if(cosTheta > 0) {
                float tMin = 0.001;
                float tMax = lightDistance;
                vec3 origin = worldPos;
                vec3 rayDir = L;
                uint flags =
                        gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT;

                traceRayEXT(topLevelAS,  // acceleration structure
                            flags,       // rayFlags
                            0x02,        // cullMask - Ignore transparent for now
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

            if (isShadowed.isShadowed == 1) {
                prd.hitValue = vec3(0.0, 0.0, 0.0);
            } else {
                prd.hitValue = mat.diffuse * lightIntensity;
            }

            break;
        case 1: // Specular
        {
            vec3 reflectedDir = reflect(gl_WorldRayDirectionEXT, normal);

            float tMin = 0.001;
            float tMax = 10000.0;
            vec3 origin = worldPos;
            uint flags = gl_RayFlagsOpaqueEXT;

            recursion.recursionDepth = prd.recursionDepth + 1;

            traceRayEXT(topLevelAS,  // acceleration structure
                        flags,       // rayFlags
                        0xFF,        // cullMask
                        0,           // sbtRecordOffset
                        0,           // sbtRecordStride
                        0,           // missIndex
                        origin,      // ray origin
                        tMin,        // ray min range
                        reflectedDir,      // ray direction
                        tMax,        // ray max range
                        2           // payload (location = 2)
            );

            prd.hitValue = mat.specular * recursion.hitValue;
            break;
        }

        case 2: // Transparent
        {
            float eta = mat.refractionIndexInv;
            bool frontFace = dot(normal, gl_WorldRayDirectionEXT) < 0.0;

            if (!frontFace) {
                eta = mat.refractionIndex;
                normal = -normal;
            }

            float cosTheta = - dot(gl_WorldRayDirectionEXT, normal);

            float fresnel = fresnel(eta, cosTheta);

            vec3 rayDir = reflect(gl_WorldRayDirectionEXT, normal);

            {
                float tMin = 0.001;
                float tMax = 10000.0;
                vec3 origin = worldPos;
                uint flags = gl_RayFlagsOpaqueEXT;

                recursion.recursionDepth = prd.recursionDepth + 1;

                traceRayEXT(topLevelAS,  // acceleration structure
                            flags,       // rayFlags
                            0xFF,        // cullMask
                            0,           // sbtRecordOffset
                            0,           // sbtRecordStride
                            0,           // missIndex
                            origin,      // ray origin
                            tMin,        // ray min range
                            rayDir,      // ray direction
                            tMax,        // ray max range
                            2           // payload (location = 2)
                );
            }
            vec3 hitValueReflect = recursion.hitValue;

            rayDir = refract(gl_WorldRayDirectionEXT, normal, eta);

            if (fresnel < 1.0f)
            {
                float tMin = 0.001;
                float tMax = 10000.0;
                vec3 origin = worldPos;
                uint flags = gl_RayFlagsOpaqueEXT;

                recursion.recursionDepth = prd.recursionDepth + 1;

                traceRayEXT(topLevelAS,  // acceleration structure
                            flags,       // rayFlags
                            0xFF,        // cullMask
                            0,           // sbtRecordOffset
                            0,           // sbtRecordStride
                            0,           // missIndex
                            origin,      // ray origin
                            tMin,        // ray min range
                            rayDir,      // ray direction
                            tMax,        // ray max range
                            2           // payload (location = 2)
                );
            }

            vec3 hitValueRefract = recursion.hitValue;

            prd.hitValue = mat.specular * mix(hitValueRefract, hitValueReflect, fresnel); // Mix reflection and refraction
            break;
        }
    }



}