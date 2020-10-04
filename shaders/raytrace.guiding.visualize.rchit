#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable

#include "raycommon.glsl"
#include "guiding.visu.glsl"

//@formatter:off
layout(location = 0) rayPayloadInEXT guidingVisuInfo info;
layout(push_constant) uniform PushConstant{ pushConstant pushC; };

layout(binding = 15, set = 1, scalar) buffer Aabbs { aabb aabbs[]; };
//@formatter:on

void main() {
    aabb bb = aabbs[gl_PrimitiveID];
    vec3 extent = bb.max - bb.min;
    float radius = min(extent.x, min(extent.y, extent.z)) * 0.5 * pushC.guidingVisuScale;

    // Center of AABB
    vec3 center;
    if (pushC.guidingVisuMove) {
        center = getVisuCenter(pushC.time, pushC.guidingVisuPhiScale, pushC.guidingVisuThetaScale, bb, radius);
    } else {
        center = bb.min + 0.5 * extent;
    }

    // Get direction from center to find out what direction of the VMM to query
    info.worldPos = gl_WorldRayOriginEXT + gl_HitTEXT * gl_WorldRayDirectionEXT;
    info.direction = normalize(info.worldPos - center);
    info.center = center;
    info.t = gl_HitTEXT;
    info.iRegion = gl_PrimitiveID;
    info.isMiss = false;
}