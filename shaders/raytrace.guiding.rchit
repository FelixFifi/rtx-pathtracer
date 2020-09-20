#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_nonuniform_qualifier : enable

#include "raycommon.glsl"

//@formatter:off
layout(location = 0) rayPayloadInEXT guidingInfo info;
//@formatter:on

void main() {
    info.iRegion = gl_PrimitiveID;
}