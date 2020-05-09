#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

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
    hitValue = pushC.clearColor.xyz;
}