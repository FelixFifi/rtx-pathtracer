struct hitPayload
{
    vec3 hitValue;
    uint recursionDepth;
};

struct pushConstant
{
    vec4  clearColor;
    vec3  lightPosition;
    float lightIntensity;
    int   lightType;
    uint   maxRecursion;
};