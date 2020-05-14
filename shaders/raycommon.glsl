struct hitPayload
{
    vec3 hitValue;
    uint recursionDepth;
};

struct shadowCheck {
    int isShadowed;
};

struct pushConstant
{
    vec4  clearColor;
    vec3  lightPosition;
    float lightIntensity;
    int   lightType;
    uint   maxRecursion;
    vec2 pad2;
};