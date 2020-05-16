struct hitPayload
{
    vec3 hitValue;
    uint recursionDepth;
};

struct hitInfo
{
    vec3 worldPos;
    vec3 normal;
    uint matIndex;
    float t;
    bool isLight; // TODO: extract to light material and check via matIndex
    vec3 lightColor;
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
};