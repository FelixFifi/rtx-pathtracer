struct hitInfo
{
    vec3 worldPos;
    vec3 normal;
    int matIndex;
    float t;
    bool isLight; // TODO: extract to light material and check via matIndex
    vec3 lightColor;
};

struct shadowCheck {
    int isShadowed;
};

struct pushConstant
{
    vec4  skyColor1;
    vec4  skyColor2;
    vec3  lightPosition;
    float lightIntensity;
    vec2 uvOffset;
    int   lightType;
    uint previousFrames;
    int maxDepth;
    int samplesPerPixel;
};