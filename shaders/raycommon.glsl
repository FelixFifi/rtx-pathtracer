struct hitInfo
{
    vec3 worldPos;
    vec3 normal;
    vec2 textureUV;
    int matIndex;
    float t;
    bool isMiss;// TODO: Find more consistent handling of miss
    vec3 missColor;
    uint instanceIndex;
};

struct shadowCheck {
    bool isShadowed;
};

struct pushConstant
{
    vec4  skyColor1;
    vec4  skyColor2;
    uint randomUInt;
    int   lightType;
    uint previousFrames;
    int maxDepth;
    int samplesPerPixel;
    bool enableRR;
    bool enableNEE;
    bool enableAverageInsteadOfMix;
    bool enableMIS;
    bool showIrradianceCache;
};

struct sphere {
    vec3 center;
    float radius;
    int materialIndex;
    int iLight;
};

struct aabb {
    vec3 min;
    vec3 max;
};

struct updateCommandsHeader {
    uint maxCommands;
    uint nextCommandSlot;
    uint nextSphereSlot;
    uint maxSpheres;
};

struct updateCommand {
    vec3 center;
    float radius;
    bool isFilled;
    bool isModify;
    uint iSphere;
};

struct cacheData {
    vec3 color;
    vec3 normal;
};