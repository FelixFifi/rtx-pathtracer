struct hitInfo
{
    vec3 worldPos;
    vec3 normal;
    vec2 textureUV;
    int matIndex;
    float t;
    bool isMiss;
    bool isFrontFace;
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
    bool useIrradiance;
    float irradianceA;
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

struct cacheHeader {
    uint nextCacheSlot;
    uint maxCaches;
    uint nextUpdateSlot;
};

struct cacheData {
    vec3 color;
    vec3 normal;
    float harmonicR;
    uint numUpdates;
};

#define MAX_CACHES 10

struct cacheHits {
    vec3 origin;
    vec3 normal;
    uint nextSlot;
    vec3 cacheValues[MAX_CACHES];
    float weights[MAX_CACHES];
};