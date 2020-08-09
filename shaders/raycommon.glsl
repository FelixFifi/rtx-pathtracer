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
    bool showIrradianceCacheOnly;
    bool useIrradiance;
    bool highlightIrradianceCacheColor;
    float irradianceA;
    float irradianceUpdateProb;
    float irradianceCreateProb;
    float irradianceVisualizationScale;
    bool useIrradianceGradients;
    bool useVisibleSphereSampling;
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
    vec3 rotGrad;
    vec3 transGrad;
    float harmonicR;
    uint numUpdates;
};

struct cacheHits {
    vec3 origin;
    vec3 normal;
    float totalWeight;
    vec3 cacheValueSum;
};