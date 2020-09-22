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
    uint previousFrames;
    int maxDepth;
    int maxFollowDiscrete;
    int samplesPerPixel;
    bool enableRR;
    bool enableNEE;
    int numNEE;
    bool enableAverageInsteadOfMix;
    bool enableMIS;
    bool storeEstimate;
    int visualizeMode;
    bool showIrradianceCacheOnly;
    bool showIrradianceGradients;
    bool useIrradianceCache;
    bool highlightIrradianceCacheColor;
    float irradianceA;
    float irradianceUpdateProb;
    float irradianceCreateProb;
    float irradianceVisualizationScale;
    bool useIrradianceGradients;
    bool useIrradianceCacheOnGlossy;
    float irradianceGradientsMaxLength;
    bool isIrradiancePrepareFrame;
    int irradianceNumNEE;
    float irradianceCacheMinRadius;
    bool irradianceCachePerformVisibilityCheck;
    bool useVisibleSphereSampling;
    bool useADRRS;
    float adrrsS;
    bool adrrsSplit;
    bool splitOnFirst;
    bool useGuiding;
    float guidingProb;
    float guidingVisuScale;
    float guidingVisuMax;
    bool guidingVisuIgnoreOcclusioon;
    bool guidingTest;
    float guidingTestK;
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

struct guidingInfo {
    uint iRegion;
};

struct guidingVisuInfo {
    vec3 worldPos;
    vec3 direction;
    float t;
    uint iRegion;
    bool isMiss;
};