struct Vertex
{
    vec3 pos;
    vec3 normal;
    vec2 texCoord;
    int materialIndex;
};

struct Material {
    vec3 lightColor;
    vec3 diffuse;
    vec3 specular;
    float specularHighlight;
    float transparency;
    float refractionIndex;
    float refractionIndexInv;
    float eta;
    float k;
    float roughness;
    int textureIdDiffuse;
    int textureIdSpecular;
    int type;
};

struct InstanceInfo {
    mat4 transform;
    mat4 normalTransform;
    int modelIndex;
    int iLight;
};

struct Light {
    vec3 color;
    vec3 pos;
    uint instanceIndex;
    float sampleProb;
    float area;
    int type;
};

struct FaceSample {
    int index;
    float sampleProb;
    float faceArea;
};
