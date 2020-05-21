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
    int type;
};

struct InstanceInfo {
    mat4 transform;
    mat4 normalTransform;
    int modelIndex;
};

struct Light {
    vec3 color;
    vec3 pos;
    bool isPointLight;
    uint instanceIndex;
};
