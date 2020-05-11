struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

struct RtPushConstant {
    vec4 clearColor;
    vec3 lightPosition;
    float lightIntensity;
    int lightType;
};

struct Material {
    vec3 diffuse;
    vec3 specular;
    float specularHighlight;
    float transparency;
    float refractionIndex;
    int type;
};