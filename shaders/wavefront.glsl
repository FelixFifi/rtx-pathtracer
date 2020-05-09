struct Vertex {
    vec3 pos;
    vec3 normal;
    vec3 color;
    vec2 uv;
};

struct RtPushConstant {
    vec4 clearColor;
    vec3 lightPosition;
    float lightIntensity;
    int lightType;
};