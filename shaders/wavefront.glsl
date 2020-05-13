struct Vertex
{
    vec3 pos;
    vec3 normal;
    vec2 texCoord;
};

struct Material {
    vec3 diffuse;
    vec3 specular;
    float specularHighlight;
    float transparency;
    float refractionIndex;
    int type;
};