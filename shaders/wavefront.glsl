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