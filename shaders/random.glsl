#include "transform_fwd.glsl"

#define M_PI 3.1415926535897932384626433832795
#define E 2.7182818284590452353602874713527

uint seed;

// Generate a random unsigned int from two unsigned int values, using 16 pairs
// of rounds of the Tiny Encryption Algorithm. See Zafar, Olano, and Curtis,
// "GPU Random Numbers via the Tiny Encryption Algorithm"
uint tea(uint val0, uint val1)
{
    uint v0 = val0;
    uint v1 = val1;
    uint s0 = 0;

    for (uint n = 0; n < 16; n++)
    {
        s0 += 0x9e3779b9;
        v0 += ((v1 << 4) + 0xa341316c) ^ (v1 + s0) ^ ((v1 >> 5) + 0xc8013ea4);
        v1 += ((v0 << 4) + 0xad90777d) ^ (v0 + s0) ^ ((v0 >> 5) + 0x7e95761e);
    }

    return v0;
}

// Generate a random unsigned int in [0, 2^24) given the previous RNG state
// using the Numerical Recipes linear congruential generator
uint lcg(inout uint prev)
{
    uint LCG_A = 1664525u;
    uint LCG_C = 1013904223u;
    prev       = (LCG_A * prev + LCG_C);
    return prev & 0x00FFFFFF;
}

// Generate a random float in [0, 1) given the previous RNG state
float rnd(inout uint prev)
{
    return (float(lcg(prev)) / float(0x01000000));
}

float rnd() {
    return rnd(seed);
}

// Random float [-1, 1)
float getRandomNegPos() {
    return rnd(seed) * 2 - 1;
}

// Random int [0, max]
int getRandomInteger(int max) {
    return int(rnd(seed) * (max + 1));
}

// https://math.stackexchange.com/a/1163278
vec3 randomOnUnitSphere() {
    vec3 res;
    do {
        res = vec3(rnd(seed), rnd(seed), rnd(seed));
    } while (length(res) > 1);

    res = normalize(res);
    return res;
}

// PDF 1 / (2pi)
vec3 randomInHemisphere(vec3 normal) {
    vec3 res;
    do {
        res = vec3(getRandomNegPos(), getRandomNegPos(), getRandomNegPos());
    } while (length(res) > 1);

    res = normalize(res);

    if (dot(normal, res) < 0) {
        res = reflect(res, normal);
    }
    return res;
}

// PDF cos(theta) / pi
vec3 randomInHemisphereCosine(vec3 normal) {
    float u = rnd(seed);
    float sqrt_u = sqrt(u);
    float phi = 2 * M_PI * rnd(seed);

    vec3 local = vec3(sqrt_u * cos(phi), sqrt_u * sin(phi), sqrt(1 - u));

    return toWorld(local, normal);
}

// (p + 1) * dot(reflected, res)^p/(2pi)
vec3 randomInHemisphereCosinePower(vec3 reflected, float p) {
    float u = rnd(seed);
    float cosTheta = pow(u, 1.0 / (p + 1));
    float phi = 2 * M_PI * rnd(seed);

    float sinTheta = sqrt(1 - cosTheta * cosTheta);

    vec3 local = vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
    return toWorld(local, reflected);
}

vec3 randomOnSphere(sphere s, out vec3 normal) {
    do {
        normal = vec3(getRandomNegPos(), getRandomNegPos(), getRandomNegPos());
    } while (length(normal) > 1);
    normal = normalize(normal);

    return s.center + normal * s.radius;
}

vec3 randomBeckmannNormal(Material mat, vec3 normal) {
    float thetaM = atan(sqrt(- mat.roughness * mat.roughness * log(1 - rnd(seed))));
    float phiM = 2 * M_PI * rnd(seed);

    float cosThetaNM = cos(thetaM);

    vec3 localM = vec3(sin(thetaM) * cos(phiM), sin(thetaM) * sin(phiM), cosThetaNM);

    return toWorld(localM, normal);
}