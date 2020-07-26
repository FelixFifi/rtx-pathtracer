#include "random_fwd.glsl"

#define M_PI 3.1415926535897932384626433832795
#define E 2.7182818284590452353602874713527

// Rendering Vorlesung - Nori src/common.cpp
void coordinateAxis(in vec3 z, out vec3 x, out vec3 y) {
    if (abs(z.x) > abs(z.y)) {
        float invLen = 1.0f / sqrt(z.x * z.x + z.z * z.z);
        y = vec3(z.z * invLen, 0.0f, -z.x * invLen);
    } else {
        float invLen = 1.0f / sqrt(z.y * z.y + z.z * z.z);
        y = vec3(0.0f, z.z * invLen, -z.y * invLen);
    }
    x = cross(y, z);
}

vec3 toLocal(vec3 v, vec3 x, vec3 y, vec3 z) {
    return vec3(dot(v, x), dot(v, y), dot(v, z));
}

vec3 toLocal(vec3 v, vec3 n) {
    vec3 x, y;
    coordinateAxis(n, x, y);

    return toLocal(v, x, y, n);
}

vec3 toWorld(vec3 v, vec3 x, vec3 y, vec3 z) {
    return v.x * x + v.y * y + v.z * z;
}

vec3 toWorld(vec3 v, vec3 n) {
    vec3 x, y;
    coordinateAxis(n, x, y);

    return toWorld(v, x, y, n);
}

vec3 sphericalToCartesian(float theta, float phi) {
    return vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}