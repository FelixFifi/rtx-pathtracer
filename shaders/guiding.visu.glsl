#define M_PI 3.1415926535897932384626433832795

vec3 sphericalToCartesian(float theta, float phi) {
    return vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

vec3 getVisuCenter(float time, float phiScale, float thetaScale, aabb bb, float radius) {
    vec3 extent = bb.max - bb.min;
    vec3 center = bb.min + 0.5 * extent;
    vec3 pos = sphericalToCartesian(0.5 * M_PI + thetaScale * time, phiScale * time);

    // Scale to AABB size
    return center + pos * 0.5 * (extent - 2 * radius.xxx);
}