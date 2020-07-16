void coordinateAxis(in vec3 z, out vec3 x, out vec3 y);

vec3 toLocal(vec3 v, vec3 x, vec3 y, vec3 z);

vec3 toLocal(vec3 v, vec3 n);

vec3 toWorld(vec3 v, vec3 x, vec3 y, vec3 z);

vec3 toWorld(vec3 v, vec3 n);

vec3 sphericalToCartesian(float theta, float phi);