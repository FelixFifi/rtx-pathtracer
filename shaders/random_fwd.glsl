// Generate a random unsigned int from two unsigned int values, using 16 pairs
// of rounds of the Tiny Encryption Algorithm. See Zafar, Olano, and Curtis,
// "GPU Random Numbers via the Tiny Encryption Algorithm"
uint tea(uint val0, uint val1);

// Generate a random unsigned int in [0, 2^24) given the previous RNG state
// using the Numerical Recipes linear congruential generator
uint lcg(inout uint prev);

// Generate a random float in [0, 1) given the previous RNG state
float rnd(inout uint prev);
float rnd();

// Random float [-1, 1)
float getRandomNegPos();

// Random int [0, max]
int getRandomInteger(int max);

// https://math.stackexchange.com/a/1163278
vec3 randomOnUnitSphere();

// PDF 1 / (2pi)
vec3 randomInHemisphere(vec3 normal);

// PDF cos(theta) / pi
vec3 randomInHemisphereCosine(vec3 normal);

// (p + 1) * dot(reflected, res)^p/(2pi)
vec3 randomInHemisphereCosinePower(vec3 reflected, float p);

vec3 randomOnSphere(sphere s, out vec3 normal);

vec3 randomBeckmannNormal(Material mat, vec3 normal);