#include "random_fwd.glsl"

#define M_PI 3.1415926535897932384626433832795
#define M_2PI 6.28318530718

struct Theta {
    vec3 mu;
    float k;
    float norm; // = k / (M_2PI * (1 - exp(- 2 * theta.k)))
    float eMin2K; // exp(-2.0f * k)
};

Theta updateK(Theta theta, float k) {
    theta.k = k;
    theta.norm = k / (M_2PI * (1 - exp(- 2 * theta.k)));
    theta.eMin2K = exp(-2.0 * k);
    return theta;
}

float vMF(vec3 wo, Theta theta) {
    return theta.norm * exp(theta.k * (dot(theta.mu, wo) - 1));
}

#define MAX_DISTRIBUTIONS 10


struct VMM_Theta {
    Theta thetas[MAX_DISTRIBUTIONS];
    float pi[MAX_DISTRIBUTIONS];
    float pi_sum;
};


float VMM(vec3 wo, VMM_Theta vmmTheta) {
    float res = 0;

    for (int i = 0; i < MAX_DISTRIBUTIONS; i++) {
        res += vmmTheta.pi[i] * vMF(wo, vmmTheta.thetas[i]);
    }
    return res;
}

vec3 sampleVMF(Theta theta) {
    if (theta.k > 0.0) {
        const float r1 = rnd();
        const float r2 = rnd();
        const float cosTheta = 1.0 + log(1 + theta.eMin2K * r1 - r1) / theta.k; // log1p not available
        const float sinTheta = 1.0 - cosTheta * cosTheta <= 0.0 ? 0.0 : sqrt(1.0 - cosTheta * cosTheta);
        const float phi = 2.f * M_PI * r2;

        const float cosPhi = cos(phi);
        const float sinPhi = sin(phi);

        return vec3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
    } else {
        return randomOnUnitSphere();
    }
}

vec3 sampleVMM(VMM_Theta vmmTheta) {
    float rndDist = rnd() * vmmTheta.pi_sum;

    int iDistribution = 0;
    while (vmmTheta.pi[iDistribution] < rndDist) {
        iDistribution++;
    }

    return sampleVMF(vmmTheta.thetas[iDistribution]);
}