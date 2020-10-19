#include "random_fwd.glsl"
#include "transform_fwd.glsl"

#define M_PI 3.1415926535897932384626433832795
#define M_2PI 6.28318530718
#define M_INV_4PI 0.07957747155

// Taken from lightpmm/VMFKernel.h

// the minumum value of kappa before it gets set to 0.0 for numerical stability
#define VMF_MinKappa 1e-3f

struct VMF_Theta {
    vec3 mu;
    float k;
    float norm;// = k / (M_2PI * (1 - exp(- 2 * theta.k)))
    float eMin2K;// exp(-2.0f * k)
    float distance;
    vec3 target;
};

VMF_Theta updateK(VMF_Theta theta, float k) {
    k = k < VMF_MinKappa ? 0.0 : k;

    theta.k = k;
    theta.norm = k / (M_2PI * (1 - exp(- 2 * k)));
    theta.eMin2K = exp(-2.0 * k);
    return theta;
}

float vMF(vec3 wo, VMF_Theta theta, vec3 worldPos, bool useParallaxCompensation) {
    if (theta.k == 0.0) {
        // Homogene sphere
        return M_INV_4PI;
    }

    vec3 mu = theta.mu;
    if (useParallaxCompensation && theta.distance > 0) {
        mu = normalize(theta.target - worldPos);
    }

    return theta.norm * exp(theta.k * (dot(mu, wo) - 1));
}

#define MAX_DISTRIBUTIONS 32


struct VMM_Theta {
    VMF_Theta thetas[MAX_DISTRIBUTIONS];
    float pi[MAX_DISTRIBUTIONS];
    vec3 meanPosition;
    int usedDistributions;
};

float VMM(vec3 wo, VMM_Theta vmmTheta, vec3 worldPos, bool useParallaxCompensation) {
    float res = 0;

    for (int i = 0; i < vmmTheta.usedDistributions; i++) {
        res += vmmTheta.pi[i] * vMF(wo, vmmTheta.thetas[i], worldPos, useParallaxCompensation);
    }
    return res;
}

vec3 sampleVMF(VMF_Theta theta, vec3 worldPos, bool useParallaxCompensation) {
    if (theta.k > 0.0) {
        const float r1 = rnd();
        const float r2 = rnd();
        const float cosTheta = 1.0 + log(1 + theta.eMin2K * r1 - r1) / theta.k;// log1p not available
        const float sinTheta = 1.0 - cosTheta * cosTheta <= 0.0 ? 0.0 : sqrt(1.0 - cosTheta * cosTheta);
        const float phi = 2.f * M_PI * r2;

        const float cosPhi = cos(phi);
        const float sinPhi = sin(phi);

        vec3 mu = theta.mu;
        if (useParallaxCompensation && theta.distance > 0) {
            mu = normalize(theta.target - worldPos);
        }

        return toWorld(vec3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta), mu);
    } else {
        return randomOnUnitSphere();
    }
}

vec3 sampleVMM(VMM_Theta vmmTheta, vec3 worldPos, bool useParallaxCompensation) {
    float rndDist = rnd();

    int iDistribution = 0;
    int maxDistribution = vmmTheta.usedDistributions - 1;
    float piSum = vmmTheta.pi[0];
    while (piSum < rndDist && iDistribution < maxDistribution) {
        iDistribution++;
        piSum += vmmTheta.pi[iDistribution];
    }

    return sampleVMF(vmmTheta.thetas[iDistribution], worldPos, useParallaxCompensation);
}

// Taken from lightpmm/DirectionalData.h
struct DirectionalData {
// the positin of the sample in 3D (e.g, photon position)
    vec3 position;
// direction of the sample (e.g., direction pointing to the origin of the photon)
    vec3 direction;
// the weight or value associated to this direction (e.g., photon power)
    float weight;
// the PDF for sampling/generating this directinal sample
    float pdf;
// the distance associated to the directional sample (e.g., distance to the photon's origin)
    float distance;
    uint flags;
};

#define INVALID 4294967295u
#define MAX_DIRECTIONAL_DATA_PER_PIXEL 16