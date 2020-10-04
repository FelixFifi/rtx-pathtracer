///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, Sebastian Herholz, Lukas Ruppert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the author nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#ifndef PMM_VMFKERNEL_H_
#define PMM_VMFKERNEL_H_

#include "pmm.h"
#include "vector.h"

#include <iostream>
#include <math.h>

PMM_NAMESPACE_BEGIN

// the minumum value of kappa before it gets set to 0.0 for numerical stability
#define VMF_MinKappa 1e-3f

#define INV_FOURPI 0.07957747155

template<typename TScalar>
PMM_INLINE TScalar kappaToMeanCosine(const TScalar& kappa){
    //coth goes to 1.0 fast. no need to evaluate it
    //exact for kappa > 19.061546, still extremely close for kappa > 5.0f
    if (lightpmm::all(kappa > 5.0f))
        return 1.0f-1.0f/kappa;

    TScalar r = 1.0f/lightpmm::tanh(kappa)-1.0f/kappa;
    return lightpmm::ifthen(kappa > 0.0f, r, 0.0f);
}

template<typename TScalar>
PMM_INLINE TScalar meanCosineToKappa(const TScalar& r){
    PMM_ASSERT(lightpmm::all(r < 1.0f));
    PMM_ASSERT(lightpmm::all(r>= 0.0f));
    const TScalar ones(1.0f);
    const TScalar d(3.f);

    //approximation by Banerjee et al. 2005
    return (r*d - (r*r*r)) / (ones - r*r);
}

/**
 * @brief von Mises-Fisher distribution kernel for use in \c ParametricMixtureModel.
 */
template <typename TScalar2>
class VMFKernel
{
    // VMFKernel is only implemented for scalars wrapped using the Scalar struct defined in pmm.h,
    // since it depends on the constants defined there.
    using TScalar = Scalar<typename TScalar2::IntegralType,
                    TScalar2::Width::value, TScalar2::Alignment::value>;
    static_assert (std::is_same<TScalar, TScalar2>::value, "lightpmm::Scalar is required for TScalar2");
public:

    typedef TScalar ScalarType;
    TScalar m_weights;      /// mixture weights
    Vec<TScalar> m_mu;      /// mean directions of the vMF lobes

private:

    TScalar m_kappa;        /// vMF concentration parameter
    TScalar m_r;            /// mean cosine of the lobe (used to estimate kappa)
    TScalar m_norm;         /// precomputed normalization factors for the vMF lobes
    TScalar m_eMin2Kappa;   /// precomputed exp(-2*kappa)

public:

    VMFKernel() = default;

    VMFKernel(const TScalar weights, const TScalar kappas, const Vec<TScalar> mus)
        : m_weights{weights}, m_mu{mus}, m_kappa{ifthen(kappas < VMF_MinKappa, 0.0f, kappas)},
          m_r{kappaToMeanCosine<TScalar>(m_kappa)}
    {
        calNormalization();
    }

    PMM_INLINE TScalar pdf(const Vec<TScalar>& dir) const
    {
        const TScalar cosTheta = dot(m_mu, dir);

        // ensure that cosTheta is <= 1.0 -> t <= 0.0
        const TScalar t = lightpmm::min(cosTheta-1.0f, 0.0f);
        const TScalar e = lightpmm::exp(m_kappa*t);
        PMM_ASSERT(lightpmm::isfinite(e));
        return m_norm * e;
     }

    /// Function that evalautes the PDF of the VMF distribution
    /// after a convolution by another VMF kernel.
    /// dir: ther direction the convolved VMF is evaluated
    /// conv: the VMF kernel used for the convolution
    PMM_INLINE TScalar pdfConvolve(const Vec<TScalar>& dir, const VMFKernel<TScalar> &conv) const
    {
        const TScalar ones(1.0f);
        const TScalar zeros(0.0f);
        const TScalar invFourPi(1.0f/(4.0f*M_PI));
        const TScalar cosTheta = dot(m_mu, dir);

        const TScalar r0 = m_r;
        const TScalar r1 = conv.m_r;

        // calculate the mean cosine of the convolved distribution
        const TScalar r = r0*r1;
        PMM_ASSERT(lightpmm::isfinite(r));

        TScalar kappa = meanCosineToKappa<TScalar>(r);
        PMM_ASSERT(lightpmm::isfinite(kappa));
        kappa = ifthen(kappa < VMF_MinKappa, zeros, kappa);

        const TScalar eMin2Kappa = lightpmm::exp(TScalar(-2.0f)*kappa);
        TScalar norm = kappa/(2.0f*M_PI*(1.0f-eMin2Kappa));
        norm = lightpmm::ifthen(kappa > 0.0f, norm, invFourPi);
        PMM_ASSERT(lightpmm::isfinite(norm));

        // ensure that cosTheta is <= 1.0 -> t <= 0.0
        TScalar t = lightpmm::min(cosTheta-ones, zeros);
        TScalar e = lightpmm::exp(kappa*t);
        PMM_ASSERT(lightpmm::isfinite(e));
        return norm * e;
     }

    PMM_INLINE Vector3 sample(const uint32_t idx, const Point2 random) const
    {
        PMM_ASSERT(random[0] < 1.0f);
        PMM_ASSERT(random[1] < 1.0f);

        Vector3 dir;

        const float kappa = m_kappa[idx];

        if (kappa > 0.0f)
        {
            const float expMin2Kappa = m_eMin2Kappa[idx];
            const Vector3 mu = m_mu[idx];

            const float cosTheta = 1.f + std::log1p(expMin2Kappa*random[0]-random[0])/kappa;
            const float sinTheta = (1.0f-cosTheta*cosTheta <= 0.0f) ? 0.0f : std::sqrt(1.f-cosTheta*cosTheta);
            const float phi = 2.f * M_PI * random[1];

            float sinPhi,cosPhi;
            sincosf(phi, &sinPhi, &cosPhi);
            dir = lightpmm::sphericalDirection(cosTheta, sinTheta, cosPhi, sinPhi);
            PMM_ASSERT(std::isfinite(dir[0]) && std::isfinite(dir[1]) && std::isfinite(dir[2]));

            dir = lightpmm::frameToWorld(mu,dir);
        }
        else
        {
            dir = lightpmm::squareToUniformSphere(random);
            PMM_ASSERT(std::isfinite(dir[0]) && std::isfinite(dir[1]) && std::isfinite(dir[2]));
        }

        return dir;
    }

    PMM_INLINE void setWeight(const TScalar& weight)
    {
        m_weights = weight;
    }

    PMM_INLINE void setWeight(const uint32_t& idx, const float& value)
    {
        lightpmm::insert(m_weights, idx, value);
    }

    PMM_INLINE float getWeight(const uint32_t& idx) const
    {
        return m_weights[idx];
    }

    PMM_INLINE void setKappa(const uint32_t& idx, const float& kappa)
    {
        const float _kappa = ifthen(kappa < VMF_MinKappa, 0.0f, kappa);
        lightpmm::insert(m_kappa, idx, _kappa);
        lightpmm::insert(m_r, idx, kappaToMeanCosine<float>(_kappa));
        calNormalization();
    }

    PMM_INLINE void setKappa(const TScalar& kappa)
    {
        m_kappa = ifthen(kappa < VMF_MinKappa, 0.0f, kappa);
        m_r = kappaToMeanCosine<TScalar>(m_kappa);
        calNormalization();
    }

    PMM_INLINE void setKappaAndR(const uint32_t& idx, const float& kappa, const float& r)
    {
        const float _kappa = ifthen(kappa < VMF_MinKappa, 0.0f, kappa);
        const float _r = ifthen(kappa < VMF_MinKappa, 0.0f, r);
        lightpmm::insert(m_kappa, idx, _kappa);
        lightpmm::insert(m_r, idx, _r);
        calNormalization();
    }

    PMM_INLINE void setKappaAndR(const TScalar& kappa, const TScalar &r)
    {
        const auto checkKappa = (kappa < VMF_MinKappa);
        m_kappa = ifthen(checkKappa, 0.0f, kappa);
        m_r = ifthen(checkKappa, 0.0f, r);
        calNormalization();
    }

    PMM_INLINE TScalar getKappa() const
    {
        return m_kappa;
    }

    PMM_INLINE float getKappa(const uint32_t& idx) const
    {
        return m_kappa[idx];
    }

    /// Returns the mean cosine of the VMF kernel
    PMM_INLINE TScalar getR() const
    {
        return m_r;
    }

    PMM_INLINE void setMu(const Vec<TScalar>& mu)
    {
        m_mu = mu;
    }

    PMM_INLINE void setMu(const Vector3& mu)
    {
        m_mu.x = TScalar(mu[0]);
        m_mu.y = TScalar(mu[1]);
        m_mu.z = TScalar(mu[2]);
    }

    PMM_INLINE void setMu(const uint32_t& idx, const Vector3& value)
    {
        lightpmm::insert(m_mu.x, idx, value[0]);
        lightpmm::insert(m_mu.y, idx, value[1]);
        lightpmm::insert(m_mu.z, idx, value[2]);
    }

    PMM_INLINE Vector3 getMu(const uint32_t& idx) const
    {
        return Vector3(m_mu.x[idx],m_mu.y[idx],m_mu.z[idx]);
    }


    PMM_INLINE void normWeights()
    {
        const float sumW = lightpmm::sum(m_weights);
        PMM_ASSERT(sumW > 0.0f);
        m_weights *= TScalar(1.0f/sumW);
    }


    ///broadcast the data from index idx to all components in the kernel
    PMM_INLINE void broadcast(const uint32_t idx)
    {
        m_weights    = m_weights[idx];
        m_mu         = m_mu[idx];
        m_kappa      = m_kappa[idx];
        m_r          = m_r[idx];
        m_norm       = m_norm[idx];
        m_eMin2Kappa = m_eMin2Kappa[idx];
    }

    ///extract a VMFKernel with TScalar::Width copies of the component at index idx
    PMM_INLINE VMFKernel<TScalar> extract(const uint32_t idx) const
    {
        VMFKernel<TScalar> copy{*this};
        copy.broadcast(idx);
        return copy;
    }

    ///copy component at index idx from source
    PMM_INLINE void insert(const VMFKernel<TScalar>& source, const uint32_t idx)
    {
        lightpmm::insert(m_weights,    idx, source.m_weights[idx]);
        lightpmm::insert(m_mu.x,       idx, source.m_mu.x[idx]);
        lightpmm::insert(m_mu.y,       idx, source.m_mu.y[idx]);
        lightpmm::insert(m_mu.z,       idx, source.m_mu.z[idx]);
        lightpmm::insert(m_kappa,      idx, source.m_kappa[idx]);
        lightpmm::insert(m_r,          idx, source.m_r[idx]);
        lightpmm::insert(m_norm,       idx, source.m_norm[idx]);
        lightpmm::insert(m_eMin2Kappa, idx, source.m_eMin2Kappa[idx]);
    }

    /**
     * @brief product computes the vMF product distribution
     * @param a vMF distribution(s) to compute the product with
     * @return product integral (scaled by mixture weights)
     */
    PMM_INLINE TScalar product(const VMFKernel<TScalar>& a)
    {
        Vec<TScalar> newMu = m_mu*m_kappa + a.m_mu*a.m_kappa;
        TScalar newKappa = newMu.length();

        // check if the new kappa is in the numerical stabe region
        // or if it is 0.0
        const auto checkKappa = (newKappa < VMF_MinKappa);
        newKappa = ifthen(checkKappa, 0.0f, newKappa);

        // only update mu if kappa is > 0.0 (or minKappa)
        newMu = lightpmm::ifthen(checkKappa, m_mu, newMu/newKappa);

        PMM_ASSERT(lightpmm::isfinite(new_mu));

        const TScalar C_i = m_norm;
        const TScalar C_j = a.m_norm;

        const TScalar new_eMin2Kappa = lightpmm::exp(TScalar(-2.0f)*newKappa);

        const float zeroKappaNorm(1.0f/(4.0f*M_PI));
        const TScalar C_ij = lightpmm::ifthen(checkKappa, zeroKappaNorm, (1.0f/(2.0f*M_PI))*newKappa/(1.0f-new_eMin2Kappa));

        PMM_ASSERT(lightpmm::isfinite(C_ij));
        PMM_ASSERT(lightpmm::all(C_ij > 0.0f));

        const TScalar ti = dot(newMu, m_mu);
        const TScalar tj = dot(newMu, a.m_mu);

        const TScalar t = m_kappa*(ti-1.0f)+a.m_kappa*(tj-1.0f);
        const TScalar e = lightpmm::exp(t);

        const TScalar s = e*m_weights*a.m_weights*(C_i*C_j)/C_ij;

        m_weights = s;
        m_kappa = newKappa;
        m_r = kappaToMeanCosine<TScalar>(m_kappa);
        m_mu = newMu;
        m_norm = C_ij;
        m_eMin2Kappa = new_eMin2Kappa;

        return s;
    }

    /**
     * @brief division computes the vMF division distribution
     * @param other vMF distribution(s) to compute the product with
     * @return division integral (scaled by mixture weights)
     */
    PMM_INLINE TScalar division(const VMFKernel<TScalar>& other)
    {
        Vec<TScalar> newMu = m_mu*m_kappa-other.m_mu*other.m_kappa;
        TScalar newKappa = newMu.length();

        // check if the new kappa is in the numerical stabe region
        // or if it is 0.0
        const auto checkKappa = (newKappa < VMF_MinKappa);
        newKappa = ifthen(checkKappa, 0.0f, newKappa);

        // only update mu if kappa is > 0.0 (or minKappa)
        newMu = lightpmm::ifthen(checkKappa, m_mu, newMu/newKappa);

        PMM_ASSERT(lightpmm::isfinite(new_mu));

        const TScalar C_i = m_norm;
        const TScalar C_j = other.m_norm;

        const TScalar newExpMin2Kappa = lightpmm::exp(TScalar(-2.0f)*newKappa);

        const float zeroKappaNorm(1.0f/(4.0f*M_PI));
        const TScalar C_ij = lightpmm::ifthen(checkKappa, zeroKappaNorm, (1.0f/(2.0f*M_PI))*newKappa/(1.0f-newExpMin2Kappa));

        PMM_ASSERT(lightpmm::isfinite(C_ij));
        PMM_ASSERT(lightpmm::all(C_ij > 0.0f));

        const TScalar muIDotNewMu = dot(newMu, m_mu);
        const TScalar muJDotNewMu = dot(newMu, other.m_mu);

        const TScalar t = m_kappa*(muIDotNewMu-1.0f)-other.m_kappa*(muJDotNewMu-1.0f);
        //NOTE: this can go to infinity for large other.m_kappa
        const TScalar e = lightpmm::exp(t);

        const TScalar s = e*m_weights*C_i/(other.m_weights*C_j*C_ij);

        m_weights = s;
        m_kappa = newKappa;
        m_r = kappaToMeanCosine<TScalar>(m_kappa);
        m_mu = newMu;
        m_norm = C_ij;
        m_eMin2Kappa = newExpMin2Kappa;

        return s;
    }

    ///compute the integral of each component's product with itself
    PMM_INLINE TScalar selfProductIntegral() const
    {
        return m_weights*m_weights*m_kappa*INV_FOURPI*lightpmm::ifthen((1.0f-m_eMin2Kappa)*(1.0f-m_eMin2Kappa) > 0.0f, (1.0f-m_eMin2Kappa*m_eMin2Kappa)/((1.0f-m_eMin2Kappa)*(1.0f-m_eMin2Kappa)), 1.0f);
    }

    /// Convolves the current VMF kernel with another VMF kernel
    PMM_INLINE void convolve(const VMFKernel<TScalar>& a)
    {
        const TScalar r_i = m_r;
        const TScalar r_j = a.m_r;

        // calculate the mean cosine of the convolved distribution
        const TScalar r = r_i*r_j;

        m_kappa = meanCosineToKappa<TScalar>(r);
        m_kappa = ifthen(m_kappa < VMF_MinKappa, 0.0f, m_kappa);
        m_r = r;
        calNormalization();
    }

    /**
     * @brief swapComponent swaps a component with one from a potentially foreign kernel
     * @param k0 component index in kernel
     * @param k1 component index in foreign kernel
     * @param a foreign kernel
     */
    PMM_INLINE void swapComponent(const int &k0, const int &k1,  VMFKernel<TScalar>& a)
    {
        float weight = m_weights[k0];
        float kappa = m_kappa[k0];
        float r = m_r[k0];

        float mu_x = m_mu.x[k0];
        float mu_y = m_mu.y[k0];
        float mu_z = m_mu.z[k0];

        float norm = m_norm[k0];
        float eMin2Kappa = m_eMin2Kappa[k0];

        lightpmm::insert(m_weights, k0, a.m_weights[k1]);
        lightpmm::insert(m_kappa, k0, a.m_kappa[k1]);
        lightpmm::insert(m_r, k0, a.m_r[k1]);

        lightpmm::insert(m_mu.x, k0, a.m_mu.x[k1]);
        lightpmm::insert(m_mu.y, k0, a.m_mu.y[k1]);
        lightpmm::insert(m_mu.z, k0, a.m_mu.z[k1]);

        lightpmm::insert(m_norm, k0, a.m_norm[k1]);
        lightpmm::insert(m_eMin2Kappa, k0, a.m_eMin2Kappa[k1]);

        lightpmm::insert(a.m_weights, k1, weight);
        lightpmm::insert(a.m_kappa, k1, kappa);
        lightpmm::insert(a.m_r, k1, r);

        lightpmm::insert(a.m_mu.x, k1, mu_x);
        lightpmm::insert(a.m_mu.y, k1, mu_y);
        lightpmm::insert(a.m_mu.z, k1, mu_z);

        lightpmm::insert(a.m_norm, k1, norm);
        lightpmm::insert(a.m_eMin2Kappa, k1, eMin2Kappa);
    }

    /**
     * @brief mergeComponent merges a component with one from a potentially foreign kernel, which is then reset
     * @param k0 component index in kernel
     * @param k1 component index in foreign kernel
     * @param a foreign kernel
     */
    PMM_INLINE void mergeComponent(const int &k0, const int &k1,  VMFKernel<TScalar>& a)
    {
        float weight0 = m_weights[k0];
        float weight1 = a.m_weights[k1];

        float r0 = m_r[k0];
        float r1 = a.m_r[k1];

        float kappa = 0.0f;
        float norm = 1.0f/(4.0f*M_PI);
        float eMin2Kappa = 1.0f;

        float weight = weight0 + weight1;

        float mu_x = weight0*r0*m_mu.x[k0] + weight1*r1*a.m_mu.x[k1];
        float mu_y = weight0*r0*m_mu.y[k0] + weight1*r1*a.m_mu.y[k1];
        float mu_z = weight0*r0*m_mu.z[k0] + weight1*r1*a.m_mu.z[k1];

        PMM_ASSERT(weight > 0.0f);

        mu_x /= weight;
        mu_y /= weight;
        mu_z /= weight;

        float r = mu_x*mu_x + mu_y*mu_y + mu_z*mu_z;
        if(r > 0.0f){
            r = std::sqrt(r);
            kappa = meanCosineToKappa<float>(r);
            kappa = ifthen(kappa < VMF_MinKappa, 0.0f, kappa);

            eMin2Kappa = std::exp(-2.0f*kappa);
            norm = kappa/(2.0f*M_PI*(1.0f-eMin2Kappa));

            mu_x /= r;
            mu_y /= r;
            mu_z /= r;
        }else{
            mu_x = m_mu.x[k0];
            mu_y = m_mu.y[k0];
            mu_z = m_mu.z[k0];
        }

        lightpmm::insert(m_weights, k0, weight);
        lightpmm::insert(m_kappa, k0, kappa);
        lightpmm::insert(m_r, k0, r);

        lightpmm::insert(m_mu.x, k0, mu_x);
        lightpmm::insert(m_mu.y, k0, mu_y);
        lightpmm::insert(m_mu.z, k0, mu_z);

        lightpmm::insert(m_norm, k0, norm);
        lightpmm::insert(m_eMin2Kappa, k0, eMin2Kappa);

        a.reset(k1);
    }

    /**
     * @brief reset resets the entire kernel
     */
    PMM_INLINE void reset()
    {
        m_weights = TScalar(0.0f);
        m_kappa = TScalar(0.0f);
        m_r = TScalar(0.0f);

        m_mu.x = TScalar(0.0f);
        m_mu.y = TScalar(0.0f);
        m_mu.z = TScalar(1.0f);

        m_norm = TScalar(1.0f/(4.0f*M_PI));
        m_eMin2Kappa = TScalar(1.0f);
    }

    /**
     * @brief reset resets component \c k
     * @param k component index
     */
    PMM_INLINE void reset(const uint32_t& k)
    {
        lightpmm::insert(m_weights, k, 0.0f);
        lightpmm::insert(m_kappa, k, 0.0f);
        lightpmm::insert(m_r, k, 0.0f);

        lightpmm::insert(m_mu.x, k, 0.0f);
        lightpmm::insert(m_mu.y, k, 0.0f);
        lightpmm::insert(m_mu.z, k, 1.0f);

        lightpmm::insert(m_norm, k, 1.0f/(4.0f*M_PI));
        lightpmm::insert(m_eMin2Kappa, k, 1.0f);
    }

    /**
     * @brief cutoff resets the last \c TScalar::Size-k components
     * @param k number of active components in the kernel
     */
    PMM_INLINE void cutoff(const uint32_t& k)
    {
        if (k==0)
            reset();
        else
        {
            const typename TScalar::BooleanType activeComponents {!!(TScalar{1.0f}.cutoff(k))};

            m_weights    = lightpmm::ifthen(activeComponents, m_weights,    0.0f);
            m_kappa      = lightpmm::ifthen(activeComponents, m_kappa,      0.0f);
            m_r          = lightpmm::ifthen(activeComponents, m_r,          0.0f);
            m_mu         = lightpmm::ifthen(activeComponents, m_mu,         Vec<TScalar>{0.0f, 0.0f, 1.0f});
            m_norm       = lightpmm::ifthen(activeComponents, m_norm,       1.0f/(4.0f*M_PI));
            m_eMin2Kappa = lightpmm::ifthen(activeComponents, m_eMin2Kappa, 1.0f);
        }
    }

    void serialize(std::ostream& stream) const
    {
        static_assert(std::is_trivially_copyable<VMFKernel<TScalar>>::value, "not trivially serializable");

        stream.write(reinterpret_cast<const char*>(this), sizeof(VMFKernel<TScalar>));
    }

    void deserialize(std::istream& stream)
    {
        stream.read(reinterpret_cast<char*>(this), sizeof(VMFKernel<TScalar>));
    }

    /**
     * @brief toString returns a string representation of the vMF kernel
     * @return string representation of the vMF kernel
     */
    PMM_INLINE const std::string toString() const
    {
        std::ostringstream oss;
        oss << "VMFKernel<"<< TScalar::Width::value << ">[" << endl;
        for(uint32_t k = 0; k < TScalar::Width::value; k++){
            oss << "\t" << "[" << k << "]: " << "weight = " << m_weights[k]<< "\tkappa = " << m_kappa[k]<< "\tr = " << m_r[k] << "\tmu: [" << m_mu.x[k] << "\t" << m_mu.y[k] << "\t" << m_mu.z[k] << "]" << "\tnorm = " << m_norm[k] << "\teMin2Kappa = " << m_eMin2Kappa[k] <<endl;
        }
        oss << "]";
        return oss.str();
    }

    /**
     * @brief calNormalization precomputes \c m_norm and \c m_eMin2Kappa
     */
    PMM_INLINE void calNormalization()
    {
        const TScalar zeroKappaNorm(1.0f/(4.0f*M_PI));
        m_eMin2Kappa = lightpmm::exp(TScalar(-2.0f)*m_kappa);

        const TScalar norm = m_kappa/(2.0f*M_PI*(1.0f-m_eMin2Kappa));
        m_norm = lightpmm::ifthen(m_kappa > 0.0f, norm, zeroKappaNorm);
        PMM_ASSERT(lightpmm::isfinite(m_norm));
    }

    /**
     * @brief valid checks the vMF parameters for validity
     * @return wheter the parameters are valid
     */
    bool valid() const
    {
        if (lightpmm::any(m_kappa < 0.0f || vcl::is_nan(m_kappa) || vcl::is_inf(m_kappa)))
        {
            PMM_WARN("invalid kappa value in VMFKernel\n%s", toString().c_str());
            return false;
        }
        if (lightpmm::any(m_r < 0.0f || m_r > 1.0f || vcl::is_nan(m_r) || vcl::is_inf(m_r)))
        {
            PMM_WARN("invalid r value in VMFKernel\n%s", toString().c_str());
            return false;
        }
        if (lightpmm::any(m_mu.length() < 0.9f || m_mu.length() > 1.1f
                          || vcl::is_nan(m_mu.x) || vcl::is_inf(m_mu.x)
                          || vcl::is_nan(m_mu.y) || vcl::is_inf(m_mu.y)
                          || vcl::is_nan(m_mu.z) || vcl::is_inf(m_mu.z)))
        {
            PMM_WARN("invalid mu in VMFKernel\n%s", toString().c_str());
            return false;
        }

        return true;
    }

    /**
     * @brief The SufficientStats struct provides an easy way to compute the necessary sufficient statistics
     * for the vMF distribution from arbitrary sample data
     */
    struct SufficientStats
    {
        /// r*W
        Vec<TScalar> muTimesAvgCosineTimesSumWeight {Vector3{0.0f}};
        /// W
        TScalar sumWeight {0.0f};

        /**
         * @brief accumulateSample accumulates the statistics for a given sample and given assignment
         * @param sample sample data, e.g. \c lightpmm::DirectionalData
         * @param assignment assignment of the sample to the individual components
         */
        template<typename TDirectionalData>
        PMM_INLINE void accumulateSample(const TDirectionalData& sample, const TScalar assignment)
        {
            const TScalar softAssignmentTimesSampleWeight = assignment*sample.weight;

            muTimesAvgCosineTimesSumWeight += Vec<TScalar>{sample.direction}*softAssignmentTimesSampleWeight;
            sumWeight += softAssignmentTimesSampleWeight;
        }

        void clear()
        {
            *this = std::move(SufficientStats{});
        }

        SufficientStats& operator+=(const SufficientStats& other)
        {
            muTimesAvgCosineTimesSumWeight += other.muTimesAvgCosineTimesSumWeight;
            sumWeight += other.sumWeight;

            return *this;
        }

        SufficientStats operator+(const SufficientStats& other) const
        {
            SufficientStats copy{*this};
            copy += other;
            return copy;
        }

        SufficientStats& operator*=(const float factor)
        {
            muTimesAvgCosineTimesSumWeight *= factor;
            sumWeight *= factor;

            return *this;
        }

        SufficientStats operator*(const float factor) const
        {
            SufficientStats copy{*this};
            copy *= factor;
            return copy;
        }

        std::string toString() const
        {
            std::ostringstream oss;

            oss << "VMFKernel::SufficientStats [\n"
                << "  muTimesAvgCosineTimesSumWeight (r*W) = " << muTimesAvgCosineTimesSumWeight.toString() << '\n'
                << "  sumWeight (W) = " << sumWeight.toString() << '\n'
                << "]";

            return oss.str();
        }
    };
};

PMM_NAMESPACE_END

#endif /* INCLUDE_MITSUBA_GUIDING_PMM_VMFKERNEL_H_ */
