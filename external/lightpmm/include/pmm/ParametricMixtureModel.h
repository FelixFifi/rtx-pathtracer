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

#ifndef PMM_PARAMETRICMIXTUREMODEL_H_
#define PMM_PARAMETRICMIXTUREMODEL_H_

#include "pmm.h"
#include "vector.h"

#include <fstream>
#include <iostream>
#include <cmath>
#include <array>

PMM_NAMESPACE_BEGIN

using std::endl;

/**
 * @brief Parametric mixture model implementation for arbitrary kernel distributions.
 *
 * The kernel distribution must typedef its scalar type wrapped in \c lightpmm::Scalar as \c ScalarType.
 * It must further contain a \c TScalar \c m_weights to store its mixture component weights.
 * It further needs to support several operations to enable the functionality provided
 * by the \c ParametricMixtureModel class, for instance \c pdf and \c sample functions.
 */
template <typename TKernel, uint32_t NKernels>
class ParametricMixtureModel
{
public:
    typedef TKernel KernelType;
    typedef Scalar<typename TKernel::ScalarType::IntegralType,
                   TKernel::ScalarType::Width::value,
                   TKernel::ScalarType::Alignment::value> ScalarType;
    static_assert (std::is_same<ScalarType, typename TKernel::ScalarType>::value, "lightpmm::Scalar is required for TScalar");
    typedef std::integral_constant<uint32_t, ScalarType::Width::value> ScalarWidth;
    typedef std::integral_constant<uint32_t, NKernels> NumKernels;
    typedef std::integral_constant<uint32_t, NKernels*ScalarWidth::value> MaxK;

    ///number of active components
    uint32_t m_K {MaxK::value};
    ///avg sample weight times number of samples
    float m_sampleWeight {0.0f};
    ///estimation of the number of samples (might include decay for updated fits)
    float m_numSamples {0.0f};
    ///total number of samples processed by all fits since initialization
    size_t m_totalNumSamples {0};
    ///number of EM iterations processed since initialization
    uint32_t m_numEMIterations {0};
    ///per component probability distribution data
    std::array<TKernel, NumKernels::value> m_comps;

public:

    ParametricMixtureModel() = default;

    /**
     * @brief pdf evaluates the probability density for the whole mixture
     * @param dir direction vector
     * @return probability density in solid angle
     */
    PMM_INLINE float pdf(const Vector3& dir)const
    {
        const Vec<ScalarType> sdir(dir);
        ScalarType sumPDF{0.0f};
        ScalarType sumWeight{0.0f};

        uint32_t k = 0;
        //loop over all but the last kernel
        while (k<(m_K-1)/ScalarWidth::value)
        {
            sumWeight += m_comps[k].m_weights;
            sumPDF    += m_comps[k].m_weights*m_comps[k].pdf(sdir);
            ++k;
        }
        //sumWeight should be close to 1, this takes care of slight deviations.
        sumWeight += m_comps[k].m_weights;
        ScalarType lastWeights = m_comps[k].m_weights;
        const float totalWeight = lightpmm::sum(sumWeight);
        const float lastComponentAdjustedWeight = m_comps[k].m_weights[(m_K-1)%ScalarWidth::value]+1.0f-totalWeight;
        PMM_ASSERT(lastComponentAdjustedWeight > 0.0f);
        lastWeights.insert((m_K-1)%ScalarWidth::value, lastComponentAdjustedWeight);
        sumPDF += lastWeights*m_comps[k].pdf(sdir);

        return lightpmm::sum(sumPDF);
    }

    /**
     * @brief pdfK evaluates the probability density for a single component
     * @param k component index
     * @param dir direction vector
     * @return probability density in solid angle for the given component
     */
    PMM_INLINE float pdfK(const uint32_t& k, const Vector3& dir)const
    {
        return m_comps[k/ScalarWidth::value].pdf(dir)[k%ScalarWidth::value];
    }

    /**
     * @brief weightK returns the weight of a mixture component
     * @param k component index
     * @return mixture weight of the component
     */
    PMM_INLINE float weightK(const uint32_t& k)const
    {
        return m_comps[k/ScalarWidth::value].m_weights[k%ScalarWidth::value];
    }

    /**
     * @brief pdf evaluates the probability density for the whole mixture when convolved with \c conv
     * @param dir direction vector
     * @param component(s) to convolve the mixture with
     * @return probability density in solid angle
     */
    PMM_INLINE float pdfConvolve(const Vector3& dir, const TKernel& conv) const
    {
        const Vec<ScalarType> sdir(dir);
        ScalarType sumPDFConvolve{0.0f};
        ScalarType sumWeight{0.0f};

        uint32_t k = 0;
        //loop over all but the last kernel
        while (k<(m_K-1)/ScalarWidth::value)
        {
            sumWeight      += m_comps[k].m_weights;
            sumPDFConvolve += m_comps[k].m_weights*m_comps[k].pdfConvolve(sdir, conv);
            ++k;
        }
        //sumWeight should be close to 1, this takes care of slight deviations.
        sumWeight += m_comps[k].m_weights;
        ScalarType lastWeights = m_comps[k].m_weights;
        const float totalWeight = lightpmm::sum(sumWeight);
        const float lastComponentAdjustedWeight = m_comps[k].m_weights[(m_K-1)%ScalarWidth::value]+1.0f-totalWeight;
        PMM_ASSERT(lastComponentAdjustedWeight > 0.0f);
        lastWeights.insert((m_K-1)%ScalarWidth::value, lastComponentAdjustedWeight);
        sumPDFConvolve += lastWeights*m_comps[k].pdfConvolve(sdir, conv);

        return lightpmm::sum(sumPDFConvolve);
    }

    /**
     * @brief sample returns a random direction vector distributed according to the mixture
     * @param random 2D random value in [0,1)
     * @return direction vector
     */
    PMM_INLINE Vector3 sample(const Point2& random)const
    {
        float sumWeight = 0.0f;

        uint32_t k;
        //loop over all but the last kernel
        for (k=0; k<(m_K-1)/ScalarWidth::value; ++k)
        {
            const float kernelWeight = lightpmm::sum(m_comps[k].m_weights);
            if (sumWeight+kernelWeight > random.y)
                break;
            sumWeight += kernelWeight;
        }
        uint32_t i;
        //loop over all but the last component in the kernel
        for (i=0; i+1<ScalarWidth::value && i+1+k*ScalarWidth::value<m_K; ++i)
        {
            const float componentWeight = m_comps[k].m_weights[i];
            if (sumWeight+componentWeight > random.y)
                break;
            sumWeight += componentWeight;
        }

        const Point2 reusedSample{random.x, std::min(std::nexttowardf(1.0f, 0.0f), (random.y-sumWeight)/m_comps[k].m_weights[i])};

        return m_comps[k].sample(i, reusedSample);
    }

    /**
     * @brief setComponent sets the components at \c idx to the values in \c comp
     * @param idx kernel index
     * @param comp mixture kernel (multiple components)
     */
    PMM_INLINE void setComponent(const uint32_t& idx, const TKernel& comp)
    {
        m_comps[idx] = comp;
    }

    /**
     * @brief getComponent retrieves the components at \c idx from the mixture (const version)
     * @param idx kernel index
     * @return mixture kernel (multiple components)
     */
    PMM_INLINE const TKernel &getComponent(const uint32_t& idx)const
    {
        return m_comps[idx];
    }

    /**
     * @brief getComponent retrieves the components at \c idx from the mixture (non-const version)
     * @param idx kernel index
     * @return mixture kernel (multiple components)
     */
    PMM_INLINE TKernel &getComponent(const uint32_t& idx)
    {
        return m_comps[idx];
    }

    /**
     * @brief normWeights normalizes the mixture's weights
     * @return sum of mixture weights before normalization
     */
    PMM_INLINE float normWeights()
    {
        const uint32_t numActiveKernels = (m_K+ScalarWidth::value-1)/ScalarWidth::value;
        ScalarType partialSumWeights {0.0f};

        //set the weight of inactive components to 0
        if (m_K%ScalarWidth::value != 0)
            m_comps[numActiveKernels-1].m_weights.cutoff(m_K%ScalarWidth::value);

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            partialSumWeights += m_comps[k].m_weights;
        }

        const float sumWeights = lightpmm::sum(partialSumWeights);
        ScalarType inv_sumWeights(1.0f/sumWeights);

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            m_comps[k].m_weights *= inv_sumWeights;
        }

        return sumWeights;
    }

    /**
     * @brief product computes the product mixture
     * @param a mixture kernel (component(s)) to compute the product with
     * @return product integral
     */
    PMM_INLINE float product(const TKernel& a)
    {
        const uint32_t numActiveKernels = (m_K+ScalarWidth::value-1)/ScalarWidth::value;
        ScalarType partialSumScale {0.0f};

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            partialSumScale += m_comps[k].product(a);
        }

        const float sumScale = lightpmm::sum(partialSumScale);
        ScalarType inv_sumScale(1.0f/sumScale);

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            m_comps[k].m_weights *= inv_sumScale;
        }

        return sumScale;
    }

    /**
     * @brief convolve computes the convolved mixture
     * @param a mixture kernel (component(s)) to compute the convolution with
     */
    PMM_INLINE void convolve(const TKernel& a)
    {
        const int cnt = (m_K+ScalarWidth::value-1) / ScalarWidth::value;
        for(int k=0;k<cnt;k++){
            m_comps[k].convolve(a);
        }
    }

    /**
     * @brief swapComponents swaps components
     * @param k0 first component index
     * @param k1 second component index
     */
    PMM_INLINE void swapComponents(const uint32_t &k0, const uint32_t &k1){
        if (k0 == k1)
            return;

        const uint32_t k0Kernel = k0/ScalarWidth::value;
        const uint32_t k0Component = k0%ScalarWidth::value;
        const uint32_t k1Kernel = k1/ScalarWidth::value;
        const uint32_t k1Component = k1%ScalarWidth::value;

        m_comps[k0Kernel].swapComponent(k0Component, k1Component, m_comps[k1Kernel]);

    }

    /**
     * @brief mergeComponents merges components into the first index
     * and swaps the component at the second index with the last component
     * @param k0 first component index
     * @param k1 second component index
     */
    PMM_INLINE void mergeComponents(const uint32_t &k0, const uint32_t &k1){
        if (k0 == k1)
            return;

        const uint32_t k0Kernel = k0/ScalarWidth::value;
        const uint32_t k0Component = k0%ScalarWidth::value;
        const uint32_t k1Kernel = k1/ScalarWidth::value;
        const uint32_t k1Component = k1%ScalarWidth::value;

        m_comps[k0Kernel].mergeComponent(k0Component, k1Component, m_comps[k1Kernel]);
        m_K -= 1;
        swapComponents(k1,m_K);
    }

    /**
     * @brief removeWeightPrior removes weight prior applied by \c addWeightPrior
     * @param weightPrior approximately minimum weight per component
     */
    PMM_INLINE void removeWeightPrior(const float weightPrior) {
        const uint32_t numComponents = m_K;
        const uint32_t numActiveKernels = (m_K+ScalarWidth::value-1)/ScalarWidth::value;

        const float invWPriorNormalization = weightPrior*static_cast<float>(numComponents)+1.0f;
        const float wPriorNormalization = 1.0f/invWPriorNormalization;
        const float wPriorTimesNormalization = weightPrior*wPriorNormalization;

        for (uint32_t k=0; k<numActiveKernels; ++k)
            m_comps[k].m_weights = (m_comps[k].m_weights-wPriorTimesNormalization)*invWPriorNormalization;

        if (m_K%ScalarWidth::value != 0)
            m_comps[numActiveKernels-1].m_weights.cutoff(m_K%ScalarWidth::value);
    }

    /**
     * @brief applyWeightPrior applies a prior on component weight
     * @param weightPrior approximately minimum weight per component
     */
    PMM_INLINE void applyWeightPrior(const float weightPrior) {
        const uint32_t numComponents = m_K;
        const uint32_t numActiveKernels = (m_K+ScalarWidth::value-1)/ScalarWidth::value;

        const float invWPriorNormalization = weightPrior*static_cast<float>(numComponents)+1.0f;
        const float wPriorNormalization = 1.0f/invWPriorNormalization;
        const float wPriorTimesNormalization = weightPrior*wPriorNormalization;

        for (uint32_t k=0; k<numActiveKernels; ++k)
            m_comps[k].m_weights = m_comps[k].m_weights*wPriorNormalization+wPriorTimesNormalization;

        if (m_K%ScalarWidth::value != 0)
            m_comps[numActiveKernels-1].m_weights.cutoff(m_K%ScalarWidth::value);
    }

    void serialize(std::ostream& stream)const
    {
        stream.write(reinterpret_cast<const char*>(&m_K), sizeof(m_K));
        stream.write(reinterpret_cast<const char*>(&m_sampleWeight), sizeof(m_sampleWeight));
        stream.write(reinterpret_cast<const char*>(&m_numSamples), sizeof(m_numSamples));
        stream.write(reinterpret_cast<const char*>(&m_totalNumSamples), sizeof(m_totalNumSamples));
        stream.write(reinterpret_cast<const char*>(&m_numEMIterations), sizeof(m_numEMIterations));

        for(uint32_t k=0;k<NKernels;k++){
            m_comps[k].serialize(stream);
        }
    }

    void deserialize(std::istream& stream)
    {
        stream.read(reinterpret_cast<char*>(&m_K), sizeof(m_K));
        stream.read(reinterpret_cast<char*>(&m_sampleWeight), sizeof(m_sampleWeight));
        stream.read(reinterpret_cast<char*>(&m_numSamples), sizeof(m_numSamples));
        stream.read(reinterpret_cast<char*>(&m_totalNumSamples), sizeof(m_totalNumSamples));
        stream.read(reinterpret_cast<char*>(&m_numEMIterations), sizeof(m_numEMIterations));

        for(uint32_t k=0;k<NKernels;k++){
            m_comps[k].deserialize(stream);
        }
    }

    /// Returns a string representation of the mixture class.
    PMM_INLINE const std::string toString()const
    {
        std::ostringstream oss;
        float sumW = 0.0f;
        oss << "PMM<"<< NKernels*4<< ">[" << endl; //
        for(uint32_t k = 0; k <NKernels; k++){
            oss << m_comps[k].toString() <<endl;
            sumW += lightpmm::sum(m_comps[k].m_weights);
        }
        oss << "K = " << m_K << endl;
        oss << "sampleWeight = "<< m_sampleWeight << endl;
        oss << "numSamples = "<< m_numSamples << endl;
        oss << "totalNumSamples = "<< m_totalNumSamples << endl;
        oss << "numEMIterations = "<< m_numEMIterations << endl;
        oss << "sumMixtureWeight = "<< sumW << endl;

        oss << "]";
        return oss.str();

    }

    /**
     * @brief setK sets the number of mixture components
     * components beyond \c _K are reset
     * @param _K new number of mixture components
     */
    PMM_INLINE void setK(const uint32_t& _K)
    {
        m_K = _K;
        const uint32_t fullKernels = m_K/ScalarWidth::value;
        const uint32_t remainingComponents = m_K%ScalarWidth::value;

        if (remainingComponents > 0)
            m_comps[fullKernels].cutoff(remainingComponents);

        //this should be unnecessary. all functions should check for the last active kernel and ignore the remaining kernels
        for (uint32_t k = fullKernels+1; k<NKernels;k++)
        {
            m_comps[k].reset();
        }
    }

    /**
     * @brief getK returns the number of mixture components
     * @return number of mixture components
     */
    PMM_INLINE uint32_t getK() const
    {
        return m_K;
    }

    bool valid(const float minComponentWeight=0.0f) const
    {
        ScalarType partialSumWeights{0.0f};

        for (uint32_t k=0; k<(getK()+ScalarWidth::value-1)/ScalarWidth::value; k++)
        {
            const KernelType& kernel = m_comps[k];

            if (!kernel.valid())
            {
                PMM_WARN("invalid kernel in ParametricMixtureModel\n%s", toString().c_str());
                return false;
            }

            typename ScalarType::BooleanType activeComponent;
            for (uint32_t i=0; i<ScalarType::Width::value; ++i)
                activeComponent.insert(i, k*ScalarWidth::value+i < getK());

            if (lightpmm::any((kernel.m_weights < minComponentWeight && activeComponent) || kernel.m_weights < 0.0f || kernel.m_weights > 1.001f || vcl::is_nan(kernel.m_weights) || vcl::is_inf(kernel.m_weights)))
            {
                PMM_WARN("invalid weight in ParametricMixtureModel\n%s", toString().c_str());
                return false;
            }
            partialSumWeights += kernel.m_weights;
        }

        const float sumWeights = lightpmm::sum(partialSumWeights);

        if (sumWeights < 0.99f || sumWeights > 1.01f)
        {
            PMM_WARN("ParametricMixtureModel weights do not sum up to 1.0\n%s", toString().c_str());
            return false;
        }

        return true;
    }

public:
    class SoftAssignmentWeights
    {
    private:
        ScalarType softAssignments[NKernels];
        float mixturePDF {0.0f};

    public:
        /**
         * @brief SoftAssignmentWeights computes soft assignment weights for Expectation Maximization
         * @param pmm parametric mixture model
         * @param dir direction vector
         */
        SoftAssignmentWeights(const ParametricMixtureModel<TKernel, NKernels>& pmm, const Vec<ScalarType>& dir)
        {
            const uint32_t numComponents = pmm.getK();
            const uint32_t numActiveKernels = (numComponents+ScalarWidth::value-1)/ScalarWidth::value;

            ScalarType partialSumPDF{0.0f};

            for(uint32_t k=0; k<numActiveKernels; ++k)
            {
                softAssignments[k] = pmm.getComponent(k).m_weights*pmm.getComponent(k).pdf(dir);
                partialSumPDF += softAssignments[k];
            }

            mixturePDF = sum(partialSumPDF);
            const float invMixturePDF = 1.0f/mixturePDF;

            for(uint32_t k=0; k<numActiveKernels; ++k)
                softAssignments[k] *= invMixturePDF;

            for(uint32_t k=numActiveKernels; k<NKernels; ++k)
                softAssignments[k] = 0.0f;
        }

        /**
         * @brief getSoftAssignment returns the soft assignment to the given component index
         * @param componentIndex component index
         * @return soft assignment to the given component index
         */
        PMM_INLINE float getSoftAssignment(const uint32_t componentIndex) const
        {
            return softAssignments[componentIndex/ScalarWidth::value][componentIndex%ScalarWidth::value];
        }

        /**
         * @brief getSoftAssignment returns the soft assignments to the given kernel index
         * @param kernelIndex kernel index
         * @return soft assignments for the given kernel index
         */
        PMM_INLINE ScalarType getSoftAssignments(const uint32_t kernelIndex) const
        {
            return softAssignments[kernelIndex];
        }

        /**
         * @brief valid checks if the sample has a sufficient PDF in the mixture to avoid numerical issues
         * at a too low PDF, assignments become numerically unstable and negatively impact the fitting process
         * @return sample is safe to use
         */
        PMM_INLINE bool valid() const
        {
            return mixturePDF > PMM_EPSILON;
        }

        /**
         * @brief getMixturePDF returns the pdf of the sample in the mixture in solid angle
         * @return pdf of the sample in the mixture in solid angle
         */
        PMM_INLINE float getMixturePDF() const
        {
            return mixturePDF;
        }

        /**
         * @brief toString returns a string representation of the soft assignments
         * @return string representation
         */
        std::string toString() const {
            std::ostringstream oss;
            int count = 0;
            oss << "SoftAssignmentWeights<"<< NKernels*ScalarWidth::value<< ">[" << endl;

            for(uint32_t k=0; k<NKernels; ++k){
                for(uint32_t i=0; i<ScalarWidth::value; ++i, ++count){
                    oss << "  softAssignments["<< count <<"]: " << softAssignments[k][i] << endl;
                }
            }
            oss << "  mixturePDF = " << mixturePDF << endl;
            oss << "]";
            return oss.str();
        }
    };
};

PMM_NAMESPACE_END

#endif /* PMM_PARAMETRICMIXTUREMODEL_H_ */
