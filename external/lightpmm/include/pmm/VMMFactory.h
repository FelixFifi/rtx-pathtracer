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

#ifndef PMM_VMMFACTORY_H_
#define PMM_VMMFACTORY_H_

#include "pmm.h"
#include "vector.h"
#include "ParametricMixtureModel.h"
#include "VMFKernel.h"

#include <fstream>
#include <sstream>

#include <type_traits>

PMM_NAMESPACE_BEGIN

struct VMMFactoryProperties
{
    uint32_t minItr {1};
    uint32_t maxItr {100};
    float relLogLikelihoodThreshold{0.005f};

    uint32_t numInitialComponents {8};

    float initKappa {5.0f};
    float maxKappa {10000.0f};

    //weight priors:
    // percentage of minimal weight for a component (e.g. 0.01 = 1.0%)
    float vPrior {0.01f};
    // kappa prior
    float rPrior {0.0f};
    float rPriorWeight {1.0f};

    void serialize(std::ostream& stream) const
    {
        static_assert(std::is_trivially_copyable<VMMFactoryProperties>::value, "not trivially serializable");

        stream.write(reinterpret_cast<const char*>(this), sizeof(VMMFactoryProperties));
    }
    void deserialize(std::istream& stream)
    {
        stream.read(reinterpret_cast<char*>(this), sizeof(VMMFactoryProperties));
    }

    const std::string toString()const
    {
        std::ostringstream oss;
        oss << "VMMFactoryProperties["<< endl
            << "  minItr = " << minItr << endl
            << "  maxItr = " << maxItr << endl
            << "  relLogLikelihoodThreshold = " << relLogLikelihoodThreshold << endl
            << "  numInitialComponents = " << numInitialComponents << endl
            << "  initKappa = " << initKappa << endl
            << "  maxKappa = " << maxKappa << endl
            << "  vPrior = " << vPrior << endl
            << "  rPrior = " << rPrior << endl
            << "  rPriorWeight = " << rPriorWeight << endl
            << "]";
        return oss.str();
    }
};

/**
 * @brief The VMMFactory class provides fitting utilities for parametric mixture models using the von Mises-Fisher distribution.
 *
 * While much of the code is general and could be used for arbitrary distributions,
 * the usage of distribution-dependent initialization, priors and parameter updates currently prevents it from being more general.
 * By moving the handling of these operations to the distribution kernel, it could be applied to a wider range of directional distribution models.
 */
template<typename TVMM2>
class VMMFactory
{
private:
    using TScalar = Scalar<typename TVMM2::ScalarType::IntegralType,
                           TVMM2::ScalarType::Width::value,
                           TVMM2::ScalarType::Alignment::value>;
    using TSufficientStats = typename VMFKernel<TScalar>::SufficientStats;
    using TVMM = ParametricMixtureModel<VMFKernel<TScalar>, TVMM2::NumKernels::value>;
    static_assert (std::is_same<TVMM, TVMM2>::value, "Only ParametricMixtureModel<VMFKernel<Scalar<...>, N>> is supported by the VMMFactory.");
public:
    typedef TVMM PMMType;

    void resetInactiveComponents(TVMM& model) const
    {
        const uint32_t numComponents = model.getK();
        const uint32_t lastKernel = numComponents/TScalar::Width::value;
        const uint32_t componentsInLastKernel = numComponents%TScalar::Width::value;

        if (componentsInLastKernel > 0)
            model.m_comps[lastKernel].cutoff(componentsInLastKernel);
    }

    PMM_INLINE VMMFactory(const VMMFactoryProperties& props = VMMFactoryProperties())
        : m_properties{props}
    {

    }

    /**
     * @brief initialize the VMM based on the factories properties
     * @param model VMM to be initialized
     */
    PMM_INLINE void initialize(TVMM& model) const
    {
        model.setK(m_properties.numInitialComponents);

        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;
        const TScalar initWeight {1.0f/static_cast<float>(numComponents)};
        const TScalar initKappa {m_properties.initKappa};

        for (uint32_t k=0; k<maxKernel; ++k)
            model.m_comps[k] = VMFKernel<TScalar>{initWeight, initKappa, m_initMus[numComponents-1][k]};

        model.m_sampleWeight = 0.0f;
        model.m_numSamples = 0.0f;
        model.m_totalNumSamples = 0;
        model.m_numEMIterations = 0;
    }

    /**
     * @brief fit the VMM to the given sample data (EM)
     * @param dataBegin iterator pointing at the beginning of the sample data
     * @param dataEnd iterator pointing at the end of the sample data
     * @param model VMM to be fit
     * @param initialize whether to initialize the VMM before fitting
     */
    template<typename TFwdIterator>
    void fit(const TFwdIterator& dataBegin, const TFwdIterator& dataEnd, TVMM& model, const bool initialize) const
    {
        static_assert(std::is_base_of<std::forward_iterator_tag, typename TFwdIterator::iterator_category>::value, "forward iterator required");

        /// the number of active components of the mixture
        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;

        // init the components of the mixture
        if (initialize)
            this->initialize(model);

        // if the last used kernel contains deactivated components, we need to reset them
        resetInactiveComponents(model);

        float sumWeight, weightedLogLikelihood, lastWeightedLogLikelihood = 0.0f, absInvLastWeightedLogLikelihood = 0.0f;

        // checks if we have enough samples to do the fitting.
        // if not we do not fit, use the initialized mixture
        uint32_t maxItr = std::distance(dataBegin, dataEnd) > static_cast<uint32_t>(numComponents*2) ? m_properties.maxItr : 0;
        // MAP-EM fit
        uint32_t i = 0;
        for(; i<maxItr; ++i)
        {
            const std::array<TSufficientStats, TVMM::NumKernels::value> sufficientStats = computeSufficentStatsFromSamples(dataBegin, dataEnd, model,
                                                                                                                           (i==0) ? &sumWeight : nullptr,
                                                                                                                           (i >= m_properties.minItr) ? &weightedLogLikelihood : nullptr);

            // update mixture statistics
            if (i==0)
            {
                model.m_sampleWeight = sumWeight;
                model.m_numSamples = static_cast<float>(std::distance(dataBegin, dataEnd));
                model.m_totalNumSamples = std::distance(dataBegin, dataEnd);
            }

            // final M-Step (generate the model)
            for (uint32_t k=0;k<maxKernel;k++)
                parameterUpdate(model, k, sufficientStats[k]);

            // if the last used kernel contains deactivated componets, we need to reset them
            resetInactiveComponents(model);

            if (i >= m_properties.minItr)
            {
                if (i > m_properties.minItr)
                {
                    const float relativeLogLikelihood = (weightedLogLikelihood-lastWeightedLogLikelihood)*absInvLastWeightedLogLikelihood;

                    if (relativeLogLikelihood < m_properties.relLogLikelihoodThreshold)
                        break;
                }
                lastWeightedLogLikelihood = weightedLogLikelihood;
                absInvLastWeightedLogLikelihood = 1.0f/fabs(weightedLogLikelihood);
            }
        }

        model.m_numEMIterations += i;
    }

    /**
     * @brief maskedFit fits the active components in the VMM to the given sample data (partial EM)
     * @param dataBegin iterator pointing at the beginning of the sample data
     * @param dataEnd iterator pointing at the end of the sample data
     * @param model VMM to be fit
     * @param mask determines which components are considered active
     */
    template<typename TFwdIterator>
    void maskedFit(const TFwdIterator& dataBegin, const TFwdIterator& dataEnd, TVMM& model, const std::array<typename TScalar::BooleanType, TVMM::NumKernels::value> mask) const
    {
        static_assert(std::is_base_of<std::forward_iterator_tag, typename TFwdIterator::iterator_category>::value, "forward iterator required");

        /// the number of active components of the mixture
        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;

        // if the last used kernel contains deactivated componets, we need to reset them
        resetInactiveComponents(model);

        float sumWeight, weightedLogLikelihood, lastWeightedLogLikelihood = 0.0f, absInvLastWeightedLogLikelihood = 0.0f;

        // sum up the inital weight of the masked components which needs to be conserved

        TScalar oldActiveComponentWeightPartialSum{0.0f};
        for(uint32_t k=0;k<maxKernel;k++){
            oldActiveComponentWeightPartialSum += lightpmm::ifthen(mask[k], removeWeightPrior(model.m_comps[k].m_weights, numComponents), 0.0f);
        }
        const float oldActiveComponentWeight = lightpmm::sum(std::move(oldActiveComponentWeightPartialSum));

        // checks if we have enough samples to do the fitting.
        // if not we do not fit, use the initialized mixture
        uint32_t maxItr = std::distance(dataBegin, dataEnd) > static_cast<uint32_t>(numComponents*2) ? m_properties.maxItr : 0;
        // MAP-EM fit
        uint32_t i = 0;
        for(; i<maxItr; ++i)
        {
            std::array<TSufficientStats, TVMM::NumKernels::value> sufficientStats = computeSufficentStatsFromSamples(dataBegin, dataEnd, model,
                                                                                                                     (i==0) ? &sumWeight : nullptr,
                                                                                                                     (i >= m_properties.minItr) ? &weightedLogLikelihood : nullptr);

            //normalize weight of active components
            TScalar activeComponentWeightPartialSum = 0.0f;
            for (uint32_t k=0;k<maxKernel;k++)
                if (lightpmm::any(mask[k]))
                    activeComponentWeightPartialSum += lightpmm::ifthen(mask[k], sufficientStats[k].sumWeight, 0.0f);

            const float invActiveComponentWeight = sumWeight/lightpmm::sum(std::move(activeComponentWeightPartialSum));
            const float activeComponentWeightNormalization = oldActiveComponentWeight*invActiveComponentWeight;

            for (uint32_t k=0;k<maxKernel;k++)
            {
                if (lightpmm::any(mask[k]))
                {
                    sufficientStats[k].muTimesAvgCosineTimesSumWeight = lightpmm::ifthen(mask[k], sufficientStats[k].muTimesAvgCosineTimesSumWeight*activeComponentWeightNormalization, sufficientStats[k].muTimesAvgCosineTimesSumWeight);
                    sufficientStats[k].sumWeight = lightpmm::ifthen(mask[k], sufficientStats[k].sumWeight*activeComponentWeightNormalization, sufficientStats[k].sumWeight);
                }
            }

            // final M-Step (generate the model)
            for (uint32_t k=0;k<maxKernel;k++)
                if (lightpmm::any(mask[k]))
                    maskedParameterUpdate(model, k, sufficientStats[k], mask[k], sumWeight);

            // if the last used kernel contains deactivated componets, we need to reset them
            resetInactiveComponents(model);

            if (i >= m_properties.minItr)
            {
                if (i > m_properties.minItr)
                {
                    const float relativeLogLikelihood = (weightedLogLikelihood-lastWeightedLogLikelihood)*absInvLastWeightedLogLikelihood;

                    if (relativeLogLikelihood < m_properties.relLogLikelihoodThreshold)
                        break;
                }
                lastWeightedLogLikelihood = weightedLogLikelihood;
                absInvLastWeightedLogLikelihood = 1.0f/fabs(weightedLogLikelihood);
            }
        }
    }

    /**
     * @brief updateFit updates the fit of the VMM based on the given sample data and the previous mixture state (incremental MAP EP)
     * @param dataBegin iterator pointing at the beginning of the sample data
     * @param dataEnd iterator pointing at the end of the sample data
     * @param model VMM to be fit
     */
    template<typename TFwdIterator>
    void updateFit(const TFwdIterator& dataBegin, const TFwdIterator& dataEnd, TVMM& model) const
    {
        static_assert(std::is_base_of<std::forward_iterator_tag, typename TFwdIterator::iterator_category>::value, "forward iterator required");

        /// the number of active components of the mixture
        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;

        const std::array<TSufficientStats, TVMM::NumKernels::value> oldStats = computeSufficentStatsFromMixture(model);

        // if the last used kernel contains deactivated componets, we need to reset them
        resetInactiveComponents(model);

        float sumWeight, weightedLogLikelihood, lastWeightedLogLikelihood = 0.0f, absInvLastWeightedLogLikelihood = 0.0f;

        // checks if we have enough samples to do the fitting.
        // if not we do not fit, use the initialized mixture
        uint32_t maxItr = std::distance(dataBegin, dataEnd) > static_cast<uint32_t>(numComponents*2) ? m_properties.maxItr : 0;
        // MAP-EM fit
        uint32_t i = 0;
        for(; i<maxItr; ++i)
        {
            const std::array<TSufficientStats, TVMM::NumKernels::value> sufficientStats = computeSufficentStatsFromSamples(dataBegin, dataEnd, model,
                                                                                                                           (i==0) ? &sumWeight : nullptr,
                                                                                                                           (i >= m_properties.minItr) ? &weightedLogLikelihood : nullptr);

            // update mixture statistics
            if (i==0)
            {
                model.m_sampleWeight += sumWeight;
                model.m_numSamples += static_cast<float>(std::distance(dataBegin, dataEnd));
                model.m_totalNumSamples += std::distance(dataBegin, dataEnd);
            }

            // final M-Step (generate the model)
            for (uint32_t k=0;k<maxKernel;k++)
                parameterUpdate(model, k, oldStats[k]+sufficientStats[k]);

            // if the last used kernel contains deactivated componets, we need to reset them
            resetInactiveComponents(model);

            if (i >= m_properties.minItr)
            {
                if (i > m_properties.minItr)
                {
                    const float relativeLogLikelihood = (weightedLogLikelihood-lastWeightedLogLikelihood)*absInvLastWeightedLogLikelihood;

                    if (relativeLogLikelihood < m_properties.relLogLikelihoodThreshold)
                        break;
                }
                lastWeightedLogLikelihood = weightedLogLikelihood;
                absInvLastWeightedLogLikelihood = 1.0f/fabs(weightedLogLikelihood);
            }
        }

        model.m_numEMIterations += i;
    }

    /**
     * @brief computeMinComponentWeight computes the minimum component weight,
     * that may result after fitting when using the weight prior
     * @param numComponents number of mixture components
     * @return minimum valid component weight that may result for the configured weight prior
     */
    PMM_INLINE float computeMinComponentWeight(const uint32_t numComponents) const
    {
        const float invWPriorNormalization = m_properties.vPrior*static_cast<float>(numComponents)+1.0f;
        const float wPriorNormalization = 1.0f/invWPriorNormalization;
        const float wPriorTimesNormalization = m_properties.vPrior*wPriorNormalization;

        return wPriorTimesNormalization;
    }

    /**
     * @brief applyWeightPrior applies the weight prior on top of the weights computed by the EM
     * @param normalizedSampleWeight_k weights computed by the regular EM
     * @param numComponents number of components in the mixture
     * @return weights with applied weight prior
     */
    PMM_INLINE TScalar applyWeightPrior(const TScalar normalizedSampleWeight_k, const uint32_t numComponents) const
    {
        const float invWPriorNormalization = m_properties.vPrior*static_cast<float>(numComponents)+1.0f;
        const float wPriorNormalization = 1.0f/invWPriorNormalization;
        const float wPriorTimesNormalization = m_properties.vPrior*wPriorNormalization;

        return normalizedSampleWeight_k*wPriorNormalization+wPriorTimesNormalization;
    }

    /**
     * @brief removeWeightPrior removes the weight prior from the given mixture weights
     * @param normalizedSampleWeightWithPrior_k weights with prior
     * @param numComponents number of components in the mixture
     * @return weights without prior
     */
    PMM_INLINE TScalar removeWeightPrior(const TScalar normalizedSampleWeightWithPrior_k, const uint32_t numComponents) const
    {
        const float invWPriorNormalization = m_properties.vPrior*static_cast<float>(numComponents)+1.0f;
        const float wPriorNormalization = 1.0f/invWPriorNormalization;
        const float wPriorTimesNormalization = m_properties.vPrior*wPriorNormalization;

        return lightpmm::ifthen(normalizedSampleWeightWithPrior_k > wPriorTimesNormalization, (normalizedSampleWeightWithPrior_k-wPriorTimesNormalization)*invWPriorNormalization, 0.0f);
    }

    /**
     * @brief applyAvgCosinePrior applies the prior on the average cosine
     * @param avgCosine_k average cosine from sufficient statistics
     * @param numSamplesInComponent number of samples observed by the component
     * @return average cosine with applied prior
     */
    PMM_INLINE TScalar applyAvgCosinePrior(const TScalar avgCosine_k, const TScalar numSamplesInComponent) const
    {
        const float rPriorTimesWeight = m_properties.rPrior*m_properties.rPriorWeight;

        return (avgCosine_k*numSamplesInComponent+rPriorTimesWeight)/(numSamplesInComponent+m_properties.rPriorWeight);
    }

    /**
     * @brief removeAvgCosinePrior removes the prior on the average cosine
     * @param avgCosineWithPrior_k average cosine with applied prior
     * @param numSamplesInComponent number of samples observed by the component
     * @return average cosine without prior
     */
    PMM_INLINE TScalar removeAvgCosinePrior(const TScalar avgCosineWithPrior_k, const TScalar numSamplesInComponent) const
    {
        const float rPriorTimesWeight = m_properties.rPrior*m_properties.rPriorWeight;

        return (avgCosineWithPrior_k*(numSamplesInComponent+m_properties.rPriorWeight)-rPriorTimesWeight)/numSamplesInComponent;
    }

    /// update of mixture parameters with priors
    void parameterUpdate(TVMM& model, const uint32_t k, const TSufficientStats stats_k) const
    {
        const TScalar maxMeanCosine = kappaToMeanCosine<TScalar>(m_properties.maxKappa);
        const uint32_t numComponents = model.getK();
        typename TVMM::KernelType& kernel = model.getComponent(k);

        const TScalar mixtureWeight_k = applyWeightPrior(stats_k.sumWeight/model.m_sampleWeight, numComponents);

        const TScalar rLength_k = stats_k.muTimesAvgCosineTimesSumWeight.length();
        const Vec<TScalar> mu_k = ifthen(rLength_k > 0.0f, stats_k.muTimesAvgCosineTimesSumWeight/rLength_k, kernel.m_mu);

        const TScalar avgCosine_k = rLength_k/stats_k.sumWeight;
        const TScalar avgCosineWithPrior_k = lightpmm::min(applyAvgCosinePrior(avgCosine_k, static_cast<float>(model.m_totalNumSamples)*mixtureWeight_k), maxMeanCosine);

        const TScalar kappa_k = meanCosineToKappa<TScalar>(avgCosineWithPrior_k);

        kernel.m_weights = mixtureWeight_k;
        kernel.m_mu = mu_k;
        kernel.setKappaAndR(kappa_k, avgCosineWithPrior_k);
    }

    /// masked update of mixture parameters with priors
    void maskedParameterUpdate(TVMM& model, const uint32_t k, const TSufficientStats stats_k, const typename TScalar::BooleanType mask_k, const float sampleWeight) const
    {
        const TScalar maxMeanCosine = kappaToMeanCosine<TScalar>(m_properties.maxKappa);
        const uint32_t numComponents = model.getK();
        typename TVMM::KernelType& kernel = model.getComponent(k);

        TScalar mixtureWeight_k = applyWeightPrior(stats_k.sumWeight/sampleWeight, numComponents);

        const TScalar rLength_k = stats_k.muTimesAvgCosineTimesSumWeight.length();
        Vec<TScalar> mu_k = ifthen(rLength_k > 0.0f, stats_k.muTimesAvgCosineTimesSumWeight/rLength_k, kernel.m_mu);

        const TScalar avgCosine_k = rLength_k/stats_k.sumWeight;
        TScalar avgCosineWithPrior_k = lightpmm::min(applyAvgCosinePrior(avgCosine_k, static_cast<float>(model.m_totalNumSamples)*mixtureWeight_k), maxMeanCosine);

        TScalar kappa_k = meanCosineToKappa<TScalar>(avgCosineWithPrior_k);

        //reset parameters for inactive components
        if (!lightpmm::all(mask_k))
        {
            mixtureWeight_k      = lightpmm::ifthen(mask_k, mixtureWeight_k,      kernel.m_weights);
            mu_k                 = lightpmm::ifthen(mask_k, mu_k,                 kernel.m_mu);
            avgCosineWithPrior_k = lightpmm::ifthen(mask_k, avgCosineWithPrior_k, kernel.getR());
            kappa_k              = lightpmm::ifthen(mask_k, kappa_k,              kernel.getKappa());
        }

        kernel.m_weights = mixtureWeight_k;
        kernel.m_mu = mu_k;
        kernel.setKappaAndR(kappa_k, avgCosineWithPrior_k);
    }

    /**
     * @brief computeSufficentStatsFromSamples computes sufficient statistics from sample data
     * @param dataBegin iterator pointing at the beginning of the sample data
     * @param dataEnd iterator pointing at the end of the sample data
     * @param model VMM
     * @param sumWeightOut set to sum of sample weights if not a \c nullptr
     * @param weightedLogLikelihoodOut set to the weighted log likelihood of the sample data if not a \c nullptr
     * @return sufficent statistics of the given sample data per mixture component
     */
    template<typename TInIterator>
    std::array<TSufficientStats, TVMM::NumKernels::value> computeSufficentStatsFromSamples(const TInIterator& dataBegin, const TInIterator& dataEnd, const TVMM& model,
                                                                                           float* sumWeightOut=nullptr, float* weightedLogLikelihoodOut=nullptr) const
    {
        static_assert(std::is_base_of<std::input_iterator_tag, typename TInIterator::iterator_category>::value, "input iterator required");

        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;

        if (sumWeightOut)
            *sumWeightOut = 0.0f;
        if (weightedLogLikelihoodOut)
            *weightedLogLikelihoodOut = 0.0f;

        std::array<TSufficientStats, TVMM::NumKernels::value> sufficientStats;
        for(TInIterator it=dataBegin; it != dataEnd; ++it)
        {
            const typename TInIterator::value_type& data_n = *it;

            //iterative E-Step
            const typename TVMM::SoftAssignmentWeights softAssignments{model, data_n.direction};

            // if a sample is not covered by any component at all, we skip it. (?? can we avoid this ??)
            if(!softAssignments.valid())
                continue;

            //iterative M-Step
            for (uint32_t k=0;k<maxKernel;k++)
                sufficientStats[k].accumulateSample(data_n, softAssignments.getSoftAssignments(k));

            //compute total sample weight for normalization
            if (sumWeightOut)
                *sumWeightOut += data_n.weight;

            if (weightedLogLikelihoodOut)
                *weightedLogLikelihoodOut += data_n.weight*std::log(softAssignments.getMixturePDF());
        }

        return sufficientStats;
    }

    /**
     * @brief computeSufficentStatsFromMixture reconstructs the sufficent statistics per mixture component from its parameters
     * @param model VMM
     * @return sufficient statistics per mixture component
     */
    std::array<TSufficientStats, TVMM::NumKernels::value> computeSufficentStatsFromMixture(TVMM& model) const
    {
        const uint32_t numComponents = model.getK();
        const uint32_t maxKernel = (numComponents+TScalar::Width::value-1)/TScalar::Width::value;

        std::array<TSufficientStats, TVMM::NumKernels::value> sufficientStats;
        for (uint32_t k=0; k<maxKernel; ++k)
        {
            sufficientStats[k].sumWeight = removeWeightPrior(model.m_comps[k].m_weights, numComponents)*(model.m_sampleWeight);
            sufficientStats[k].muTimesAvgCosineTimesSumWeight = model.m_comps[k].m_mu*(removeAvgCosinePrior(model.m_comps[k].getR(), model.m_totalNumSamples*model.m_comps[k].m_weights)*sufficientStats[k].sumWeight);
        }
        return sufficientStats;
    }

    /**
     * @brief toString returns a string representation of the VMMFactory
     * @return string representation with individual properties
     */
    const std::string toString()const
    {
        std::ostringstream oss;
        oss << "VMMFactory[" << endl
            << "properties = " << m_properties.toString() << endl
            << "]";
        return oss.str();
    }

    void serialize(std::ostream& stream) const
    {
        m_properties.serialize(stream);
    }
    void deserialize(std::istream& stream)
    {
        m_properties.deserialize(stream);
    }

private:
    /*
    Scalar dirichletPrior(const Scalar& itrSumWeightedStats, const Scalar& sumWeightedStats, const Scalar& numComp, const Scalar& sumStats, const Scalar& nData, const Scalar& weightN0)
    {
        Scalar w = itrSumWeightedStats / sumWeightedStats;
        w = (m_vMinusOne + (w*sumStats+weightN0))/(m_vMinusOne*numComp+(sumStats+nData));
        return w;
    }
*/
    std::array<std::array<Vec<TScalar>, TVMM::NumKernels::value>, TVMM::MaxK::value> m_initMus {computeUniformMus()};

    /**
     * @brief computeUniformMus computes direction vectors according to the spherical fibonacci distribution
     * @return direction vectors distributed according to spherical fibonacci for increasing mixture sizes
     */
    static std::array<std::array<Vec<TScalar>, TVMM::NumKernels::value>, TVMM::MaxK::value> computeUniformMus() {
        std::array<std::array<Vec<TScalar>, TVMM::NumKernels::value>, TVMM::MaxK::value> uniformMus;

        //uint32_t count = 0;
        const float gr = 1.618033988749895f;

        for(uint32_t l=0;l<TVMM::MaxK::value;l++){

            /// distributes samples l+1 uniform samples over the sphere
            /// based on "Spherical Fibonacci Point Sets for Illumination Integrals"
            uint32_t n = 0;
            for(uint32_t k=0;k<TVMM::NumKernels::value;k++){

                for(uint32_t i=0; i< TScalar::Width::value;i++){

                    if(n<l+1){
                        float phi = 2.0f*M_PI*(static_cast<float>(n) / gr);
                        float z = 1.0f - ((2.0f*n + 1.0f) / float(l+1));
                        float theta = std::acos(z);

                        Vector3 mu = sphericalDirection(theta, phi);
                        lightpmm::insert(uniformMus[l][k].x, i, mu[0]);
                        lightpmm::insert(uniformMus[l][k].y, i, mu[1]);
                        lightpmm::insert(uniformMus[l][k].z, i, mu[2]);
                    }else{
                        lightpmm::insert(uniformMus[l][k].x, i, 0.0f);
                        lightpmm::insert(uniformMus[l][k].y, i, 0.0f);
                        lightpmm::insert(uniformMus[l][k].z, i, 1.0f);
                    }

                    n++;

                    //count++;
                }
            }
        }

        return uniformMus;
    }

public:
    VMMFactoryProperties m_properties;

};

PMM_NAMESPACE_END

#endif /* VMM_PMMFACTORY_H_ */
