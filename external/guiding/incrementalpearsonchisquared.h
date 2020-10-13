/*
    This file is part of the implementation of the SIGGRAPH 2020 paper
    "Robust Fitting of Parallax-Aware Mixtures for Path Guiding".
    The implementation extends Mitsuba, a physically based rendering system.

    Copyright (c) 2020 Lukas Ruppert, Sebastian Herholz.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCREMENTALPEARSONCHISQUARED_H
#define INCREMENTALPEARSONCHISQUARED_H

#include <pmm/pmm.h>
#include <pmm/vector.h>
#include <pmm/ParametricMixtureModel.h>

#include <array>
#include <algorithm>
#include <string>

namespace guiding {


template <typename TDistribution>
struct IncrementalPearsonChiSquared;

template<typename TKernel, uint32_t NKernels>
struct IncrementalPearsonChiSquared<lightpmm::ParametricMixtureModel<TKernel, NKernels>>
{
    using TDistribution = lightpmm::ParametricMixtureModel<TKernel, NKernels>;
    using ScalarType = typename TKernel::ScalarType;

    std::array<ScalarType, TDistribution::NumKernels::value> divergencePlusOneTimesIntegralSqr;
    std::array<ScalarType, TDistribution::NumKernels::value> numSamples;

    IncrementalPearsonChiSquared()
    {
        std::fill(divergencePlusOneTimesIntegralSqr.begin(), divergencePlusOneTimesIntegralSqr.end(), ScalarType{0.0f});
        std::fill(numSamples.begin(), numSamples.end(), ScalarType{0.0f});
    }

    template<typename TSampleRange>
    void updateDivergence(const TDistribution& distribution, const TSampleRange& samples)
    {
        if (samples.size() == 0)
            return;

        const uint32_t numComponents = distribution.getK();

        std::array<ScalarType, TDistribution::NumKernels::value> batchDivergence;
        std::fill(batchDivergence.begin(), batchDivergence.end(), ScalarType{0.0f});

        for (const auto& sample : samples)
        {
            std::array<ScalarType, TDistribution::NumKernels::value> componentPDFs;

            ScalarType mixturePDFPartialSum{0.0f};
            for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; k++)
            {
                componentPDFs[k] = distribution.getComponent(k).pdf(sample.direction);
                mixturePDFPartialSum += distribution.getComponent(k).m_weights*componentPDFs[k];
            }

            const float mixturePDF = lightpmm::sum(mixturePDFPartialSum);

            if (EXPECT_NOT_TAKEN(!(mixturePDF > PMM_EPSILON)))
                continue;

            const float mixturePDFSqr = mixturePDF*mixturePDF;
            const float idealPDFSqrDivMixturePDFSqrTimesSamplePDF = sample.weight*sample.weight*sample.pdf/mixturePDFSqr;

            for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; ++k)
                batchDivergence[k] += componentPDFs[k]*idealPDFSqrDivMixturePDFSqrTimesSamplePDF;
        }

        for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; ++k)
        {
            const ScalarType newNumSamples = numSamples[k]+static_cast<float>(samples.size());
            divergencePlusOneTimesIntegralSqr[k] = (divergencePlusOneTimesIntegralSqr[k]*numSamples[k]+batchDivergence[k])/newNumSamples;
            numSamples[k] = newNumSamples;
        }
    }

    template<typename TSampleRange>
    void updateDivergenceMasked(const TDistribution& distribution, const TSampleRange& samples, const std::array<typename ScalarType::BooleanType, TDistribution::NumKernels::value>& mask)
    {
        if (samples.size() == 0)
            return;

        const uint32_t numComponents = distribution.getK();

        std::array<ScalarType, TDistribution::NumKernels::value> batchDivergence;
        std::fill(batchDivergence.begin(), batchDivergence.end(), ScalarType{0.0f});

        for (const auto& sample : samples)
        {
            std::array<ScalarType, TDistribution::NumKernels::value> componentPDFs;

            ScalarType mixturePDFPartialSum{0.0f};
            for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; k++)
            {
                componentPDFs[k] = distribution.getComponent(k).pdf(sample.direction);
                mixturePDFPartialSum += distribution.getComponent(k).m_weights*componentPDFs[k];
            }

            const float mixturePDF = lightpmm::sum(mixturePDFPartialSum);

            if (EXPECT_NOT_TAKEN(!(mixturePDF > PMM_EPSILON)))
                continue;

            const float mixturePDFSqr = mixturePDF*mixturePDF;
            const float idealPDFSqrDivMixturePDFSqrTimesSamplePDF = sample.weight*sample.weight*sample.pdf/mixturePDFSqr;

            for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; ++k)
            {
                if (!lightpmm::any(mask[k]))
                    continue;
                batchDivergence[k] = lightpmm::ifthen(mask[k], batchDivergence[k]+componentPDFs[k]*idealPDFSqrDivMixturePDFSqrTimesSamplePDF, batchDivergence[k]);
            }
        }

        for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; ++k)
        {
            if (!lightpmm::any(mask[k]))
                continue;
            const ScalarType newNumSamples = numSamples[k]+static_cast<float>(samples.size());
            divergencePlusOneTimesIntegralSqr[k] = lightpmm::ifthen(mask[k], (divergencePlusOneTimesIntegralSqr[k]*numSamples[k]+batchDivergence[k])/newNumSamples, divergencePlusOneTimesIntegralSqr[k]);
            numSamples[k] = lightpmm::ifthen(mask[k], newNumSamples, numSamples[k]);
        }
    }

    std::array<ScalarType, TDistribution::NumKernels::value> computeDivergence(const TDistribution& distribution) const
    {
        const uint32_t numComponents = distribution.getK();
        const float invAvgSampleWeightSqr = (distribution.m_numSamples*distribution.m_numSamples)/(distribution.m_sampleWeight*distribution.m_sampleWeight);

        std::array<ScalarType, TDistribution::NumKernels::value> result;

        for (uint32_t k=0; k<(numComponents+ScalarType::Width::value-1)/ScalarType::Width::value; ++k)
            result[k] = divergencePlusOneTimesIntegralSqr[k]*invAvgSampleWeightSqr-1.0f;

        return result;
    }

    void operator *=(const float factor)
    {
        for (uint32_t k=0; k<TDistribution::NumKernels::value; ++k)
            numSamples[k] *= factor;
    }

    void clear()
    {
        *this = IncrementalPearsonChiSquared{};
    }

    void merge(const TDistribution& distribution, const uint32_t componentA, const uint32_t componentB)
    {
        const float weightA = distribution.getComponent(componentA/ScalarType::Width::value).getWeight(componentA%ScalarType::Width::value);
        const float weightB = distribution.getComponent(componentB/ScalarType::Width::value).getWeight(componentB%ScalarType::Width::value);
        const float mergedWeight = weightA+weightB;
        const float invMergedWeight = 1.0f/mergedWeight;

        if (EXPECT_TAKEN(!std::isinf(invMergedWeight)))
        {
            const float scaleA = weightA*invMergedWeight;
            const float scaleB = weightB*invMergedWeight;

            const float divergencePlusOneTimesIntegralSqrA = divergencePlusOneTimesIntegralSqr[componentA/ScalarType::Width::value][componentA%ScalarType::Width::value];
            const float divergencePlusOneTimesIntegralSqrB = divergencePlusOneTimesIntegralSqr[componentB/ScalarType::Width::value][componentB%ScalarType::Width::value];
            const float mergedDivergencePlusOneTimesIntegralSqr = scaleA*divergencePlusOneTimesIntegralSqrA+scaleB*divergencePlusOneTimesIntegralSqrB;

            const float numSamplesA = numSamples[componentA/ScalarType::Width::value][componentA%ScalarType::Width::value];
            const float numSamplesB = numSamples[componentB/ScalarType::Width::value][componentB%ScalarType::Width::value];
            const float mergedNumSamples = scaleA*numSamplesA+scaleB*numSamplesB;

            divergencePlusOneTimesIntegralSqr[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, mergedDivergencePlusOneTimesIntegralSqr);
            numSamples[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, mergedNumSamples);
        }
        else
        {
            divergencePlusOneTimesIntegralSqr[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, 0.0f);
            numSamples[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, 0.0f);
        }

        const uint32_t lastComponent = distribution.getK()-1;
        if (componentB != lastComponent)
        {
            divergencePlusOneTimesIntegralSqr[componentB/ScalarType::Width::value].insert(componentB%ScalarType::Width::value, divergencePlusOneTimesIntegralSqr[lastComponent/ScalarType::Width::value][lastComponent%ScalarType::Width::value]);
            numSamples[componentB/ScalarType::Width::value].insert(componentB%ScalarType::Width::value, numSamples[lastComponent/ScalarType::Width::value][lastComponent%ScalarType::Width::value]);
        }
    }

    void split(const TDistribution& distribution, const uint32_t component)
    {
        const uint32_t newComponent = distribution.getK()-1;

        //reset divergence estimate for split components
        divergencePlusOneTimesIntegralSqr[component/ScalarType::Width::value].insert(component%ScalarType::Width::value, 0.0f);
        divergencePlusOneTimesIntegralSqr[newComponent/ScalarType::Width::value].insert(newComponent%ScalarType::Width::value, 0.0f);
        numSamples[component/ScalarType::Width::value].insert(component%ScalarType::Width::value, 0.0f);
        numSamples[newComponent/ScalarType::Width::value].insert(newComponent%ScalarType::Width::value, 0.0f);
    }
};

}

#endif
