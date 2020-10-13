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

#ifndef INCREMENTALCOVARIANCE2D_H
#define INCREMENTALCOVARIANCE2D_H

#include <pmm/pmm.h>
#include <pmm/vector.h>
#include <pmm/ParametricMixtureModel.h>

#include <array>
#include <algorithm>
#include <string>

namespace guiding {

template <typename TDistribution>
struct IncrementalCovariance2D;

template<typename TKernel, uint32_t NKernels>
struct IncrementalCovariance2D<lightpmm::ParametricMixtureModel<TKernel, NKernels>>
{
    using TDistribution = lightpmm::ParametricMixtureModel<TKernel, NKernels>;
    using ScalarType = typename TKernel::ScalarType;

    //var_x, var_y, cov_{xy}
    std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> varianceAndCovariance;
    std::array<ScalarType, TDistribution::NumKernels::value> sumWeights;

    IncrementalCovariance2D()
    {
        std::fill(varianceAndCovariance.begin(), varianceAndCovariance.end(), lightpmm::Vec<ScalarType>{0.0f, 0.0f, 0.0f});
        std::fill(sumWeights.begin(), sumWeights.end(), ScalarType{0.0f});
    }

    template<typename TSampleRange>
    void updateStatistics(const TDistribution& distribution, const TSampleRange& samples)
    {
        const uint32_t numComponents = distribution.getK();
        const uint32_t numActiveKernels = (numComponents+ScalarType::Width::value-1)/ScalarType::Width::value;

        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> s;
        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> t;

        std::array<ScalarType, TDistribution::NumKernels::value> batchWeight;
        std::fill(batchWeight.begin(), batchWeight.end(), ScalarType{0.0f});
        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> batchUnnormalizedVariance;
        std::fill(batchUnnormalizedVariance.begin(), batchUnnormalizedVariance.end(), lightpmm::Vec<ScalarType>{0.0f, 0.0f, 0.0f});

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            const lightpmm::Vec<ScalarType> n = distribution.getComponent(k).m_mu;
            const lightpmm::Vec<ScalarType> nSqr = n*n;

            const typename ScalarType::BooleanType xAbsGreaterThanY = nSqr.x > nSqr.y;

            ScalarType invLen = 1.0f/lightpmm::sqrt(lightpmm::ifthen(xAbsGreaterThanY, nSqr.x, nSqr.y)+nSqr.z);
            t[k] = lightpmm::ifthen(xAbsGreaterThanY, lightpmm::Vec<ScalarType>(n.z*invLen, 0.0f, -n.x*invLen), lightpmm::Vec<ScalarType>(0.0f, n.z*invLen, -n.y*invLen));
            s[k] = lightpmm::cross(t[k], n);
        }

        for (const auto& sample : samples)
        {
            const typename TDistribution::SoftAssignmentWeights softAssignments {distribution, sample.direction};

            if (EXPECT_NOT_TAKEN(!softAssignments.valid()))
                continue;

            for (uint32_t k=0; k<numActiveKernels; ++k)
            {
                const ScalarType weightTimesSoftAssignment = sample.weight*softAssignments.getSoftAssignments(k);

                batchWeight[k] += weightTimesSoftAssignment;

                const lightpmm::Vec<ScalarType> dir {sample.direction};

                const ScalarType localX = lightpmm::dot(s[k], dir);
                const ScalarType localY = lightpmm::dot(t[k], dir);

                batchUnnormalizedVariance[k].x += localX*localX*weightTimesSoftAssignment;
                batchUnnormalizedVariance[k].y += localY*localY*weightTimesSoftAssignment;
                //covariance
                batchUnnormalizedVariance[k].z += localX*localY*weightTimesSoftAssignment;
            }
        }

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            const ScalarType newSumWeight = sumWeights[k]+batchWeight[k];
            varianceAndCovariance[k] = lightpmm::ifthen(newSumWeight > 0.0f, (varianceAndCovariance[k]*sumWeights[k]+batchUnnormalizedVariance[k])/newSumWeight, lightpmm::Vec<ScalarType>{0.0f, 0.0f, 0.0f});
            sumWeights[k] = newSumWeight;
        }
    }

    template<typename TSampleRange>
    void updateStatisticsMasked(const TDistribution& distribution, const TSampleRange& samples, const std::array<typename ScalarType::BooleanType, TDistribution::NumKernels::value>& mask)
    {
        const uint32_t numComponents = distribution.getK();
        const uint32_t numActiveKernels = (numComponents+ScalarType::Width::value-1)/ScalarType::Width::value;

        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> s;
        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> t;

        std::array<ScalarType, TDistribution::NumKernels::value> batchWeight;
        std::fill(batchWeight.begin(), batchWeight.end(), ScalarType{0.0f});
        std::array<lightpmm::Vec<ScalarType>, TDistribution::NumKernels::value> batchUnnormalizedVariance;
        std::fill(batchUnnormalizedVariance.begin(), batchUnnormalizedVariance.end(), lightpmm::Vec<ScalarType>{0.0f, 0.0f, 0.0f});

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            if (!lightpmm::any(mask[k]))
                continue;

            const lightpmm::Vec<ScalarType> n = distribution.getComponent(k).m_mu;
            const lightpmm::Vec<ScalarType> nSqr = n*n;

            const typename ScalarType::BooleanType xAbsGreaterThanY = nSqr.x > nSqr.y;

            ScalarType invLen = 1.0f/lightpmm::sqrt(lightpmm::ifthen(xAbsGreaterThanY, nSqr.x, nSqr.y)+nSqr.z);
            t[k] = lightpmm::ifthen(xAbsGreaterThanY, lightpmm::Vec<ScalarType>(n.z*invLen, 0.0f, -n.x*invLen), lightpmm::Vec<ScalarType>(0.0f, n.z*invLen, -n.y*invLen));
            s[k] = lightpmm::cross(t[k], n);
        }

        for (const auto& sample : samples)
        {
            const typename TDistribution::SoftAssignmentWeights softAssignments {distribution, sample.direction};

            if (EXPECT_NOT_TAKEN(!softAssignments.valid()))
                continue;

            for (uint32_t k=0; k<numActiveKernels; ++k)
            {
                if (!lightpmm::any(mask[k]))
                    continue;

                const ScalarType weightTimesSoftAssignment = sample.weight*softAssignments.getSoftAssignments(k);

                batchWeight[k] += weightTimesSoftAssignment;

                const lightpmm::Vec<ScalarType> dir {sample.direction};

                const ScalarType localX = lightpmm::dot(s[k], dir);
                const ScalarType localY = lightpmm::dot(t[k], dir);

                batchUnnormalizedVariance[k].x += localX*localX*weightTimesSoftAssignment;
                batchUnnormalizedVariance[k].y += localY*localY*weightTimesSoftAssignment;
                //covariance
                batchUnnormalizedVariance[k].z += localX*localY*weightTimesSoftAssignment;
            }
        }

        for (uint32_t k=0; k<numActiveKernels; ++k)
        {
            if (!lightpmm::any(mask[k]))
                continue;

            const ScalarType newSumWeight = sumWeights[k]+batchWeight[k];
            varianceAndCovariance[k] = lightpmm::ifthen(mask[k], lightpmm::ifthen(newSumWeight > 0.0f, (varianceAndCovariance[k]*sumWeights[k]+batchUnnormalizedVariance[k])/newSumWeight, lightpmm::Vec<ScalarType>{0.0f, 0.0f, 0.0f}), varianceAndCovariance[k]);
            sumWeights[k] = lightpmm::ifthen(mask[k], newSumWeight, sumWeights[k]);
        }
    }

    lightpmm::Matrix2x2 computeCovarianceMatrix(uint32_t component) const
    {
        const float componentSumWeights = sumWeights[component/ScalarType::Width::value][component%ScalarType::Width::value];
        const lightpmm::Vector3 variance{EXPECT_TAKEN(componentSumWeights > 0.0f) ? varianceAndCovariance[component/ScalarType::Width::value][component%ScalarType::Width::value] :  lightpmm::Vector3{0.0f}};
        return {variance.x, variance.z, variance.z, variance.y};
    }

    void operator *=(const float factor)
    {
        for (uint32_t k=0; k<TDistribution::NumKernels::value; ++k)
            sumWeights[k] *= factor;
    }

    void clear()
    {
        *this = IncrementalCovariance2D{};
    }

    void merge(const TDistribution& distribution, const uint32_t componentA, const uint32_t componentB)
    {
        const float weightA = distribution.getComponent(componentA/ScalarType::Width::value).getWeight(componentA%ScalarType::Width::value);
        const float weightB = distribution.getComponent(componentB/ScalarType::Width::value).getWeight(componentB%ScalarType::Width::value);
        const float mergedWeight = weightA+weightB;
        const float invMergedWeight = 1.0f/mergedWeight;

        if (EXPECT_TAKEN(!std::isinf(invMergedWeight)))
        {
            const Vector muA = distribution.getComponent(componentA/ScalarType::Width::value).m_mu[componentA%ScalarType::Width::value];
            const Vector muB = distribution.getComponent(componentB/ScalarType::Width::value).m_mu[componentB%ScalarType::Width::value];

            Vector mergedMu = weightA*muA+weightB*muB;
            mergedMu *= 1.0f/mergedMu.length();

            lightpmm::Frame mergedMuFrame{mergedMu};

            const Vector muALocal = mergedMuFrame.toLocal(muA);
            const Vector muBLocal = mergedMuFrame.toLocal(muB);

            const float meanXA = muALocal.x;
            const float meanXB = muBLocal.x;

            const float meanYA = muALocal.y;
            const float meanYB = muBLocal.y;

            const float sumWeightA = sumWeights[componentA/ScalarType::Width::value][componentA%ScalarType::Width::value];
            const float sumWeightB = sumWeights[componentB/ScalarType::Width::value][componentB%ScalarType::Width::value];
            const float mergedSumWeight = sumWeightA+sumWeightB;



            const Vector varianceA = varianceAndCovariance[componentA/ScalarType::Width::value][componentA%ScalarType::Width::value];
            const Vector varianceB = varianceAndCovariance[componentB/ScalarType::Width::value][componentB%ScalarType::Width::value];
            const Vector mergedVarianceAndCovariance = (weightA*varianceA+weightB*varianceB+weightA*weightB/mergedWeight
                                                        *Vector{(meanXA-meanXB)*(meanXA-meanXB), (meanYA-meanYB)*(meanYA-meanYB), (meanXA-meanXB)*(meanYA-meanYB)})
                                                       *invMergedWeight;

            varianceAndCovariance[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, mergedVarianceAndCovariance);
            sumWeights[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, mergedSumWeight);
        }
        else
        {
            varianceAndCovariance[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, Vector{0.0f});
            sumWeights[componentA/ScalarType::Width::value].insert(componentA%ScalarType::Width::value, 0.0f);
        }

        const uint32_t lastComponent = distribution.getK()-1;
        if (componentB != lastComponent)
        {
            varianceAndCovariance[componentB/ScalarType::Width::value].insert(componentB%ScalarType::Width::value, varianceAndCovariance[lastComponent/ScalarType::Width::value][lastComponent%ScalarType::Width::value]);
            sumWeights[componentB/ScalarType::Width::value].insert(componentB%ScalarType::Width::value, sumWeights[lastComponent/ScalarType::Width::value][lastComponent%ScalarType::Width::value]);
        }
    }

    void split(const TDistribution& distribution, const uint32_t component)
    {
        const uint32_t newComponent = distribution.getK()-1;

        //reset covariance for split components
        varianceAndCovariance[component/ScalarType::Width::value].insert(component%ScalarType::Width::value, Vector{0.0f});
        sumWeights[component/ScalarType::Width::value].insert(component%ScalarType::Width::value, 0.0f);

        varianceAndCovariance[newComponent/ScalarType::Width::value].insert(newComponent%ScalarType::Width::value, Vector{0.0f});
        sumWeights[newComponent/ScalarType::Width::value].insert(newComponent%ScalarType::Width::value, 0.0f);
    }
};


}

#endif
