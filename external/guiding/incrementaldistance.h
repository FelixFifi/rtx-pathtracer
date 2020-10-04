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

#ifndef INCREMENTALDISTANCE_H
#define INCREMENTALDISTANCE_H

// Custom defines
#define SLog(x,y) std::cout << x << y << std::endl;
#define EXPECT_NOT_TAKEN(arg) arg
#define SAssert(arg)

#include <pmm/pmm.h>
#include <pmm/vector.h>
#include <pmm/ParametricMixtureModel.h>

#include <array>
#include <algorithm>
#include <string>

namespace guiding {

typedef lightpmm::Vector3 Vector;

    template<typename TDistribution>
    struct IncrementalDistance;

    template<typename TKernel, uint32_t NKernels>
    struct IncrementalDistance<lightpmm::ParametricMixtureModel<TKernel, NKernels>> {
        using TDistribution = lightpmm::ParametricMixtureModel<TKernel, NKernels>;
        using ScalarType = typename TKernel::ScalarType;

        std::array<ScalarType, TDistribution::NumKernels::value> distances;
        std::array<ScalarType, TDistribution::NumKernels::value> sumWeights;

        IncrementalDistance() {
            std::fill(distances.begin(), distances.end(), ScalarType{std::numeric_limits<float>::infinity()});
            std::fill(sumWeights.begin(), sumWeights.end(), ScalarType{0.0f});
        }

        template<typename TSampleRange>
        void updateDistances(const TDistribution &distribution, const TSampleRange &samples) {
            const uint32_t numComponents = distribution.getK();

            std::array<ScalarType, TDistribution::NumKernels::value> batchWeightedDistance;
            std::array<ScalarType, TDistribution::NumKernels::value> batchSumWeight;
            std::fill(batchWeightedDistance.begin(), batchWeightedDistance.end(), ScalarType{0.0f});
            std::fill(batchSumWeight.begin(), batchSumWeight.end(), ScalarType{0.0f});

            for (const auto &sample : samples) {
                if (EXPECT_NOT_TAKEN(!(sample.distance > 0.0f))) {
                    SLog("invalid sample distance %f. skipping sample.", sample.distance);
                    continue;
                }

                std::array<ScalarType, TDistribution::NumKernels::value> componentPDFs;
                std::array<ScalarType, TDistribution::NumKernels::value> weightedComponentPDFs;

                ScalarType mixturePDFPartialSum{0.0f};
                for (uint32_t i = 0;
                     i < (numComponents + ScalarType::Width::value - 1) / ScalarType::Width::value; i++) {
                    componentPDFs[i] = distribution.getComponent(i).pdf(sample.direction);
                    weightedComponentPDFs[i] = distribution.getComponent(i).m_weights * componentPDFs[i];
                    mixturePDFPartialSum += weightedComponentPDFs[i];
                }

                const float mixturePDF = lightpmm::sum(mixturePDFPartialSum);

                if (EXPECT_NOT_TAKEN(!(mixturePDF > PMM_EPSILON)))
                    continue;

                const float sampleWeightDivMixturePDF = sample.weight / mixturePDF;

                for (uint32_t k = 0;
                     k < (numComponents + ScalarType::Width::value - 1) / ScalarType::Width::value; ++k) {
                    const ScalarType sampleWeightTimesSoftAssignmentTimesPDF =
                            weightedComponentPDFs[k] * componentPDFs[k] * sampleWeightDivMixturePDF;

                    batchSumWeight[k] += sampleWeightTimesSoftAssignmentTimesPDF;
                    //using the harmonic mean
                    batchWeightedDistance[k] += sampleWeightTimesSoftAssignmentTimesPDF / sample.distance;
                }
            }

            for (uint32_t k = 0; k < (numComponents + ScalarType::Width::value - 1) / ScalarType::Width::value; ++k) {
                const ScalarType newSumWeight = sumWeights[k] + batchSumWeight[k];
                //using the harmonic mean
                distances[k] = newSumWeight / (sumWeights[k] / distances[k] + batchWeightedDistance[k]);
                sumWeights[k] = newSumWeight;
            }
        }

        void reposition(TDistribution &distribution, const Vector newPosToOldPos) {
            SAssert(!std::isinf(newPosToOldPos.x) && !std::isnan(newPosToOldPos.x));
            SAssert(!std::isinf(newPosToOldPos.y) && !std::isnan(newPosToOldPos.y));
            SAssert(!std::isinf(newPosToOldPos.z) && !std::isnan(newPosToOldPos.z));

            for (uint32_t k = 0;
                 k < (distribution.getK() + ScalarType::Width::value - 1) / ScalarType::Width::value; k++) {
                typename TDistribution::KernelType &kernel = distribution.getComponent(k);

                const typename ScalarType::BooleanType isInfiniteOrInvalid =
                        lightpmm::is_inf(distances[k]) || sumWeights[k] <= 0.0f;

                if (lightpmm::all(isInfiniteOrInvalid))
                    continue;

                const lightpmm::Vec<ScalarType> newPosToLightSource = kernel.m_mu * distances[k] + newPosToOldPos;
                const ScalarType newPosToLightSourceDistance = newPosToLightSource.length();

                distances[k] = lightpmm::ifthen(isInfiniteOrInvalid, distances[k], newPosToLightSourceDistance);
                kernel.m_mu = lightpmm::ifthen(isInfiniteOrInvalid, kernel.m_mu,
                                               newPosToLightSource / newPosToLightSourceDistance);
            }
        }

        void repositionDistribution(TDistribution &distribution, const Vector newPosToOldPos) const {
            SAssert(!std::isinf(newPosToOldPos.x) && !std::isnan(newPosToOldPos.x));
            SAssert(!std::isinf(newPosToOldPos.y) && !std::isnan(newPosToOldPos.y));
            SAssert(!std::isinf(newPosToOldPos.z) && !std::isnan(newPosToOldPos.z));

            for (uint32_t k = 0;
                 k < (distribution.getK() + ScalarType::Width::value - 1) / ScalarType::Width::value; k++) {
                typename TDistribution::KernelType &kernel = distribution.getComponent(k);

                const typename ScalarType::BooleanType isInfiniteOrInvalid =
                        lightpmm::is_inf(distances[k]) || sumWeights[k] <= 0.0f;

                if (lightpmm::all(isInfiniteOrInvalid))
                    continue;

                const lightpmm::Vec<ScalarType> newPosToLightSource = kernel.m_mu * distances[k] + newPosToOldPos;
                const ScalarType newPosToLightSourceDistance = newPosToLightSource.length();

                kernel.m_mu = lightpmm::ifthen(isInfiniteOrInvalid, kernel.m_mu,
                                               newPosToLightSource / newPosToLightSourceDistance);
            }
        }

        void operator*=(const float factor) {
            for (uint32_t k = 0; k < TDistribution::NumKernels::value; ++k)
                sumWeights[k] *= factor;
        }

        void clear() {
            *this = IncrementalDistance{};
        }

        void merge(const TDistribution &distribution, const uint32_t componentA, const uint32_t componentB) {
            const float weightA = distribution.getComponent(componentA / ScalarType::Width::value).getWeight(
                    componentA % ScalarType::Width::value);
            const float weightB = distribution.getComponent(componentB / ScalarType::Width::value).getWeight(
                    componentB % ScalarType::Width::value);
            const float mergedWeight = weightA + weightB;

            const float sumWeightA = sumWeights[componentA / ScalarType::Width::value][componentA %
                                                                                       ScalarType::Width::value];
            const float sumWeightB = sumWeights[componentB / ScalarType::Width::value][componentB %
                                                                                       ScalarType::Width::value];
            //using harmonic mean
            const float mergedSumWeight = mergedWeight / (weightA / sumWeightA + weightB / sumWeightB);

            const float distanceA = distances[componentA / ScalarType::Width::value][componentA %
                                                                                     ScalarType::Width::value];
            const float distanceB = distances[componentB / ScalarType::Width::value][componentB %
                                                                                     ScalarType::Width::value];
            //using harmonic mean
            const float mergedDistance = mergedWeight / (weightA / distanceA + weightB / distanceB);

            distances[componentA / ScalarType::Width::value].insert(componentA % ScalarType::Width::value,
                                                                    mergedDistance);
            sumWeights[componentA / ScalarType::Width::value].insert(componentA % ScalarType::Width::value,
                                                                     mergedSumWeight);

            const uint32_t lastComponent = distribution.getK() - 1;
            if (componentB != lastComponent) {
                distances[componentB / ScalarType::Width::value].insert(componentB % ScalarType::Width::value,
                                                                        distances[lastComponent /
                                                                                  ScalarType::Width::value][
                                                                                lastComponent %
                                                                                ScalarType::Width::value]);
                sumWeights[componentB / ScalarType::Width::value].insert(componentB % ScalarType::Width::value,
                                                                         sumWeights[lastComponent /
                                                                                    ScalarType::Width::value][
                                                                                 lastComponent %
                                                                                 ScalarType::Width::value]);
            }
        }

        void split(const TDistribution &distribution, const uint32_t component) {
            const uint32_t newComponent = distribution.getK() - 1;
            distances[newComponent / ScalarType::Width::value].insert(newComponent % ScalarType::Width::value,
                                                                      distances[component / ScalarType::Width::value][
                                                                              component % ScalarType::Width::value]);
            const float splitWeight =
                    0.5f * sumWeights[component / ScalarType::Width::value][component % ScalarType::Width::value];
            sumWeights[component / ScalarType::Width::value].insert(component % ScalarType::Width::value, splitWeight);
            sumWeights[newComponent / ScalarType::Width::value].insert(newComponent % ScalarType::Width::value,
                                                                       splitWeight);
        }

        std::string toString() const {
            std::ostringstream oss;

            oss << "IncrementalDistance [\n";
            for (size_t i = 0; i < TDistribution::MaxK::value; ++i) {
                oss << "  [" << i << "]:"
                    << " distance = "
                    << (sumWeights[i / ScalarType::Width::value][i % ScalarType::Width::value] > 0.0f ? distances[i /
                                                                                                                  ScalarType::Width::value][
                            i % ScalarType::Width::value] : std::numeric_limits<float>::infinity())
                    << ", sumWeights = " << sumWeights[i / ScalarType::Width::value][i % ScalarType::Width::value]
                    << '\n';
            }
            oss << ']';

            return oss.str();
        }
    };

}
#endif
