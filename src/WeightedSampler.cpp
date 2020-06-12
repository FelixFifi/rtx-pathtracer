//
// Created by felixfifi on 21.05.20.
//

#include "WeightedSampler.h"

std::mt19937 WeightedSampler::generator;
std::uniform_real_distribution<float> WeightedSampler::distribution;

int WeightedSampler::sample() {
    if (values.empty()) {
        return -1;
    }

    float random = distribution(generator);
    random *= total;

    float sum = values[0];
    for (int i = 1; i < values.size(); ++i) {
        if (sum > random) {
            return i - 1;
        }
        sum += values[i];
    }

    return values.size() - 1;
}

WeightedSampler::WeightedSampler(const std::vector<float> &values) : values(values) {
    total = 0.0f;
    for (auto value : values) {
        total += value;
    }
}

std::vector<float> WeightedSampler::getProbabilities() {
    std::vector<float> probs(values.size());

    for (int iValue = 0; iValue < values.size(); ++iValue) {
        probs[iValue] = values[iValue] / total;
    }

    return probs;
}
