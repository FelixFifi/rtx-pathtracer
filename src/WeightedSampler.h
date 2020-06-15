//
// Created by felixfifi on 21.05.20.
//

#ifndef RTX_RAYTRACER_WEIGHTEDSAMPLER_H
#define RTX_RAYTRACER_WEIGHTEDSAMPLER_H

#include <vector>
#include <random>

class WeightedSampler {
public:
    WeightedSampler(const std::vector<float> &values);

    /**
     * @return The index of the chosen object, weighted by values
     */
    int sample();

    std::vector<float> getProbabilities();
private:
    std::vector<float> values;
    float total;
public:
    float getTotal() const;

private:

    static std::mt19937 generator;
    static std::uniform_real_distribution<float> distribution;
};

#endif //RTX_RAYTRACER_WEIGHTEDSAMPLER_H
