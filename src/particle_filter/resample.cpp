// resample.cpp
#include "resample.hpp"
#include <random>
#include <numeric>

std::vector<Particle> resampleParticles(const std::vector<Particle>& particles) {
    double totalWeight = std::accumulate(particles.begin(), particles.end(), 0.0,
                                         [](double sum, const Particle& p) { return sum + p.weight; });

    std::vector<Particle> resampledParticles;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, totalWeight);

    for (size_t i = 0; i < particles.size(); ++i) {
        double x = distribution(generator);
        double wSum = 0;
        for (const auto& particle : particles) {
            wSum += particle.weight;
            if (wSum > x) {
                resampledParticles.push_back(particle);
                break;
            }
        }
    }

    return resampledParticles;
}
