// resample.hpp
#ifndef RESAMPLE_HPP
#define RESAMPLE_HPP

#include <vector>

struct Particle {
    int id; // Identifier for particles
    double weight;
};

/**
 * Resamples particles based on their weights.
 *
 * @param particles The vector of particles to be resampled.
 * @return The resampled vector of particles.
 */
std::vector<Particle> resampleParticles(const std::vector<Particle>& particles);

#endif // RESAMPLE_HPP
