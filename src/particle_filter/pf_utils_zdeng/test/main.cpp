// main.cpp
#include "resample.hpp"
#include <iostream>
#include <random>

int main() {
    std::vector<Particle> particles;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> weightDistribution(0.0, 1.0);

    for (int i = 0; i < 20; ++i) {
        Particle p = {i, weightDistribution(generator)};
        particles.push_back(p);
    }

    std::cout << "Original particles:" << std::endl;
    for (const auto& p : particles) {
        std::cout << "ID: " << p.id << ", Weight: " << p.weight << std::endl;
    }

    auto resampled = resampleParticles(particles);

    std::cout << "\nResampled particles:" << std::endl;
    for (const auto& p : resampled) {
        std::cout << "ID: " << p.id << ", Weight: " << p.weight << std::endl;
    }

    return 0;
}

