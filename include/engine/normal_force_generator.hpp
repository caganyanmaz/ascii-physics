#pragma once
#include "force_generator.hpp"

// Calculates normal forces for resting collisions
// Must be called after every other force calculation
class NormalForceGenerator : public ForceGenerator {
public:
    void generate(std::vector<Particle>& particles)override;
};