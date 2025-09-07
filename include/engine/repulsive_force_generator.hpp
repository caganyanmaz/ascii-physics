// Creates a repulsive force between two objects
// affected_particles: list of particle index / particle repulsiveness
// The force itself acts on each particle in the list, it repulses other objects based
// on the distance squared and the product of their repulsiveness, it gets affected by all particles separately
// So it can be used to model any repulsive force that obeys inverse-square and superposition laws
#pragma once
#include "force_generator.hpp"
#include <vector>
#include <utility>

class RepulsiveForceGenerator : public ForceGenerator {
    const std::vector<std::pair<size_t, double>> affected_particles;
public:
    RepulsiveForceGenerator(std::vector<std::pair<size_t, double>>&& affected_particles);
    void generate(std::vector<Particle>& particles)override;
};