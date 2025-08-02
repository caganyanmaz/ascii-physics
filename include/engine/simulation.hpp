#pragma once
#include "particle.hpp"
#include <vector>

class Simulation {
    std::vector<Particle> particles;
public:
    void step(double dt);
    const std::vector<Particle>& get_particles()const;
    void add_particle(Particle&& particle);
};
