#pragma once
#include "particle.hpp"
#include <vector>

class Simulation {
    std::vector<Particle> particles;
public:
    void step(double dt);
    const std::vector<Particle>& get_particles()const;
    void add_particle(Particle&& particle);
    double get_total_energy()const;
    double get_total_kinetic_energy()const;
    double get_total_potential_energy()const;
    Vec2 get_total_momentum()const;
private:
    Vec2 calculate_particle_force(int particle_index)const;
};
