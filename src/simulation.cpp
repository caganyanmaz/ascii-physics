#include "engine/simulation.hpp"
#include <iostream>

void Simulation::step(double dt) {
    // Stub logic
    for (Particle& particle : particles) {
        particle.position += dt * particle.velocity + dt * dt * 0.5 * particle.acceleration;
        particle.velocity += dt * particle.acceleration;
    }
}

const std::vector<Particle>& Simulation::get_particles()const {
    return particles;
}

void Simulation::add_particle(Particle&& particle) {
    particles.push_back(std::move(particle));
}