#include "engine/simulation.hpp"
#include <iostream>

constexpr static double BOUNDARY = 0.95;

void Simulation::step(double dt) {
    // Stub logic
    for (Particle& particle : particles) {
        particle.position += dt * particle.velocity + dt * dt * 0.5 * particle.acceleration;
        particle.velocity += dt * particle.acceleration;
        // Collision logic on boundaries
        if (abs(particle.position.x) > BOUNDARY)
            particle.velocity.x *= -1;
        if (abs(particle.position.y) > BOUNDARY)
            particle.velocity.y *= -1;
    }
}

const std::vector<Particle>& Simulation::get_particles()const {
    return particles;
}

void Simulation::add_particle(Particle&& particle) {
    particles.push_back(std::move(particle));
}