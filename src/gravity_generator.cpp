#include "engine/gravity_generator.hpp"

GravityGenerator::GravityGenerator(double gravitational_acceleration) : gravitational_acceleration(gravitational_acceleration) {}

void GravityGenerator::generate(std::vector<Particle>& particles) {
    for (Particle& particle : particles) {
        particle.force_accumulator += Vec2(0, particle.mass * gravitational_acceleration);
    }
}