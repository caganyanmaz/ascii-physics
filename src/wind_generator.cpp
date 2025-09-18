#include "engine/wind_generator.hpp"
WindGenerator::WindGenerator(Vec2<double> wind_velocity) : wind_velocity(wind_velocity) {}

void WindGenerator::generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies) {
    for (Particle& particle : particles) {
        particle.force_accumulator += wind_velocity;
    }
    for (RigidBody& rigid_body : rigid_bodies) {
        rigid_body.force_accumulator += wind_velocity;
    }
}