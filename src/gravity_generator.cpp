#include "engine/gravity_generator.hpp"

GravityGenerator::GravityGenerator(double gravitational_acceleration) : gravitational_acceleration(gravitational_acceleration) {}

void GravityGenerator::generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies) {
    for (Particle& particle : particles) {
        particle.force_accumulator += Vec2<double>(0, particle.mass * gravitational_acceleration);
    }
    for (RigidBody& rigid_body : rigid_bodies) {
        rigid_body.force_accumulator += Vec2<double>(0, rigid_body.mass * gravitational_acceleration);
    }
}