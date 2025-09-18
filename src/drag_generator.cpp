#include "engine/drag_generator.hpp"

DragGenerator::DragGenerator(double drag_coefficient) : drag_coefficient(drag_coefficient) {}

void DragGenerator::generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies) {
    for (Particle& particle : particles) {
        particle.force_accumulator += (drag_coefficient * particle.velocity.norm()) * particle.velocity;
    }
    for (RigidBody& rigid_body : rigid_bodies) {
        rigid_body.force_accumulator += (drag_coefficient * rigid_body.velocity.norm()) * rigid_body.velocity;
    }
}