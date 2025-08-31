#include "engine/drag_generator.hpp"

DragGenerator::DragGenerator(double drag_coefficient) : drag_coefficient(drag_coefficient) {}

void DragGenerator::generate(std::vector<Particle>& particles) {
    for (Particle& particle : particles) {
        particle.force_accumulator += (drag_coefficient * particle.velocity.norm()) * particle.velocity;
    }
}