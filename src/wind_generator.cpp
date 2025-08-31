#include "engine/wind_generator.hpp"
WindGenerator::WindGenerator(Vec2 wind_velocity) : wind_velocity(wind_velocity) {}

void WindGenerator::generate(std::vector<Particle>& particles) {
    for (Particle& particle : particles) {
        particle.force_accumulator += wind_velocity;
    }
}