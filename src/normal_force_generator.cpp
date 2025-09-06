#include "engine/normal_force_generator.hpp"
#include <cassert>

void NormalForceGenerator::generate(std::vector<Particle>& particles) {
    for (Particle& particle : particles) {
        if (particle.fixed)
            continue;
        Particle::NormalState normal_state = particle.get_normal_state();
        if (normal_state == Particle::NormalState::CONSTRAINED) {
            particle.force_accumulator = Vec2<double>(0, 0);
            continue;
        }
        // Really this one shouldn't happen, as a particle can go one direction
        assert(normal_state != Particle::NormalState::COLLINEAR);
        assert(particle.normal_count != 2); // TODO: Implement this later
        if (particle.normal_count == 0)
            continue;
        double force_magnitude = -(particle.force_accumulator * particle.normals[0]);
        assert(force_magnitude >= 0);
        particle.force_accumulator += force_magnitude * particle.normals[0];
    }
}
