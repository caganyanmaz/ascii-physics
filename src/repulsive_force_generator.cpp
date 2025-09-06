#include "engine/repulsive_force_generator.hpp"
#include <unordered_set>



#include <iostream> 
RepulsiveForceGenerator::RepulsiveForceGenerator(std::vector<std::pair<int, double>>&& affected_particles) 
    : affected_particles(std::move(affected_particles)) {
    // Verifying data
    std::unordered_set<int> seen;
    std::cout << affected_particles.size();
    for (auto [ idx, repulsiveness ] : affected_particles) {
        assert(seen.find(idx) == seen.end());
        assert(repulsiveness > 0);
        seen.insert(idx);
    }
}

void repulse_particle(int a_idx, double a_repulsiveness, int b_idx, double b_repulsiveness, std::vector<Particle>& particles);

void RepulsiveForceGenerator::generate(std::vector<Particle>& particles) {
    for (auto [ a_idx, a_repulsiveness ] : affected_particles) {
        assert(0 <= a_idx && a_idx < particles.size());
        for (auto [ b_idx, b_repulsiveness ] : affected_particles) {
            assert(0 <= b_idx && b_idx < particles.size());
            if (a_idx == b_idx)
                continue;
            repulse_particle(a_idx, a_repulsiveness, b_idx, b_repulsiveness, particles);
        }
    }
    
}

void repulse_particle(int a_idx, double a_repulsiveness, int b_idx, double b_repulsiveness, std::vector<Particle>& particles) {
    const Vec2<double> dist = particles[a_idx].position - particles[b_idx].position;
    const double repulsion_coefficient = a_repulsiveness * b_repulsiveness;
    const double inv_dist_squared = 1 / dist.norm_squared();
    particles[a_idx].force_accumulator += (repulsion_coefficient * inv_dist_squared) * dist.normalize();
}