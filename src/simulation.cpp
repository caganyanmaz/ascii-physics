#include "engine/simulation.hpp"
#include <cmath>
#include <iostream>

constexpr static double BOUNDARY                              = 0.95;
constexpr static double GRAVITY_ACCELERATION                  = 9.8;
constexpr static double PARTICLE_ELECTROMAGNETISM_COEFFICIENT = 1;

void Simulation::step(double dt) {
    std::vector<Vec2> forces(particles.size());
    for (int i = 0; i < particles.size(); i++){
        forces[i] = calculate_particle_force(i);
    }
    for (int i = 0; i < particles.size(); i++) {
        Particle& particle = particles[i];
        const Vec2 particle_acceleration = (1 / particle.mass) * forces[i];
        particle.position += dt * particle.velocity + dt * dt * 0.5 * particle_acceleration;
        particle.velocity += dt * particle_acceleration;
        // Collision logic on boundaries
        if (abs(particle.position.x) > BOUNDARY)
            particle.velocity.x *= -1;
        if (abs(particle.position.y) > BOUNDARY)
            particle.velocity.y *= -1;
    }
}

Vec2 Simulation::calculate_particle_force(int particle_index)const {
    // Adding gravity
    Vec2 res(0, GRAVITY_ACCELERATION * particles[particle_index].mass);
    for (int i = 0; i < particles.size(); i++) {
        if (i == particle_index)
            continue;
        Vec2 dist = particles[particle_index].position - particles[i].position;
        res += pow(1 / dist.norm(), 3) *  dist;
    }
    return res;
}

double Simulation::get_total_energy()const {
    return get_total_kinetic_energy() + get_total_potential_energy();
}

double Simulation::get_total_kinetic_energy()const {
    double res = 0;
    for (const Particle& particle : particles) {
        res += 0.5 * particle.mass * particle.velocity.norm_squared();
    }
    return res;
}

double Simulation::get_total_potential_energy()const {
    double res = 0;
    for (const Particle& particle : particles) {
        res += particle.mass * (1 - particle.position.y) * GRAVITY_ACCELERATION;
    }
    return res;
}

Vec2 Simulation::get_total_momentum()const {
    Vec2 res(0, 0);
    for (const Particle& particle : particles) {
        res += particle.mass * particle.velocity;
    }
    return res;
}

const std::vector<Particle>& Simulation::get_particles()const {
    return particles;
}

void Simulation::add_particle(Particle&& particle) {
    particles.push_back(std::move(particle));
}