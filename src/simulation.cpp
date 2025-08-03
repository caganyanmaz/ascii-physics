#include "engine/simulation.hpp"
#include <cmath>
#include <iostream>

constexpr static double BOUNDARY                              = 0.95;
constexpr static double GRAVITY_ACCELERATION                  = 9.8;
constexpr static double DRAG_COEFFICIENT = -1;
constexpr static double RESTITUTION = 0.99;
const static Vec2 WIND_VELOCITY(0.1, -0.1);
//constexpr static double PARTICLE_ELECTROMAGNETISM_COEFFICIENT = 1;

Simulation::Simulation() {
    // Adding boundaries
    surfaces = {
        Surface(Vec2(0, 0.9), Vec2(0, -1)),
        Surface(Vec2(0, -0.9), Vec2(0, 1)),
        Surface(Vec2(0.9, 0), Vec2(-1, 0)),
        Surface(Vec2(-0.9, 0), Vec2(1, 0))
    };
}

void Simulation::step(double dt) {
    std::vector<Vec2> forces(particles.size());
    for (Particle& particle : particles) {
        Vec2 new_position = particle.position + (dt * particle.velocity);
        auto [collided, total_impact] = process_collisions(particle, new_position, dt);
        const Vec2 total_force  = collided ? total_impact : calculate_particle_force(particle);
        const Vec2 acceleration = total_force / particle.mass;
        particle.position = new_position;
        particle.velocity += dt * acceleration;
    }
}

std::pair<bool, Vec2> Simulation::process_collisions(const Particle& particle, Vec2& new_position, double dt)const {
    bool collided = false;
    Vec2 total_impact(0, 0);
    for (const Surface& surface : surfaces) {
        double dist = (new_position - surface.position) * surface.normal;
        if (dist < particle.radius && surface.normal * particle.velocity < 0) {
            //std::cout << "Collision: " << particle << " " << surface << " " << dist << " "<< new_position<< "\n";
            collided = true;
            const double impact_coefficient = -(particle.velocity * surface.normal) * (RESTITUTION + 1) * particle.mass;
            total_impact += impact_coefficient * surface.normal / dt;
            const double velocity_change = (particle.radius - dist) / (particle.velocity * surface.normal);
            new_position += velocity_change * particle.velocity;
        }
        else if (dist < particle.radius) {
            new_position += (particle.radius - dist) * surface.normal;
        }
    }
    return std::make_pair(collided, total_impact);
}

Vec2 Simulation::calculate_particle_force(const Particle& particle)const {
    // Adding gravity
    Vec2 res(0, GRAVITY_ACCELERATION * particle.mass);
    // Adding drag
    res += (DRAG_COEFFICIENT * particle.velocity.norm()) * particle.velocity;
    // Adding wind effect
    res += WIND_VELOCITY;
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