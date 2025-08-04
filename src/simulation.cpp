#include "engine/simulation.hpp"
#include <cmath>
#include <iostream>

constexpr static double GRAVITY_ACCELERATION                  = 9.8;
constexpr static double DRAG_COEFFICIENT = -1;
constexpr static double PARTICLE_SURFACE_RESTITUTION = 0.99;
constexpr static double PARTICLE_PARTICLE_RESTITUTION = 1;
const static Vec2 WIND_VELOCITY(0.1, -0.1);
//constexpr static double PARTICLE_ELECTROMAGNETISM_COEFFICIENT = 1;

Simulation::Simulation() {
    // Adding boundaries
    surfaces = {
        Surface(Vec2(0, 0.9), Vec2(0, -1)),
        Surface(Vec2(0, -0.9), Vec2(0, 1)),
        Surface(Vec2(4.5, 0), Vec2(-1, 0)),
        Surface(Vec2(-4.5, 0), Vec2(1, 0))
    };
}

void Simulation::init() {
    for (Particle& particle : particles) {
        if (particle.fixed) {
            static_particles.push_back(particle);
        } else {
            dynamic_particles.push_back(particle);
        }
    }
}

void Simulation::step(double dt) {
    std::vector<Vec2> forces(particles.size());
    for (Particle& particle : dynamic_particles) {
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
        auto [ collided_with_surface, surface_impact ] = process_particle_surface_collision(particle, new_position, dt, surface);
        collided = collided || collided_with_surface;
        total_impact += surface_impact;
    }
    for (const Particle& obstacle : static_particles) {
        auto [ collided_with_obstacle, obstacle_impact ] = process_particle_particle_collision(particle, new_position, dt, obstacle);
        collided = collided || collided_with_obstacle;
        total_impact += obstacle_impact;
    }
    return std::make_pair(collided, total_impact);
}

std::pair<bool, Vec2> Simulation::process_particle_surface_collision(const Particle& particle, Vec2& new_position, double dt, const Surface& surface)const {
    double dist = (new_position - surface.position) * surface.normal;
    if (dist > particle.radius) {
        return std::make_pair(false, Vec2(0, 0));
    }
    if (surface.normal * particle.velocity >= 0) {
        new_position += (particle.radius - dist) * surface.normal;
        return std::make_pair(false, Vec2(0, 0));
    }
    const double impact_coefficient = -(particle.velocity * surface.normal) * (PARTICLE_SURFACE_RESTITUTION + 1) * particle.mass;
    const double velocity_change = (particle.radius - dist) / (particle.velocity * surface.normal);
    new_position += velocity_change * particle.velocity;
    return std::make_pair(true, impact_coefficient * surface.normal / dt);
}

std::pair<bool, Vec2> Simulation::process_particle_particle_collision(const Particle& particle, Vec2& new_position, double dt, const Particle& other_particle)const {
    const double min_distance     = particle.radius + other_particle.radius;
    const Vec2 current_difference = particle.position - other_particle.position;
    const double current_distance = current_difference.norm();
    const Vec2 relative_velocity  = particle.velocity - other_particle.velocity;
    const Vec2 normal             = current_difference.normalize();
    if (current_distance > min_distance) {
        return std::make_pair(false, Vec2(0, 0));
    }
    if ((normal * relative_velocity) >= 0) {
        new_position += (min_distance - current_distance) * normal;
        return std::make_pair(false, Vec2(0, 0));
    }
    const double impact_coefficient = -(relative_velocity * normal) * (PARTICLE_PARTICLE_RESTITUTION + 1) * particle.mass;
    const double rollback_needed    = (min_distance - current_distance) / (-(normal * relative_velocity));
    new_position += rollback_needed * particle.velocity;  // TODO: Fix this so it works for moving particles as well
    return std::make_pair(true, impact_coefficient * normal / dt);
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