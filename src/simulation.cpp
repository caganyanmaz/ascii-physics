#include "engine/simulation.hpp"
#include "engine/wind_generator.hpp"
#include "engine/spring_generator.hpp"
#include "engine/gravity_generator.hpp"
#include "engine/simulation_config.hpp"
#include <cmath>
#include <iostream>

Simulation::Simulation(SimulationConfig&& config, std::vector<Particle>&& particles) 
    :   config(std::move(config)), 
        particles(std::move(particles)),
        is_gravity_included(config.gravity)
{
    // Adding force generators
    if (config.gravity)
        force_generators.push_back(std::unique_ptr<ForceGenerator>(new GravityGenerator(config.gravitational_acceleration)));
    if (config.drag)
        force_generators.push_back(std::unique_ptr<ForceGenerator>(new DragGenerator(config.drag_coefficient)));
    if (config.wind)
        force_generators.push_back(std::unique_ptr<ForceGenerator>(new WindGenerator(config.wind_velocity)));

    // Adding boundaries
    surfaces = {
        Surface(Vec2<double>(0, 0.9), Vec2<double>(0, -1)),
        Surface(Vec2<double>(0, -0.9), Vec2<double>(0, 1)),
        Surface(Vec2<double>(4.5, 0), Vec2<double>(-1, 0)),
        Surface(Vec2<double>(-4.5, 0), Vec2<double>(1, 0))
    };

    // Setting up particles 
    for (Particle& particle : this->particles) {
        if (particle.fixed) {
            static_particles.push_back(particle);
        } else {
            dynamic_particles.push_back(particle);
        }
    }
}

void Simulation::step(double dt) {
    clear_forces();
    add_forces();
    for (Particle& particle : dynamic_particles) {
        Vec2<double> new_position = particle.position + (dt * particle.velocity);
        auto [collided, total_impact] = process_collisions(particle, new_position, dt);
        const Vec2<double> total_force  = collided ? total_impact : particle.force_accumulator;
        const Vec2<double> acceleration = total_force / particle.mass;
        particle.position = new_position;
        particle.velocity += dt * acceleration;
    }
}

void Simulation::add_spring(int a_id, int b_id, double spring_constant, double damping_constant, double rest_length) {
    force_generators.push_back(std::unique_ptr<ForceGenerator>(new SpringGenerator(a_id, b_id, spring_constant, damping_constant, rest_length)));
}

void Simulation::clear_forces() {
    for (Particle& particle : particles) {
        particle.force_accumulator = Vec2<double>(0, 0);
    }
}

void Simulation::add_forces() {    
    for (std::unique_ptr<ForceGenerator>& force_generator : force_generators) {
        force_generator->generate(particles);
    }
}

std::pair<bool, Vec2<double>> Simulation::process_collisions(const Particle& particle, Vec2<double>& new_position, double dt)const {
    bool collided = false;
    Vec2<double> total_impact(0, 0);
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

std::pair<bool, Vec2<double>> Simulation::process_particle_surface_collision(const Particle& particle, Vec2<double>& new_position, double dt, const Surface& surface)const {
    double dist = (new_position - surface.position) * surface.normal;
    if (dist > particle.radius) {
        return std::make_pair(false, Vec2<double>(0, 0));
    }
    if (surface.normal * particle.velocity >= 0) {
        new_position += (particle.radius - dist) * surface.normal;
        return std::make_pair(false, Vec2<double>(0, 0));
    }
    const double impact_coefficient = -(particle.velocity * surface.normal) * (config.particle_surface_restitution + 1) * particle.mass;
    const double velocity_change = (particle.radius - dist) / (particle.velocity * surface.normal);
    new_position += velocity_change * particle.velocity;
    return std::make_pair(true, impact_coefficient * surface.normal / dt);
}

std::pair<bool, Vec2<double>> Simulation::process_particle_particle_collision(const Particle& particle, Vec2<double>& new_position, double dt, const Particle& other_particle)const {
    const double min_distance     = particle.radius + other_particle.radius;
    const Vec2<double> current_difference = particle.position - other_particle.position;
    const double current_distance = current_difference.norm();
    const Vec2<double> relative_velocity  = particle.velocity - other_particle.velocity;
    const Vec2<double> normal             = current_difference.normalize();
    if (current_distance > min_distance) {
        return std::make_pair(false, Vec2<double>(0, 0));
    }
    if ((normal * relative_velocity) >= 0) {
        new_position += (min_distance - current_distance) * normal;
        return std::make_pair(false, Vec2<double>(0, 0));
    }
    const double impact_coefficient = -(relative_velocity * normal) * (config.particle_particle_restitution + 1) * particle.mass;
    const double rollback_needed    = (min_distance - current_distance) / (-(normal * relative_velocity));
    new_position += rollback_needed * particle.velocity;  // TODO: Fix this so it works for moving particles as well
    return std::make_pair(true, impact_coefficient * normal / dt);
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
    if (!is_gravity_included)
        return 0;
    double res = 0;
    for (const Particle& particle : particles) {
        res += particle.mass * (1 - particle.position.y) * config.gravitational_acceleration;
    }
    return res;
}

Vec2<double> Simulation::get_total_momentum()const {
    Vec2<double> res(0, 0);
    for (const Particle& particle : particles) {
        res += particle.mass * particle.velocity;
    }
    return res;
}

const std::vector<Particle>& Simulation::get_particles()const {
    return particles;
}

