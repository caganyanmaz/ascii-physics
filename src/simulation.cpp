#include "engine/simulation.hpp"
#include "engine/wind_generator.hpp"
#include "engine/spring_generator.hpp"
#include "engine/gravity_generator.hpp"
#include "engine/simulation_config.hpp"
#include "engine/normal_force_generator.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

constexpr static double COLLISION_TIME_ERROR_TOLERANCE = 1e-9;

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
    force_generators.push_back(std::make_unique<NormalForceGenerator>());

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
    if (dt < COLLISION_TIME_ERROR_TOLERANCE) {
        return;
    }
    while (dt > COLLISION_TIME_ERROR_TOLERANCE) {
        double cur = get_dt_until_collision(dt);
        process_without_collisions(cur);
        process_all_collisions();
        dt -= cur;
    }
    assert(COLLISION_TIME_ERROR_TOLERANCE >= dt && dt >= 0);
    process_without_collisions(dt);
    process_all_collisions();
    flush_normals();
}

double Simulation::get_dt_until_collision(double dt) {
    auto [ tmp_y, tmp_dy ] = create_state_containers();
    dump_state(tmp_y, tmp_dy);
    double l = 0, r = dt;
    std::vector<int> candidates;
    for (int i = 0; i < particles.size(); i++) {
        if (particles[i].fixed)
            candidates.push_back(i);    
    }
    while (r - l > COLLISION_TIME_ERROR_TOLERANCE) {
        double m = (l+r) * 0.5;
        process_without_collisions(m);
        std::vector<int> new_candidates = is_there_an_inter_penetration(candidates);
        assert(new_candidates.size() <= candidates.size()); // We should get fewer candidates at each run
        if (new_candidates.empty()) {
            l = m;
        } else {
            r = m;
            std::swap(candidates, new_candidates);
        }
        load_state(tmp_y);
    }
    return r;
}

void Simulation::process_without_collisions(double dt) {
    auto [ y, dy ] = create_state_containers();
    std::unique_ptr<OdeSolver> ode_solver = config.ode_solver_factory->make(y, dy, dt);

    clear_forces();
    add_forces();
    dump_state(y, dy);

    while (!ode_solver->is_over()) {
        ode_solver->step();
        load_state(y);
        clear_forces();
        add_forces();
        dump_state(y, dy);
    }
    ode_solver->end();
    load_state(y);
}

void Simulation::process_all_collisions() {
    auto [ y, dy ] = create_state_containers();
    dump_state(y, dy);
    process_without_collisions(COLLISION_TIME_ERROR_TOLERANCE);
    std::vector<Particle> particles_after_unit_time = particles;
    load_state(y);
    clear_forces();
    for (int i = 0; i < particles.size(); i++) {
        if (particles[i].fixed) {
            continue;
        }
        process_collisions(particles[i], particles_after_unit_time[i]);
    }
}

void Simulation::process_collisions(Particle& particle, const Particle& particle_after_unit_time) {
    for (const Particle& obstacle : static_particles) {
        if (is_there_a_particle_particle_inter_penetration(particle_after_unit_time, obstacle)) {
            process_particle_particle_collision(particle, obstacle);
        }
    }
    for (const Surface& surface : surfaces) {
        if (is_there_a_particle_surface_inter_penetration(particle_after_unit_time, surface)) {
            process_particle_surface_collision(particle, surface);
        }
    }

}

std::vector<int> Simulation::is_there_an_inter_penetration(const std::vector<int>& current_candidates)const {
    std::vector<int> inter_penetrations;
    for (int i : current_candidates) {
        const Particle& particle = particles[i];
        if (particle.fixed)
            continue;
        if (is_there_an_inter_penetration(particle)) {
            inter_penetrations.push_back(i);
        }
    }
    return inter_penetrations;
}

bool Simulation::is_there_an_inter_penetration(const Particle& particle)const {
    for (const Particle& obstacle: static_particles) {
        if (is_there_a_particle_particle_inter_penetration(particle, obstacle)) {
            return true;
        }
    }
    for (const Surface& surface: surfaces) {
        if (is_there_a_particle_surface_inter_penetration(particle, surface)) {
            return true;
        }
    }
    return false;
}

bool Simulation::is_there_a_particle_surface_inter_penetration(const Particle& particle, const Surface& surface)const {
    double distance = (particle.position - surface.position) * surface.normal;
    double relative_speed = - (particle.velocity * surface.normal);
    return distance < particle.radius && relative_speed > 0;
}

bool Simulation::is_there_a_particle_particle_inter_penetration(const Particle& a, const Particle& b)const {
    Vec2<double> position_difference     = a.position - b.position;
    double distance                      = position_difference.norm();
    double min_distance                  = (a.radius + b.radius);
    Vec2<double> relative_velocity       = a.velocity - b.velocity;
    Vec2<double> current_difference_norm = position_difference.normalize();
    return (distance < min_distance) && (current_difference_norm * relative_velocity < 0);
}

void Simulation::process_particle_surface_collision(Particle& particle, const Surface& surface) {
    assert(surface.normal * particle.velocity < 1e9);
    // Resting collision, this should only happen once per step (per particle surface collision)
    // Assumes surfaces are fixed
    if(surface.normal * particle.velocity >= 0) {
        particle.add_normal(surface.normal);
        particle.velocity += -(particle.velocity * surface.normal) * surface.normal;
        return;
    }
    const double impact_coefficient = -(particle.velocity * surface.normal) * (config.particle_surface_restitution + 1);
    particle.velocity += impact_coefficient * surface.normal;
}

void Simulation::process_particle_particle_collision(Particle& a, const Particle& b) {
    const Vec2<double> current_difference = a.position - b.position;
    const double current_distance = current_difference.norm();
    const Vec2<double> relative_velocity  = a.velocity - b.velocity;
    const Vec2<double> normal             = current_difference.normalize();
    assert(normal * relative_velocity < 1e9);
    // If it's a resting collision, it'll be detected only once per cycle, then we'll add the normal of particle b to normal forces influencing a 
    // so that a won't collide with the b again and again, causing the process to slow down
    // NOTE: This assumes a moving particle only collides with a fixed object, and the particle won't affect the momentum of the other particle.
    // Fix this logic when adding other types of collisions
    if (normal * relative_velocity >= 0) {
        a.add_normal(normal);
        a.velocity += -(a.velocity * normal) * normal;
        return;
    }
    const double impact_coefficient = -(relative_velocity * normal) * (config.particle_particle_restitution + 1);
    a.velocity += impact_coefficient * normal;
}

void Simulation::flush_normals() {
    for (Particle& particle : particles) {
        particle.flush_normals();
    }
}

constexpr static int PARTICLE_VECTOR_SIZE = 4;
    
std::pair<std::vector<double>, std::vector<double>> Simulation::create_state_containers()const {
    const int n = get_state_vector_size();
    return {std::vector<double>(n), std::vector<double>(n)};
}

void Simulation::dump_state(std::vector<double>& y, std::vector<double>& dy)const {
    const int n = get_state_vector_size();
    assert(y.size() == n && dy.size() == n);
    for (int i = 0; i < dynamic_particles.size(); i++) {
        const Particle& particle = dynamic_particles[i];
        y[i*PARTICLE_VECTOR_SIZE]    = particle.position.x;
        y[i*PARTICLE_VECTOR_SIZE+1]  = particle.position.y;
        y[i*PARTICLE_VECTOR_SIZE+2]  = particle.velocity.x;
        y[i*PARTICLE_VECTOR_SIZE+3]  = particle.velocity.y;
        dy[i*PARTICLE_VECTOR_SIZE]   = particle.velocity.x;
        dy[i*PARTICLE_VECTOR_SIZE+1] = particle.velocity.y;
        dy[i*PARTICLE_VECTOR_SIZE+2] = particle.force_accumulator.x / particles[i].mass;
        dy[i*PARTICLE_VECTOR_SIZE+3] = particle.force_accumulator.y / particles[i].mass;
    }
}

void Simulation::load_state(const std::vector<double>& y) {
    const int n = dynamic_particles.size() * PARTICLE_VECTOR_SIZE;
    assert(y.size() == n);
    for (int i = 0; i < dynamic_particles.size(); i++) {
        Particle& particle = dynamic_particles[i];
        particle.position.x = y[i*PARTICLE_VECTOR_SIZE];
        particle.position.y = y[i*PARTICLE_VECTOR_SIZE+1];
        particle.velocity.x = y[i*PARTICLE_VECTOR_SIZE+2];
        particle.velocity.y = y[i*PARTICLE_VECTOR_SIZE+3];
    }
}

int Simulation::get_state_vector_size()const {
    return PARTICLE_VECTOR_SIZE * dynamic_particles.size();
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
