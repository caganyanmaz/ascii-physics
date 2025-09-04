#pragma once
#include "particle.hpp"
#include "surface.hpp"
#include "drag_generator.hpp"
#include "simulation_config.hpp"
#include <vector>
#include <memory>

class Simulation {
    bool is_gravity_included;
    std::vector<Particle> particles;
    std::vector<Surface> surfaces;
    std::vector<std::reference_wrapper<Particle>> static_particles;
    std::vector<std::reference_wrapper<Particle>> dynamic_particles;
    const SimulationConfig config;
    std::vector<std::unique_ptr<ForceGenerator>> force_generators;
public:
    Simulation(SimulationConfig&& config, std::vector<Particle>&& particles);
    void step(double dt);
    const std::vector<Particle>& get_particles()const;
    void add_spring(int a_id, int b_id, double spring_constant, double damping_constant, double rest_length);
    double get_total_energy()const;
    double get_total_kinetic_energy()const;
    double get_total_potential_energy()const;
    Vec2<double> get_total_momentum()const;
private:  
    std::pair<bool, Vec2<double>> process_collisions(const Particle& particle, Vec2<double>& new_position, double dt)const;
    std::pair<bool, Vec2<double>> process_particle_particle_collision(const Particle& particle, Vec2<double>& new_position, double dt, const Particle& other_particle)const;
    std::pair<bool, Vec2<double>> process_particle_surface_collision(const Particle& particle, Vec2<double>& new_position, double dt, const Surface& surface)const;
    void clear_forces();
    void add_forces();
};

