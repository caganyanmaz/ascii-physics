#pragma once
#include "particle.hpp"
#include "surface.hpp"
#include "drag_generator.hpp"
#include "simulation_config.hpp"
#include <vector>
#include <memory>

class Simulation {
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
    double get_total_energy()const;
    double get_total_kinetic_energy()const;
    double get_total_potential_energy()const;
    Vec2 get_total_momentum()const;
private:
    std::pair<bool, Vec2> process_collisions(const Particle& particle, Vec2& new_position, double dt)const;
    std::pair<bool, Vec2> process_particle_particle_collision(const Particle& particle, Vec2& new_position, double dt, const Particle& other_particle)const;
    std::pair<bool, Vec2> process_particle_surface_collision(const Particle& particle, Vec2& new_position, double dt, const Surface& surface)const;
    void clear_forces();
    void add_forces();
};

