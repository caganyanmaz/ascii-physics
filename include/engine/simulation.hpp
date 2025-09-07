#pragma once
#include "particle.hpp"
#include "surface.hpp"
#include "drag_generator.hpp"
#include "simulation_config.hpp"
#include "ode_solver.hpp"
#include <vector>
#include <memory>

class Simulation {
    const SimulationConfig config;
    std::vector<Particle> particles;
    std::vector<Surface> surfaces;
    std::vector<std::reference_wrapper<Particle>> static_particles;
    std::vector<std::reference_wrapper<Particle>> dynamic_particles;
    std::vector<std::unique_ptr<ForceGenerator>> force_generators;
public:
    Simulation(SimulationConfig&& config, std::vector<Particle>&& particles);
    Simulation(SimulationConfig&& config, std::vector<Particle>&& particles, std::vector<std::unique_ptr<ForceGenerator>>&& additional_force_generators);
    void step(double dt);
    const std::vector<Particle>& get_particles()const;
    void add_spring(int a_id, int b_id, double spring_constant, double damping_constant, double rest_length);
    double get_total_energy()const;
    double get_total_kinetic_energy()const;
    double get_total_potential_energy()const;
    Vec2<double> get_total_momentum()const;
private:  
    void clear_forces();
    void add_forces(); 
    std::pair<std::vector<double>, std::vector<double>> create_state_containers()const;
    void dump_state(std::vector<double>& y, std::vector<double>& dy)const;
    void load_state(const std::vector<double>& y);
    void process_without_collisions(double dt);
    std::vector<int> is_there_an_inter_penetration(const std::vector<int>& current_candidates)const;
    bool is_there_an_inter_penetration(const Particle& particle)const;
    bool is_there_a_particle_particle_inter_penetration(const Particle& a, const Particle& b)const;
    bool is_there_a_particle_surface_inter_penetration(const Particle& a, const Surface& surface)const;
    void process_all_collisions();
    void process_collisions(Particle& particle, const Particle& particle_after_unit_time);
    void process_particle_particle_collision(Particle& particle, const Particle& other_particle);
    void process_particle_surface_collision(Particle& particle, const Surface& surface);
    size_t get_state_vector_size()const;
    double get_dt_until_collision(double dt);
    void flush_normals();
};

