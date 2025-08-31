#pragma once
#include "particle.hpp"
#include "surface.hpp"
#include <vector>
#include "simulation_config.hpp"

template<bool gravity, bool drag, bool wind>
class Simulation {
    std::vector<Particle> particles;
    std::vector<Surface> surfaces;
    std::vector<std::reference_wrapper<Particle>> static_particles;
    std::vector<std::reference_wrapper<Particle>> dynamic_particles;
    const SimulationConfig config;
public:
    Simulation(SimulationConfig&& config, std::vector<Particle>&& particles);
    void step(double dt);
    const std::vector<Particle>& get_particles()const;
    double get_total_energy()const;
    double get_total_kinetic_energy()const;
    double get_total_potential_energy()const;
    Vec2 get_total_momentum()const;
private:
    Vec2 calculate_particle_force(const Particle& particle)const;
    std::pair<bool, Vec2> process_collisions(const Particle& particle, Vec2& new_position, double dt)const;
    std::pair<bool, Vec2> process_particle_particle_collision(const Particle& particle, Vec2& new_position, double dt, const Particle& other_particle)const;
    std::pair<bool, Vec2> process_particle_surface_collision(const Particle& particle, Vec2& new_position, double dt, const Surface& surface)const;
};


extern template class Simulation<false, false, false>;
extern template class Simulation<false, false, true >;
extern template class Simulation<false, true , false>;
extern template class Simulation<false, true , true>;
extern template class Simulation<true , false, false>;
extern template class Simulation<true , false, true >;
extern template class Simulation<true , true , false>;
extern template class Simulation<true , true , true>;

