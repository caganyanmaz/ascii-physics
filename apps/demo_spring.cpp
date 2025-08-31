#include "engine/simulation.hpp"
#include "engine/renderer.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <random>


//constexpr static int FPS = 144;
constexpr static int FRAMES_PER_RENDER = 100;
//constexpr static double WAIT_TIME = 1000000 / 144;
using namespace std::literals::chrono_literals;
//constexpr static auto WAIT_DURATION = 70us;

void print_physics_invariants(const Simulation& simulation) {
    double kinetic_energy   = simulation.get_total_kinetic_energy();
    double potential_energy = simulation.get_total_potential_energy();
    Vec2 momentum = simulation.get_total_momentum();
    std::cout << "Energy(without springs): " << (kinetic_energy + potential_energy) << "J (kinetic: " << kinetic_energy << "J, potential" << potential_energy << "J)                   \n";
    std::cout << "Momentum: " << momentum << "kg . m / s                \n";

}

double _random() {
    return ((double) rand() / (RAND_MAX));
}

Simulation create_simulation();



int main() {
    srand(42);
    using clock = std::chrono::steady_clock;
    Renderer renderer;
    Simulation sim = create_simulation();
    renderer.add_spring(0, 1);

    int frame_counter = 0;
    auto last = clock::now();
    while (true) {
        auto duration = clock::now() - last;
        //if (duration < WAIT_DURATION) {
        //    std::this_thread::sleep_for(WAIT_DURATION - duration); 
        //}
        //duration = clock::now() - last;
        last = clock::now();
        //renderer.debug_render(sim.get_particles());
        sim.step(std::chrono::duration_cast<std::chrono::microseconds>(duration).count() * 0.000001);
        frame_counter++;
        if (frame_counter >= FRAMES_PER_RENDER) {
            frame_counter = 0;
            renderer.render(sim.get_particles());
            print_physics_invariants(sim);
        }
    }

    return 0;
}

Simulation create_simulation() {
    std::vector<Particle> particles;
    Particle fixed_particle;
    fixed_particle.position = Vec2<double>(0, -0.8);
    fixed_particle.fixed = true;
    fixed_particle.radius = 0.2;
    fixed_particle.mass = 10;
    fixed_particle.symbol = 'O';
    particles.push_back(fixed_particle);
    Particle moving_particle;
    moving_particle.position = Vec2<double>(-0.5, 0.5);
    moving_particle.radius = 0.1;
    moving_particle.symbol = 'O';
    particles.push_back(moving_particle);

    SimulationConfig simulation_config;
    simulation_config.drag = false;
    simulation_config.wind = false;
    Simulation sim(std::move(simulation_config), std::move(particles));
    sim.add_spring(0, 1, (fixed_particle.position - moving_particle.position).norm() / 2, 50, 0);
    return sim;
}

