#include "engine/simulation.hpp"
#include "engine/renderer.hpp"
#include "engine/utils.hpp"
#include "engine/repulsive_force_generator.hpp"
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
    Vec2<double> momentum = simulation.get_total_momentum();
    std::cout << "Energy: " << (kinetic_energy + potential_energy) << "J (kinetic: " << kinetic_energy << "J, potential" << potential_energy << "J)                   \n";
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
    std::vector<char> symbols({
        '.', '*', '+', 'x', 'o', 'O'
    });
    for (int i = 0; i < 20; i++) {
        Particle particle;
        particle.position = {(_random() - 0.5) * 7, _random() * 0.1 - 0.9};
        particle.velocity = {_random() * 2 - 1, _random() * 2 - 1};
        particle.symbol = random_choice(symbols);
        particle.radius = (_random() * 0.3);
        //particle.velocity = {_random() * 0.02 - 0.01, _random() * 0.02 - 0.01};
        particles.push_back(std::move(particle));
    }
    SimulationConfig simulation_config;
    simulation_config.wind = false;
    simulation_config.gravity = false;
    simulation_config.drag = false;
    simulation_config.boundaries = true;
    return Simulation(std::move(simulation_config), std::move(particles));
}
