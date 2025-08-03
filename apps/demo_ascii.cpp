#include "engine/simulation.hpp"
#include "engine/renderer.hpp"
#include <chrono>
#include <thread>
#include <iostream>


constexpr static int FPS = 144;

constexpr static double WAIT_TIME = 1000000 / 144;
using namespace std::literals::chrono_literals;
constexpr static auto WAIT_DURATION = 7000us;

void print_physics_invariants(const Simulation& simulation) {
    double kinetic_energy   = simulation.get_total_kinetic_energy();
    double potential_energy = simulation.get_total_potential_energy();
    Vec2 momentum = simulation.get_total_momentum();
    std::cout << "Energy: " << (kinetic_energy + potential_energy) << "J (kinetic: " << kinetic_energy << "J, potential" << potential_energy << "J)\n";
    std::cout << "Momentum: " << momentum << "\n";

}

int main() {
    using clock = std::chrono::steady_clock;
    Simulation sim;
    Renderer renderer;

    Particle particle1, particle2;
    particle1.velocity = {1, 0};
    particle2.position = {0.5, 0.5};
    sim.add_particle(std::move(particle1));
    sim.add_particle(std::move(particle2));
    auto last = clock::now();
    while (true) {
        auto duration = clock::now() - last;
        if (duration < WAIT_DURATION) {
            std::this_thread::sleep_for(WAIT_DURATION - duration); 
        }
        duration = clock::now() - last;
        last = clock::now();
        sim.step(std::chrono::duration_cast<std::chrono::microseconds>(duration).count() * 0.000001);
        renderer.render(sim.get_particles());
        print_physics_invariants(sim);
        //renderer.debug_render(sim.get_particles());
    }

    return 0;
}

