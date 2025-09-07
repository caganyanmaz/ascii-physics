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
constexpr static int WIDTH = 4;
constexpr static int HEIGHT = 4;

constexpr static double GRID_DISTANCE = 0.4;

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

void add_springs(Simulation& sim, Renderer& renderer);

Simulation create_simulation();



int main() {
    srand(42);
    using clock = std::chrono::steady_clock;
    Renderer renderer;
    Simulation sim = create_simulation();
    add_springs(sim, renderer);

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
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            Particle particle;
            particle.position = {y * GRID_DISTANCE - HEIGHT * GRID_DISTANCE * 0.5, x * GRID_DISTANCE - GRID_DISTANCE * WIDTH * 0.5};

            particle.velocity = (x == 0 && y == 0) ? Vec2<double>(-1, -1) : Vec2<double>(0, 0);
            particle.mass = 1;
            particle.symbol = (x == 0 && y == 0) ? '*' : 'O';
            particle.radius = 0.05;
            particles.push_back(std::move(particle));
        }
    }
    SimulationConfig simulation_config;
    simulation_config.wind = false;
    simulation_config.gravity = false;
    simulation_config.drag = false;
    simulation_config.boundaries = true;
    return Simulation(std::move(simulation_config), std::move(particles));
}

void add_springs(Simulation& sim, Renderer& renderer) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (y < HEIGHT - 1) {
                sim.add_spring(y * WIDTH + x, (y + 1) * WIDTH + x, GRID_DISTANCE, 50, 1);
                renderer.add_spring(y * WIDTH + x, (y + 1) * WIDTH + x);
            }
            if (x < WIDTH - 1) {
                sim.add_spring(y * WIDTH + x, y * WIDTH + x + 1, GRID_DISTANCE, 50, 1);
                renderer.add_spring(y * WIDTH + x, y * WIDTH + x + 1);
            }
        }
    }
}