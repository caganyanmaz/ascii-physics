#include "engine/simulation.hpp"
#include "engine/renderer.hpp"
#include <chrono>
#include <thread>


constexpr static int FPS = 144;

constexpr static double WAIT_TIME = 1000000 / 144;
using namespace std::literals::chrono_literals;
constexpr static auto WAIT_DURATION = 7000us;

int main() {
    using clock = std::chrono::steady_clock;
    Simulation sim;
    Renderer renderer;

    sim.step(1.0 / 60.0);
    Particle particle;
    particle.position = {0, 0};
    particle.velocity = {1, 0};
    particle.acceleration = {0, 9.8};
    sim.add_particle(std::move(particle));
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
    }

    return 0;
}

