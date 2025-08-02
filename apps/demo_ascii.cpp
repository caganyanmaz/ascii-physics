#include "engine/simulation.hpp"
#include "engine/renderer.hpp"

int main() {
    Simulation sim;
    Renderer renderer;

    sim.step(1.0 / 60.0);
    Particle particle;
    particle.position = {0, 0};
    sim.add_particle(std::move(particle));
    renderer.render(sim.get_particles());

    return 0;
}

