#include "engine/simulation.hpp"
#include "engine/renderer.hpp"

int main() {
    Simulation sim;
    Renderer renderer;

    sim.step(1.0 / 60.0);
    Particle particle;
    particle.position = {0, 0};
    renderer.render();

    return 0;
}

