#include "engine/simulation.hpp"
#include "engine/renderer.hpp"

int main() {
    Simulation sim;
    Renderer renderer;

    sim.step(1.0 / 60.0);
    renderer.render();

    return 0;
}

