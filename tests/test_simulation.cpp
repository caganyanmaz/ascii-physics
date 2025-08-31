#include <gtest/gtest.h>
#include "engine/simulation.hpp"

#define MACHINE_EPSILON 1e-15
TEST(SimulationTest, ParticleSimple) {
    SimulationConfig simulation_config;
    simulation_config.gravity = false;
    simulation_config.drag = false;
    simulation_config.wind = false;
    Simulation<false, false, false> sim(std::move(simulation_config));
    Particle particle;
    particle.velocity = Vec2(0.5, -0.1);
    sim.add_particle(std::move(particle));
    sim.init();
    for (int i = 0; i < 100; i++) {
        sim.step(0.01);
    }
    ASSERT_EQ(sim.get_particles().size(), 1);
    ASSERT_NEAR(sim.get_particles()[0].position.x, 0.5, 100 * MACHINE_EPSILON);
    ASSERT_NEAR(sim.get_particles()[0].position.y, -0.1, 100 * MACHINE_EPSILON);
}

