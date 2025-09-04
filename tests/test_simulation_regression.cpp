#include <gtest/gtest.h>
#include "engine/simulation.hpp"
#include "engine/simulation_config.hpp"
#include "engine/particle.hpp"
#include "golden.hpp"
#include <iomanip>

static std::string serialize_state(const Simulation& sim) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6);

    int i = 0;
    for (const auto& p : sim.get_particles()) {
        os << "p" << i++ << " "
           << p.position.x << " " << p.position.y << " "
           << p.velocity.x << " " << p.velocity.y << "\n";
    }
    os << "KE " << sim.get_total_kinetic_energy() << "\n";
    os << "PE " << sim.get_total_potential_energy() << "\n";
    os << "P  " << sim.get_total_momentum().x << " " << sim.get_total_momentum().y << "\n";
    return os.str();
}

TEST(SimulationRegression, simple_free_flight) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;

    Particle p;
    p.position = {0.0, 0.0};
    p.velocity = {0.5, -0.1};
    p.mass = 1.0;

    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    for (int i = 0; i < 100; ++i) sim.step(0.01);

    const std::string got = serialize_state(sim);

    golden::verify("sim/sim_free_flight.txt", got);
}