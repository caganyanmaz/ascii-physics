#include <gtest/gtest.h>
#include "engine/simulation.hpp"
#include "engine/simulation_config.hpp"
#include "engine/particle.hpp"
#include "engine/ode_solver.hpp"
#include "engine/euler_ode_solver.hpp"
#include "golden.hpp"
#include <iomanip>

static Particle make_particle(Vec2<double> pos, Vec2<double> vel, double mass = 1.0) {
    Particle p;
    p.position = pos;
    p.velocity = vel;
    p.mass = mass;
    return p;
}

// compact state serializer: pos/vel for each, then KE/PE/P
static std::string snapshot(const Simulation& sim) {
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
    Vec2<double> P = sim.get_total_momentum();
    os << "P  " << P.x << " " << P.y << "\n";
    return os.str();
}



TEST(SimulationRegression, simple_free_flight) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false; 
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();

    Particle p;
    p.position = {0.0, 0.0};
    p.velocity = {0.5, -0.1};
    p.mass = 1.0;

    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    for (int i = 0; i < 100; ++i) sim.step(0.01);

    const std::string got = snapshot(sim);

    golden::verify("sim/sim_free_flight.txt", got);
}


// 1) two-body free flight (no forces): stable positions / velocities / totals
TEST(SimulationRegression, free_flight_two_body) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {0.5, -0.1}, 1.0));
    ps.push_back(make_particle({-1.0, 2.0}, {0.0, 0.2}, 2.0));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.01;
    for (int i = 0; i < 250; ++i) sim.step(dt); // T=2.5

    golden::verify("sim/free_flight_two_body.txt", snapshot(sim));
}

// 2) gravity drop (your +y downward): lock current integrator behavior
TEST(SimulationRegression, gravity_drop_single) {
    SimulationConfig cfg; cfg.gravity=true; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();
    cfg.gravitational_acceleration = 9.81;

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {0.0, 0.0}, 1.5));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.02;
    for (int i = 0; i < 100; ++i) sim.step(dt); // T=2.0

    golden::verify("sim/gravity_drop_single.txt", snapshot(sim));
}

// 3) wind impulse only: momentum change driven by wind every step
TEST(SimulationRegression, wind_impulse_line) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=true;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();
    cfg.wind_velocity = Vec2<double>(0.3, -0.15);

    std::vector<Particle> ps;
    ps.push_back(make_particle({-0.2, 0.0}, {0.0, 0.0}, 1.0));
    ps.push_back(make_particle({ 0.2, 0.0}, {0.0, 0.0}, 1.0));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.05;
    for (int i = 0; i < 60; ++i) sim.step(dt); // T=3.0

    golden::verify("sim/wind_impulse_line.txt", snapshot(sim));
}

// 4) drag only: decay trajectory — we snapshot the whole state
TEST(SimulationRegression, drag_decay) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=true; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();
    cfg.drag_coefficient = -0.2;

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {3.0, 4.0}, 1.0));
    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.01;
    for (int i = 0; i < 400; ++i) sim.step(dt); // T=4.0

    golden::verify("sim/drag_decay.txt", snapshot(sim));
}

// 5) internal spring only (no gravity/wind/drag): oscillation snapshot
TEST(SimulationRegression, spring_oscillation_two_body) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();

    Particle a = make_particle({-0.5, 0.0}, { 0.0,  0.5}, 1.0);
    Particle b = make_particle({ 0.5, 0.0}, { 0.0, -0.5}, 1.5);

    Simulation sim(std::move(cfg), std::vector<Particle>{a, b});
    sim.add_spring(/*a=*/0, /*b=*/1, /*k=*/30.0, /*d=*/0.7, /*rest=*/0.9);

    const double dt = 0.005;
    for (int i = 0; i < 800; ++i) sim.step(dt); // T=4.0

    golden::verify("sim/spring_oscillation_two_body.txt", snapshot(sim));
}

// 6) mixed forces: gravity + drag + wind with two particles
TEST(SimulationRegression, mixed_forces_two_body) {
    SimulationConfig cfg;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<EulerOdeSolver>>();
    cfg.gravity = true;
    cfg.drag    = true;
    cfg.wind    = true;
    cfg.gravitational_acceleration = 9.8;
    cfg.drag_coefficient = -0.1;
    cfg.wind_velocity = Vec2<double>(0.05, -0.02);

    std::vector<Particle> ps;
    ps.push_back(make_particle({-0.2, 0.0}, {0.4, 0.0}, 1.0));
    ps.push_back(make_particle({ 0.2, 0.0}, {-0.1, 0.0}, 2.0));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.01;
    for (int i = 0; i < 500; ++i) sim.step(dt); // T=5.0

    golden::verify("sim/mixed_forces_two_body.txt", snapshot(sim));
}
