#include <gtest/gtest.h>
#include "engine/simulation.hpp"
#include "engine/simulation_config.hpp"
#include "engine/particle.hpp"
#include "engine/ode_solver.hpp"
#include "engine/midpoint_ode_solver.hpp"
#include "golden.hpp"
#include <iomanip>
#include <sstream>
#include "test_utils.hpp"
#include "test_simulation_utils.hpp"
using namespace test_utils;

// 1) single-body free flight (no forces) with Midpoint
TEST(SimulationRegressionMidpoint, simple_free_flight_midpoint) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();

    Particle p;
    p.position = {0.0, 0.0};
    p.velocity = {0.5, -0.1};
    p.mass = 1.0;

    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    for (int i = 0; i < 100; ++i) sim.step(0.01);

    golden::verify("sim_midpoint/sim_free_flight_midpoint.txt", snapshot(sim));
}

// 2) two-body free flight (no forces)
TEST(SimulationRegressionMidpoint, free_flight_two_body_midpoint) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {0.5, -0.1}, 1.0));
    ps.push_back(make_particle({-1.0, 2.0}, {0.0, 0.2}, 2.0));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.01;
    for (int i = 0; i < 250; ++i) sim.step(dt); // T=2.5

    golden::verify("sim_midpoint/free_flight_two_body_midpoint.txt", snapshot(sim));
}

// 3) gravity drop (your +y downward)
TEST(SimulationRegressionMidpoint, gravity_drop_single_midpoint) {
    SimulationConfig cfg; cfg.gravity=true; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();
    cfg.gravitational_acceleration = 9.81;

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {0.0, 0.0}, 1.5));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.02;
    for (int i = 0; i < 100; ++i) sim.step(dt); // T=2.0

    golden::verify("sim_midpoint/gravity_drop_single_midpoint.txt", snapshot(sim));
}

// 4) wind impulse only
TEST(SimulationRegressionMidpoint, wind_impulse_line_midpoint) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=true;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();
    cfg.wind_velocity = Vec2<double>(0.3, -0.15);

    std::vector<Particle> ps;
    ps.push_back(make_particle({-0.2, 0.0}, {0.0, 0.0}, 1.0));
    ps.push_back(make_particle({ 0.2, 0.0}, {0.0, 0.0}, 1.0));

    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.05;
    for (int i = 0; i < 60; ++i) sim.step(dt); // T=3.0

    golden::verify("sim_midpoint/wind_impulse_line_midpoint.txt", snapshot(sim));
}

// 5) drag only: decay trajectory
TEST(SimulationRegressionMidpoint, drag_decay_midpoint) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=true; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();
    cfg.drag_coefficient = -0.2;

    std::vector<Particle> ps;
    ps.push_back(make_particle({0.0, 0.0}, {3.0, 4.0}, 1.0));
    Simulation sim(std::move(cfg), std::move(ps));

    const double dt = 0.01;
    for (int i = 0; i < 400; ++i) sim.step(dt); // T=4.0

    golden::verify("sim_midpoint/drag_decay_midpoint.txt", snapshot(sim));
}

// 6) internal spring only (no gravity/wind/drag): oscillation snapshot
TEST(SimulationRegressionMidpoint, spring_oscillation_two_body_midpoint) {
    SimulationConfig cfg; cfg.gravity=false; cfg.drag=false; cfg.wind=false;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();

    Particle a = make_particle({-0.5, 0.0}, { 0.0,  0.5}, 1.0);
    Particle b = make_particle({ 0.5, 0.0}, { 0.0, -0.5}, 1.5);

    Simulation sim(std::move(cfg), std::vector<Particle>{a, b});
    sim.add_spring(/*a=*/0, /*b=*/1, /*k=*/30.0, /*d=*/0.7, /*rest=*/0.9);

    const double dt = 0.005;
    for (int i = 0; i < 800; ++i) sim.step(dt); // T=4.0

    golden::verify("sim_midpoint/spring_oscillation_two_body_midpoint.txt", snapshot(sim));
}

// 7) mixed forces: gravity + drag + wind with two particles
TEST(SimulationRegressionMidpoint, mixed_forces_two_body_midpoint) {
    SimulationConfig cfg;
    cfg.ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();
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

    golden::verify("sim_midpoint/mixed_forces_two_body_midpoint.txt", snapshot(sim));
}