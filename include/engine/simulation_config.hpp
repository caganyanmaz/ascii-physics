#pragma once
#include "math.hpp"
#include "ode_solver.hpp"
#include "midpoint_ode_solver.hpp"

struct SimulationConfig {
    bool gravity = true;
    bool drag = true;
    bool wind = true;
    bool boundaries = false;
    double gravitational_acceleration = 9.8;
    double drag_coefficient = -1;
    double particle_surface_restitution = 0.99;
    double particle_particle_restitution = 1;
    Vec2<double> wind_velocity = Vec2<double>(0.1, -0.1);
    std::unique_ptr<OdeSolverFactory> ode_solver_factory = std::make_unique<OdeSolverFactoryInstance<MidpointOdeSolver>>();
};