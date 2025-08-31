#pragma once
#include "math.hpp"

struct SimulationConfig {
    bool gravity = true;
    bool drag = true;
    bool wind = true;
    double gravitational_acceleration = 9.8;
    double drag_coefficient = -1;
    double particle_surface_restitution = 0.99;
    double particle_particle_restitution = 1;
    Vec2 wind_velocity = Vec2(0.1, -0.1);
};