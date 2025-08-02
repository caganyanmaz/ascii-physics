#pragma once
#include "math.hpp"

struct Particle {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    double mass;
    Particle() : mass(0) {}
};