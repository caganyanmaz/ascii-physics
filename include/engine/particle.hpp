#pragma once
#include "math.hpp"
#include <ostream>

struct Particle {
    Vec2 position;
    Vec2 velocity;
    double mass;
    double radius;
    Particle() : mass(1), radius(0.01) {}
};

std::ostream& operator<<(std::ostream& os, const Particle& particle);