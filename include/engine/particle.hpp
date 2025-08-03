#pragma once
#include "math.hpp"
#include <ostream>

struct Particle {
    Vec2 position;
    Vec2 velocity;
    double mass;
    Particle() : mass(1) {}
};

std::ostream& operator<<(std::ostream& os, const Particle& particle);