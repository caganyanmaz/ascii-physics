#pragma once
#include "math.hpp"
#include <ostream>

struct Particle {
    Vec2 position;
    Vec2 velocity;
    double mass;
    double radius;
    char symbol;
    bool fixed;
    Particle() : mass(1), radius(0.01), symbol('.'), fixed(false) {}
};

std::ostream& operator<<(std::ostream& os, const Particle& particle);