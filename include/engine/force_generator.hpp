#pragma once
#include <vector>
#include "particle.hpp"

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    virtual void generate(std::vector<Particle>& particles) = 0;
};