#pragma once
#include <vector>
#include "particle.hpp"

class ForceGenerator {
public:
    virtual void generate(std::vector<Particle>& particles) = 0;
};