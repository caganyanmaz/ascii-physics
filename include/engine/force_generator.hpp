#pragma once
#include <vector>
#include "particle.hpp"
#include "rigid_body.hpp"

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    virtual void generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies) = 0;
};