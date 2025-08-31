#pragma once
#include "force_generator.hpp"


class GravityGenerator : public ForceGenerator {
    const double gravitational_acceleration;
public:
    GravityGenerator(double gravitational_acceleration);
    void generate(std::vector<Particle>& particles)override;
};