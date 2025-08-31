#pragma once
#include "force_generator.hpp"


class WindGenerator : public ForceGenerator {
    const Vec2 wind_velocity;
public:
    WindGenerator(Vec2 wind_velocity);
    void generate(std::vector<Particle>& particles)override;
};