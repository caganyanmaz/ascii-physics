#pragma once
#include "force_generator.hpp"


class WindGenerator : public ForceGenerator {
    const Vec2<double> wind_velocity;
public:
    WindGenerator(Vec2<double> wind_velocity);
    void generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies)override;
};