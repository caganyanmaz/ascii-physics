#pragma once
#include "force_generator.hpp"

class SpringGenerator : public ForceGenerator {
private:
    size_t a_id, b_id;
    double rest_length, spring_constant, damping_constant;
public:
    SpringGenerator(size_t a_id, size_t b_id, double rest_length, double spring_constant, double damping_constant);
    void generate(std::vector<Particle>& particles, std::vector<RigidBody>& rigid_bodies)override;
};