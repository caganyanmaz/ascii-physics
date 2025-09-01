#pragma once
#include "force_generator.hpp"

class SpringGenerator : public ForceGenerator {
private:
    int a_id, b_id;
    double rest_length, spring_constant, damping_constant;
public:
    SpringGenerator(int a_id, int b_id, double rest_length, double spring_constant, double damping_constant);
    void generate(std::vector<Particle>& particles)override;
};