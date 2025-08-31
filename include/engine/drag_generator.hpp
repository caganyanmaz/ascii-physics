#pragma once
#include "force_generator.hpp"


class DragGenerator : public ForceGenerator {
    const double drag_coefficient;
public:
    DragGenerator(double drag_coefficient);
    void generate(std::vector<Particle>& particles)override;
};