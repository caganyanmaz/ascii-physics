#pragma once
#include "math.hpp"
#include <array>
#include <ostream>


struct Particle {
    enum NormalState {
        REGULAR,
        COLLINEAR,
        CONSTRAINED
    };
    Vec2<double> position;
    Vec2<double> velocity;
    Vec2<double> force_accumulator;
    std::array<Vec2<double>, 2> normals;
    int normal_count = 0;
    double mass;
    double radius;
    char symbol;
    bool fixed;
    Particle() : mass(1), radius(0.01), symbol('.'), fixed(false) {}
    void add_normal(Vec2<double> normal);
    void flush_normals();
    NormalState get_normal_state()const;
};

std::ostream& operator<<(std::ostream& os, const Particle& particle);