#include <cassert>
#include "engine/spring_generator.hpp"

SpringGenerator::SpringGenerator(int a_id, int b_id, double rest_length, double spring_constant, double damping_constant)
    : a_id(a_id), b_id(b_id), rest_length(rest_length), spring_constant(spring_constant), damping_constant(damping_constant) {}

void SpringGenerator::generate(std::vector<Particle>& particles) {  
    assert(0 <= a_id && a_id < particles.size() && 0 <= b_id && b_id < particles.size());
    Vec2<double> I        = particles[a_id].position - particles[b_id].position;
    Vec2<double> diff_I   = particles[a_id].velocity - particles[b_id].velocity;
    double norm_I = I.norm();
    double spring_factor = spring_constant * (norm_I - rest_length);
    double damping_factor = damping_constant * (I * diff_I) / norm_I;
    Vec2<double> f_a = -(spring_factor + damping_factor) * I / norm_I;
    Vec2<double> f_b = (-1.0) * f_a;
    particles[a_id].force_accumulator += f_a;
    particles[b_id].force_accumulator += f_b;
}