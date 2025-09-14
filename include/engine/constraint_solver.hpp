#pragma once
#include "constraint.hpp"
#include "particle.hpp"
#include <vector>
#include <memory>


class ConstraintSolver {
    std::vector<std::unique_ptr<Constraint>> constraints;
    std::vector<double> c;
    std::vector<double> c_dot;
    std::vector<int> j_constraint_starts;
    SparseMatrix j;
    std::vector<int> j_dot_constraint_starts;
    SparseMatrix j_dot;
    double spring_constant;
    double damping_constant;
public:
    ConstraintSolver(std::vector<std::unique_ptr<Constraint>>&& constraints, int particle_count, double spring_constant, double damping_constant);
    void solve(std::vector<Particle>& particles);
};