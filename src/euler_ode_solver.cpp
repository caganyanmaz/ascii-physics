#include "engine/euler_ode_solver.hpp"
#include <cassert>

EulerOdeSolver::EulerOdeSolver(std::vector<double>& y, std::vector<double>& dy, double dt) : y(y), dy(dy), dt(dt) {}

void EulerOdeSolver::step() {
    assert(false); // Shouldn't happen as we don't enter the loop
}

bool EulerOdeSolver::is_over()const {
    return true;
}

void EulerOdeSolver::end() {
    assert(y.size() == dy.size());
    for (int i = 0; i < y.size(); i++) {
        y[i] += dy[i] * dt;
    }
}