#include "engine/midpoint_ode_solver.hpp"
#include <cassert>

MidpointOdeSolver::MidpointOdeSolver(std::vector<double>& y, std::vector<double>& dy, double dt) 
    : y(y), dy(dy), dt(dt), is_step_done(false), initial_y(y.size()) {
    for (size_t i = 0; i < y.size(); i++) {
        initial_y[i] = y[i];
    }
}

bool MidpointOdeSolver::is_over()const {
    return is_step_done;
}

void MidpointOdeSolver::step() {
    // Euler step 
    for (size_t i = 0; i < y.size(); i++) {
        y[i] += dt * 0.5 * dy[i];
    }
    is_step_done = true;
}

void MidpointOdeSolver::end() {
    // Calculate y using dy at the beginning
    for (size_t i = 0; i < y.size(); i++) {
        y[i] = initial_y[i] + dt * dy[i];
    }
}