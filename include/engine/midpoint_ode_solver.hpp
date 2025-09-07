#pragma once
#include "ode_solver.hpp"

class MidpointOdeSolver : public OdeSolver {
    std::vector<double>& y;
    std::vector<double>& dy;
    double dt;
    bool is_step_done;
    std::vector<double> initial_y;
public:
    MidpointOdeSolver(std::vector<double>& y, std::vector<double>& dy, double dt);
    bool is_over()const override;
    void step()override;
    void end()override;
};