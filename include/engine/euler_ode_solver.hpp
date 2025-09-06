#pragma once 
#include "ode_solver.hpp"

class EulerOdeSolver : public OdeSolver {
    std::vector<double>& y;
    std::vector<double>& dy;
    double dt;
public:
    EulerOdeSolver(std::vector<double>& y, std::vector<double>& dy, double dt);
    void step()override;
    bool is_over()const override;
    void end()override;
};