#pragma once
#include <vector>
#include <memory>

// General working logic: The ODE solver might evaluate the new y values and might need to re-evalate dy/dt at a newer point.
// Instead of providing a calculate_derivate function, the class will preserve the necessary stuff to solve the ODE in its internal state,
// and decide to continue calculating or not by passing a flag as a return value.

class OdeSolver {
public:
    virtual ~OdeSolver() = default;
    virtual void step() = 0;
    virtual bool is_over()const = 0;
    virtual void end() = 0;
};

struct OdeSolverFactory {
    virtual std::unique_ptr<OdeSolver> make(std::vector<double>& y, std::vector<double>& dy, double dt) const = 0;
};


template<class OdeSolverInstance>
struct OdeSolverFactoryInstance : public OdeSolverFactory {
    std::unique_ptr<OdeSolver> make(std::vector<double>& y, std::vector<double>& dy, double dt) const override;
};