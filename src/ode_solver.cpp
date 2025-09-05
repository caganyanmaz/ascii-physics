#include "engine/ode_solver.hpp"
#include "engine/euler_ode_solver.hpp"


template<class OdeSolverInstance>
std::unique_ptr<OdeSolver> OdeSolverFactoryInstance<OdeSolverInstance>::make(std::vector<double>& y, std::vector<double>& dy, double dt) const {
    return std::make_unique<OdeSolverInstance>(y, dy, dt);
}


template class OdeSolverFactoryInstance<EulerOdeSolver>;