#include "engine/simulation.hpp"

static Particle make_particle(Vec2<double> pos, Vec2<double> vel, double mass = 1.0) {
    Particle p;
    p.position = pos;
    p.velocity = vel;
    p.mass = mass;
    return p;
}

// compact state serializer: pos/vel for each, then KE/PE/P
static std::string snapshot(const Simulation& sim) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6);

    int i = 0;
    for (const auto& p : sim.get_particles()) {
        os << "p" << i++ << " "
        << p.position.x << " " << p.position.y << " "
        << p.velocity.x << " " << p.velocity.y << "\n";
    }
    os << "KE " << sim.get_total_kinetic_energy() << "\n";
    os << "PE " << sim.get_total_potential_energy() << "\n";
    Vec2<double> P = sim.get_total_momentum();
    os << "P  " << P.x << " " << P.y << "\n";
    return os.str();
}