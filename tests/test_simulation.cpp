#include <gtest/gtest.h>
#include "engine/simulation.hpp"
#include "engine/simulation_config.hpp"
#include "test_utils.hpp"

using test_utils::vec2_near;
using test_utils::TOL;

static Particle make_particle(Vec2<double> pos, Vec2<double> vel, double mass = 1.0) {
    Particle p;
    p.position = pos;
    p.velocity = vel;
    p.mass = mass;
    p.force_accumulator = Vec2<double>(0.0, 0.0);
    return p;
}

#define MACHINE_EPSILON 1e-15
TEST(SimulationTest, ParticleSimple) {
    std::vector<Particle> particles;
    Particle particle;
    particle.velocity = Vec2(0.5, -0.1);
    particles.push_back(std::move(particle));
    SimulationConfig simulation_config;
    simulation_config.gravity = false;
    simulation_config.drag = false;
    simulation_config.wind = false;
    Simulation sim(std::move(simulation_config), std::move(particles));
    for (int i = 0; i < 100; i++) {
        sim.step(0.01);
    }
    ASSERT_EQ(sim.get_particles().size(), 1);
    ASSERT_NEAR(sim.get_particles()[0].position.x, 0.5, 100 * MACHINE_EPSILON);
    ASSERT_NEAR(sim.get_particles()[0].position.y, -0.1, 100 * MACHINE_EPSILON);
}



TEST(SimulationPublicTest, constructs_and_exposes_particles) {
    SimulationConfig cfg; // defaults ok
    std::vector<Particle> particles;
    particles.push_back(make_particle(Vec2<double>(0.0, 0.0), Vec2<double>(0.0, 0.0)));
    particles.push_back(make_particle(Vec2<double>(1.0, 2.0), Vec2<double>(-0.5, 0.25)));

    Simulation sim(std::move(cfg), std::move(particles));

    const auto& ps = sim.get_particles();
    ASSERT_EQ(ps.size(), 2u);
    EXPECT_TRUE(vec2_near(ps[0].position, Vec2<double>(0.0, 0.0)));
    EXPECT_TRUE(vec2_near(ps[1].position, Vec2<double>(1.0, 2.0)));
}

TEST(SimulationPublicTest, no_forces_linear_motion_ke_and_momentum_constant) {
    SimulationConfig cfg;
    cfg.gravity = false;
    cfg.drag = false;
    cfg.wind = false;

    const Vec2<double> v0(0.5, -0.1);
    auto p = make_particle(Vec2<double>(0.0, 0.0), v0, 1.0);
    std::vector<Particle> particles{p};

    Simulation sim(std::move(cfg), std::move(particles));

    const double dt = 0.02;
    const int steps = 50;
    const double total_time = dt * steps;

    const double ke0 = sim.get_total_kinetic_energy();
    const Vec2<double> p0 = sim.get_total_momentum();

    for (int i = 0; i < steps; ++i) sim.step(dt);

    const auto& out = sim.get_particles()[0];
    EXPECT_TRUE(vec2_near(out.position, Vec2<double>(v0.x * total_time, v0.y * total_time), 1e-9));
    EXPECT_TRUE(vec2_near(out.velocity, v0, 1e-12));

    EXPECT_NEAR(sim.get_total_kinetic_energy(), ke0, 1e-12);
    Vec2<double> p_final = sim.get_total_momentum();
    EXPECT_TRUE(vec2_near(p_final, p0, 1e-12));

    // with no gravity, potential energy should not contribute
    EXPECT_NEAR(sim.get_total_energy(), sim.get_total_kinetic_energy(), 1e-12);
}

TEST(SimulationPublicTest, gravity_only_delta_potential_matches_mg_delta_y) {
    SimulationConfig cfg;
    cfg.gravity = true;
    cfg.drag = false;
    cfg.wind = false;
    cfg.gravitational_acceleration = 9.8; // +y is downward

    auto p = make_particle(Vec2<double>(0.0, 0.0), Vec2<double>(0.0, 0.0), /*mass=*/2.0);
    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    const double dt = 0.1;

    double pe_before = sim.get_total_potential_energy();
    double y_before = sim.get_particles()[0].position.y;

    sim.step(dt);

    double pe_after = sim.get_total_potential_energy();
    double y_after = sim.get_particles()[0].position.y;

    // ΔPE should be m * g * Δy with +y downward convention
    double expected_delta_pe = 2.0 * 9.8 * (y_after - y_before);
    EXPECT_NEAR(pe_after - pe_before, expected_delta_pe, 1e-6);

    // sanity: reported total energy equals KE + PE getters
    EXPECT_NEAR(sim.get_total_energy(),
                sim.get_total_kinetic_energy() + sim.get_total_potential_energy(), 1e-12);
}

TEST(SimulationPublicTest, wind_only_total_momentum_changes_by_w_dt) {
    SimulationConfig cfg;
    cfg.gravity = false;
    cfg.drag = false;
    cfg.wind = true;
    cfg.wind_velocity = Vec2<double>(0.5, -0.25); // this is the *force* per your WindGenerator spec

    auto p = make_particle(Vec2<double>(0.0, 0.0), Vec2<double>(0.0, 0.0), /*mass=*/1.0);
    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    const double dt = 0.2;
    Vec2<double> p_before = sim.get_total_momentum();
    sim.step(dt);
    Vec2<double> p_after = sim.get_total_momentum();

    // Δp = F * Δt = wind_velocity * dt  (mass = 1)
    EXPECT_TRUE(vec2_near(p_after,
                          Vec2<double>(p_before.x + 0.5 * dt, p_before.y - 0.25 * dt),
                          1e-12));
}

TEST(SimulationPublicTest, drag_only_reduces_momentum_magnitude) {
    SimulationConfig cfg;
    cfg.gravity = false;
    cfg.drag = true;
    cfg.wind = false;
    cfg.drag_coefficient = -0.5; // negative per your convention

    auto p = make_particle(Vec2<double>(0.0, 0.0), Vec2<double>(3.0, 4.0), /*mass=*/1.0); // |v|=5
    Simulation sim(std::move(cfg), std::vector<Particle>{p});

    const double dt = 0.05;
    Vec2<double> p0 = sim.get_total_momentum();
    double p0_mag = std::sqrt(p0.x * p0.x + p0.y * p0.y);

    sim.step(dt);

    Vec2<double> p1 = sim.get_total_momentum();
    double p1_mag = std::sqrt(p1.x * p1.x + p1.y * p1.y);

    EXPECT_LT(p1_mag, p0_mag); // should strictly decrease with viscous drag
}

TEST(SimulationPublicTest, internal_spring_conserves_total_momentum_no_external_forces) {
    SimulationConfig cfg;
    cfg.gravity = false;
    cfg.drag = false;
    cfg.wind = false;

    Particle a = make_particle(Vec2<double>(-1.0, 0.0), Vec2<double>( 1.0, 0.0), 1.0);
    Particle b = make_particle(Vec2<double>( 1.0, 0.0), Vec2<double>(-1.0, 0.0), 1.0);
    Simulation sim(std::move(cfg), std::vector<Particle>{a, b});

    // connect with a spring (parameters arbitrary)
    sim.add_spring(/*a_id=*/0, /*b_id=*/1, /*spring_constant=*/20.0, /*damping_constant=*/0.5, /*rest_length=*/1.0);

    Vec2<double> p_initial = sim.get_total_momentum();

    const double dt = 0.01;
    for (int i = 0; i < 200; ++i) sim.step(dt);

    Vec2<double> p_final = sim.get_total_momentum();
    EXPECT_TRUE(vec2_near(p_final, p_initial, 1e-8)); // internal forces only → conserve total momentum
}

TEST(SimulationPublicTest, spring_at_rest_length_no_initial_velocity_no_effect) {
    SimulationConfig cfg;
    cfg.gravity = false;
    cfg.drag = false;
    cfg.wind = false;

    // distance 2.0 between (0,0) and (2,0) equals rest_length
    Particle a = make_particle(Vec2<double>(0.0, 0.0), Vec2<double>(0.0, 0.0));
    Particle b = make_particle(Vec2<double>(2.0, 0.0), Vec2<double>(0.0, 0.0));
    Simulation sim(std::move(cfg), std::vector<Particle>{a, b});

    sim.add_spring(/*a_id=*/0, /*b_id=*/1, /*spring_constant=*/10.0, /*damping_constant=*/0.0, /*rest_length=*/2.0);

    const double dt = 0.1;
    sim.step(dt);

    const auto& ps = sim.get_particles();
    EXPECT_TRUE(vec2_near(ps[0].velocity, Vec2<double>(0.0, 0.0)));
    EXPECT_TRUE(vec2_near(ps[1].velocity, Vec2<double>(0.0, 0.0)));
    EXPECT_TRUE(vec2_near(ps[0].position, Vec2<double>(0.0, 0.0)));
    EXPECT_TRUE(vec2_near(ps[1].position, Vec2<double>(2.0, 0.0)));
}

TEST(SimulationPublicTest, kinetic_plus_potential_equals_reported_total_energy) {
    SimulationConfig cfg;
    cfg.gravity = true;   // include PE term
    cfg.drag = false;
    cfg.wind = false;
    cfg.gravitational_acceleration = 9.8;

    Particle a = make_particle(Vec2<double>(0.0, 1.0), Vec2<double>(1.0, 0.0), 2.0);
    Simulation sim(std::move(cfg), std::vector<Particle>{a});

    sim.step(0.05);

    double ke = sim.get_total_kinetic_energy();
    double pe = sim.get_total_potential_energy();
    double e  = sim.get_total_energy();
    EXPECT_NEAR(ke + pe, e, 1e-12);
}
