#include <gtest/gtest.h>
#include "engine/particle.hpp"
#include "engine/drag_generator.hpp"
#include "test_utils.hpp"   
#include <vector>

using namespace test_utils;

TEST(DragGeneratorTest, ZeroVelocity_YieldsNoForce) {
    Particle p;
    p.velocity = Vec2<double>(0.0, 0.0);
    p.force_accumulator = Vec2<double>(0.1, -0.2); // ensure we don't overwrite

    const double drag_coefficient = -0.1;
    DragGenerator drag(drag_coefficient);

    std::vector<Particle> particles{p};
    std::vector<RigidBody> rigid_bodies{};
    drag.generate(particles, rigid_bodies);

    // unchanged
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.1, -0.2)));
}

TEST(DragGeneratorTest, AppliesOpposingForce_ProportionalToSpeedTimesVelocity) {
    Particle p;
    p.velocity = Vec2<double>(3.0, 4.0); // |v| = 5
    p.force_accumulator = Vec2<double>(0.0, 0.0);

    const double drag_coefficient = -0.1; // already negative per convention
    DragGenerator drag(drag_coefficient);

    std::vector<Particle> particles{p};
    std::vector<RigidBody> rigid_bodies{};
    drag.generate(particles, rigid_bodies);

    // ΔF = k * |v| * v = -0.1 * 5 * (3, 4) = (-1.5, -2.0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-1.5, -2.0)));

    // Opposes motion: dot(F, v) <= 0 (allow tiny numerical noise)
    const double dot = particles[0].force_accumulator.x * p.velocity.x +
                       particles[0].force_accumulator.y * p.velocity.y;
    EXPECT_LE(dot, TOL);
}

TEST(DragGeneratorTest, AccumulatesIntoExistingForces) {
    Particle p;
    p.velocity = Vec2<double>(-2.0, 1.0);  // |v| = sqrt(5)
    p.force_accumulator = Vec2<double>(0.5, 0.5);

    const double drag_coefficient = -0.25;
    DragGenerator drag(drag_coefficient);

    std::vector<Particle> particles{p};
    std::vector<RigidBody> rigid_bodies{};
    drag.generate(particles, rigid_bodies);

    // k*|v| = -0.25 * sqrt(5) ≈ -0.5590169943749475
    // ΔF = (-0.5590169943749475) * (-2.0, 1.0) = (1.118033988749895, -0.5590169943749475)
    // accumulated = (0.5, 0.5) + ΔF = (1.618033988749895, -0.0590169943749475)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator,
                          Vec2<double>(1.618033988749895, -0.0590169943749475)));
}

TEST(DragGeneratorTest, DoesNotModifyOtherParticleState) {
    Particle p;
    p.position = Vec2<double>(10.0, -7.0);
    p.velocity = Vec2<double>(1.0, 0.0);
    p.mass = 3.0;
    p.radius = 0.02;
    p.symbol = '#';
    p.fixed = true; // generator should only touch force_accumulator

    const double drag_coefficient = -1.0;
    DragGenerator drag(drag_coefficient);

    std::vector<Particle> particles{p};
    std::vector<RigidBody> rigid_bodies{};
    drag.generate(particles, rigid_bodies);

    // position/velocity/mass/radius/symbol/fixed unchanged
    EXPECT_TRUE(vec2_near(particles[0].position, p.position));
    EXPECT_TRUE(vec2_near(particles[0].velocity, p.velocity));
    EXPECT_DOUBLE_EQ(particles[0].mass, p.mass);
    EXPECT_DOUBLE_EQ(particles[0].radius, p.radius);
    EXPECT_EQ(particles[0].symbol, p.symbol);
    EXPECT_EQ(particles[0].fixed, p.fixed);
}

TEST(DragGeneratorTest, HandlesMultipleParticlesIndependently) {
    Particle p1; p1.velocity = Vec2<double>(2.0, 0.0);  p1.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle p2; p2.velocity = Vec2<double>(0.0, -3.0); p2.force_accumulator = Vec2<double>(1.0, -1.0);
    Particle p3; p3.velocity = Vec2<double>(0.0, 0.0);  p3.force_accumulator = Vec2<double>(-0.2, 0.3);

    const double drag_coefficient = -0.5;
    DragGenerator drag(drag_coefficient);

    std::vector<Particle> particles{p1, p2, p3};
    std::vector<RigidBody> rigid_bodies{};
    drag.generate(particles, rigid_bodies);

    // p1: |v|=2 → ΔF = -0.5*2*(2,0) = (-2.0, 0.0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-2.0, 0.0)));

    // p2: |v|=3 → ΔF = -0.5*3*(0,-3) = (0.0, 4.5); accumulated = (1.0, -1.0) + (0.0, 4.5) = (1.0, 3.5)
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>(1.0, 3.5)));

    // p3: zero velocity → unchanged
    EXPECT_TRUE(vec2_near(particles[2].force_accumulator, Vec2<double>(-0.2, 0.3)));
}
