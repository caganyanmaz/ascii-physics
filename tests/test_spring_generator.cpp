#include <gtest/gtest.h>
#include "engine/particle.hpp"
#include "engine/spring_generator.hpp"
#include "test_utils.hpp"
#include <vector>

using namespace test_utils;

// Hooke + damping reference used during test design (hard-coded below):
// F_a = - [ k_s (|l| - r) + k_d * ( (v_a - v_b) · l / |l| ) ] * (l / |l|)
// F_b = -F_a, where l = a.pos - b.pos

TEST(SpringGeneratorTest, applies_restoring_force_when_extended_no_damping) {
    // a at (3,0), b at (0,0): |l|=3, r=2 → extension = +1
    Particle a; a.position = Vec2<double>(3.0, 0.0); a.velocity = Vec2<double>(0.0, 0.0); a.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle b; b.position = Vec2<double>(0.0, 0.0); b.velocity = Vec2<double>(0.0, 0.0); b.force_accumulator = Vec2<double>(0.0, 0.0);

    std::vector<Particle> particles{a, b};

    const double rest_length = 2.0;
    const double spring_constant = 10.0;   
    const double damping_constant = 0.0;   
    SpringGenerator spring(0, 1, rest_length, spring_constant, damping_constant);

    spring.generate(particles);

    // l = (3,0), |l|=3, unit=(1,0)
    // term = k_s(|l|-r) = 10*(1) = 10
    // F_a = -10*(1,0) = (-10,0); F_b = (10,0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-10.0, 0.0)));
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>( 10.0, 0.0)));
}

TEST(SpringGeneratorTest, applies_restoring_force_when_compressed_no_damping) {
    // a at (1,0), b at (0,0): |l|=1, r=2 → extension = -1 (compressed)
    Particle a; a.position = Vec2<double>(1.0, 0.0); a.velocity = Vec2<double>(0.0, 0.0); a.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle b; b.position = Vec2<double>(0.0, 0.0); b.velocity = Vec2<double>(0.0, 0.0); b.force_accumulator = Vec2<double>(0.0, 0.0);

    std::vector<Particle> particles{a, b};

    const double rest_length = 2.0;
    const double spring_constant = 10.0;   
    const double damping_constant = 0.0;   
    SpringGenerator spring(0, 1, rest_length, spring_constant, damping_constant);

    spring.generate(particles);

    // l = (1,0), |l|=1, unit=(1,0)
    // term = k_s(|l|-r) = 10*(-1) = -10
    // F_a = -(-10)*(1,0) = (10,0); F_b = (-10,0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>( 10.0, 0.0)));
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>(-10.0, 0.0)));
}

TEST(SpringGeneratorTest, includes_linear_damping_term_in_direction_of_l) {
    // a at (3,0), b at (0,0) → |l|=3, unit=(1,0), r=2 → extension=+1
    // velocities: v_a=(1,0), v_b=(0,0) → (v_a - v_b)·l = (1,0)·(3,0) = 3
    // k_s=10, k_d=2 → term = 10*(1) + 2*(3/3) = 10 + 2 = 12
    Particle a; a.position = Vec2<double>(3.0, 0.0); a.velocity = Vec2<double>(1.0, 0.0); a.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle b; b.position = Vec2<double>(0.0, 0.0); b.velocity = Vec2<double>(0.0, 0.0); b.force_accumulator = Vec2<double>(0.0, 0.0);

    std::vector<Particle> particles{a, b};

    const double rest_length = 2.0;
    const double spring_constant = 10.0;   
    const double damping_constant = 2.0;   
    SpringGenerator spring(0, 1, rest_length, spring_constant, damping_constant);

    spring.generate(particles);

    // F_a = -12*(1,0) = (-12,0); F_b = (12,0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-12.0, 0.0)));
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>( 12.0, 0.0)));
}

TEST(SpringGeneratorTest, accumulates_on_existing_forces_and_leaves_other_fields_unchanged) {
    // choose an oblique configuration to verify both components
    // a(3,4), b(0,0) → l=(3,4), |l|=5, unit=(0.6,0.8), r=2 → extension=3
    // no damping
    Particle a;
    a.position = Vec2<double>(3.0, 4.0);
    a.velocity = Vec2<double>(-2.0, 1.0);
    a.mass = 2.5;
    a.radius = 0.05;
    a.symbol = 'A';
    a.fixed = false;
    a.force_accumulator = Vec2<double>(0.5, -1.0);

    Particle b;
    b.position = Vec2<double>(0.0, 0.0);
    b.velocity = Vec2<double>(0.0, 0.0);
    b.mass = 3.0;
    b.radius = 0.02;
    b.symbol = 'B';
    b.fixed = true;
    b.force_accumulator = Vec2<double>(-0.25, 0.75);

    std::vector<Particle> particles{a, b};

    const double rest_length = 2.0;
    const double spring_constant = 10.0;
    const double damping_constant = 0.0;
    SpringGenerator spring(0, 1, rest_length, spring_constant, damping_constant);

    spring.generate(particles);

    // term = k_s(|l|-r) = 10*(5-2) = 30
    // F_a = -30 * (0.6, 0.8) = (-18, -24)
    // accumulate:
    // a.F = (0.5, -1.0) + (-18, -24)   = (-17.5, -25.0)
    // b.F = (-0.25, 0.75) + (18, 24)   = (17.75, 24.75)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-17.5, -25.0)));
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>( 17.75, 24.75)));

    // verify other fields remained unchanged
    EXPECT_TRUE(vec2_near(particles[0].position, Vec2<double>(3.0, 4.0)));
    EXPECT_TRUE(vec2_near(particles[0].velocity, Vec2<double>(-2.0, 1.0)));
    EXPECT_DOUBLE_EQ(particles[0].mass, 2.5);
    EXPECT_DOUBLE_EQ(particles[0].radius, 0.05);
    EXPECT_EQ(particles[0].symbol, 'A');
    EXPECT_FALSE(particles[0].fixed);

    EXPECT_TRUE(vec2_near(particles[1].position, Vec2<double>(0.0, 0.0)));
    EXPECT_TRUE(vec2_near(particles[1].velocity, Vec2<double>(0.0, 0.0)));
    EXPECT_DOUBLE_EQ(particles[1].mass, 3.0);
    EXPECT_DOUBLE_EQ(particles[1].radius, 0.02);
    EXPECT_EQ(particles[1].symbol, 'B');
    EXPECT_TRUE(particles[1].fixed);
}

TEST(SpringGeneratorTest, handles_unrelated_particles_untouched) {
    // third particle should remain unchanged
    Particle a; a.position = Vec2<double>(3.0, 0.0); a.velocity = Vec2<double>(0.0, 0.0); a.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle b; b.position = Vec2<double>(0.0, 0.0); b.velocity = Vec2<double>(0.0, 0.0); b.force_accumulator = Vec2<double>(1.0, 1.0);
    Particle c; c.position = Vec2<double>(5.0, 5.0); c.velocity = Vec2<double>(0.5, 0.5); c.force_accumulator = Vec2<double>(-0.2, 0.3);

    std::vector<Particle> particles{a, b, c};

    SpringGenerator spring(0, 1, 2.0, 10.0, 0.0);
    spring.generate(particles);

    // from first test: F_a = (-10, 0), F_b = (10, 0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(-10.0, 0.0)));
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>( 11.0, 1.0))); // (1,1) + (10,0)
    // particle c untouched
    EXPECT_TRUE(vec2_near(particles[2].force_accumulator, Vec2<double>(-0.2, 0.3)));
}
