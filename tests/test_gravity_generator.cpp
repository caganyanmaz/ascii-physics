// tests/gravity_generator_test.cpp
#include <gtest/gtest.h>
#include "engine/particle.hpp"
#include "engine/gravity_generator.hpp"
#include <vector>
#include "test_utils.hpp"

using namespace test_utils;

TEST(GravityGeneratorTest, ZeroMassParticleProducesNoForce) {
    Particle p;
    p.mass = 0.0;
    p.force_accumulator = Vec2<double>(0.1, 0.2);

    GravityGenerator gravity(9.81);
    std::vector<Particle> particles{p};
    gravity.generate(particles);

    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.1, 0.2)));
}

TEST(GravityGeneratorTest, AppliesForceEqualToMassTimesG) {
    Particle p;
    p.mass = 2.0;
    p.force_accumulator = Vec2<double>(0.0, 0.0);

    const double g = 9.81;
    GravityGenerator gravity(g);
    std::vector<Particle> particles{p};
    gravity.generate(particles);

    // +y is downward → (0, +19.62)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.0, 19.62)));
}

TEST(GravityGeneratorTest, AccumulatesOnExistingForces) {
    Particle p;
    p.mass = 1.5;
    p.force_accumulator = Vec2<double>(3.0, -1.0);

    const double g = 10.0;
    GravityGenerator gravity(g);
    std::vector<Particle> particles{p};
    gravity.generate(particles);

    // delta = (0, +15.0) → (3.0, -1.0) + (0, 15.0) = (3.0, 14.0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(3.0, 14.0)));
}

TEST(GravityGeneratorTest, DoesNotChangeOtherFields) {
    Particle p;
    p.position = Vec2<double>(5.0, 6.0);
    p.velocity = Vec2<double>(-1.0, 2.0);
    p.mass = 4.2;
    p.radius = 0.02;
    p.symbol = 'G';
    p.fixed = true;
    p.force_accumulator = Vec2<double>(0.0, 0.0);

    GravityGenerator gravity(9.81);
    std::vector<Particle> particles{p};
    gravity.generate(particles);

    EXPECT_TRUE(vec2_near(particles[0].position, Vec2<double>(5.0, 6.0)));
    EXPECT_TRUE(vec2_near(particles[0].velocity, Vec2<double>(-1.0, 2.0)));
    EXPECT_DOUBLE_EQ(particles[0].mass, 4.2);
    EXPECT_DOUBLE_EQ(particles[0].radius, 0.02);
    EXPECT_EQ(particles[0].symbol, 'G');
    EXPECT_EQ(particles[0].fixed, true);
}

TEST(GravityGeneratorTest, MultipleParticlesHandledIndependently) {
    Particle p1; p1.mass = 1.0; p1.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle p2; p2.mass = 2.0; p2.force_accumulator = Vec2<double>(1.0, -1.0);
    Particle p3; p3.mass = 0.0; p3.force_accumulator = Vec2<double>(-0.5, 0.5);

    const double g = 5.0;
    GravityGenerator gravity(g);
    std::vector<Particle> particles{p1, p2, p3};
    gravity.generate(particles);

    // p1: (0, +5.0)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.0, 5.0)));
    // p2: (1.0, -1.0) + (0, +10.0) = (1.0, 9.0)
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>(1.0, 9.0)));
    // p3: unchanged (mass = 0)
    EXPECT_TRUE(vec2_near(particles[2].force_accumulator, Vec2<double>(-0.5, 0.5)));
}
