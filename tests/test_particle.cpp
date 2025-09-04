#include <gtest/gtest.h>
#include "engine/particle.hpp"

constexpr double TOL = 1e-12;

TEST(ParticleTest, DefaultConstructorInitializesFields) {
    Particle p;

    EXPECT_NEAR(p.position.x, 0.0, TOL);
    EXPECT_NEAR(p.position.y, 0.0, TOL);

    EXPECT_NEAR(p.velocity.x, 0.0, TOL);
    EXPECT_NEAR(p.velocity.y, 0.0, TOL);

    EXPECT_NEAR(p.force_accumulator.x, 0.0, TOL);
    EXPECT_NEAR(p.force_accumulator.y, 0.0, TOL);

    EXPECT_DOUBLE_EQ(p.mass, 1.0);
    EXPECT_DOUBLE_EQ(p.radius, 0.01);
    EXPECT_EQ(p.symbol, '.');
    EXPECT_FALSE(p.fixed);
}

TEST(ParticleTest, FieldsAreMutable) {
    Particle p;
    p.position = Vec2<double>(1.0, 2.0);
    p.velocity = Vec2<double>(-0.5, 0.25);
    p.force_accumulator = Vec2<double>(0.1, -0.2);
    p.mass = 3.14;
    p.radius = 2.0;
    p.symbol = 'o';
    p.fixed = true;

    EXPECT_NEAR(p.position.x, 1.0, TOL);
    EXPECT_NEAR(p.position.y, 2.0, TOL);
    EXPECT_NEAR(p.velocity.x, -0.5, TOL);
    EXPECT_NEAR(p.velocity.y, 0.25, TOL);
    EXPECT_NEAR(p.force_accumulator.x, 0.1, TOL);
    EXPECT_NEAR(p.force_accumulator.y, -0.2, TOL);
    EXPECT_DOUBLE_EQ(p.mass, 3.14);
    EXPECT_DOUBLE_EQ(p.radius, 2.0);
    EXPECT_EQ(p.symbol, 'o');
    EXPECT_TRUE(p.fixed);
}