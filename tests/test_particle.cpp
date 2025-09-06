#include "engine/particle.hpp"
#include "engine/math.hpp"
#include "test_utils.hpp"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

using test_utils::vec2_near;
namespace {
    constexpr double EQ_TOL = 1e-12; // for exact-ish value compares
    constexpr double COL_TOL = 1e-9; // collinearity tolerance (from spec)
}

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

// Helper: set-membership with order-insensitive compare for two normals
static ::testing::AssertionResult contains_both(const std::array<Vec2<double>,2>& arr,
                                                const Vec2<double>& u,
                                                const Vec2<double>& v,
                                                double tol = EQ_TOL) {
    bool first_match  = (vec2_near(arr[0], u, tol) && vec2_near(arr[1], v, tol));
    bool second_match = (vec2_near(arr[0], v, tol) && vec2_near(arr[1], u, tol));
    if (first_match || second_match) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
        << "Array does not contain both normals within tol="
        << tol << ". Got: (" << arr[0].x << "," << arr[0].y
        << "), (" << arr[1].x << "," << arr[1].y
        << ") vs expected any ordering of (" << u.x << "," << u.y
        << ") and (" << v.x << "," << v.y << ")";
}


TEST(ParticleNormals, DefaultState) {
    Particle p;
    EXPECT_EQ(p.get_normal_state(), Particle::REGULAR);
    EXPECT_EQ(p.normal_count, 0);
}

TEST(ParticleNormals, FlushNormalsResetsStateAndCount) {
    Particle p;
    p.add_normal({1.0, 0.0});
    p.add_normal({0.0, 1.0});
    ASSERT_EQ(p.normal_count, 2);
    ASSERT_EQ(p.get_normal_state(), Particle::REGULAR);

    p.flush_normals();
    EXPECT_EQ(p.get_normal_state(), Particle::REGULAR);
    EXPECT_EQ(p.normal_count, 0);
}

TEST(ParticleNormals, SingleNormal_StoredAtIndex0_StateRegular) {
    Particle p;
    const Vec2<double> n{1.0, 0.0};
    p.add_normal(n);

    EXPECT_EQ(p.get_normal_state(), Particle::REGULAR);
    EXPECT_EQ(p.normal_count, 1);
    EXPECT_TRUE(vec2_near(p.normals[0], n, EQ_TOL));
    EXPECT_NEAR(std::hypot(p.normals[0].x, p.normals[0].y), 1.0, EQ_TOL);
}

TEST(ParticleNormals, TwoNonCollinearNormals_StateRegular_BothStored) {
    Particle p;
    const Vec2<double> n1{1.0, 0.0};
    const Vec2<double> n2{0.0, 1.0};

    p.add_normal(n1);
    p.add_normal(n2);

    EXPECT_EQ(p.get_normal_state(), Particle::REGULAR);
    EXPECT_EQ(p.normal_count, 2);
    EXPECT_TRUE(contains_both(p.normals, n1, n2));
    EXPECT_NEAR(std::hypot(p.normals[0].x, p.normals[0].y), 1.0, EQ_TOL);
    EXPECT_NEAR(std::hypot(p.normals[1].x, p.normals[1].y), 1.0, EQ_TOL);
}

TEST(ParticleNormals, TwoCollinearOppositeNormals_StateCollinear_BothStored) {
    Particle p;
    // Hard-code exact opposites
    const Vec2<double> n1{ 1.0, 0.0};
    const Vec2<double> n2{-1.0, 0.0};

    p.add_normal(n1);
    p.add_normal(n2);

    EXPECT_EQ(p.get_normal_state(), Particle::COLLINEAR);
    EXPECT_TRUE(contains_both(p.normals, n1, n2));
}

TEST(ParticleNormals, ThreeNormalsNotFittingSemicircle_StateConstrained_CountUnchanged) {
    Particle p;
    // 0°, 120°, 240° → cannot fit in any semicircle
    const Vec2<double> n0{ 1.0, 0.0};
    const Vec2<double> n1{-0.5,  std::sqrt(3.0)/2.0};
    const Vec2<double> n2{-0.5, -std::sqrt(3.0)/2.0};

    p.add_normal(n0);
    p.add_normal(n1);
    ASSERT_EQ(p.normal_count, 2);

    p.add_normal(n2);

    EXPECT_EQ(p.get_normal_state(), Particle::CONSTRAINED);

    // Further adds are no-ops; still must pass unit vectors
    p.add_normal({0.0, 1.0});
    EXPECT_EQ(p.get_normal_state(), Particle::CONSTRAINED);
}

TEST(ParticleNormals, AddingCoincidingNormalWhileCollinear_StaysCollinear_CountTwo) {
    Particle p;
    const Vec2<double> n1{ 1.0, 0.0};
    const Vec2<double> n2{-1.0, 0.0}; // opposite → collinear

    p.add_normal(n1);
    p.add_normal(n2);
    ASSERT_EQ(p.get_normal_state(), Particle::COLLINEAR);

    // Add a third normal coinciding with one of the stored collinear normals
    p.add_normal(n1);

    EXPECT_EQ(p.get_normal_state(), Particle::COLLINEAR);
    EXPECT_TRUE(contains_both(p.normals, n1, n2));
}

TEST(ParticleNormals, OtherFieldsUnaffectedByNormalOps) {
    Particle p;

    p.position = {3.0, -2.0};
    p.velocity = {0.5, 0.25};
    p.force_accumulator = {7.0, -4.0};
    p.mass = 2.5;
    p.radius = 0.42;
    p.symbol = '@';
    p.fixed = true;

    p.add_normal({1.0, 0.0});
    p.add_normal({0.0, 1.0});
    p.flush_normals();

    EXPECT_TRUE(vec2_near(p.position, {3.0, -2.0}, EQ_TOL));
    EXPECT_TRUE(vec2_near(p.velocity, {0.5, 0.25}, EQ_TOL));
    EXPECT_TRUE(vec2_near(p.force_accumulator, {7.0, -4.0}, EQ_TOL));
    EXPECT_DOUBLE_EQ(p.mass, 2.5);
    EXPECT_DOUBLE_EQ(p.radius, 0.42);
    EXPECT_EQ(p.symbol, '@');
    EXPECT_TRUE(p.fixed);
}