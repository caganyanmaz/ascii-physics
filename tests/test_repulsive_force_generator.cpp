#include <gtest/gtest.h>
#include <vector>
#include "engine/force_generator.hpp"
#include "engine/repulsive_force_generator.hpp"
#include "engine/math.hpp"
#include "test_utils.hpp"

using test_utils::vec2_near;
using test_utils::TOL;

// Helper to make a particle with minimal noise.
static Particle make_particle(Vec2<double> pos,
                              Vec2<double> vel = {0.0, 0.0},
                              double mass = 1.0,
                              double radius = 0.01,
                              bool fixed = false,
                              char symbol='.') {
    Particle p;
    p.position = pos;
    p.velocity = vel;
    p.force_accumulator = {0.0, 0.0};
    p.mass = mass;
    p.radius = radius;
    p.fixed = fixed;
    p.symbol = symbol;
    p.normal_count = 0;
    p.normals = {Vec2<double>{0,0}, Vec2<double>{0,0}};
    return p;
}

// For these tests, place particles at unit distances to avoid ambiguity in direction vs 1/r^2.
// With |r| = 1, the vector form q_i*q_j * r_hat / r^2 simplifies to q_i*q_j * r_hat.
TEST(RepulsiveForceGenerator, TwoParticles_SymmetricOpposite_ForcesAtUnitDistance) {
    // p0 at (0,0), p1 at (1,0) => unit distance along +x
    // repulsiveness q0=2, q1=3 => product 6
    // Force on p0 is away from p1 -> (-6, 0), on p1 is away from p0 -> (+6, 0)
    std::vector<Particle> particles;
    particles.push_back(make_particle({0.0, 0.0}));
    particles.push_back(make_particle({1.0, 0.0}));

    // (index, repulsiveness)
    RepulsiveForceGenerator gen({{0, 2.0}, {1, 3.0}});
    gen.generate(particles);

    ASSERT_TRUE(vec2_near(particles[0].force_accumulator, {-6.0, 0.0}));
    ASSERT_TRUE(vec2_near(particles[1].force_accumulator, {+6.0, 0.0}));

    // No other fields changed
    EXPECT_TRUE(vec2_near(particles[0].position, {0.0, 0.0}));
    EXPECT_TRUE(vec2_near(particles[1].position, {1.0, 0.0}));
    EXPECT_TRUE(vec2_near(particles[0].velocity, {0.0, 0.0}));
    EXPECT_TRUE(vec2_near(particles[1].velocity, {0.0, 0.0}));
}

TEST(RepulsiveForceGenerator, Superposition_ThreeParticles_AllUnitDistancesToCenter) {
    // p0 at origin, p1 at (1,0), p2 at (0,1). |p0-p1|=|p0-p2|=1
    // q0=1.5, q1=2, q2=4
    // Force on p0 = from p1: -q0*q1 xhat = -3.0 xhat, from p2: -q0*q2 yhat = -6.0 yhat
    // => F0 = (-3, -6)
    // Force on p1 gets contribution from p0 ( +3,0 ) and from p2 ( direction (1,-1) with |r|=sqrt(2) so magnitude q1*q2 / r^2 = 8/2=4 along r_hat=(1,-1)/√2 => vector 4*(1/√2, -1/√2) = (2√2, -2√2) )
    // Force on p2 similarly.
    std::vector<Particle> particles;
    particles.push_back(make_particle({0.0, 0.0})); // p0
    particles.push_back(make_particle({1.0, 0.0})); // p1
    particles.push_back(make_particle({0.0, 1.0})); // p2

    RepulsiveForceGenerator gen({{0, 1.5}, {1, 2.0}, {2, 4.0}});
    gen.generate(particles);

    // p0 expectation: only unit-distance contributions -> clean integers
    ASSERT_TRUE(vec2_near(particles[0].force_accumulator, {-3.0, -6.0}));

    // p1: from p0 is (+3, 0). From p2: see comment above => (2√2, -2√2).
    Vec2<double> F1_expected{3.0 + 2.0 * std::sqrt(2.0), 0.0 - 2.0 * std::sqrt(2.0)};
    ASSERT_TRUE(vec2_near(particles[1].force_accumulator, F1_expected, 1e-12));

    // p2: from p0 is (0, +6). From p1: vector is (-2√2, +2√2).
    Vec2<double> F2_expected{0.0 - 2.0 * std::sqrt(2.0), 6.0 + 2.0 * std::sqrt(2.0)};
    ASSERT_TRUE(vec2_near(particles[2].force_accumulator, F2_expected, 1e-12));
}

TEST(RepulsiveForceGenerator, Accumulates_WithoutOverwriting) {
    // Start with pre-existing accumulators; generator should add to them.
    std::vector<Particle> particles;
    particles.push_back(make_particle({0.0, 0.0}));
    particles.push_back(make_particle({1.0, 0.0}));

    particles[0].force_accumulator = {10.0, 1.0};
    particles[1].force_accumulator = {-5.0, -2.0};

    RepulsiveForceGenerator gen({{0, 1.0}, {1, 1.0}});
    gen.generate(particles); // unit distance => product 1, so (-1,0) and (+1,0) added

    ASSERT_TRUE(vec2_near(particles[0].force_accumulator, {9.0, 1.0}));
    ASSERT_TRUE(vec2_near(particles[1].force_accumulator, {-4.0, -2.0}));
}

TEST(RepulsiveForceGenerator, NonAffectedParticlesRemainUntouched) {
    // Include a third particle not in the affected list.
    std::vector<Particle> particles;
    particles.push_back(make_particle({0.0, 0.0}));
    particles.push_back(make_particle({1.0, 0.0}));
    particles.push_back(make_particle({0.0, 2.0})); // p2 untouched

    particles[2].force_accumulator = {7.0, -3.0};

    RepulsiveForceGenerator gen({{0, 2.0}, {1, 3.0}}); // p2 not included
    gen.generate(particles);

    // p2 accumulator must not change
    ASSERT_TRUE(vec2_near(particles[2].force_accumulator, {7.0, -3.0}));

    // spot-check p0/p1 as well (same as first test, since p2 is not in the set)
    ASSERT_TRUE(vec2_near(particles[0].force_accumulator, {-6.0, 0.0}));
    ASSERT_TRUE(vec2_near(particles[1].force_accumulator, {+6.0, 0.0}));
}

TEST(RepulsiveForceGenerator, DoesNotModifyOtherFields) {
    std::vector<Particle> particles;
    particles.push_back(make_particle({0.0, 0.0}, {1.0, 2.0}, /*mass=*/3.0, /*radius=*/0.4, /*fixed=*/true, /*symbol=*/'#'));
    particles.push_back(make_particle({1.0, 0.0}, {-3.0, 4.0}, /*mass=*/5.0, /*radius=*/0.2, /*fixed=*/false, /*symbol=*/'@'));

    // Capture originals
    auto p0 = particles[0];
    auto p1 = particles[1];

    RepulsiveForceGenerator gen({{0, 2.0}, {1, 3.0}});
    gen.generate(particles);

    // Only force_accumulator may change
    EXPECT_TRUE(vec2_near(particles[0].position,  p0.position));
    EXPECT_TRUE(vec2_near(particles[1].position,  p1.position));
    EXPECT_TRUE(vec2_near(particles[0].velocity,  p0.velocity));
    EXPECT_TRUE(vec2_near(particles[1].velocity,  p1.velocity));
    EXPECT_EQ(particles[0].mass,   p0.mass);
    EXPECT_EQ(particles[1].mass,   p1.mass);
    EXPECT_EQ(particles[0].radius, p0.radius);
    EXPECT_EQ(particles[1].radius, p1.radius);
    EXPECT_EQ(particles[0].fixed,  p0.fixed);
    EXPECT_EQ(particles[1].fixed,  p1.fixed);
    EXPECT_EQ(particles[0].symbol, p0.symbol);
    EXPECT_EQ(particles[1].symbol, p1.symbol);
    EXPECT_EQ(particles[0].normal_count, p0.normal_count);
    EXPECT_EQ(particles[1].normal_count, p1.normal_count);
    // normals array unchanged
    EXPECT_TRUE(vec2_near(particles[0].normals[0], p0.normals[0]));
    EXPECT_TRUE(vec2_near(particles[0].normals[1], p0.normals[1]));
    EXPECT_TRUE(vec2_near(particles[1].normals[0], p1.normals[0]));
    EXPECT_TRUE(vec2_near(particles[1].normals[1], p1.normals[1]));
}