#include <gtest/gtest.h>
#include "engine/normal_force_generator.hpp"  
#include "engine/particle.hpp"                
#include "engine/math.hpp"                    

static constexpr double TOL = 1e-9;

TEST(NormalForceGenerator, RegularZeroNormals_NoChange) {
    Particle p{};
    p.force_accumulator = Vec2{3.0, 4.0};

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    EXPECT_NEAR(particles[0].force_accumulator.x, 3.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, 4.0, TOL);
}

TEST(NormalForceGenerator, RegularOneNormal_IntoSurface_ComponentCanceled_AxisAligned) {
    Particle p{};
    p.force_accumulator = Vec2{5.0, -7.0};
    // unit normal +y
    p.add_normal(Vec2{0.0, 1.0});

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    // y pushed into surface -> canceled; tangential x preserved
    EXPECT_NEAR(particles[0].force_accumulator.x, 5.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, 0.0, TOL);

    // dot(F', n) == 0
    const Vec2 Fp = particles[0].force_accumulator;
    const Vec2 n{0.0, 1.0};
    const double dot_after = Fp.x * n.x + Fp.y * n.y;
    EXPECT_NEAR(dot_after, 0.0, TOL);
}

TEST(NormalForceGenerator, RegularOneNormal_TangentOnly_Unchanged) {
    Particle p{};
    p.force_accumulator = Vec2{5.0, 0.0};
    // unit normal +y
    p.add_normal(Vec2{0.0, 1.0});

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    EXPECT_NEAR(particles[0].force_accumulator.x, 5.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, 0.0, TOL);
}

TEST(NormalForceGenerator, RegularOneNormal_ObliqueNormal_IntoSurface_ComponentCanceled) {
    Particle p{};
    p.force_accumulator = Vec2{2.0, -10.0};

    // oblique unit normal (1,1)/sqrt(2), hard-coded
    p.add_normal(Vec2{0.7071067811865475, 0.7071067811865475});

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    // Hard-coded math:
    // t = dot(F, n) = (2*0.70710678) + (-10*0.70710678) = (-8)/sqrt(2) ≈ -5.656854249
    // t*n ≈ (-4.0, -4.0)
    // F' = F - t*n ≈ (2, -10) - (-4, -4) = (6, -6)
    EXPECT_NEAR(particles[0].force_accumulator.x, 6.0, 1e-8);
    EXPECT_NEAR(particles[0].force_accumulator.y, -6.0, 1e-8);

    const Vec2 Fp = particles[0].force_accumulator;
    const Vec2 n{0.7071067811865475, 0.7071067811865475};
    const double dot_after = Fp.x * n.x + Fp.y * n.y;
    EXPECT_NEAR(dot_after, 0.0, 1e-8);
}

TEST(NormalForceGenerator, RegularOneNormal_NearTolerance_SmallIntoSurface_NoChange) {
    Particle p{};
    p.force_accumulator = Vec2{1.0, -1e-10};  // dot with +y is -1e-10
    p.add_normal(Vec2{0.0, 1.0});

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    EXPECT_NEAR(particles[0].force_accumulator.x, 1.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, -1e-10, TOL);
}

TEST(NormalForceGenerator, RegularZeroNormals_ZeroForce_NoChange) {
    Particle p{};
    p.force_accumulator = Vec2{0.0, 0.0};

    std::vector<Particle> particles{p};
    NormalForceGenerator gen;
    gen.generate(particles);

    EXPECT_NEAR(particles[0].force_accumulator.x, 0.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, 0.0, TOL);
}