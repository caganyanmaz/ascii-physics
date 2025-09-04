#include <gtest/gtest.h>
#include "engine/particle.hpp"
#include "engine/wind_generator.hpp"
#include "tests/test_utils.hpp"
#include <vector>

using test_utils;

TEST(WindGeneratorTest, zero_wind_yields_no_force) {
    Particle p;
    p.force_accumulator = Vec2<double>(0.3, -0.4);

    WindGenerator wind(Vec2<double>(0.0, 0.0));
    std::vector<Particle> particles{p};
    wind.generate(particles);

    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.3, -0.4)));
}

TEST(WindGeneratorTest, adds_wind_vector_to_force_accumulator) {
    Particle p;
    p.force_accumulator = Vec2<double>(0.0, 0.0);

    WindGenerator wind(Vec2<double>(1.2, -0.5));
    std::vector<Particle> particles{p};
    wind.generate(particles);

    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(1.2, -0.5)));
}

TEST(WindGeneratorTest, accumulates_on_existing_force) {
    Particle p;
    p.force_accumulator = Vec2<double>(2.0, -1.0);

    WindGenerator wind(Vec2<double>(-0.3, 0.7));
    std::vector<Particle> particles{p};
    wind.generate(particles);

    // (2.0, -1.0) + (-0.3, 0.7) = (1.7, -0.3)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(1.7, -0.3)));
}

TEST(WindGeneratorTest, does_not_modify_other_particle_state) {
    Particle p;
    p.position = Vec2<double>(10.0, -7.0);
    p.velocity = Vec2<double>(1.5, 2.5);
    p.mass = 3.14;
    p.radius = 0.02;
    p.symbol = '@';
    p.fixed = true;
    p.force_accumulator = Vec2<double>(0.0, 0.0);

    WindGenerator wind(Vec2<double>(0.5, -0.25));
    std::vector<Particle> particles{p};
    wind.generate(particles);

    // only force_accumulator changes
    EXPECT_TRUE(vec2_near(particles[0].position, p.position));
    EXPECT_TRUE(vec2_near(particles[0].velocity, p.velocity));
    EXPECT_DOUBLE_EQ(particles[0].mass, p.mass);
    EXPECT_DOUBLE_EQ(particles[0].radius, p.radius);
    EXPECT_EQ(particles[0].symbol, p.symbol);
    EXPECT_EQ(particles[0].fixed, p.fixed);
}

TEST(WindGeneratorTest, handles_multiple_particles_independently) {
    Particle p1; p1.force_accumulator = Vec2<double>(0.0, 0.0);
    Particle p2; p2.force_accumulator = Vec2<double>(1.0, -1.0);
    Particle p3; p3.force_accumulator = Vec2<double>(-0.2, 0.3);

    WindGenerator wind(Vec2<double>(0.5, -0.25));
    std::vector<Particle> particles{p1, p2, p3};
    wind.generate(particles);

    // p1: (0.0, 0.0) + (0.5, -0.25) = (0.5, -0.25)
    EXPECT_TRUE(vec2_near(particles[0].force_accumulator, Vec2<double>(0.5, -0.25)));
    // p2: (1.0, -1.0) + (0.5, -0.25) = (1.5, -1.25)
    EXPECT_TRUE(vec2_near(particles[1].force_accumulator, Vec2<double>(1.5, -1.25)));
    // p3: (-0.2, 0.3) + (0.5, -0.25) = (0.3, 0.05)
    EXPECT_TRUE(vec2_near(particles[2].force_accumulator, Vec2<double>(0.3, 0.05)));
}
