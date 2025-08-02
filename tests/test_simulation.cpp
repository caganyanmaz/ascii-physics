#include <gtest/gtest.h>
#include "engine/simulation.hpp"

TEST(SimulationTest, DummyStepTest) {
    Simulation sim;
    sim.step(0.01);
    SUCCEED();
}

