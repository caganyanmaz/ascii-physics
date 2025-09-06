#include <gtest/gtest.h>
#include <vector>

// Adjust include paths as needed
#include "engine/ode_solver.hpp"
#include "engine/euler_ode_solver.hpp"

namespace { constexpr double TOL = 1e-12; }

TEST(EulerOdeSolver, IsOverInitiallyTrue_AndEndAppliesEulerUpdate) {
    std::vector<double> y{1.0, -2.0, 0.5};
    std::vector<double> dy{2.0, 3.0, -4.0};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = 0.1;

    EulerOdeSolver solver(y, dy, dt);

    // Contract: we don't enter the loop => is_over() is true
    EXPECT_TRUE(solver.is_over());

    // Single Euler update occurs on end()
    EXPECT_NO_THROW(solver.end());

    ASSERT_EQ(y.size(), y0.size());
    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y0[i] + dt * dy0[i], TOL);
    }
    ASSERT_EQ(dy.size(), dy0.size());
    for (size_t i = 0; i < dy.size(); ++i) {
        EXPECT_NEAR(dy[i], dy0[i], TOL);
    }
}

TEST(EulerOdeSolver, ZeroDt_LeavesYUnchanged_OnEnd) {
    std::vector<double> y{10.0, -7.0};
    std::vector<double> dy{1.234, -9.876};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = 0.0;

    EulerOdeSolver solver(y, dy, dt);
    EXPECT_TRUE(solver.is_over());

    EXPECT_NO_THROW(solver.end());

    ASSERT_EQ(y.size(), y0.size());
    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y0[i], TOL);
    }
    for (size_t i = 0; i < dy.size(); ++i) {
        EXPECT_NEAR(dy[i], dy0[i], TOL);
    }
}

TEST(EulerOdeSolver, NegativeDt_BackwardStep_OnEnd) {
    std::vector<double> y{1.0, 2.0};
    std::vector<double> dy{5.0, -1.0};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = -0.25;

    EulerOdeSolver solver(y, dy, dt);
    EXPECT_TRUE(solver.is_over());

    EXPECT_NO_THROW(solver.end());

    ASSERT_EQ(y.size(), y0.size());
    EXPECT_NEAR(y[0], y0[0] + dt * dy0[0], TOL);
    EXPECT_NEAR(y[1], y0[1] + dt * dy0[1], TOL);

    for (size_t i = 0; i < dy.size(); ++i) {
        EXPECT_NEAR(dy[i], dy0[i], TOL);
    }
}

TEST(EulerOdeSolver, DoesNotResizeOrReallocate) {
    std::vector<double> y(4, 1.0);
    std::vector<double> dy(4, 2.0);
    const double dt = 0.5;

    const auto* y_ptr_before  = y.data();
    const auto* dy_ptr_before = dy.data();
    const auto y_size_before  = y.size();
    const auto dy_size_before = dy.size();

    EulerOdeSolver solver(y, dy, dt);
    EXPECT_TRUE(solver.is_over());

    EXPECT_NO_THROW(solver.end());

    EXPECT_EQ(y.size(),  y_size_before);
    EXPECT_EQ(dy.size(), dy_size_before);
    EXPECT_EQ(y.data(),  y_ptr_before);
    EXPECT_EQ(dy.data(), dy_ptr_before);

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], 1.0 + dt * 2.0, TOL);
    }
}

TEST(EulerOdeSolver, SingleUse_NewSolverForAnotherUpdate) {
    // Show that a second Euler update requires a fresh solver instance.
    std::vector<double> y{0.0, 1.0};
    std::vector<double> dy{1.0, -2.0};
    const double dt = 0.25;

    // First update
    {
        const auto y0 = y;
        EulerOdeSolver s1(y, dy, dt);
        EXPECT_TRUE(s1.is_over());
        s1.end();
        EXPECT_NEAR(y[0], y0[0] + dt * dy[0], TOL);
        EXPECT_NEAR(y[1], y0[1] + dt * dy[1], TOL);
    }

    // Change dy and require a new solver for the second update
    dy = {3.0, 0.5};
    {
        const auto y1 = y;
        EulerOdeSolver s2(y, dy, dt);
        EXPECT_TRUE(s2.is_over());
        s2.end();
        EXPECT_NEAR(y[0], y1[0] + dt * dy[0], TOL);
        EXPECT_NEAR(y[1], y1[1] + dt * dy[1], TOL);
    }
}