#include <gtest/gtest.h>
#include <vector>
#include <cmath>

// Adjust include paths as needed
#include "engine/ode_solver.hpp"
#include "engine/euler_ode_solver.hpp"

namespace { constexpr double TOL = 1e-12; }

// ------------------------------------------------------------
// Single step: constant dy
// y' = y + dt * dy  (dy constant)
// ------------------------------------------------------------
TEST(EulerOdeSolver, SingleStep_ConstantDerivative) {
    std::vector<double> y{1.0, -2.0, 0.5};
    std::vector<double> dy{2.0, 3.0, -4.0};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = 0.1;

    EulerOdeSolver ode(y, dy, dt);
    while (!ode.is_over()) {
        ode.step();
        // Recompute dy from new y (constant here): overwrite explicitly
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = dy0[i];
    }
    ode.end();

    ASSERT_EQ(y.size(), y0.size());
    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y0[i] + dt * dy0[i], TOL);
    }
    for (size_t i = 0; i < dy.size(); ++i) {
        EXPECT_NEAR(dy[i], dy0[i], TOL);
    }
}

// ------------------------------------------------------------
// Single step: dt = 0 (no change), but still run loop + end()
// ------------------------------------------------------------
TEST(EulerOdeSolver, SingleStep_ZeroDt) {
    std::vector<double> y{10.0, -7.0};
    std::vector<double> dy{1.234, -9.876};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = 0.0;

    EulerOdeSolver ode(y, dy, dt);
    while (!ode.is_over()) {
        ode.step();
        // Arbitrary recompute; result shouldn't matter because dt=0
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = dy0[i] * 2.0;
    }
    ode.end();

    for (size_t i = 0; i < y.size(); ++i) EXPECT_NEAR(y[i], y0[i], TOL);
}

// ------------------------------------------------------------
// Single step: negative dt (backwards step)
// ------------------------------------------------------------
TEST(EulerOdeSolver, SingleStep_NegativeDt) {
    std::vector<double> y{1.0, 2.0};
    std::vector<double> dy{5.0, -1.0};
    const auto y0 = y;
    const auto dy0 = dy;
    const double dt = -0.25;

    EulerOdeSolver ode(y, dy, dt);
    while (!ode.is_over()) {
        ode.step();
        // Keep derivative constant
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = dy0[i];
    }
    ode.end();

    ASSERT_EQ(y.size(), y0.size());
    EXPECT_NEAR(y[0], y0[0] + dt * dy0[0], TOL);
    EXPECT_NEAR(y[1], y0[1] + dt * dy0[1], TOL);
    for (size_t i = 0; i < dy.size(); ++i) EXPECT_NEAR(dy[i], dy0[i], TOL);
}

// ------------------------------------------------------------
// Single step: nonlinear, state-dependent dy
// dy(y) = [ y0^2, sin(y1), exp(y2) ]
// Expected: y1 = y0 + dt * dy(y0)
// Also verify that after the loop dy equals dy(y1) (because we overwrite)
// ------------------------------------------------------------
TEST(EulerOdeSolver, SingleStep_NonlinearStateDependentDerivative) {
    std::vector<double> y{1.2, -0.4, 0.1};
    // dy at y0 (for expected y)
    std::vector<double> dy{
        y[0] * y[0],
        std::sin(y[1]),
        std::exp(y[2])
    };
    const auto y0 = y;
    const auto dy_at_y0 = dy;
    const double dt = 0.2;


    EulerOdeSolver ode(y, dy, dt);
    while (!ode.is_over()) {
        ode.step();
        // Overwrite dy based on *updated* y
        dy[0] = y[0] * y[0];
        dy[1] = std::sin(y[1]);
        dy[2] = std::exp(y[2]);
    }
    ode.end();

    // Expected y after single Euler update using dy(y0)
    EXPECT_NEAR(y[0], y0[0] + dt * dy_at_y0[0], 1e-12);
    EXPECT_NEAR(y[1], y0[1] + dt * dy_at_y0[1], 1e-12);
    EXPECT_NEAR(y[2], y0[2] + dt * dy_at_y0[2], 1e-12);
}

// ------------------------------------------------------------
// Multi-step (new solver each step): constant derivative
// y' = c  => y_N = y_0 + (sum dt_k) * c   (vary dt per step)
// ------------------------------------------------------------
TEST(EulerOdeSolver, MultiStep_VaryingDt_ConstantDerivative) {
    std::vector<double> y{0.0, 1.0, -2.0};
    std::vector<double> dy(3);
    const std::vector<double> c{1.0, -2.0, 0.5}; // constant derivative per component

    // Use a varying dt schedule
    const std::vector<double> dts{0.01, 0.02, 0.005, 0.03, 0.015, 0.04};
    double total_dt = 0.0;

    // Initialize dy for step 0
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];

    const auto y_start = y;

    for (double dt : dts) {
        EulerOdeSolver ode(y, dy, dt);
        while (!ode.is_over()) {
            ode.step();
            // Overwrite dy = c (constant)
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];
        }
        ode.end();
        total_dt += dt;
        // do NOT reuse 'ode'
    }

    for (size_t i = 0; i < y.size(); ++i) {
        const double expected = y_start[i] + total_dt * c[i];
        EXPECT_NEAR(y[i], expected, 1e-12);
    }
}

// ------------------------------------------------------------
// Multi-step (new solver each step): linear state-dependent dy
// y' = -k y. Euler => y_{n+1} = (1 - k*dt_n) y_n
// With varying dt, y_N = (Π_n (1 - k*dt_n)) * y_0
// ------------------------------------------------------------
TEST(EulerOdeSolver, MultiStep_VaryingDt_StateDependentDerivative) {
    std::vector<double> y{1.0, -2.0, 0.5};
    std::vector<double> dy(y.size());
    const double k = 1.3;

    const std::vector<double> dts{0.01, 0.02, 0.005, 0.03, 0.015, 0.04, 0.02, 0.01};
    double factor = 1.0;

    // initial dy = -k*y
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = -k * y[i];
    const auto y0 = y;

    for (double dt : dts) {
        EulerOdeSolver ode(y, dy, dt);
        while (!ode.is_over()) {
            ode.step();
            // Overwrite dy from the updated y
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = -k * y[i];
        }
        ode.end();
        factor *= (1.0 - k * dt);
        // do NOT reuse 'ode'
    }

    for (size_t i = 0; i < y.size(); ++i) {
        const double expected = y0[i] * factor;
        EXPECT_NEAR(y[i], expected, 0.1); // looser for compounded FP error
    }
}

// ------------------------------------------------------------
// In-place behavior: solver must not resize/reallocate y or dy
// ------------------------------------------------------------
TEST(EulerOdeSolver, SingleStep_DoesNotResizeOrReallocate) {
    std::vector<double> y(4, 1.0);
    std::vector<double> dy(4, 2.0);
    const double dt = 0.5;

    const auto* y_ptr_before  = y.data();
    const auto* dy_ptr_before = dy.data();
    const auto y_size_before  = y.size();
    const auto dy_size_before = dy.size();

    EulerOdeSolver ode(y, dy, dt);
    while (!ode.is_over()) {
        ode.step();
        // Overwrite dy explicitly (constant here)
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = 2.0;
    }
    ode.end();

    EXPECT_EQ(y.size(),  y_size_before);
    EXPECT_EQ(dy.size(), dy_size_before);
    EXPECT_EQ(y.data(),  y_ptr_before);
    EXPECT_EQ(dy.data(), dy_ptr_before);

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], 1.0 + dt * 2.0, TOL);
    }
}

// ------------------------------------------------------------
// Demonstrate single-use: two sequential updates require NEW solvers.
// Step 1 with dy = a, Step 2 with dy = b (different), each with own dt.
// ------------------------------------------------------------
TEST(EulerOdeSolver, SequentialSteps_RequireNewSolverEachStep) {
    std::vector<double> y{0.0, 1.0};
    std::vector<double> dy(2);

    // Step 1: dy = a, dt1
    const std::vector<double> a{1.0, -2.0};
    const double dt1 = 0.25;
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = a[i];

    {
        const auto y_before = y;
        EulerOdeSolver ode(y, dy, dt1);
        while (!ode.is_over()) {
            ode.step();
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = a[i];
        }
        ode.end();

        EXPECT_NEAR(y[0], y_before[0] + dt1 * a[0], TOL);
        EXPECT_NEAR(y[1], y_before[1] + dt1 * a[1], TOL);
    }

    // Step 2: dy = b, dt2 (NEW solver)
    const std::vector<double> b{3.0, 0.5};
    const double dt2 = 0.1;
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = b[i];

    {
        const auto y_before = y;
        EulerOdeSolver ode(y, dy, dt2);
        while (!ode.is_over()) {
            ode.step();
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = b[i];
        }
        ode.end();

        EXPECT_NEAR(y[0], y_before[0] + dt2 * b[0], TOL);
        EXPECT_NEAR(y[1], y_before[1] + dt2 * b[1], TOL);
    }
}