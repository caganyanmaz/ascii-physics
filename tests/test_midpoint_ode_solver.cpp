#include <gtest/gtest.h>
#include <vector>
#include <cmath>

// Adjust include paths as needed
#include "engine/ode_solver.hpp"
#include "engine/midpoint_ode_solver.hpp"

namespace { constexpr double TOL = 1e-12; }

// ------------------------------------------------------------
// Single step: constant derivative
// f(y) = c  => y1 = y0 + h * c
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SingleStep_ConstantDerivative) {
    std::vector<double> y{1.0, -2.0, 0.5};
    std::vector<double> dy(y.size());
    const std::vector<double> c{2.0, 3.0, -4.0};  // hard-coded

    // dy accurate before construction
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];

    const auto y0 = y;
    const double h = 0.1;

    MidpointOdeSolver ode(y, dy, h);
    while (!ode.is_over()) {
        ode.step();
        // recompute dy from *updated* y (constant here)
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];
    }
    ode.end();

    for (size_t i = 0; i < y.size(); ++i) {
        const double expected = y0[i] + h * c[i];
        EXPECT_NEAR(y[i], expected, TOL);
    }
}

// ------------------------------------------------------------
// Single step: h = 0 (no change), still run loop + end()
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SingleStep_ZeroStepSize) {
    std::vector<double> y{10.0, -7.0};
    std::vector<double> dy(y.size());

    // choose any f(y); h == 0 so result shouldn't change y
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = (i == 0 ? 1.234 : -9.876);

    const auto y0 = y;
    const double h = 0.0;

    MidpointOdeSolver ode(y, dy, h);
    while (!ode.is_over()) {
        ode.step();
        // recompute dy (arbitrary here)
        for (size_t i = 0; i < dy.size(); ++i) dy[i] *= 2.0;
    }
    ode.end();

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y0[i], TOL);
    }
}

// ------------------------------------------------------------
// Single step: negative step size with constant derivative
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SingleStep_NegativeStep_ConstantDerivative) {
    std::vector<double> y{1.0, 2.0};
    std::vector<double> dy(y.size());
    const std::vector<double> c{5.0, -1.0};

    // dy accurate before construction
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];

    const auto y0 = y;
    const double h = -0.25;

    MidpointOdeSolver ode(y, dy, h);
    while (!ode.is_over()) {
        ode.step();
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];
    }
    ode.end();

    EXPECT_NEAR(y[0], y0[0] + h * c[0], TOL);
    EXPECT_NEAR(y[1], y0[1] + h * c[1], TOL);
}

// ------------------------------------------------------------
// Single step: nonlinear state-dependent derivative
// f(y) = [ y0^2, sin(y1), exp(y2) ]
// midpoint: y1 = y0 + h * f( y0 + (h/2) * f(y0) )
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SingleStep_NonlinearStateDependent) {
    std::vector<double> y{1.2, -0.4, 0.1};
    std::vector<double> dy(y.size());
    const auto y_start = y;
    const double h = 0.2;

    // dy = f(y) at y0 (before construction)
    dy[0] = y[0] * y[0];
    dy[1] = std::sin(y[1]);
    dy[2] = std::exp(y[2]);

    // Pre-compute gold midpoint result
    const std::vector<double> f0{ y_start[0]*y_start[0],
                                  std::sin(y_start[1]),
                                  std::exp(y_start[2]) };
    const std::vector<double> y_mid{ y_start[0] + 0.5*h*f0[0],
                                     y_start[1] + 0.5*h*f0[1],
                                     y_start[2] + 0.5*h*f0[2] };
    const std::vector<double> f_mid{ y_mid[0]*y_mid[0],
                                     std::sin(y_mid[1]),
                                     std::exp(y_mid[2]) };

    MidpointOdeSolver ode(y, dy, h);
    while (!ode.is_over()) {
        ode.step();
        // overwrite dy = f(y) at updated y (undefined after end; we don't check)
        dy[0] = y[0] * y[0];
        dy[1] = std::sin(y[1]);
        dy[2] = std::exp(y[2]);
    }
    ode.end();

    EXPECT_NEAR(y[0], y_start[0] + h * f_mid[0], TOL);
    EXPECT_NEAR(y[1], y_start[1] + h * f_mid[1], TOL);
    EXPECT_NEAR(y[2], y_start[2] + h * f_mid[2], TOL);
}

// ------------------------------------------------------------
// Multi-step (new solver each time): constant derivative, varying h
// Still reduces to y = y0 + (sum h_k) * c
// ------------------------------------------------------------
TEST(MidpointOdeSolver, MultiStep_VaryingH_ConstantDerivative) {
    std::vector<double> y{0.0, 1.0, -2.0};
    std::vector<double> dy(y.size());
    const std::vector<double> c{1.0, -2.0, 0.5};
    const std::vector<double> hs{0.01, 0.02, 0.005, 0.03, 0.015, 0.04};
    const auto y0 = y;

    double Hsum = 0.0;
    for (double h : hs) {
        // dy accurate before construction
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];

        MidpointOdeSolver ode(y, dy, h);
        while (!ode.is_over()) {
            ode.step();
            // keep derivative constant
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = c[i];
        }
        ode.end();
        Hsum += h;
    }

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y0[i] + Hsum * c[i], TOL);
    }
}

// ------------------------------------------------------------
// Multi-step (new solver each time): linear state-dependent f
// f(y) = -k y
// Midpoint factor per step: (1 - k h + (k^2 h^2)/2)
// With varying h: y_N = (Π (1 - k h_n + (k^2 h_n^2)/2)) y_0
// ------------------------------------------------------------
TEST(MidpointOdeSolver, MultiStep_VaryingH_StateDependentLinear) {
    std::vector<double> y{1.0, -2.0, 0.5};
    std::vector<double> dy(y.size());
    const double k = 1.3;
    const std::vector<double> hs{0.01, 0.02, 0.005, 0.03, 0.015, 0.04, 0.02, 0.01};
    const auto y_init = y;

    double factor = 1.0;
    for (double h : hs) {
        // dy accurate before construction
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = -k * y[i];

        MidpointOdeSolver ode(y, dy, h);
        while (!ode.is_over()) {
            ode.step();
            // overwrite dy from updated y
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = -k * y[i];
        }
        ode.end();

        factor *= (1.0 - k * h + 0.5 * k * k * h * h);
    }

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], y_init[i] * factor, 1e-10);
    }
}

// ------------------------------------------------------------
// Multi-step: harmonic oscillator via midpoint (RK2)
// z = [x, v], f(z) = [v, -ω^2 x]
// ------------------------------------------------------------
TEST(MidpointOdeSolver, MultiStep_HarmonicOscillator_MatchesExplicitMidpointRecurrence) {
    const double omega = 2.0;
    const double h = 0.01;
    const int N = 400;

    // z = [x, v]
    std::vector<double> y{1.0, 0.0};
    std::vector<double> dy(2);

    // gold sequence using the same midpoint recurrences
    double gx = y[0];
    double gv = y[1];

    for (int n = 0; n < N; ++n) {
        // dy accurate before construction: f(y) = [v, -ω^2 x]
        dy[0] = y[1];
        dy[1] = -omega * omega * y[0];

        MidpointOdeSolver ode(y, dy, h);
        while (!ode.is_over()) {
            ode.step();
            // overwrite dy from updated y
            dy[0] = y[1];
            dy[1] = -omega * omega * y[0];
        }
        ode.end();

        // gold
        const double mid_x = gx + 0.5 * h * gv;
        const double mid_v = gv - 0.5 * h * omega * omega * gx;
        const double fmx   = mid_v;
        const double fmv   = -omega * omega * mid_x;
        gx += h * fmx;
        gv += h * fmv;
    }

    EXPECT_NEAR(y[0], gx, 1e-12);
    EXPECT_NEAR(y[1], gv, 1e-12);
}

// ------------------------------------------------------------
// In-place behavior: solver must not resize/reallocate y or dy
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SingleStep_DoesNotResizeOrReallocate) {
    std::vector<double> y(4, 1.0);
    std::vector<double> dy(4);

    // dy accurate before construction (constant 2.0)
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = 2.0;

    const double h = 0.5;

    const auto* y_ptr_before  = y.data();
    const auto* dy_ptr_before = dy.data();
    const auto y_size_before  = y.size();
    const auto dy_size_before = dy.size();

    MidpointOdeSolver ode(y, dy, h);
    while (!ode.is_over()) {
        ode.step();
        for (size_t i = 0; i < dy.size(); ++i) dy[i] = 2.0;
    }
    ode.end();

    EXPECT_EQ(y.size(),  y_size_before);
    EXPECT_EQ(dy.size(), dy_size_before);
    EXPECT_EQ(y.data(),  y_ptr_before);
    EXPECT_EQ(dy.data(), dy_ptr_before);

    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_NEAR(y[i], 1.0 + h * 2.0, TOL);
    }
}

// ------------------------------------------------------------
// Single-use demonstration: two sequential updates require NEW solvers
// with different f and h
// ------------------------------------------------------------
TEST(MidpointOdeSolver, SequentialSteps_RequireNewSolverEachStep) {
    std::vector<double> y{0.0, 1.0};
    std::vector<double> dy(2);

    // Step 1: f = a, h1
    const std::vector<double> a{1.0, -2.0};
    const double h1 = 0.25;

    // dy accurate before construction
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = a[i];

    {
        const auto y_before = y;
        MidpointOdeSolver ode(y, dy, h1);
        while (!ode.is_over()) {
            ode.step();
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = a[i];
        }
        ode.end();

        // constant derivative → midpoint reduces to y + h * a
        EXPECT_NEAR(y[0], y_before[0] + h1 * a[0], TOL);
        EXPECT_NEAR(y[1], y_before[1] + h1 * a[1], TOL);
    }

    // Step 2: f = b, h2 (new solver)
    const std::vector<double> b{3.0, 0.5};
    const double h2 = 0.1;

    // dy accurate before construction
    for (size_t i = 0; i < dy.size(); ++i) dy[i] = b[i];

    {
        const auto y_before = y;
        MidpointOdeSolver ode(y, dy, h2);
        while (!ode.is_over()) {
            ode.step();
            for (size_t i = 0; i < dy.size(); ++i) dy[i] = b[i];
        }
        ode.end();

        EXPECT_NEAR(y[0], y_before[0] + h2 * b[0], TOL);
        EXPECT_NEAR(y[1], y_before[1] + h2 * b[1], TOL);
    }
}