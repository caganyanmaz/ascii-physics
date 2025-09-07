#pragma once
#include <gtest/gtest.h>
#include "engine/math.hpp"

#include <cmath>

namespace test_utils {

    constexpr double TOL = 1e-12;

    inline ::testing::AssertionResult vec2_near(const Vec2<double>& a,
                                                const Vec2<double>& b,
                                                double tol = TOL) {
        if (std::abs(a.x - b.x) <= tol && std::abs(a.y - b.y) <= tol)
            return ::testing::AssertionSuccess();
        return ::testing::AssertionFailure()
            << "Vec2 mismatch: (" << a.x << ", " << a.y << ") vs ("
            << b.x << ", " << b.y << "), tol=" << tol;
    }
   
}