#include <gtest/gtest.h>
#include "engine/math.hpp"
#include <limits>
#include "test_utils.hpp"

using namespace test_utils;

// ---------- Integer Vec2 tests ----------
TEST(Vec2IntTest, DefaultConstructsToZero) {
    Vec2<int> v;
    EXPECT_EQ(v.x, 0);
    EXPECT_EQ(v.y, 0);
}

TEST(Vec2IntTest, ConstructAndAccess) {
    Vec2<int> v(3, -4);
    EXPECT_EQ(v.x, 3);
    EXPECT_EQ(v.y, -4);
}

TEST(Vec2IntTest, NormSquaredAndNorm) {
    Vec2<int> v(3, 4);
    EXPECT_EQ(v.norm_squared(), 25);
    EXPECT_NEAR(v.norm(), 5.0, TOL);
}

TEST(Vec2IntTest, AdditionSubtraction) {
    Vec2<int> a(2, -5);
    Vec2<int> b(-3, 7);

    Vec2<int> s = a + b;
    Vec2<int> d = a - b;

    EXPECT_EQ(s.x, -1);
    EXPECT_EQ(s.y, 2);

    EXPECT_EQ(d.x, 5);
    EXPECT_EQ(d.y, -12);

    Vec2<int> s2 = b + a;
    EXPECT_EQ(s, s2);
}

TEST(Vec2IntTest, ScalarMultiplyLeft) {
    Vec2<int> v(2, -3);
    int c = -4;
    Vec2<int> r = c * v;
    EXPECT_EQ(r.x, -8);
    EXPECT_EQ(r.y, 12);
}

TEST(Vec2IntTest, DivisionByScalar_TruncatesTowardZero) {
    Vec2<int> v(5, -5);
    int d = 2;
    Vec2<int> q = v / d; 
    EXPECT_EQ(q.x, 5 / 2);   
    EXPECT_EQ(q.y, -5 / 2);  
}

TEST(Vec2IntTest, PlusEqualsAccumulates) {
    Vec2<int> a(1, 2);
    Vec2<int> b(3, 4);
    Vec2<int>& ref = (a += b);
    EXPECT_EQ(&ref, &a); 
    EXPECT_EQ(a.x, 4);
    EXPECT_EQ(a.y, 6);

    Vec2<int> c(-5, 1);
    a += c;
    EXPECT_EQ(a.x, -1);
    EXPECT_EQ(a.y, 7);
}

TEST(Vec2IntTest, EqualityAndInequality) {
    Vec2<int> a(10, -2);
    Vec2<int> b(10, -2);
    Vec2<int> c(10, -3);
    Vec2<int> d(11, -2);

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);

    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);

    EXPECT_FALSE(a == d);
    EXPECT_TRUE(a != d);
}

TEST(Vec2IntTest, DotProduct) {
    Vec2<int> a(2, 3);
    Vec2<int> b(-4, 5);
    int dot = a * b;
    EXPECT_EQ(dot, 7);
}

// ---------- Double Vec2 tests ----------
TEST(Vec2DoubleTest, DefaultConstructsToZero) {
    Vec2<double> v;
    EXPECT_NEAR(v.x, 0.0, TOL);
    EXPECT_NEAR(v.y, 0.0, TOL);
}

TEST(Vec2DoubleTest, ConstructAndAccess) {
    Vec2<double> v(0.5, -0.25);
    EXPECT_NEAR(v.x, 0.5, TOL);
    EXPECT_NEAR(v.y, -0.25, TOL);
}

TEST(Vec2DoubleTest, NormSquaredAndNorm) {
    Vec2<double> v(0.6, 0.8);
    EXPECT_NEAR(v.norm_squared(), 1.0, TOL);
    EXPECT_NEAR(v.norm(), 1.0, TOL);
}

TEST(Vec2DoubleTest, AdditionSubtraction) {
    Vec2<double> a(1.25, -2.5);
    Vec2<double> b(-0.75, 4.0);

    Vec2<double> s = a + b;
    Vec2<double> d = a - b;

    EXPECT_TRUE(vec2_near(s, Vec2<double>(0.5, 1.5), TOL));
    EXPECT_TRUE(vec2_near(d, Vec2<double>(2.0, -6.5), TOL));

    Vec2<double> s2 = b + a;
    EXPECT_TRUE(vec2_near(s, s2, TOL));
}

TEST(Vec2DoubleTest, DistributivityWithLeftScalar) {
    Vec2<double> a(1.0, -3.0);
    Vec2<double> b(2.0, 5.0);
    double c = -2.25;

    Vec2<double> left = c * (a + b);
    Vec2<double> right = (c * a) + (c * b);
    EXPECT_TRUE(vec2_near(left, right, TOL));
}

TEST(Vec2DoubleTest, DivisionByScalar) {
    Vec2<double> v(1.2, -3.6);
    double d = -0.3;
    Vec2<double> q = v / d;
    EXPECT_TRUE(vec2_near(q, Vec2<double>(-4.0, 12.0), TOL));
}

TEST(Vec2DoubleTest, PlusEqualsAccumulates) {
    Vec2<double> a(0.1, 0.2);
    Vec2<double> b(0.3, 0.4);
    a += b;
    EXPECT_TRUE(vec2_near(a, Vec2<double>(0.4, 0.6), TOL));

    Vec2<double> c(-0.5, 1.0);
    a += c;
    EXPECT_TRUE(vec2_near(a, Vec2<double>(-0.1, 1.6), TOL));
}

TEST(Vec2DoubleTest, DotProduct) {
    Vec2<double> a(0.5, -1.25);
    Vec2<double> b(-0.2, 0.8);
    double dot = a * b;
    EXPECT_NEAR(dot, -1.1, TOL);
}

TEST(Vec2DoubleTest, NormalizeProducesUnitVectorAndPreservesDirection) {
    Vec2<double> v(3.0, 4.0);
    Vec2<double> u = v.normalize();
    EXPECT_NEAR(u.norm(), 1.0, TOL);
    EXPECT_NEAR(u.x / v.x, u.y / v.y, TOL); // both ~ 1/5
    EXPECT_TRUE(u.x * v.x >= 0.0);
    EXPECT_TRUE(u.y * v.y >= 0.0);
}
