#pragma once
#include <cmath>
#include <ostream>

struct Vec2 {
    double x;
    double y;
    Vec2(double x, double y) : x(x), y(y) {}
    Vec2() : Vec2(0, 0) {}
    inline double norm_squared()const { return x * x + y * y; }
    inline double norm()const { return sqrt(norm_squared()); }
    inline Vec2 normalize()const {
        double _norm = norm();
        return Vec2(x / _norm, y / _norm);
    }
};

Vec2 operator+(const Vec2& a, const Vec2& b);
Vec2 operator-(const Vec2& a, const Vec2& b);
Vec2 operator*(const double& c, const Vec2& v); 
Vec2 operator/(const Vec2& a, double b);
Vec2& operator+=(Vec2& a, const Vec2& b);
std::ostream &operator<<(std::ostream &os, const Vec2 &v);
double operator*(const Vec2& a, const Vec2& b);