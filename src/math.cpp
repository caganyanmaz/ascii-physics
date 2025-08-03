#include "engine/math.hpp"

Vec2 operator+(const Vec2& a, const Vec2& b) {
    Vec2 c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}


Vec2 operator*(const double& c, const Vec2& v) {
    Vec2 res;
    res.x = c * v.x;
    res.y = c * v.y;
    return res;
}

Vec2& operator+=(Vec2& a, const Vec2& b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}

Vec2 operator/(const Vec2& a, double b) {
    return Vec2(a.x / b, a.y / b);
}

double operator*(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

Vec2 operator-(const Vec2& a, const Vec2& b) {
    return Vec2(a.x - b.x, a.y - b.y);
}

std::ostream &operator<<(std::ostream &os, Vec2 const &v) {
    return os << "(" <<v.x << ", " << v.y << ")";
}