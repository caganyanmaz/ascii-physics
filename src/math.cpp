#include "engine/math.hpp"

Vec2 operator+(Vec2 a, Vec2 b) {
    Vec2 c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

Vec2 operator*(double c, Vec2 v) {
    Vec2 res;
    res.x = c * v.x;
    res.y = c * v.y;
    return res;
}

Vec2& operator+=(Vec2& a, Vec2 b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}