#pragma once

struct Vec2 {
    double x;
    double y;
    Vec2(double x, double y) : x(x), y(y) {}
    Vec2() : Vec2(0, 0) {}
};

Vec2 operator+(Vec2 a, Vec2 b);
Vec2 operator*(double c, Vec2 v); 
Vec2& operator+=(Vec2& a, Vec2 b);