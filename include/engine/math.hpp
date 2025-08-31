#pragma once
#include <cmath>
#include <ostream>

template<typename T>
struct Vec2 {
    T x;
    T y;
    Vec2(T x, T y) : x(x), y(y) {}
    Vec2() : Vec2(0, 0) {}
    inline T norm_squared()const { return x * x + y * y; }
    inline double norm()const { return sqrt(static_cast<double>(norm_squared())); }
    inline Vec2 normalize()const {
        T _norm = norm();
        return Vec2(x / _norm, y / _norm);
    }
};

template<typename T>
Vec2<T> operator+(const Vec2<T>& a, const Vec2<T>& b);
template<typename T>
Vec2<T> operator-(const Vec2<T>& a, const Vec2<T>& b);
template<typename T>
Vec2<T> operator*(const T& c, const Vec2<T>& v); 
template<typename T>
Vec2<T> operator/(const Vec2<T>& a, T b);
template<typename T>
Vec2<T>& operator+=(Vec2<T>& a, const Vec2<T>& b);
template<typename T>
bool operator==(const Vec2<T>& a, const Vec2<T>& b);
template<typename T>
bool operator!=(const Vec2<T>& a, const Vec2<T>& b);
template<typename T>
std::ostream &operator<<(std::ostream &os, const Vec2<T> &v);

template<typename T>
T operator*(const Vec2<T>& a, const Vec2<T>& b);

extern template struct Vec2<double>;
extern template struct Vec2<int>;