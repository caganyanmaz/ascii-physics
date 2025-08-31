#include "engine/math.hpp"

template<typename T>
Vec2<T> operator+(const Vec2<T>& a, const Vec2<T>& b) {
    Vec2<T> c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

template<typename T>
Vec2<T> operator*(const T& c, const Vec2<T>& v) {
    Vec2<T> res;
    res.x = c * v.x;
    res.y = c * v.y;
    return res;
}


template<typename T>
Vec2<T>& operator+=(Vec2<T>& a, const Vec2<T>& b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}

template<typename T>
Vec2<T> operator/(const Vec2<T>& a, T b) {
    return Vec2<T>(a.x / b, a.y / b);
}

template<typename T>
T operator*(const Vec2<T>& a, const Vec2<T>& b) {
    return a.x * b.x + a.y * b.y;
}

template<typename T>
Vec2<T> operator-(const Vec2<T>& a, const Vec2<T>& b) {
    return Vec2<T>(a.x - b.x, a.y - b.y);
}

template<typename T>
bool operator==(const Vec2<T>& a, const Vec2<T>& b) {
    return a.x == b.x && a.y == b.y;
}

template<typename T>
bool operator!=(const Vec2<T>& a, const Vec2<T>& b) {
    return !(a == b);
}

template<typename T>
std::ostream &operator<<(std::ostream &os, Vec2<T> const &v) {
    return os << "(" <<v.x << ", " << v.y << ")";
}

template struct Vec2<double>;
template Vec2<double> operator+(const Vec2<double>& a, const Vec2<double>& b); 
template Vec2<double> operator*(const double& c, const Vec2<double>& v); 
template Vec2<double>& operator+=(Vec2<double>& a, const Vec2<double>& b); 
template Vec2<double> operator/(const Vec2<double>& a, double b); 
template double operator*(const Vec2<double>& a, const Vec2<double>& b); 
template Vec2<double> operator-(const Vec2<double>& a, const Vec2<double>& b); 
template bool operator==(const Vec2<double>& a, const Vec2<double>& b);
template bool operator!=(const Vec2<double>& a, const Vec2<double>& b);
template std::ostream &operator<<(std::ostream &os, Vec2<double> const &v);

template struct Vec2<int>;
template Vec2<int> operator+(const Vec2<int>& a, const Vec2<int>& b); 
template Vec2<int> operator*(const int& c, const Vec2<int>& v); 
template Vec2<int>& operator+=(Vec2<int>& a, const Vec2<int>& b); 
template Vec2<int> operator/(const Vec2<int>& a, int b); 
template int operator*(const Vec2<int>& a, const Vec2<int>& b); 
template Vec2<int> operator-(const Vec2<int>& a, const Vec2<int>& b); 
template bool operator==(const Vec2<int>& a, const Vec2<int>& b);
template bool operator!=(const Vec2<int>& a, const Vec2<int>& b);
template std::ostream &operator<<(std::ostream &os, Vec2<int> const &v);