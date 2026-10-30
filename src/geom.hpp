// mech — M1 walking skeleton of a mechanism language
// geom.hpp: minimal 2-D vector algebra used by the model and the solver
#pragma once
#include <cmath>

namespace mech {

struct Vec2 {
    double x = 0.0, y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(double s, Vec2 a) { return a * s; }
inline Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { a.x += b.x; a.y += b.y; return a; }

inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double crossz(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// rotate v by 90 degrees counter-clockwise
inline Vec2 perp(Vec2 v) { return {-v.y, v.x}; }
// rotate v by angle th (radians)
inline Vec2 rotate(Vec2 v, double th) {
    double c = std::cos(th), s = std::sin(th);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}
inline double vlen(Vec2 v) { return std::hypot(v.x, v.y); }
inline Vec2 normalize(Vec2 v) {
    double l = vlen(v);
    return l > 1e-12 ? Vec2{v.x / l, v.y / l} : Vec2{0.0, 0.0};
}

const double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// smallest signed difference between two angles (radians)
inline double angDiff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
}

} // namespace mech
