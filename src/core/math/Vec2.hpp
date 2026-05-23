// src/core/math/Vec2.hpp
//
// Vecteur 2D pur, sans dependance SFML.
// Destine a remplacer sf::Vector2f a l'interieur du noyau de simulation.
//
// Interop SFML volontairement absente ici : la conversion vit dans
// render/SfmlInterop.hpp, pour eviter de polluer le core.
#pragma once

#include <cmath>

namespace core {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float xx, float yy) : x(xx), y(yy) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(float s) const { return {x / s, y / s}; }

    Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(float s) { x *= s; y *= s; return *this; }
    Vec2& operator/=(float s) { x /= s; y /= s; return *this; }

    constexpr bool operator==(Vec2 o) const { return x == o.x && y == o.y; }
    constexpr bool operator!=(Vec2 o) const { return !(*this == o); }

    float lengthSq() const { return x * x + y * y; }
    float length()   const { return std::sqrt(lengthSq()); }

    Vec2 normalized() const {
        const float l = length();
        return (l > 1e-6f) ? Vec2{x / l, y / l} : Vec2{0.f, 0.f};
    }
};

constexpr Vec2 operator*(float s, Vec2 v) { return v * s; }

inline float dot(Vec2 a, Vec2 b)      { return a.x * b.x + a.y * b.y; }
inline float cross(Vec2 a, Vec2 b)    { return a.x * b.y - a.y * b.x; }
inline float distance(Vec2 a, Vec2 b) { return (a - b).length(); }

} // namespace core
