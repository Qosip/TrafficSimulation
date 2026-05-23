// src/core/math/Constants.hpp
//
// Constantes mathematiques de la simulation.
// Remplace les redefinitions locales de PI / DEG2RAD / RAD2DEG dans les
// modules Perception, Vehicle, World, Intersection...
//
// Aucune dependance externe. Utilisable depuis le core comme la couche render.
#pragma once

#include <numbers>

namespace core::math {

inline constexpr float PI      = std::numbers::pi_v<float>;
inline constexpr float TWO_PI  = 2.0f * PI;
inline constexpr float HALF_PI = 0.5f * PI;

inline constexpr float DEG2RAD = PI / 180.0f;
inline constexpr float RAD2DEG = 180.0f / PI;

// Normalise un angle (degres) dans [-180, 180].
inline float wrapDeg180(float deg) {
    while (deg < -180.0f) deg += 360.0f;
    while (deg >  180.0f) deg -= 360.0f;
    return deg;
}

// Normalise un angle (degres) dans [0, 360[.
inline float wrapDeg360(float deg) {
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

} // namespace core::math
