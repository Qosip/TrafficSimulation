// src/Lane.hpp
//
// Trajectoire 1D parametrique (curvilineaire).
// Etape 4 du refactor : sf::Vector2f remplace par core::Vec2.
#pragma once

#include <vector>

#include "core/math/Vec2.hpp"

class Lane {
private:
    std::vector<core::Vec2> points;
    std::vector<float>      accumulatedDistances;
    float                   totalLength;

public:
    explicit Lane(const std::vector<core::Vec2>& waypoints);

    // Position (x, y) le long de la courbe a la distance curvilineaire s.
    core::Vec2 getPositionAt(float s) const;

    // Heading (en degres) a la distance s.
    float getHeadingAt(float s) const;

    float getLength() const { return totalLength; }
    const std::vector<core::Vec2>& getPoints() const { return points; }
};
