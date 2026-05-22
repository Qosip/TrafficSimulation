#include "Lane.hpp"
#include <cmath>

Lane::Lane(const std::vector<sf::Vector2f>& waypoints) : points(waypoints), totalLength(0.f) {
    accumulatedDistances.push_back(0.f);
    // Calcule la distance cumulée (s) pour chaque point de la courbe
    for (size_t i = 1; i < points.size(); ++i) {
        sf::Vector2f diff = points[i] - points[i-1];
        float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        totalLength += dist;
        accumulatedDistances.push_back(totalLength);
    }
}

sf::Vector2f Lane::getPositionAt(float s) const {
    if (points.empty()) return {0.f, 0.f};
    if (s <= 0.f) return points.front();
    if (s >= totalLength) return points.back();

    // Interpolation linéaire entre les deux points encadrant s
    for (size_t i = 1; i < accumulatedDistances.size(); ++i) {
        if (s <= accumulatedDistances[i]) {
            float s0 = accumulatedDistances[i-1];
            float s1 = accumulatedDistances[i];
            float ratio = (s - s0) / (s1 - s0);
            sf::Vector2f p0 = points[i-1];
            sf::Vector2f p1 = points[i];
            return p0 + (p1 - p0) * ratio;
        }
    }
    return points.back();
}

float Lane::getHeadingAt(float s) const {
    if (points.size() < 2) return 0.f;

    // Lissage par "Look-ahead" : on regarde 5 pixels en avant
    float lookAhead = 5.0f;

    sf::Vector2f currentPos = getPositionAt(s);
    sf::Vector2f nextPos = getPositionAt(std::min(s + lookAhead, totalLength));
    sf::Vector2f diff = nextPos - currentPos;

    // Sécurité : si on est à la toute fin de la voie, on calcule l'angle en regardant en arrière
    if (diff.x == 0.f && diff.y == 0.f) {
        sf::Vector2f prevPos = getPositionAt(std::max(0.f, s - lookAhead));
        diff = currentPos - prevPos;
    }

    return std::atan2(diff.y, diff.x) * 180.f / 3.14159265f;
}