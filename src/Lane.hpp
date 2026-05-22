#pragma once
#include <SFML/Graphics.hpp>
#include <vector>

class Lane {
private:
    std::vector<sf::Vector2f> points;
    std::vector<float> accumulatedDistances;
    float totalLength;

public:
    Lane(const std::vector<sf::Vector2f>& waypoints);

    // Évalue la position (x, y) pour une distance s parcourue
    sf::Vector2f getPositionAt(float s) const;

    // Évalue l'angle (heading) en degrés à la distance s
    float getHeadingAt(float s) const;

    float getLength() const { return totalLength; }
    const std::vector<sf::Vector2f>& getPoints() const { return points; }
};