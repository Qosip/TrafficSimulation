// src/Vehicle.hpp
#pragma once
#include "IAgent.hpp"
#include <vector>

class Vehicle : public IAgent {
protected:
    sf::Vector2f position;
    sf::Vector2f velocity;

    float maxSpeed;
    float maxAcceleration;
    sf::RectangleShape shape;

    std::vector<sf::Vector2f> densePath;
    size_t currentPathIndex;
    bool hasFinishedPath;
    float tileSize;

    float currentAngle;
    float currentSpeed;
    bool isHeadingInitialized;

    // Debug
    bool isSelected = false;
public:
    Vehicle(float startX, float startY, float tSize = 50.f);
    virtual ~Vehicle() = default;

    void update(float dt) override;
    void draw(sf::RenderWindow& window) override;
    sf::Vector2f getPosition() const override;

    void setPath(const std::vector<sf::Vector2i>& newPath);
    bool isFinished() const;

    // Debug
    bool contains(sf::Vector2f point) const override;
    void setSelected(bool selected) override;
    void drawDebug(sf::RenderWindow& window) override;
};