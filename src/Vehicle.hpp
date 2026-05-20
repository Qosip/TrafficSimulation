// src/Vehicle.hpp
#pragma once
#include "IAgent.hpp"
#include "Perception.hpp"
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

    // Perception
    VisionParams visionParams;
    PerceptionResult lastPerception;

    // Tracking intersection
    int currentIntersectionId = -1;   // -1 = pas dans une intersection
    bool hasEnteredIntersection = false;  // true = on est dedans, on ne freine plus
public:
    Vehicle(float startX, float startY, float tSize = 50.f);
    virtual ~Vehicle() = default;

    void update(float dt,
            const std::vector<std::unique_ptr<IAgent>>& agents,
            const World& world) override;
    void draw(sf::RenderWindow& window) override;
    sf::Vector2f getPosition() const override;

    void setPath(const std::vector<sf::Vector2i>& newPath);
    bool isFinished() const;

    // Debug
    bool contains(sf::Vector2f point) const override;
    void setSelected(bool selected) override;
    void drawDebug(sf::RenderWindow& window) override;

    float getHeading() const override;
    float getSpeed() const override;
};