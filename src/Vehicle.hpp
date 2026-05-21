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
    float lastRoadSpeedLimit = 100.f;

    // Tracking intersection
    bool isCommittedToPass = false;
    int committedIntersectionId = -1;

    // Sauvegarde du scénario originel
    sf::Vector2i startTile;
    sf::Vector2i goalTile;
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
    float getLength() const override;

    sf::Vector2i getStartTile() const override { return startTile; }
    sf::Vector2i getGoalTile() const override { return goalTile; }

    // Mode Construction
    sf::Vector2i getCurrentTile() const override;
    void recalculatePath(const World& world) override;
    void resetToStart(const World& world) override;
};