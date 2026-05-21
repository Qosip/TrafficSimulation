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

    // Statut d'interface
    bool isSelectedFlag = false;

    // Périmètre de vision et capteurs
    VisionParams visionParams;
    PerceptionResult lastPerception;
    float lastRoadSpeedLimit = 100.f;

    // Verrous de priorité aux carrefours
    bool isCommittedToPass = false;
    int committedIntersectionId = -1;

    // Coordonnées cibles du scénario immuable
    sf::Vector2i startTile;
    sf::Vector2i goalTile;

public:
    Vehicle(float startX, float startY, float tSize = 50.f);
    virtual ~Vehicle() = default;

    void update(float dt,
                const std::vector<std::unique_ptr<IAgent>>& agents,
                const World& world) override;
    void draw(sf::RenderWindow& window) override;

    sf::Vector2f getPosition() const override { return position; }
    float getHeading() const override { return currentAngle; }
    float getSpeed() const override { return currentSpeed; }
    float getLength() const override { return shape.getSize().x; }

    std::string getType() const override { return "VEHICLE"; }
    sf::Vector2i getStartTile() const override { return startTile; }
    sf::Vector2i getGoalTile() const override { return goalTile; }
    sf::Vector2i getCurrentTile() const override;

    void setPath(const std::vector<sf::Vector2i>& newPath);
    void recalculatePath(const World& world) override;
    void resetToStart(const World& world) override;

    bool contains(sf::Vector2f point) const override;
    void setSelected(bool selected) override { isSelectedFlag = selected; }
    bool isSelected() const override { return isSelectedFlag; }
    void drawDebug(sf::RenderWindow& window) override;
    float getRemainingDistance() const override;
};