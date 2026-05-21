// src/IAgent.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

// Forward declarations
class World;

class IAgent {
public:
    virtual ~IAgent() = default;

    // dt + accès aux autres agents et au monde pour la perception
    virtual void update(float dt,
                        const std::vector<std::unique_ptr<IAgent>>& agents,
                        const World& world) = 0;

    virtual void draw(sf::RenderWindow& window) = 0;

    virtual sf::Vector2f getPosition() const = 0;

    // Pour que le système de vision puisse identifier ce qu'il détecte
    virtual float getHeading() const = 0;   // Angle en degrés
    virtual float getSpeed() const = 0;     // Vitesse actuelle

    // Debug
    virtual bool contains(sf::Vector2f point) const = 0;
    virtual void setSelected(bool selected) = 0;
    virtual void drawDebug(sf::RenderWindow& window) = 0;
    virtual float getLength() const = 0;

    // Sauvegarde
    virtual std::string getType() const = 0;
    virtual sf::Vector2i getStartTile() const = 0;
    virtual sf::Vector2i getGoalTile() const = 0;

    // Mode Construction
    virtual sf::Vector2i getCurrentTile() const = 0;
    virtual void recalculatePath(const World& world) = 0;
    virtual void resetToStart(const World& world) = 0;
};