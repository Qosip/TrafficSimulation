// src/IAgent.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <string>

class World;

class IAgent {
public:
    virtual ~IAgent() = default;

    // Mise à jour de la logique et de la perception
    virtual void update(float dt,
                        const std::vector<std::unique_ptr<IAgent>>& agents,
                        const World& world) = 0;

    // Rendu graphique standard
    virtual void draw(sf::RenderWindow& window) = 0;

    // Getters physiques fondamentaux
    virtual sf::Vector2f getPosition() const = 0;
    virtual float getHeading() const = 0;   // Angle en degrés
    virtual float getSpeed() const = 0;     // Vitesse actuelle en px/s
    virtual float getLength() const = 0;    // Longueur totale de la carrosserie

    // Gestion du scénario originel et du Mode Construction
    virtual std::string getType() const = 0;
    virtual sf::Vector2i getStartTile() const = 0;
    virtual sf::Vector2i getGoalTile() const = 0;
    virtual sf::Vector2i getCurrentTile() const = 0;
    virtual void recalculatePath(const World& world) = 0;
    virtual void resetToStart(const World& world) = 0;

    // Outils de Débogage et d'Inspection UI
    virtual bool contains(sf::Vector2f point) const = 0;
    virtual void setSelected(bool selected) = 0;
    virtual bool isSelected() const = 0;
    virtual void drawDebug(sf::RenderWindow& window) = 0;
    virtual float getRemainingDistance() const = 0;
};