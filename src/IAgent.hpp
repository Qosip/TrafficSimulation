// src/IAgent.hpp
#pragma once
#include <SFML/Graphics.hpp>

class IAgent {
public:
    virtual ~IAgent() = default;

    virtual void update(float dt) = 0;
    virtual void draw(sf::RenderWindow& window) = 0;

    virtual sf::Vector2f getPosition() const = 0;

    // Méthodes pour le mode Debug
    virtual bool contains(sf::Vector2f point) const = 0;
    virtual void setSelected(bool selected) = 0;
    virtual void drawDebug(sf::RenderWindow& window) = 0;
};