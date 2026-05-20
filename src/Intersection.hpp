// src/Intersection.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

class IAgent;

enum class RegulationType {
    PRIORITY_RIGHT,
    STOP,               // Prévu
    YIELD,              // Prévu
    TRAFFIC_LIGHT,
    ROUNDABOUT,         // Prévu
};

struct Approach {
    enum class Direction { NORTH, SOUTH, EAST, WEST };

    Direction direction;
    sf::Vector2i entryTile;
    bool hasGreen = false;
};

enum class LightState {
    GREEN,
    ORANGE,
    RED
};

class Intersection {
private:
    int id;
    RegulationType type;

    std::vector<sf::Vector2i> coveredTiles;
    std::vector<Approach> approaches;

    // Feux tricolores
    float lightTimer = 0.f;
    int currentPhase = 0;
    float greenDuration = 5.f;
    float orangeDuration = 1.5f;

    void updateTrafficLight(float dt);

public:
    Intersection(int id, RegulationType type);

    void addCoveredTile(sf::Vector2i tile);
    void addApproach(const Approach& approach);
    void update(float dt);

    // "Je viens de cette direction, est-ce que je peux passer ?"
    bool canPass(Approach::Direction fromDirection,
                 const std::vector<std::unique_ptr<IAgent>>& agents) const;

    int getId() const;
    RegulationType getType() const;
    const std::vector<sf::Vector2i>& getCoveredTiles() const;
    const std::vector<Approach>& getApproaches() const;
    bool coversTile(int gridX, int gridY) const;
    LightState getLightState(Approach::Direction dir) const;

    void draw(sf::RenderWindow& window, float tileSize) const;
};