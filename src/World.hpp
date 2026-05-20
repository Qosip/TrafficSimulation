// src/World.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>

// Les différents types de blocs possibles
enum class TileType {
    GRASS,
    ROAD_UP,
    ROAD_DOWN,
    ROAD_LEFT,
    ROAD_RIGHT,
    INTERSECTION // Le centre du carrefour où les conflits se gèrent
};

class World {
private:
    int gridWidth;
    int gridHeight;
    float tileSize;

    std::vector<std::vector<TileType>> grid;

public:
    World(int windowWidth, int windowHeight, float tSize);

    void setTile(int gridX, int gridY, TileType type);

    TileType getTile(int gridX, int gridY) const;

    void draw(sf::RenderWindow& window);
};