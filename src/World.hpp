// src/World.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include "Intersection.hpp"
#include <vector>

enum class TileDirection { NONE, UP, DOWN, LEFT, RIGHT };

enum class RoadType { NONE, CITY_30, CITY_50, ROAD_80, HIGHWAY_130, INTERSECTION };

float getRoadSpeedLimit(RoadType type);
sf::Color getRoadColor(RoadType type);

struct Tile {
    RoadType roadType = RoadType::NONE;
    TileDirection direction = TileDirection::NONE;
};

class World {
private:
    int gridWidth;
    int gridHeight;
    float tileSize;

    std::vector<std::vector<Tile>> grid;
    std::vector<Intersection> intersections;

    // Moteur de rendu statique
    mutable sf::RenderTexture mapTexture;
    mutable sf::Sprite mapSprite;
    mutable bool isTextureInitialized = false;

public:
    World(int tilesX, int tilesY, float tSize);

    void setTile(int gridX, int gridY, RoadType type, TileDirection dir);
    const Tile& getTile(int gridX, int gridY) const;
    float getSpeedLimitAt(float worldX, float worldY) const;

    void redrawMap() const;
    void draw(sf::RenderWindow& window);

    int getGridWidth() const { return gridWidth; }
    int getGridHeight() const { return gridHeight; }
    float getTileSize() const { return tileSize; }
    float getWorldPixelWidth() const { return gridWidth * tileSize; }
    float getWorldPixelHeight() const { return gridHeight * tileSize; }

    std::vector<sf::Vector2i> getValidNeighbors(int x, int y) const;

    void addIntersection(const Intersection& intersection);
    void updateIntersections(float dt);
    void drawIntersections(sf::RenderWindow& window) const;

    const std::vector<Intersection>& getIntersections() const { return intersections; }
    const Intersection* getIntersectionAt(float worldX, float worldY) const;
    const Intersection* getIntersectionNear(float worldX, float worldY, float radius) const;
    Approach::Direction getApproachDirection(float headingDeg) const;
    void drawFlowDebug(sf::RenderWindow& window, float time) const;
};