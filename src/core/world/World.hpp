// src/World.hpp
//
// Etape 4 : aucune dependance SFML restante.
#pragma once

#include <vector>

#include "core/intersection/Intersection.hpp"
#include "core/Color.hpp"
#include "core/math/TileCoord.hpp"

enum class TileDirection { NONE, UP, DOWN, LEFT, RIGHT };

enum class RoadType { NONE, CITY_30, CITY_50, ROAD_80, HIGHWAY_130, INTERSECTION };

float       getRoadSpeedLimit(RoadType type);
core::Color getRoadColor(RoadType type);

struct Tile {
    RoadType      roadType  = RoadType::NONE;
    TileDirection direction = TileDirection::NONE;
};

class World {
private:
    int   gridWidth;
    int   gridHeight;
    float tileSize;

    std::vector<std::vector<Tile>> grid;
    std::vector<Intersection>      intersections;

    bool mapDirty = true;

public:
    World(int tilesX, int tilesY, float tSize);

    void        setTile(int gridX, int gridY, RoadType type, TileDirection dir);
    const Tile& getTile(int gridX, int gridY) const;
    float       getSpeedLimitAt(float worldX, float worldY) const;

    int   getGridWidth() const  { return gridWidth; }
    int   getGridHeight() const { return gridHeight; }
    float getTileSize() const   { return tileSize; }
    float getWorldPixelWidth()  const { return gridWidth  * tileSize; }
    float getWorldPixelHeight() const { return gridHeight * tileSize; }

    std::vector<core::TileCoord> getValidNeighbors(int x, int y) const;

    void addIntersection(const Intersection& intersection);
    void updateIntersections(float dt);

    const std::vector<Intersection>& getIntersections() const { return intersections; }
    const Intersection* getIntersectionAt(float worldX, float worldY) const;
    const Intersection* getIntersectionNear(float worldX, float worldY, float radius) const;
    Approach::Direction getApproachDirection(float headingDeg) const;

    bool consumeMapDirty();
    void markMapDirty() { mapDirty = true; }
};
