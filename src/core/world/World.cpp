// src/World.cpp
#include "core/world/World.hpp"

#include <algorithm>
#include <cmath>

float getRoadSpeedLimit(RoadType type) {
    switch (type) {
        case RoadType::CITY_30:      return 60.f;
        case RoadType::CITY_50:      return 100.f;
        case RoadType::ROAD_80:      return 160.f;
        case RoadType::HIGHWAY_130:  return 260.f;
        case RoadType::INTERSECTION: return 100.f;
        default:                     return 0.f;
    }
}

core::Color getRoadColor(RoadType type) {
    switch (type) {
        case RoadType::CITY_30:      return core::Color{65, 65, 65};
        case RoadType::CITY_50:      return core::Color{45, 45, 45};    // Anthracite chic
        case RoadType::ROAD_80:      return core::Color{35, 35, 35};
        case RoadType::HIGHWAY_130:  return core::Color{28, 26, 34};    // Graphite bleute
        case RoadType::INTERSECTION: return core::Color{55, 55, 58};
        default:                     return core::Color{38, 77, 44};    // Vert prairie desature
    }
}

World::World(int tilesX, int tilesY, float tSize) :
    gridWidth(tilesX), gridHeight(tilesY), tileSize(tSize)
{
    grid.resize(gridWidth, std::vector<Tile>(gridHeight));
}

void World::setTile(int gridX, int gridY, RoadType type, TileDirection dir) {
    if (gridX < 0 || gridX >= gridWidth || gridY < 0 || gridY >= gridHeight) return;

    // Suppression d'une tile : si elle appartient a une intersection,
    // on detruit l'intersection entiere (et remet ses autres tiles a NONE)
    // pour ne pas laisser une structure carrefour orpheline et incoherente.
    if (type == RoadType::NONE) {
        for (auto it = intersections.begin(); it != intersections.end(); ) {
            bool owns = false;
            for (const auto& t : it->getCoveredTiles()) {
                if (t.x == gridX && t.y == gridY) { owns = true; break; }
            }
            if (owns) {
                // Reset des autres tiles couvertes a NONE en bypass-record
                // pour eviter une recursion infinie.
                for (const auto& t : it->getCoveredTiles()) {
                    if (t.x == gridX && t.y == gridY) continue;
                    if (t.x >= 0 && t.x < gridWidth && t.y >= 0 && t.y < gridHeight) {
                        grid[t.x][t.y].roadType  = RoadType::NONE;
                        grid[t.x][t.y].direction = TileDirection::NONE;
                    }
                }
                it = intersections.erase(it);
            } else {
                ++it;
            }
        }
    }

    grid[gridX][gridY].roadType  = type;
    grid[gridX][gridY].direction = dir;
    mapDirty = true;
}

const Tile& World::getTile(int gridX, int gridY) const {
    static const Tile emptyTile;
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        return grid[gridX][gridY];
    }
    return emptyTile;
}

float World::getSpeedLimitAt(float worldX, float worldY) const {
    const int gx = static_cast<int>(worldX / tileSize);
    const int gy = static_cast<int>(worldY / tileSize);
    return getRoadSpeedLimit(getTile(gx, gy).roadType);
}

std::vector<core::TileCoord> World::getValidNeighbors(int x, int y) const {
    std::vector<core::TileCoord> neighbors;
    const Tile& currentTile = getTile(x, y);

    if (currentTile.roadType == RoadType::NONE) return neighbors;

    struct Move { int dx; int dy; TileDirection requiredExitDir; };
    const Move moves[] = {
        {0, -1, TileDirection::UP},
        {0,  1, TileDirection::DOWN},
        {-1, 0, TileDirection::LEFT},
        {1,  0, TileDirection::RIGHT}
    };

    for (const auto& move : moves) {
        const int nx = x + move.dx;
        const int ny = y + move.dy;
        if (nx < 0 || nx >= gridWidth || ny < 0 || ny >= gridHeight) continue;

        const Tile& nextTile = getTile(nx, ny);
        if (nextTile.roadType == RoadType::NONE) continue;

        if (currentTile.roadType != RoadType::INTERSECTION) {
            if (currentTile.direction == TileDirection::UP    && (move.dx != 0  || move.dy != -1)) continue;
            if (currentTile.direction == TileDirection::DOWN  && (move.dx != 0  || move.dy !=  1)) continue;
            if (currentTile.direction == TileDirection::LEFT  && (move.dx != -1 || move.dy !=  0)) continue;
            if (currentTile.direction == TileDirection::RIGHT && (move.dx !=  1 || move.dy !=  0)) continue;

            if (nextTile.roadType != RoadType::INTERSECTION) {
                bool isUTurn = false;
                if (currentTile.direction == TileDirection::UP    && nextTile.direction == TileDirection::DOWN)  isUTurn = true;
                if (currentTile.direction == TileDirection::DOWN  && nextTile.direction == TileDirection::UP)    isUTurn = true;
                if (currentTile.direction == TileDirection::LEFT  && nextTile.direction == TileDirection::RIGHT) isUTurn = true;
                if (currentTile.direction == TileDirection::RIGHT && nextTile.direction == TileDirection::LEFT)  isUTurn = true;
                if (isUTurn) continue;
            }
        } else {
            if (nextTile.roadType != RoadType::INTERSECTION && nextTile.direction != move.requiredExitDir) {
                continue;
            }
        }
        neighbors.push_back({nx, ny});
    }
    return neighbors;
}

void World::addIntersection(const Intersection& intersection) {
    intersections.push_back(intersection);
}

void World::updateIntersections(float dt) {
    for (auto& inter : intersections) inter.update(dt);
}

void World::setIntersectionRegulation(std::size_t index, RegulationType type) {
    if (index < intersections.size()) intersections[index].setRegulation(type);
}

void World::refreshRoundaboutApproaches() {
    auto isRoad = [&](int x, int y) {
        const Tile& t = getTile(x, y);
        return t.roadType != RoadType::NONE && t.roadType != RoadType::INTERSECTION;
    };

    for (auto& inter : intersections) {
        if (inter.getType() != RegulationType::ROUNDABOUT) continue;

        const auto& tiles = inter.getCoveredTiles();
        if (tiles.empty()) continue;

        // Bounding box de l'anneau.
        int minX = tiles[0].x, maxX = tiles[0].x;
        int minY = tiles[0].y, maxY = tiles[0].y;
        for (const auto& t : tiles) {
            minX = std::min(minX, t.x); maxX = std::max(maxX, t.x);
            minY = std::min(minY, t.y); maxY = std::max(maxY, t.y);
        }
        const int cx = (minX + maxX) / 2;
        const int cy = (minY + maxY) / 2;

        // Une branche n'existe que si une vraie route touche ce cote.
        inter.clearApproaches();
        auto tryAdd = [&](Approach::Direction dir, int ex, int ey) {
            if (!isRoad(ex, ey)) return;
            Approach a; a.direction = dir; a.entryTile = {ex, ey};
            inter.addApproach(a);
        };
        tryAdd(Approach::Direction::NORTH, cx,        minY - 1);
        tryAdd(Approach::Direction::SOUTH, cx,        maxY + 1);
        tryAdd(Approach::Direction::EAST,  maxX + 1,  cy);
        tryAdd(Approach::Direction::WEST,  minX - 1,  cy);
    }
}

const Intersection* World::getIntersectionNear(float worldX, float worldY, float radius) const {
    for (const auto& inter : intersections) {
        for (const auto& tile : inter.getCoveredTiles()) {
            const float cx = tile.x * tileSize + tileSize / 2.f;
            const float cy = tile.y * tileSize + tileSize / 2.f;
            const float dx = worldX - cx;
            const float dy = worldY - cy;
            if (std::sqrt(dx * dx + dy * dy) < radius) return &inter;
        }
    }
    return nullptr;
}

Approach::Direction World::getApproachDirection(float headingDeg) const {
    while (headingDeg <    0.f) headingDeg += 360.f;
    while (headingDeg >= 360.f) headingDeg -= 360.f;
    if (headingDeg >=  45.f && headingDeg < 135.f) return Approach::Direction::NORTH;
    if (headingDeg >= 135.f && headingDeg < 225.f) return Approach::Direction::EAST;
    if (headingDeg >= 225.f && headingDeg < 315.f) return Approach::Direction::SOUTH;
    return Approach::Direction::WEST;
}

const Intersection* World::getIntersectionAt(float worldX, float worldY) const {
    const int gridX = static_cast<int>(worldX / tileSize);
    const int gridY = static_cast<int>(worldY / tileSize);

    for (const auto& inter : intersections) {
        for (const auto& tile : inter.getCoveredTiles()) {
            if (tile.x == gridX && tile.y == gridY) return &inter;
        }
    }
    return nullptr;
}

bool World::consumeMapDirty() {
    const bool was = mapDirty;
    mapDirty = false;
    return was;
}
