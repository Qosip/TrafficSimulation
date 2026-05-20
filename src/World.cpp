// src/World.cpp
#include <cmath>
#include "World.hpp"

float getRoadSpeedLimit(RoadType type) {
    switch (type) {
        case RoadType::CITY_30:      return 60.f;    // Lent
        case RoadType::CITY_50:      return 100.f;   // Modéré
        case RoadType::ROAD_80:      return 160.f;   // Rapide
        case RoadType::HIGHWAY_130:  return 260.f;   // Très rapide
        case RoadType::INTERSECTION: return 40.f;    // Très lent dans le carrefour
        default:                     return 0.f;     // Pas une route
    }
}

sf::Color getRoadColor(RoadType type) {
    switch (type) {
        case RoadType::CITY_30:      return sf::Color(70, 70, 70);    // Gris clair — pavés
        case RoadType::CITY_50:      return sf::Color(50, 50, 50);    // Asphalte standard
        case RoadType::ROAD_80:      return sf::Color(40, 40, 40);    // Asphalte foncé
        case RoadType::HIGHWAY_130:  return sf::Color(35, 35, 45);    // Asphalte bleuté
        case RoadType::INTERSECTION: return sf::Color(80, 80, 80);    // Gris clair
        default:                     return sf::Color(34, 139, 34);   // Herbe
    }
}

// Tuile par défaut retournée quand on sort de la grille
static const Tile DEFAULT_TILE = { RoadType::NONE, TileDirection::NONE };

World::World(int tilesX, int tilesY, float tSize)
    : gridWidth(tilesX), gridHeight(tilesY), tileSize(tSize)
{
    grid.resize(gridWidth, std::vector<Tile>(gridHeight));
}

void World::setTile(int gridX, int gridY, RoadType type, TileDirection dir) {
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        grid[gridX][gridY].roadType = type;
        grid[gridX][gridY].direction = dir;
    }
}

const Tile& World::getTile(int gridX, int gridY) const {
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        return grid[gridX][gridY];
    }
    return DEFAULT_TILE;
}

float World::getSpeedLimitAt(float worldX, float worldY) const {
    int gx = (int)(worldX / tileSize);
    int gy = (int)(worldY / tileSize);
    return getRoadSpeedLimit(getTile(gx, gy).roadType);
}

void World::draw(sf::RenderWindow& window) {
    sf::RectangleShape tileShape(sf::Vector2f(tileSize, tileSize));
    tileShape.setOutlineThickness(-1.f);
    tileShape.setOutlineColor(sf::Color(0, 0, 0, 50));

    sf::CircleShape dirIndicator(tileSize / 4.f, 3);
    dirIndicator.setFillColor(sf::Color(255, 204, 0));
    dirIndicator.setOrigin(dirIndicator.getRadius(), dirIndicator.getRadius());

    for (int x = 0; x < gridWidth; ++x) {
        for (int y = 0; y < gridHeight; ++y) {
            float posX = x * tileSize;
            float posY = y * tileSize;
            tileShape.setPosition(posX, posY);

            const Tile& tile = grid[x][y];
            tileShape.setFillColor(getRoadColor(tile.roadType));
            window.draw(tileShape);

            // Triangle directionnel
            if (tile.direction != TileDirection::NONE) {
                dirIndicator.setPosition(posX + tileSize / 2.f, posY + tileSize / 2.f);

                if (tile.direction == TileDirection::RIGHT) dirIndicator.setRotation(90.f);
                else if (tile.direction == TileDirection::DOWN) dirIndicator.setRotation(180.f);
                else if (tile.direction == TileDirection::LEFT) dirIndicator.setRotation(270.f);
                else if (tile.direction == TileDirection::UP) dirIndicator.setRotation(0.f);

                window.draw(dirIndicator);
            }
        }
    }
}

int World::getGridWidth() const { return gridWidth; }
int World::getGridHeight() const { return gridHeight; }
float World::getTileSize() const { return tileSize; }
float World::getWorldPixelWidth() const { return gridWidth * tileSize; }
float World::getWorldPixelHeight() const { return gridHeight * tileSize; }
void World::addIntersection(const Intersection& intersection) {
    intersections.push_back(intersection);
}

void World::updateIntersections(float dt) {
    for (auto& inter : intersections) {
        inter.update(dt);
    }
}

void World::drawIntersections(sf::RenderWindow& window) const {
    for (const auto& inter : intersections) {
        inter.draw(window, tileSize);
    }
}

const Intersection* World::getIntersectionAt(float worldX, float worldY) const {
    int gx = (int)(worldX / tileSize);
    int gy = (int)(worldY / tileSize);
    for (const auto& inter : intersections) {
        if (inter.coversTile(gx, gy)) return &inter;
    }
    return nullptr;
}

const Intersection* World::getIntersectionNear(float worldX, float worldY, float radius) const {
    for (const auto& inter : intersections) {
        for (const auto& tile : inter.getCoveredTiles()) {
            float cx = tile.x * tileSize + tileSize / 2.f;
            float cy = tile.y * tileSize + tileSize / 2.f;
            float dx = worldX - cx;
            float dy = worldY - cy;
            if (std::sqrt(dx * dx + dy * dy) < radius) {
                return &inter;
            }
        }
    }
    return nullptr;
}

Approach::Direction World::getApproachDirection(float headingDeg) const {
    while (headingDeg < 0.f) headingDeg += 360.f;
    while (headingDeg >= 360.f) headingDeg -= 360.f;

    if (headingDeg >= 315.f || headingDeg < 45.f) return Approach::Direction::EAST;
    if (headingDeg >= 45.f && headingDeg < 135.f) return Approach::Direction::SOUTH;
    if (headingDeg >= 135.f && headingDeg < 225.f) return Approach::Direction::WEST;
    return Approach::Direction::NORTH;
}