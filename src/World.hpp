// src/World.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include "Intersection.hpp"
#include <vector>

// Direction de circulation sur la tuile
enum class TileDirection {
    NONE,       // Pas de direction (herbe, intersection)
    UP,
    DOWN,
    LEFT,
    RIGHT
};

// Type de route : détermine la vitesse max et l'apparence
enum class RoadType {
    NONE,           // Pas une route (herbe)
    CITY_30,        // Rue en ville, 30 km/h
    CITY_50,        // Route urbaine, 50 km/h
    ROAD_80,        // Route départementale, 80 km/h
    HIGHWAY_130,    // Autoroute, 130 km/h
    INTERSECTION    // Carrefour (vitesse réduite)
};

// Vitesse max en pixels/seconde pour chaque type de route
// (On peut ajuster l'échelle plus tard, pour l'instant c'est du gameplay)
float getRoadSpeedLimit(RoadType type);

// Couleur de l'asphalte selon le type de route (différenciation visuelle)
sf::Color getRoadColor(RoadType type);

// Données d'une tuile
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

    mutable sf::RenderTexture mapTexture;
    mutable sf::Sprite mapSprite;
    mutable bool isTextureInitialized = false;
public:
    World(int tilesX, int tilesY, float tSize);

    void setTile(int gridX, int gridY, RoadType type, TileDirection dir);
    const Tile& getTile(int gridX, int gridY) const;

    // Vitesse max à une position monde (pixels)
    float getSpeedLimitAt(float worldX, float worldY) const;

    void draw(sf::RenderWindow& window);

    int getGridWidth() const;
    int getGridHeight() const;
    float getTileSize() const;
    float getWorldPixelWidth() const;
    float getWorldPixelHeight() const;

    // Intersections
    void addIntersection(const Intersection& intersection);
    void updateIntersections(float dt);
    void drawIntersections(sf::RenderWindow& window) const;

    // Trouver l'intersection à une position monde (ou nullptr)
    const Intersection* getIntersectionAt(float worldX, float worldY) const;

    // Trouver l'intersection proche devant un véhicule (dans un rayon)
    const Intersection* getIntersectionNear(float worldX, float worldY, float radius) const;

    // Pour la perception : déterminer de quelle direction on approche une intersection
    Approach::Direction getApproachDirection(float headingDeg) const;
    std::vector<sf::Vector2i> getValidNeighbors(int x, int y) const;

    // Récupérer toutes les intersections pour la sauvegarde
    const std::vector<Intersection>& getIntersections() const { return intersections; }

    void redrawMap() const;
};