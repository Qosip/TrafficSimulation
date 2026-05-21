// src/World.cpp
#include "World.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

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

sf::Color getRoadColor(RoadType type) {
    switch (type) {
        case RoadType::CITY_30:      return sf::Color(65, 65, 65);
        case RoadType::CITY_50:      return sf::Color(45, 45, 45);     // Anthracite chic
        case RoadType::ROAD_80:      return sf::Color(35, 35, 35);
        case RoadType::HIGHWAY_130:  return sf::Color(28, 26, 34);     // Graphite bleuté haut de gamme
        case RoadType::INTERSECTION: return sf::Color(55, 55, 58);
        default:                     return sf::Color(38, 77, 44);     // Vert prairie désaturé
    }
}

World::World(int tilesX, int tilesY, float tSize) :
    gridWidth(tilesX), gridHeight(tilesY), tileSize(tSize)
{
    grid.resize(gridWidth, std::vector<Tile>(gridHeight));
}

void World::setTile(int gridX, int gridY, RoadType type, TileDirection dir) {
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        grid[gridX][gridY].roadType = type;
        grid[gridX][gridY].direction = dir;
        isTextureInitialized = false; // Flag pour redessiner la map au prochain cycle
    }
}

const Tile& World::getTile(int gridX, int gridY) const {
    static Tile emptyTile;
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        return grid[gridX][gridY];
    }
    return emptyTile;
}

float World::getSpeedLimitAt(float worldX, float worldY) const {
    int gx = (int)(worldX / tileSize);
    int gy = (int)(worldY / tileSize);
    return getRoadSpeedLimit(getTile(gx, gy).roadType);
}

std::vector<sf::Vector2i> World::getValidNeighbors(int x, int y) const {
    std::vector<sf::Vector2i> neighbors;
    const Tile& currentTile = getTile(x, y);

    if (currentTile.roadType == RoadType::NONE) return neighbors;

    struct Move { int dx; int dy; TileDirection requiredExitDir; };
    std::vector<Move> moves = {
        {0, -1, TileDirection::UP},
        {0, 1, TileDirection::DOWN},
        {-1, 0, TileDirection::LEFT},
        {1, 0, TileDirection::RIGHT}
    };

    for (const auto& move : moves) {
        int nx = x + move.dx;
        int ny = y + move.dy;

        if (nx < 0 || nx >= gridWidth || ny < 0 || ny >= gridHeight) continue;

        const Tile& nextTile = getTile(nx, ny);
        if (nextTile.roadType == RoadType::NONE) continue;

        if (currentTile.roadType != RoadType::INTERSECTION) {
            if (currentTile.direction == TileDirection::UP && (move.dx != 0 || move.dy != -1)) continue;
            if (currentTile.direction == TileDirection::DOWN && (move.dx != 0 || move.dy != 1)) continue;
            if (currentTile.direction == TileDirection::LEFT && (move.dx != -1 || move.dy != 0)) continue;
            if (currentTile.direction == TileDirection::RIGHT && (move.dx != 1 || move.dy != 0)) continue;

            if (nextTile.roadType != RoadType::INTERSECTION) {
                bool isUTurn = false;
                if (currentTile.direction == TileDirection::UP && nextTile.direction == TileDirection::DOWN) isUTurn = true;
                if (currentTile.direction == TileDirection::DOWN && nextTile.direction == TileDirection::UP) isUTurn = true;
                if (currentTile.direction == TileDirection::LEFT && nextTile.direction == TileDirection::RIGHT) isUTurn = true;
                if (currentTile.direction == TileDirection::RIGHT && nextTile.direction == TileDirection::LEFT) isUTurn = true;
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

void World::redrawMap() const {
    if (!isTextureInitialized) {
        mapTexture.create(gridWidth * tileSize, gridHeight * tileSize);
        isTextureInitialized = true;
    }

    mapTexture.clear(sf::Color(34, 73, 41)); // Tonalité pelouse élégante

    sf::RectangleShape tileShape(sf::Vector2f(tileSize, tileSize));
    tileShape.setOutlineThickness(-1.f);
    tileShape.setOutlineColor(sf::Color(0, 0, 0, 35));

    for (int x = 0; x < gridWidth; ++x) {
        for (int y = 0; y < gridHeight; ++y) {
            const Tile& tile = grid[x][y];
            if (tile.roadType == RoadType::NONE) continue;

            float posX = x * tileSize;
            float posY = y * tileSize;
            tileShape.setPosition(posX, posY);
            tileShape.setFillColor(getRoadColor(tile.roadType));
            mapTexture.draw(tileShape);

            // Génération pro des pointillés de voies
            if (tile.roadType == RoadType::CITY_50 || tile.roadType == RoadType::HIGHWAY_130) {
                sf::RectangleShape dash(sf::Vector2f(tileSize * 0.35f, 2.f));
                dash.setFillColor(sf::Color(255, 255, 255, 140));
                dash.setOrigin(dash.getSize().x / 2.f, dash.getSize().y / 2.f);
                dash.setPosition(posX + tileSize / 2.f, posY + tileSize / 2.f);

                if (tile.direction == TileDirection::UP || tile.direction == TileDirection::DOWN) {
                    dash.setRotation(90.f);
                }
                mapTexture.draw(dash);
            }
        }
    }
    mapTexture.display();
    mapSprite.setTexture(mapTexture.getTexture());
}

void World::draw(sf::RenderWindow& window) {
    if (!isTextureInitialized) redrawMap();
    window.draw(mapSprite);
}

void World::addIntersection(const Intersection& intersection) {
    intersections.push_back(intersection);
}

void World::updateIntersections(float dt) {
    for (auto& inter : intersections) inter.update(dt);
}

void World::drawIntersections(sf::RenderWindow& window) const {
    for (const auto& inter : intersections) inter.draw(window, tileSize);
}

const Intersection* World::getIntersectionNear(float worldX, float worldY, float radius) const {
    for (const auto& inter : intersections) {
        for (const auto& tile : inter.getCoveredTiles()) {
            float cx = tile.x * tileSize + tileSize / 2.f;
            float cy = tile.y * tileSize + tileSize / 2.f;
            float dx = worldX - cx;
            float dy = worldY - cy;
            if (std::sqrt(dx * dx + dy * dy) < radius) return &inter;
        }
    }
    return nullptr;
}

Approach::Direction World::getApproachDirection(float headingDeg) const {
    while (headingDeg < 0.f) headingDeg += 360.f;
    while (headingDeg >= 360.f) headingDeg -= 360.f;
    if (headingDeg >= 45.f && headingDeg < 135.f)  return Approach::Direction::NORTH;
    if (headingDeg >= 135.f && headingDeg < 225.f) return Approach::Direction::EAST;
    if (headingDeg >= 225.f && headingDeg < 315.f) return Approach::Direction::SOUTH;
    return Approach::Direction::WEST;
}

const Intersection* World::getIntersectionAt(float worldX, float worldY) const {
    // 1. Convertir les pixels en coordonnées de grille
    int gridX = static_cast<int>(worldX / tileSize);
    int gridY = static_cast<int>(worldY / tileSize);

    // 2. Chercher si une intersection possède cette tuile précise
    for (const auto& inter : intersections) {
        for (const auto& tile : inter.getCoveredTiles()) {
            if (tile.x == gridX && tile.y == gridY) {
                return &inter;
            }
        }
    }

    // Si on ne trouve rien, on n'est pas sur une intersection
    return nullptr;
}

void World::drawFlowDebug(sf::RenderWindow& window, float time) const {
    // --- PARAMÈTRES DU FLUX LASER ---
    float speed = 80.f;           // Vitesse de déplacement
    float length = 180.f;         // Une très longue traînée (presque 4 routes de long)
    float pulseSpacing = 350.f;   // Beaucoup d'espace entre chaque comète

    sf::Color coreColor = sf::Color(255, 230, 150); // Tête lumineuse (Jaune/Blanc)
    sf::Color tailColor = sf::Color(200, 20, 0);    // Queue sombre (Rouge)

    float maxThickness = 2.5f; // Épaisseur de la tête de la comète
    float minThickness = 0.2f; // Épaisseur de la pointe de la queue (très fin)

    // Un VertexArray est utilisé pour dessiner des géométries 100% lisses sans superposition
    sf::VertexArray trail(sf::Quads);

    for (int x = 0; x < gridWidth; ++x) {
        for (int y = 0; y < gridHeight; ++y) {
            const Tile& tile = grid[x][y];
            if (tile.roadType == RoadType::NONE || tile.roadType == RoadType::INTERSECTION) continue;

            // 1. Phase globale de la tuile
            float globalPhase = 0.f;
            if (tile.direction == TileDirection::RIGHT)      globalPhase = time * speed - x * tileSize;
            else if (tile.direction == TileDirection::LEFT)  globalPhase = time * speed + x * tileSize;
            else if (tile.direction == TileDirection::DOWN)  globalPhase = time * speed - y * tileSize;
            else if (tile.direction == TileDirection::UP)    globalPhase = time * speed + y * tileSize;

            // 2. Position de la tête de la comète (H)
            float H = std::fmod(globalPhase, pulseSpacing);
            if (H < 0) H += pulseSpacing;

            // 3. Calcul mathématique de la partie visible de la comète DANS cette tuile (0 à 50px)
            float startClip = std::max(0.f, H - length);
            float endClip = std::min((float)tileSize, H);

            // Si startClip < endClip, alors un bout de la comète passe par cette tuile !
            if (startClip < endClip) {
                // Ratio de 0.0 (Tête lumineuse) à 1.0 (Bout de la queue invisible)
                float prog1 = (H - startClip) / length;
                float prog2 = (H - endClip) / length;

                float inv1 = 1.f - prog1;
                float inv2 = 1.f - prog2;

                // Courbe d'évaporation de la lumière (très douce à la fin)
                sf::Uint8 a1 = static_cast<sf::Uint8>(255 * std::pow(inv1, 2.0f));
                sf::Uint8 a2 = static_cast<sf::Uint8>(255 * std::pow(inv2, 2.0f));

                // Création du dégradé de couleur
                sf::Color c1(
                    coreColor.r * inv1 + tailColor.r * prog1,
                    coreColor.g * inv1 + tailColor.g * prog1,
                    coreColor.b * inv1 + tailColor.b * prog1,
                    a1
                );
                sf::Color c2(
                    coreColor.r * inv2 + tailColor.r * prog2,
                    coreColor.g * inv2 + tailColor.g * prog2,
                    coreColor.b * inv2 + tailColor.b * prog2,
                    a2
                );

                // La ligne s'affine vers la queue
                float thick1 = minThickness + (maxThickness - minThickness) * inv1;
                float thick2 = minThickness + (maxThickness - minThickness) * inv2;

                sf::Vector2f pos1, pos2, norm1, norm2;

                // Alignement sur la route
                if (tile.direction == TileDirection::RIGHT) {
                    pos1 = sf::Vector2f(x * tileSize + startClip, y * tileSize + tileSize / 2.f);
                    pos2 = sf::Vector2f(x * tileSize + endClip, y * tileSize + tileSize / 2.f);
                    norm1 = sf::Vector2f(0.f, thick1);
                    norm2 = sf::Vector2f(0.f, thick2);
                } else if (tile.direction == TileDirection::LEFT) {
                    pos1 = sf::Vector2f((x + 1) * tileSize - startClip, y * tileSize + tileSize / 2.f);
                    pos2 = sf::Vector2f((x + 1) * tileSize - endClip, y * tileSize + tileSize / 2.f);
                    norm1 = sf::Vector2f(0.f, thick1);
                    norm2 = sf::Vector2f(0.f, thick2);
                } else if (tile.direction == TileDirection::DOWN) {
                    pos1 = sf::Vector2f(x * tileSize + tileSize / 2.f, y * tileSize + startClip);
                    pos2 = sf::Vector2f(x * tileSize + tileSize / 2.f, y * tileSize + endClip);
                    norm1 = sf::Vector2f(thick1, 0.f);
                    norm2 = sf::Vector2f(thick2, 0.f);
                } else if (tile.direction == TileDirection::UP) {
                    pos1 = sf::Vector2f(x * tileSize + tileSize / 2.f, (y + 1) * tileSize - startClip);
                    pos2 = sf::Vector2f(x * tileSize + tileSize / 2.f, (y + 1) * tileSize - endClip);
                    norm1 = sf::Vector2f(thick1, 0.f);
                    norm2 = sf::Vector2f(thick2, 0.f);
                }

                // On dessine le bout de ligne (sans AUCUNE superposition possible)
                trail.append(sf::Vertex(pos1 - norm1, c1));
                trail.append(sf::Vertex(pos1 + norm1, c1));
                trail.append(sf::Vertex(pos2 + norm2, c2));
                trail.append(sf::Vertex(pos2 - norm2, c2));
            }
        }
    }

    // On dessine toutes les lignes générées en une seule passe Additive
    window.draw(trail, sf::BlendAdd);
}