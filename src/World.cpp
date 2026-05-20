// src/World.cpp
#include "World.hpp"

World::World(int windowWidth, int windowHeight, float tSize) : tileSize(tSize) {
    gridWidth = windowWidth / (int)tileSize;
    gridHeight = windowHeight / (int)tileSize;

    // 1. On remplit d'herbe
    grid.resize(gridWidth, std::vector<TileType>(gridHeight, TileType::GRASS));

    int centerX = gridWidth / 2;
    int centerY = gridHeight / 2;

    // 2. Création des routes avec sens de circulation (Conduite à droite)
    for (int x = 0; x < gridWidth; ++x) {
        if (x != centerX && x != centerX - 1) {
            // Axe Horizontal
            setTile(x, centerY - 1, TileType::ROAD_LEFT);  // Voie du HAUT va vers la GAUCHE
            setTile(x, centerY, TileType::ROAD_RIGHT);     // Voie du BAS va vers la DROITE
        }
    }
    for (int y = 0; y < gridHeight; ++y) {
        if (y != centerY && y != centerY - 1) {
            // Axe Vertical
            setTile(centerX - 1, y, TileType::ROAD_DOWN);  // Voie de GAUCHE va vers le BAS
            setTile(centerX, y, TileType::ROAD_UP);        // Voie de DROITE va vers le HAUT
        }
    }

    // 3. Le coeur du carrefour
    setTile(centerX, centerY, TileType::INTERSECTION);
    setTile(centerX - 1, centerY, TileType::INTERSECTION);
    setTile(centerX, centerY - 1, TileType::INTERSECTION);
    setTile(centerX - 1, centerY - 1, TileType::INTERSECTION);
}

void World::setTile(int gridX, int gridY, TileType type) {
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        grid[gridX][gridY] = type;
    }
}

TileType World::getTile(int gridX, int gridY) const {
    if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        return grid[gridX][gridY];
    }
    return TileType::GRASS; // Sécurité : si on sort de la carte, c'est de l'herbe
}

void World::draw(sf::RenderWindow& window) {
    sf::RectangleShape tileShape(sf::Vector2f(tileSize, tileSize));
    tileShape.setOutlineThickness(-1.f);
    tileShape.setOutlineColor(sf::Color(0, 0, 0, 50));

    // Création d'un petit triangle pour indiquer le sens de la route
    sf::CircleShape dirIndicator(tileSize / 4.f, 3); // Un rayon de taille/4, et 3 sommets = Triangle
    dirIndicator.setFillColor(sf::Color(255, 204, 0)); // Jaune marquage au sol
    dirIndicator.setOrigin(dirIndicator.getRadius(), dirIndicator.getRadius());

    for (int x = 0; x < gridWidth; ++x) {
        for (int y = 0; y < gridHeight; ++y) {
            // Positionnement du bloc de base
            float posX = x * tileSize;
            float posY = y * tileSize;
            tileShape.setPosition(posX, posY);

            // Choix de la couleur du fond
            TileType type = grid[x][y];
            if (type == TileType::GRASS) {
                tileShape.setFillColor(sf::Color(34, 139, 34));
            } else if (type == TileType::INTERSECTION) {
                tileShape.setFillColor(sf::Color(80, 80, 80)); // Gris légèrement plus clair
            } else {
                tileShape.setFillColor(sf::Color(50, 50, 50)); // Asphalte
            }
            window.draw(tileShape);

            // Dessin du triangle directionnel si c'est une route
            if (type != TileType::GRASS && type != TileType::INTERSECTION) {
                // On place le triangle au centre de la tuile
                dirIndicator.setPosition(posX + tileSize / 2.f, posY + tileSize / 2.f);

                // On l'oriente en fonction de la direction de la route
                if (type == TileType::ROAD_RIGHT) dirIndicator.setRotation(90.f);
                else if (type == TileType::ROAD_DOWN) dirIndicator.setRotation(180.f);
                else if (type == TileType::ROAD_LEFT) dirIndicator.setRotation(270.f);
                else if (type == TileType::ROAD_UP) dirIndicator.setRotation(0.f);

                window.draw(dirIndicator);
            }
        }
    }
}