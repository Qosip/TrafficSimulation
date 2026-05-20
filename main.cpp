// main.cpp
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include "Car.hpp"
#include "Truck.hpp"
#include "World.hpp"
#include "Camera.hpp"
#include "PathFinder.hpp"

int main() {
    const int WINDOW_W = 800;
    const int WINDOW_H = 800;

    sf::RenderWindow window(sf::VideoMode(WINDOW_W, WINDOW_H), "Simulateur MAS - Sandbox");

    // Centrage de la fenêtre
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    window.setPosition(sf::Vector2i((desktop.width / 2) - WINDOW_W / 2,
                                     (desktop.height / 2) - WINDOW_H / 2));
    window.setFramerateLimit(60);

    sf::Clock clock;

    // --- INITIALISATION DE L'ENVIRONNEMENT ---
    // Le World prend maintenant des dimensions en tuiles, pas en pixels !
    // 16x16 = identique à avant (800px / 50px = 16 tuiles)
    // Tu peux passer à 32x32, 64x64... la caméra gère.

    const int GRID_W = 32;
    const int GRID_H = 32;
    const float TILE_SIZE = 50.f;

    World world(GRID_W, GRID_H, TILE_SIZE);
    // --- CONSTRUCTION DE LA MAP ---
    auto buildHRoad = [&](int y, int xStart, int xEnd) {
        for (int x = xStart; x <= xEnd; ++x) {
            world.setTile(x, y,     RoadType::CITY_50, TileDirection::RIGHT);
            world.setTile(x, y - 1, RoadType::CITY_50, TileDirection::LEFT);
        }
    };

    auto buildVRoad = [&](int x, int yStart, int yEnd) {
        for (int y = yStart; y <= yEnd; ++y) {
            world.setTile(x,     y, RoadType::CITY_50, TileDirection::UP);
            world.setTile(x - 1, y, RoadType::CITY_50, TileDirection::DOWN);
        }
    };

    auto buildCrossroad = [&](int cx, int cy, int id, RegulationType regType) {
        world.setTile(cx - 1, cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
        world.setTile(cx,     cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
        world.setTile(cx - 1, cy,     RoadType::INTERSECTION, TileDirection::NONE);
        world.setTile(cx,     cy,     RoadType::INTERSECTION, TileDirection::NONE);

        Intersection inter(id, regType);
        inter.addCoveredTile(sf::Vector2i(cx - 1, cy - 1));
        inter.addCoveredTile(sf::Vector2i(cx,     cy - 1));
        inter.addCoveredTile(sf::Vector2i(cx - 1, cy));
        inter.addCoveredTile(sf::Vector2i(cx,     cy));

        Approach north; north.direction = Approach::Direction::NORTH;
        north.entryTile = sf::Vector2i(cx, cy - 2);
        inter.addApproach(north);

        Approach south; south.direction = Approach::Direction::SOUTH;
        south.entryTile = sf::Vector2i(cx - 1, cy + 1);
        inter.addApproach(south);

        Approach east; east.direction = Approach::Direction::EAST;
        east.entryTile = sf::Vector2i(cx + 1, cy - 1);
        inter.addApproach(east);

        Approach west; west.direction = Approach::Direction::WEST;
        west.entryTile = sf::Vector2i(cx - 2, cy);
        inter.addApproach(west);

        world.addIntersection(inter);
    };

    // Routes horizontales à y=11 et y=22
    buildHRoad(11, 0, 31);
    buildHRoad(22, 0, 31);

    // Routes verticales à x=11 et x=22
    buildVRoad(11, 0, 31);
    buildVRoad(22, 0, 31);

    // 4 carrefours
    buildCrossroad(11, 11, 0, RegulationType::PRIORITY_RIGHT);
    buildCrossroad(22, 11, 1, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(11, 22, 2, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(22, 22, 3, RegulationType::PRIORITY_RIGHT);

    // --- INITIALISATION DE LA CAMÉRA ---
    // Centrée sur le milieu du monde, taille visible = taille de la fenêtre
    Camera camera(
        world.getWorldPixelWidth() / 2.f,
        world.getWorldPixelHeight() / 2.f,
        (float)WINDOW_W,
        (float)WINDOW_H
    );

    // --- AGENTS ---
    std::vector<std::unique_ptr<IAgent>> agents;

    // Voiture 1 (Anciennement: {{1,11},{5,11},{8,11},{15,11},{20,11},{25,11},{30,11}})
    {
        float sx = 1 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 11 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);

        // Génération automatique du chemin du point A (1, 11) au point B (30, 11)
        std::vector<sf::Vector2i> autoPath = Pathfinder::findPath(world, {1, 11}, {21, 30});
        dynamic_cast<Car*>(c.get())->setPath(autoPath);

        agents.push_back(std::move(c));
    }

    // Camion 2 (OUEST vers le SUD avec virage par exemple)
    {
        float sx = 30 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 10 * TILE_SIZE + TILE_SIZE / 2.f;
        auto t = std::make_unique<Truck>(sx, sy);

        // De l'Est vers le Sud (le Pathfinder gérera les carrefours pour tourner !)
        std::vector<sf::Vector2i> autoPath = Pathfinder::findPath(world, {30, 10}, {10, 30});
        dynamic_cast<Truck*>(t.get())->setPath(autoPath);

        agents.push_back(std::move(t));
    }

    // Voiture 2 : y=10 vers l'OUEST
    {
        float sx = 30 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 10 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);
        dynamic_cast<Car*>(c.get())->setPath({{30,10},{25,10},{20,10},{15,10},{8,10},{3,10},{1,10}});
        agents.push_back(std::move(c));
    }

    // Voiture 3 : x=11 vers le NORD
    {
        float sx = 11 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 30 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);
        dynamic_cast<Car*>(c.get())->setPath({{11,30},{11,25},{11,20},{11,15},{11,8},{11,3},{11,1}});
        agents.push_back(std::move(c));
    }

    // Voiture 4 : x=10 vers le SUD
    {
        float sx = 10 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 1 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);
        dynamic_cast<Car*>(c.get())->setPath({{10,1},{10,5},{10,8},{10,15},{10,20},{10,25},{10,30}});
        agents.push_back(std::move(c));
    }

    // Voiture 5 : x=22 vers le NORD, tourne OUEST au carrefour (22,11)
    /*{
        float sx = 22 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 30 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);
        dynamic_cast<Car*>(c.get())->setPath({{22,30},{22,25},{22,20},{22,15},{22,10},{18,10},{14,10},{8,10},{1,10}});
        agents.push_back(std::move(c));
    }*/

    // Camion 1 : y=22 vers l'EST
    {
        float sx = 1 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 22 * TILE_SIZE + TILE_SIZE / 2.f;
        auto t = std::make_unique<Truck>(sx, sy);
        dynamic_cast<Truck*>(t.get())->setPath({{1,22},{5,22},{8,22},{15,22},{20,22},{28,22}});
        agents.push_back(std::move(t));
    }

    // Voiture 6 : x=21 vers le SUD
    {
        float sx = 21 * TILE_SIZE + TILE_SIZE / 2.f;
        float sy = 1 * TILE_SIZE + TILE_SIZE / 2.f;
        auto c = std::make_unique<Car>(sx, sy);
        dynamic_cast<Car*>(c.get())->setPath({{21,1},{21,5},{21,8},{21,15},{21,20},{21,25},{21,30}});
        agents.push_back(std::move(c));
    }

    bool debugMode = false;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // --- La caméra consomme les événements de zoom ---
            camera.handleEvent(event, window);

            // --- Clic gauche : poser une route (convertie en coordonnées monde) ---
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                if (!debugMode) {
                    sf::Vector2f worldPos = camera.screenToWorld(window, sf::Mouse::getPosition(window));
                    int gridX = (int)(worldPos.x / TILE_SIZE);
                    int gridY = (int)(worldPos.y / TILE_SIZE);
                    world.setTile(gridX, gridY, RoadType::CITY_50, TileDirection::RIGHT);
                }
            }

            // --- Touche D : PLUS utilisée pour le debug ---
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::D) {
                debugMode = !debugMode;
                if (!debugMode) {
                    for (auto& agent : agents) agent->setSelected(false);
                }
            }

            // --- Clic de sélection debug (coordonnées monde) ---
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left && debugMode) {
                    sf::Vector2f mousePos = camera.screenToWorld(window, sf::Mouse::getPosition(window));

                    for (auto& agent : agents) {
                        if (agent->contains(mousePos)) {
                            agent->setSelected(true);
                        } else {
                            agent->setSelected(false);
                        }
                    }
                }
            }

            // --- Touche R : recentrer la caméra ---
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
                camera = Camera(
                    world.getWorldPixelWidth() / 2.f,
                    world.getWorldPixelHeight() / 2.f,
                    (float)WINDOW_W,
                    (float)WINDOW_H
                );
            }
        }

        // --- UPDATE ---
        world.updateIntersections(dt);
        for (auto& agent : agents) {
            agent->update(dt, agents, world);
        }

        // --- RENDER ---
        window.clear(sf::Color(34, 139, 34)); // Fond vert (visible si on sort de la map)

        // On applique la caméra AVANT de dessiner le monde et les agents
        camera.applyTo(window);

        world.draw(window);
        world.drawIntersections(window);

        for (auto& agent : agents) {
            agent->draw(window);
            if (debugMode) {
                agent->drawDebug(window);
            }
        }

        // Si besoin d'un HUD fixe plus tard :
        // camera.resetTo(window);
        // drawHUD(window);

        window.display();
    }

    return 0;
}