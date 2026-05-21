// main.cpp
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "portable-file-dialogs.h"
#include "Car.hpp"
#include "Truck.hpp"
#include "World.hpp"
#include "Camera.hpp"
#include "Pathfinder.hpp"

// ============================================================================
// TYPES ET INTERFACE UTILISATEUR
// ============================================================================

enum class AppState { MAIN_MENU, SIMULATION };

enum class BuildTool {
    NONE, ROAD_CITY_50, ROAD_HIGHWAY_130, INTERSECTION_PRIORITY, INTERSECTION_TRAFFIC_LIGHT, ERASE
};

struct Button {
    sf::RectangleShape rect;
    sf::Text text;
    bool isHovered = false;

    Button(float x, float y, float w, float h, const std::string& str, const sf::Font& font) {
        rect.setPosition(x, y);
        rect.setSize(sf::Vector2f(w, h));
        rect.setFillColor(sf::Color(50, 50, 50));
        rect.setOutlineThickness(2.f);
        rect.setOutlineColor(sf::Color::White);

        text.setFont(font);
        text.setString(str);
        text.setCharacterSize(24);
        text.setFillColor(sf::Color::White);

        sf::FloatRect bounds = text.getLocalBounds();
        text.setOrigin(bounds.left + bounds.width / 2.0f, bounds.top + bounds.height / 2.0f);
        text.setPosition(x + w / 2.0f, y + h / 2.0f);
    }

    void update(sf::Vector2f mousePos) {
        isHovered = rect.getGlobalBounds().contains(mousePos);
        rect.setFillColor(isHovered ? sf::Color(100, 100, 100) : sf::Color(50, 50, 50));
    }

    bool isClicked(const sf::Event& event) const {
        return isHovered && event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left;
    }

    void draw(sf::RenderWindow& window) const {
        window.draw(rect);
        window.draw(text);
    }
};

// ============================================================================
// UTILITAIRES DE CARTE
// ============================================================================

void buildCrossroad(World& w, int cx, int cy, int id, RegulationType regType) {
    w.setTile(cx - 1, cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx - 1, cy,     RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy,     RoadType::INTERSECTION, TileDirection::NONE);

    Intersection inter(id, regType);
    inter.addCoveredTile(sf::Vector2i(cx - 1, cy - 1));
    inter.addCoveredTile(sf::Vector2i(cx,     cy - 1));
    inter.addCoveredTile(sf::Vector2i(cx - 1, cy));
    inter.addCoveredTile(sf::Vector2i(cx,     cy));

    Approach north; north.direction = Approach::Direction::NORTH; north.entryTile = sf::Vector2i(cx, cy - 2); inter.addApproach(north);
    Approach south; south.direction = Approach::Direction::SOUTH; south.entryTile = sf::Vector2i(cx - 1, cy + 1); inter.addApproach(south);
    Approach east;  east.direction = Approach::Direction::EAST;   east.entryTile = sf::Vector2i(cx + 1, cy - 1);  inter.addApproach(east);
    Approach west;  west.direction = Approach::Direction::WEST;   west.entryTile = sf::Vector2i(cx - 2, cy);      inter.addApproach(west);

    w.addIntersection(inter);
}

void buildHRoad(World& world, int y, int xStart, int xEnd) {
    for (int x = xStart; x <= xEnd; ++x) {
        world.setTile(x, y, RoadType::CITY_50, TileDirection::RIGHT);
        world.setTile(x, y - 1, RoadType::CITY_50, TileDirection::LEFT);
    }
}

void buildVRoad(World& world, int x, int yStart, int yEnd) {
    for (int y = yStart; y <= yEnd; ++y) {
        world.setTile(x, y, RoadType::CITY_50, TileDirection::UP);
        world.setTile(x - 1, y, RoadType::CITY_50, TileDirection::DOWN);
    }
}

// ============================================================================
// GESTION DES FICHIERS
// ============================================================================

std::string openFileDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::open_file("Charger une simulation", "", { "Fichiers Texte (*.txt)", "*.txt", "Tous les fichiers", "*" });
    return f.result().empty() ? "" : f.result()[0];
}

std::string saveFileDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::save_file("Sauvegarder la simulation", "", { "Fichiers Texte (*.txt)", "*.txt", "Tous les fichiers", "*" });
    std::string path = f.result();
    if (!path.empty() && path.find(".txt") == std::string::npos) path += ".txt";
    return path;
}

void exportScenario(const std::string& filename, const World& world, const std::vector<std::unique_ptr<IAgent>>& agents) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    file << "W " << world.getGridWidth() << " " << world.getGridHeight() << " " << world.getTileSize() << "\n";

    for (int x = 0; x < world.getGridWidth(); ++x) {
        for (int y = 0; y < world.getGridHeight(); ++y) {
            const Tile& tile = world.getTile(x, y);
            if (tile.roadType != RoadType::NONE) {
                file << "T " << x << " " << y << " " << static_cast<int>(tile.roadType) << " " << static_cast<int>(tile.direction) << "\n";
            }
        }
    }

    for (const auto& inter : world.getIntersections()) {
        if (!inter.getCoveredTiles().empty()) {
            sf::Vector2i firstTile = inter.getCoveredTiles()[0];
            file << "I " << firstTile.x + 1 << " " << firstTile.y + 1 << " " << inter.getId() << " " << static_cast<int>(inter.getType()) << "\n";
        }
    }

    for (const auto& agent : agents) {
        sf::Vector2i start = agent->getStartTile();
        sf::Vector2i goal = agent->getGoalTile();
        file << "A " << agent->getType() << " " << start.x << " " << start.y << " " << goal.x << " " << goal.y << "\n";
    }
    file.close();
}

bool importScenario(const std::string& filename, std::unique_ptr<World>& outWorld, std::vector<std::unique_ptr<IAgent>>& outAgents) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    outAgents.clear();
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        char entryType;
        ss >> entryType;

        if (entryType == 'W') {
            int w, h; float tSize; ss >> w >> h >> tSize;
            outWorld = std::make_unique<World>(w, h, tSize);
        } else if (entryType == 'T' && outWorld) {
            int x, y, rType, dir; ss >> x >> y >> rType >> dir;
            outWorld->setTile(x, y, static_cast<RoadType>(rType), static_cast<TileDirection>(dir));
        } else if (entryType == 'I' && outWorld) {
            int cx, cy, id, rType; ss >> cx >> cy >> id >> rType;
            buildCrossroad(*outWorld, cx, cy, id, static_cast<RegulationType>(rType));
        } else if (entryType == 'A' && outWorld) {
            std::string type; int sx, sy, gx, gy; ss >> type >> sx >> sy >> gx >> gy;
            float tSize = outWorld->getTileSize();
            float px = sx * tSize + tSize / 2.f;
            float py = sy * tSize + tSize / 2.f;

            std::unique_ptr<Vehicle> v;
            if (type == "CAR") v = std::make_unique<Car>(px, py);
            else if (type == "TRUCK") v = std::make_unique<Truck>(px, py);

            if (v) {
                v->setPath(Pathfinder::findPath(*outWorld, {sx, sy}, {gx, gy}));
                outAgents.push_back(std::move(v));
            }
        }
    }
    return true;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    const int WINDOW_W = 800;
    const int WINDOW_H = 800;
    const int GRID_W = 32;
    const int GRID_H = 32;
    const float TILE_SIZE = 50.f;

    sf::RenderWindow window(sf::VideoMode(WINDOW_W, WINDOW_H), "Simulateur MAS");
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    window.setPosition(sf::Vector2i((desktop.width / 2) - WINDOW_W / 2, (desktop.height / 2) - WINDOW_H / 2));
    window.setFramerateLimit(60);

    sf::Font font;
    if (!font.loadFromFile("assets/Roboto-Regular.ttf")) {
        std::cerr << "Erreur: Impossible de charger la police roboto" << std::endl;
    }

    AppState currentState = AppState::MAIN_MENU;
    std::unique_ptr<World> world;
    std::unique_ptr<Camera> camera;
    std::vector<std::unique_ptr<IAgent>> agents;
    sf::Clock clock;

    bool debugMode = false;
    bool buildMode = false;
    BuildTool currentTool = BuildTool::ROAD_CITY_50;

    sf::Text titleText("Simulateur MAS", font, 50);
    titleText.setFillColor(sf::Color::White);
    titleText.setPosition(WINDOW_W / 2.f - titleText.getGlobalBounds().width / 2.f, 150.f);

    float btnW = 300.f, btnH = 60.f, btnX = (WINDOW_W - btnW) / 2.f;
    Button btnNew(btnX, 350.f, btnW, btnH, "Nouvelle Simulation", font);
    Button btnLoad(btnX, 450.f, btnW, btnH, "Charger Simulation", font);
    Button btnExit(btnX, 550.f, btnW, btnH, "Quitter", font);

    sf::FloatRect paletteBounds(5.f, 40.f, 170.f, 210.f);
    sf::FloatRect btnCity50(10, 50, 150, 30);
    sf::FloatRect btnHighway(10, 90, 150, 30);
    sf::FloatRect btnInterPrio(10, 130, 150, 30);
    sf::FloatRect btnInterLight(10, 170, 150, 30);
    sf::FloatRect btnErase(10, 210, 150, 30);

    while (window.isOpen()) {
        sf::Event event;
        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            if (currentState == AppState::MAIN_MENU) {
                if (btnNew.isClicked(event)) {
                    world = std::make_unique<World>(GRID_W, GRID_H, TILE_SIZE);
                    agents.clear();

                    buildHRoad(*world, 11, 0, 31);
                    buildHRoad(*world, 22, 0, 31);
                    buildVRoad(*world, 11, 0, 31);
                    buildVRoad(*world, 22, 0, 31);

                    buildCrossroad(*world, 11, 11, 0, RegulationType::PRIORITY_RIGHT);
                    buildCrossroad(*world, 22, 11, 1, RegulationType::TRAFFIC_LIGHT);
                    buildCrossroad(*world, 11, 22, 2, RegulationType::TRAFFIC_LIGHT);
                    buildCrossroad(*world, 22, 22, 3, RegulationType::PRIORITY_RIGHT);

                    camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, WINDOW_W, WINDOW_H);

                    auto addCar = [&](int sx, int sy, int gx, int gy) {
                        auto c = std::make_unique<Car>(sx * TILE_SIZE + TILE_SIZE / 2.f, sy * TILE_SIZE + TILE_SIZE / 2.f);
                        c->setPath(Pathfinder::findPath(*world, {sx, sy}, {gx, gy}));
                        agents.push_back(std::move(c));
                    };
                    auto addTruck = [&](int sx, int sy, int gx, int gy) {
                        auto t = std::make_unique<Truck>(sx * TILE_SIZE + TILE_SIZE / 2.f, sy * TILE_SIZE + TILE_SIZE / 2.f);
                        t->setPath(Pathfinder::findPath(*world, {sx, sy}, {gx, gy}));
                        agents.push_back(std::move(t));
                    };

                    addCar(1, 11, 21, 30);
                    addTruck(30, 10, 10, 30);
                    addCar(30, 10, 1, 10);
                    addCar(11, 30, 11, 1);
                    addCar(10, 1, 10, 30);
                    addTruck(1, 22, 28, 22);
                    addCar(21, 1, 21, 30);

                    world->redrawMap();
                    currentState = AppState::SIMULATION;
                    clock.restart();
                }
                else if (btnLoad.isClicked(event)) {
                    std::string filepath = openFileDialog();
                    if (!filepath.empty() && importScenario(filepath, world, agents)) {
                        camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, WINDOW_W, WINDOW_H);
                        world->redrawMap();
                        currentState = AppState::SIMULATION;
                        clock.restart();
                    }
                }
                else if (btnExit.isClicked(event)) {
                    window.close();
                }
            }
            else if (currentState == AppState::SIMULATION) {
                if (camera) camera->handleEvent(event, window);

                if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Escape) currentState = AppState::MAIN_MENU;
                    if (event.key.code == sf::Keyboard::B) { buildMode = !buildMode; debugMode = false; }
                    if (event.key.code == sf::Keyboard::D) { debugMode = !debugMode; buildMode = false; for (auto& a : agents) a->setSelected(false); }
                    if (event.key.code == sf::Keyboard::C && world) camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, WINDOW_W, WINDOW_H);

                    // --- NOUVEAU: RESET DE LA SIMULATION (Touche R) ---
                    if (event.key.code == sf::Keyboard::R && world) {
                        for (auto& agent : agents) {
                            agent->resetToStart(*world);
                        }
                    }

                    if (event.key.code == sf::Keyboard::S && world) {
                        std::string filepath = saveFileDialog();
                        if (!filepath.empty()) exportScenario(filepath, *world, agents);
                    }
                }

                if (event.type == sf::Event::MouseButtonPressed) {
                    // C'EST ICI LA CORRECTION MAGIQUE : On utilise la vue par défaut pour mapper le clic UI !
                    sf::Vector2f mPosUI = window.mapPixelToCoords(sf::Mouse::getPosition(window), window.getDefaultView());

                    // Cas 1 : Clic dans la palette UI
                    if (buildMode && paletteBounds.contains(mPosUI)) {
                        if (btnCity50.contains(mPosUI)) currentTool = BuildTool::ROAD_CITY_50;
                        else if (btnHighway.contains(mPosUI)) currentTool = BuildTool::ROAD_HIGHWAY_130;
                        else if (btnInterPrio.contains(mPosUI)) currentTool = BuildTool::INTERSECTION_PRIORITY;
                        else if (btnInterLight.contains(mPosUI)) currentTool = BuildTool::INTERSECTION_TRAFFIC_LIGHT;
                        else if (btnErase.contains(mPosUI)) currentTool = BuildTool::ERASE;
                    }
                    // Cas 2 : Clic sur la carte du Monde
                    else if (buildMode && world && camera) {
                        sf::Vector2f wPos = camera->screenToWorld(window, sf::Mouse::getPosition(window));
                        int gX = (int)(wPos.x / TILE_SIZE);
                        int gY = (int)(wPos.y / TILE_SIZE);
                        bool mapChanged = false;

                        if (event.mouseButton.button == sf::Mouse::Left) {
                            const Tile& currentTile = world->getTile(gX, gY);

                            if (currentTool == BuildTool::ROAD_CITY_50 || currentTool == BuildTool::ROAD_HIGHWAY_130) {
                                RoadType rType = (currentTool == BuildTool::ROAD_CITY_50) ? RoadType::CITY_50 : RoadType::HIGHWAY_130;
                                if (currentTile.roadType == RoadType::NONE) {
                                    world->setTile(gX, gY, rType, TileDirection::UP);
                                } else {
                                    TileDirection nd = currentTile.direction;
                                    if (nd == TileDirection::UP) nd = TileDirection::RIGHT;
                                    else if (nd == TileDirection::RIGHT) nd = TileDirection::DOWN;
                                    else if (nd == TileDirection::DOWN) nd = TileDirection::LEFT;
                                    else if (nd == TileDirection::LEFT) nd = TileDirection::UP;
                                    world->setTile(gX, gY, rType, nd);
                                }
                                mapChanged = true;
                            }
                            // --- REGLES D'INTERSECTION ---
                            else if (currentTool == BuildTool::INTERSECTION_PRIORITY || currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT) {
                                // Règle 1 : Ne pas sortir de la carte avec un 2x2
                                if (gX >= 0 && gX < world->getGridWidth() - 1 && gY >= 0 && gY < world->getGridHeight() - 1) {
                                    // Règle 2 : Exiger qu'il y ait déjà au moins une route existante ici
                                    bool hasRoad = false;
                                    for (int i = 0; i < 2; ++i) {
                                        for (int j = 0; j < 2; ++j) {
                                            if (world->getTile(gX + i, gY + j).roadType != RoadType::NONE) {
                                                hasRoad = true;
                                            }
                                        }
                                    }

                                    if (hasRoad) {
                                        RegulationType reg = (currentTool == BuildTool::INTERSECTION_PRIORITY) ? RegulationType::PRIORITY_RIGHT : RegulationType::TRAFFIC_LIGHT;
                                        buildCrossroad(*world, gX + 1, gY + 1, world->getIntersections().size() + 1, reg);
                                        mapChanged = true;
                                    } else {
                                        std::cout << "Placement refuse : Construisez d'abord des routes !\n";
                                    }
                                }
                            }
                            else if (currentTool == BuildTool::ERASE) {
                                world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                                mapChanged = true;
                            }
                        }
                        else if (event.mouseButton.button == sf::Mouse::Right) {
                            world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                            mapChanged = true;
                        }

                        if (mapChanged) {
                            world->redrawMap();
                            for (auto& agent : agents) agent->recalculatePath(*world);
                        }
                    }
                    else if (debugMode && camera && event.mouseButton.button == sf::Mouse::Left) {
                        sf::Vector2f wPos = camera->screenToWorld(window, sf::Mouse::getPosition(window));
                        for (auto& agent : agents) agent->setSelected(agent->contains(wPos));
                    }
                }
            }
        }

        if (currentState == AppState::MAIN_MENU) {
            btnNew.update(mousePos);
            btnLoad.update(mousePos);
            btnExit.update(mousePos);

            window.clear(sf::Color(40, 40, 40));
            window.draw(titleText);
            btnNew.draw(window);
            btnLoad.draw(window);
            btnExit.draw(window);
        }
        else if (currentState == AppState::SIMULATION) {
            float dt = clock.restart().asSeconds();

            if (world) {
                world->updateIntersections(dt);
                for (auto& agent : agents) agent->update(dt, agents, *world);
            }

            window.clear(sf::Color(34, 139, 34));

            if (camera) camera->applyTo(window);
            if (world) {
                world->draw(window);
                world->drawIntersections(window);
            }
            for (auto& agent : agents) {
                agent->draw(window);
                if (debugMode) agent->drawDebug(window);
            }

            if (buildMode) {
                window.setView(window.getDefaultView());

                sf::RectangleShape bgTitle(sf::Vector2f(500.f, 45.f));
                bgTitle.setFillColor(sf::Color(0, 0, 0, 200));
                bgTitle.setPosition(5.f, 5.f);
                window.draw(bgTitle);

                sf::Text buildText("MODE CONSTRUCTION (B pour quitter - R pour RESET Sim)\nClic Gauche: Utiliser | Clic Droit: Effacer", font, 16);
                buildText.setFillColor(sf::Color::Yellow);
                buildText.setPosition(10.f, 10.f);
                window.draw(buildText);

                sf::RectangleShape bgPalette(sf::Vector2f(170.f, 210.f));
                bgPalette.setFillColor(sf::Color(0, 0, 0, 150));
                bgPalette.setPosition(5.f, 40.f);
                window.draw(bgPalette);

                auto drawPaletteBtn = [&](float y, const std::string& label, BuildTool tool) {
                    sf::RectangleShape btn(sf::Vector2f(150.f, 30.f));
                    btn.setPosition(10.f, y);
                    btn.setFillColor(currentTool == tool ? sf::Color(100, 200, 100) : sf::Color(50, 50, 50));
                    btn.setOutlineThickness(1.f);
                    btn.setOutlineColor(sf::Color::White);

                    sf::Text txt(label, font, 14);
                    txt.setFillColor(currentTool == tool ? sf::Color::Black : sf::Color::White);
                    txt.setPosition(15.f, y + 5.f);
                    window.draw(btn);
                    window.draw(txt);
                };

                drawPaletteBtn(50.f, "Route Ville", BuildTool::ROAD_CITY_50);
                drawPaletteBtn(90.f, "Autoroute", BuildTool::ROAD_HIGHWAY_130);
                drawPaletteBtn(130.f, "Croisement (Prio)", BuildTool::INTERSECTION_PRIORITY);
                drawPaletteBtn(170.f, "Croisement (Feux)", BuildTool::INTERSECTION_TRAFFIC_LIGHT);
                drawPaletteBtn(210.f, "Gomme", BuildTool::ERASE);

                if (camera) camera->applyTo(window);
            }
        }
        window.display();
    }

    return 0;
}