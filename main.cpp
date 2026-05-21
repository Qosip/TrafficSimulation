// main.cpp
#include <SFML/Graphics.hpp>
#include "imgui.h"
#include "imgui-SFML.h"

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

enum class AppState { MAIN_MENU, SIMULATION };
enum class BuildTool { ROAD_CITY_50, ROAD_HIGHWAY_130, INTERSECTION_PRIORITY, INTERSECTION_TRAFFIC_LIGHT, ERASE };

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

std::string openFileDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::open_file("Charger une simulation", "", { "Fichiers Texte (*.txt)", "*.txt" });
    return f.result().empty() ? "" : f.result()[0];
}

std::string saveFileDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::save_file("Sauvegarder la simulation", "", { "Fichiers Texte (*.txt)", "*.txt" });
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
        file << "A " << agent->getType() << " " << agent->getStartTile().x << " " << agent->getStartTile().y << " " << agent->getGoalTile().x << " " << agent->getGoalTile().y << "\n";
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
        char entryType; ss >> entryType;
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
            std::unique_ptr<Vehicle> v;
            if (type == "CAR") v = std::make_unique<Car>(sx * tSize + tSize / 2.f, sy * tSize + tSize / 2.f);
            else if (type == "TRUCK") v = std::make_unique<Truck>(sx * tSize + tSize / 2.f, sy * tSize + tSize / 2.f);
            if (v) {
                v->setPath(Pathfinder::findPath(*outWorld, {sx, sy}, {gx, gy}));
                outAgents.push_back(std::move(v));
            }
        }
    }
    return true;
}

int main() {
    unsigned int winW = 1024;
    unsigned int winH = 800;
    const float PANEL_WIDTH = 280.f;
    const int GRID_W = 32;
    const int GRID_H = 32;
    const float TILE_SIZE = 50.f;

    sf::RenderWindow window(sf::VideoMode(winW, winH), "Simulateur MAS - Panel Pro", sf::Style::Default);
    window.setFramerateLimit(60);

    ImGui::SFML::Init(window);

    AppState currentState = AppState::MAIN_MENU;
    std::unique_ptr<World> world;
    std::unique_ptr<Camera> camera;
    std::vector<std::unique_ptr<IAgent>> agents;

    sf::Clock clock;
    sf::Clock deltaClock;

    // --- NOUVEAU : Horloge pour animer les flux indépendamment de la pause du jeu ---
    sf::Clock globalClock;

    bool buildMode = false;
    bool showFlowDebug = false; // --- NOUVEAU : Variable d'état du bouton ---
    BuildTool currentTool = BuildTool::ROAD_CITY_50;
    bool isPaused = false;
    float simSpeedFactor = 1.0f;

    auto initNewSimulation = [&]() {
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

        camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, (float)winW - PANEL_WIDTH, (float)winH);
        camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);

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
    };

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::Resized) {
                winW = event.size.width;
                winH = event.size.height;
                window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));
                if (camera) camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);
            }

            if (currentState == AppState::SIMULATION) {
                if (sf::Mouse::getPosition(window).x < (int)(winW - PANEL_WIDTH)) {
                    if (camera) camera->handleEvent(event, window);
                }

                if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                    currentState = AppState::MAIN_MENU;
                }

                if (event.type == sf::Event::MouseButtonPressed && sf::Mouse::getPosition(window).x < (int)(winW - PANEL_WIDTH)) {
                    sf::Vector2f wPos = camera->screenToWorld(window, sf::Mouse::getPosition(window));
                    int gX = (int)(wPos.x / TILE_SIZE);
                    int gY = (int)(wPos.y / TILE_SIZE);
                    bool mapChanged = false;

                    if (event.mouseButton.button == sf::Mouse::Left) {
                        if (buildMode && world) {
                            const Tile& tile = world->getTile(gX, gY);
                            if (currentTool == BuildTool::ROAD_CITY_50 || currentTool == BuildTool::ROAD_HIGHWAY_130) {
                                RoadType r = (currentTool == BuildTool::ROAD_CITY_50) ? RoadType::CITY_50 : RoadType::HIGHWAY_130;
                                if (tile.roadType == RoadType::NONE) {
                                    world->setTile(gX, gY, r, TileDirection::UP);
                                } else {
                                    TileDirection nd = tile.direction;
                                    if (nd == TileDirection::UP) nd = TileDirection::RIGHT;
                                    else if (nd == TileDirection::RIGHT) nd = TileDirection::DOWN;
                                    else if (nd == TileDirection::DOWN) nd = TileDirection::LEFT;
                                    else if (nd == TileDirection::LEFT) nd = TileDirection::UP;
                                    world->setTile(gX, gY, r, nd);
                                }
                                mapChanged = true;
                            }
                            else if (currentTool == BuildTool::INTERSECTION_PRIORITY || currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT) {
                                if (gX >= 0 && gX < world->getGridWidth() - 1 && gY >= 0 && gY < world->getGridHeight() - 1) {
                                    bool hasRoad = false;
                                    for(int i=0;i<2;++i)
                                        for(int j=0;j<2;++j)
                                            if(world->getTile(gX+i,gY+j).roadType != RoadType::NONE) hasRoad = true;
                                    if (hasRoad) {
                                        RegulationType reg = (currentTool == BuildTool::INTERSECTION_PRIORITY) ? RegulationType::PRIORITY_RIGHT : RegulationType::TRAFFIC_LIGHT;
                                        buildCrossroad(*world, gX + 1, gY + 1, world->getIntersections().size() + 1, reg);
                                        mapChanged = true;
                                    }
                                }
                            }
                            else if (currentTool == BuildTool::ERASE) {
                                world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                                mapChanged = true;
                            }
                        } else {
                            for (auto& agent : agents) {
                                if (agent->contains(wPos)) {
                                    agent->setSelected(!agent->isSelected());
                                    break;
                                }
                            }
                        }
                    }
                    else if (event.mouseButton.button == sf::Mouse::Right && buildMode) {
                        world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                        mapChanged = true;
                    }

                    if (mapChanged) {
                        world->redrawMap();
                        for (auto& agent : agents) agent->recalculatePath(*world);
                    }
                }
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());
        window.clear(sf::Color(30, 30, 30));

        if (currentState == AppState::MAIN_MENU) {
            window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));

            ImGui::SetNextWindowPos(ImVec2(winW / 2.f - 200.f, winH / 2.f - 150.f));
            ImGui::SetNextWindowSize(ImVec2(400.f, 300.f));
            ImGui::Begin("Menu Principal - MAS", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            ImGui::Text("Simulateur Multi-Agents Auto");
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("Nouvelle Simulation", ImVec2(-1.f, 45.f))) initNewSimulation();
            if (ImGui::Button("Charger un Scenario (TXT)", ImVec2(-1.f, 45.f))) {
                std::string path = openFileDialog();
                if (!path.empty() && importScenario(path, world, agents)) {
                    camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, (float)winW - PANEL_WIDTH, (float)winH);
                    camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);
                    world->redrawMap();
                    currentState = AppState::SIMULATION;
                    clock.restart();
                }
            }
            if (ImGui::Button("Quitter l'Application", ImVec2(-1.f, 45.f))) window.close();

            ImGui::End();
        }
        else if (currentState == AppState::SIMULATION) {
            float dt = isPaused ? 0.f : clock.restart().asSeconds() * simSpeedFactor;

            if (world && !isPaused) {
                world->updateIntersections(dt);
                for (auto& agent : agents) agent->update(dt, agents, *world);
            } else if (isPaused) {
                clock.restart();
            }

            if (camera) camera->applyTo(window);
            if (world) {
                world->draw(window);

                // --- NOUVEAU : Appel du dessin des flux dynamiques par-dessus la route ---
                if (showFlowDebug) {
                    world->drawFlowDebug(window, globalClock.getElapsedTime().asSeconds());
                }

                world->drawIntersections(window);
            }
            for (auto& agent : agents) {
                agent->draw(window);
                agent->drawDebug(window);
            }

            window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));

            ImGui::SetNextWindowPos(ImVec2(winW - PANEL_WIDTH, 0.f));
            ImGui::SetNextWindowSize(ImVec2(PANEL_WIDTH, (float)winH));
            ImGui::Begin("Dashboard de Controle", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            if (ImGui::CollapsingHeader("Fichiers & Options", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("Sauvegarder (S)", ImVec2(-1.f, 30.f))) {
                    std::string path = saveFileDialog();
                    if (!path.empty()) exportScenario(path, *world, agents);
                }
                if (ImGui::Button("Reset Simulation (R)", ImVec2(-1.f, 30.f))) {
                    for (auto& a : agents) a->resetToStart(*world);
                }
                ImGui::Separator();
                ImGui::Checkbox("Pause Generale", &isPaused);
                ImGui::SliderFloat("Vitesse Sim", &simSpeedFactor, 0.2f, 3.0f, "%.1fx");
            }

            if (ImGui::CollapsingHeader("Palette d'Edition", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Checkbox("Activer Mode Construction (B)", &buildMode);

                // --- NOUVEAU : La Checkbox pour activer la vision cyberpunk ---
                ImGui::Checkbox("Afficher les Flux (Debug)", &showFlowDebug);

                if (buildMode) {
                    ImGui::TextColored(ImVec4(1.f, 1.f, 0.f, 1.f), "Outil de pose selectionne :");
                    if (ImGui::RadioButton("Route standard (50)", currentTool == BuildTool::ROAD_CITY_50)) currentTool = BuildTool::ROAD_CITY_50;
                    if (ImGui::RadioButton("Autoroute (130)", currentTool == BuildTool::ROAD_HIGHWAY_130)) currentTool = BuildTool::ROAD_HIGHWAY_130;
                    if (ImGui::RadioButton("Carrefour Priorite", currentTool == BuildTool::INTERSECTION_PRIORITY)) currentTool = BuildTool::INTERSECTION_PRIORITY;
                    if (ImGui::RadioButton("Carrefour Feux", currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT)) currentTool = BuildTool::INTERSECTION_TRAFFIC_LIGHT;
                    if (ImGui::RadioButton("Gomme / Effacer", currentTool == BuildTool::ERASE)) currentTool = BuildTool::ERASE;
                }
            }

            if (ImGui::CollapsingHeader("Flotte de Vehicules", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("Cacher tous les traces", ImVec2(-1.f, 25.f))) {
                    for (auto& a : agents) a->setSelected(false);
                }
                ImGui::Separator();

                int vehIndex = 1;
                for (auto& agent : agents) {
                    ImGui::PushID(vehIndex);

                    std::string vehName = agent->getType() + " #" + std::to_string(vehIndex);

                    bool isSel = agent->isSelected();
                    if (ImGui::Checkbox("##sel", &isSel)) {
                        agent->setSelected(isSel);
                    }
                    ImGui::SameLine();

                    if (ImGui::TreeNode(vehName.c_str())) {
                        ImGui::Text("Vitesse : %d px/s", (int)agent->getSpeed());
                        ImGui::Text("Position : [%d, %d]", (int)agent->getPosition().x, (int)agent->getPosition().y);
                        ImGui::Text("Destination : [%d, %d]", agent->getGoalTile().x, agent->getGoalTile().y);

                        float remDist = agent->getRemainingDistance();

                        if (agent->getCurrentTile() == agent->getGoalTile()) {
                            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.f), "Statut : Arrive a destination");
                        }
                        else if (remDist <= 0.f) {
                            ImGui::TextColored(ImVec4(1.f, 0.2f, 0.2f, 1.f), "Statut : PERDU (Route coupee)");
                        }
                        else if (agent->getSpeed() > 5.f) {
                            ImGui::TextColored(ImVec4(0.f, 1.f, 0.f, 1.f), "ETA estimer : %.1f sec", remDist / agent->getSpeed());
                        }
                        else {
                            ImGui::TextColored(ImVec4(1.f, 0.5f, 0.f, 1.f), "Statut : Bloque au trafic");
                        }

                        if(ImGui::Button("Forcer Demi-Tour", ImVec2(-1.f, 25.f))) {
                            agent->recalculatePath(*world);
                        }
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                    vehIndex++;
                }
            }

            if (ImGui::Button("Retour Menu (Echap)", ImVec2(-1.f, 35.f))) currentState = AppState::MAIN_MENU;

            ImGui::End();
        }

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
    return 0;
}