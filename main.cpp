// main.cpp
#include <SFML/Graphics.hpp>
#include "imgui.h"
#include "imgui-SFML.h"

#include <vector>
#include <memory>
#include <iostream>
#include <string>

#include "portable-file-dialogs.h"
#include "core/agent/BlockReason.hpp"
#include "core/agent/Car.hpp"
#include "core/agent/Truck.hpp"
#include "core/world/World.hpp"
#include "render/Camera.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "io/SceneBuilder.hpp"
#include "io/ScenarioIO.hpp"
#include "render/SfmlRenderer.hpp"
#include "render/SfmlInterop.hpp"

enum class AppState { MAIN_MENU, SIMULATION };
enum class BuildTool { ROAD_CITY_50, ROAD_HIGHWAY_130, INTERSECTION_PRIORITY, INTERSECTION_TRAFFIC_LIGHT, ERASE };

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

    // Etape 3 du refactor : un unique point de rendu SFML pour toute l'app.
    render::SfmlRenderer renderer(window);

    AppState currentState = AppState::MAIN_MENU;
    std::unique_ptr<World> world;
    std::unique_ptr<Camera> camera;
    std::vector<std::unique_ptr<IAgent>> agents;

    sf::Clock clock;
    sf::Clock deltaClock;

    // --- NOUVEAU : Horloge pour animer les flux indépendamment de la pause du jeu ---
    sf::Clock globalClock;

    // Wave 5 : pas de simulation fixe. Decouple le solver IDM du framerate.
    constexpr float FIXED_DT = 1.0f / 60.0f;  // 60 Hz logique
    constexpr int   MAX_SUBSTEPS = 5;         // anti-spirale en cas de stall
    float           simAccumulator = 0.f;

    bool buildMode = false;
    bool showFlowDebug = false; // --- NOUVEAU : Variable d'état du bouton ---
    BuildTool currentTool = BuildTool::ROAD_CITY_50;
    bool isPaused = false;
    float simSpeedFactor = 1.0f;

    auto initNewSimulation = [&]() {
        world = std::make_unique<World>(GRID_W, GRID_H, TILE_SIZE);
        agents.clear();

        scene::buildDefaultNetwork(*world);

        camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, (float)winW - PANEL_WIDTH, (float)winH);
        camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);

        scene::spawnDefaultAgents(agents, *world);

        renderer.invalidateMapCache();
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
                                        scene::buildCrossroad(*world, gX + 1, gY + 1, world->getIntersections().size() + 1, reg);
                                        mapChanged = true;
                                    }
                                }
                            }
                            else if (currentTool == BuildTool::ERASE) {
                                world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                                mapChanged = true;
                            }
                        } else {
                            const core::Vec2 wPosCore = render::toCore(wPos);
                            for (auto& agent : agents) {
                                if (agent->contains(wPosCore)) {
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
                        renderer.invalidateMapCache();
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
                if (!path.empty() && io::importScenario(path, world, agents)) {
                    camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, (float)winW - PANEL_WIDTH, (float)winH);
                    camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);
                    renderer.invalidateMapCache();
                    currentState = AppState::SIMULATION;
                    clock.restart();
                }
            }
            if (ImGui::Button("Quitter l'Application", ImVec2(-1.f, 45.f))) window.close();

            ImGui::End();
        }
        else if (currentState == AppState::SIMULATION) {
            // Wave 5 : pas fixe + accumulateur. Garantit que IDM tourne sur un dt
            // constant quel que soit le framerate (stabilite numerique Euler).
            const float frameTime = isPaused ? 0.f : clock.restart().asSeconds() * simSpeedFactor;
            if (isPaused) { clock.restart(); simAccumulator = 0.f; }

            if (world && !isPaused) {
                simAccumulator += frameTime;
                int steps = 0;
                while (simAccumulator >= FIXED_DT && steps < MAX_SUBSTEPS) {
                    world->updateIntersections(FIXED_DT);
                    for (auto& agent : agents) agent->computeDecision(agents, *world);
                    for (auto& agent : agents) agent->integrate(FIXED_DT);
                    simAccumulator -= FIXED_DT;
                    ++steps;
                }
                // Si le frame a accumule trop de retard, on jette le surplus
                // pour eviter le scenario "spirale de la mort".
                if (steps >= MAX_SUBSTEPS) simAccumulator = 0.f;
            }

            if (camera) camera->applyTo(window);
            if (world) {
                renderer.drawWorldMap(*world);

                if (showFlowDebug) {
                    renderer.drawFlowDebug(*world, globalClock.getElapsedTime().asSeconds());
                }

                renderer.drawIntersections(*world);
            }
            for (auto& agent : agents) {
                renderer.drawAgent(*agent);
                renderer.drawAgentDebug(*agent);
            }

            window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));

            ImGui::SetNextWindowPos(ImVec2(winW - PANEL_WIDTH, 0.f));
            ImGui::SetNextWindowSize(ImVec2(PANEL_WIDTH, (float)winH));
            ImGui::Begin("Dashboard de Controle", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            if (ImGui::CollapsingHeader("Fichiers & Options", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("Sauvegarder (S)", ImVec2(-1.f, 30.f))) {
                    std::string path = saveFileDialog();
                    if (!path.empty()) io::exportScenario(path, *world, agents);
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
                        const auto reason = agent->getBlockReason();

                        // ETA seulement quand on roule reellement.
                        if (agent->getSpeed() > 5.f && reason == core::agent::BlockReason::NONE) {
                            ImGui::TextColored(ImVec4(0.f, 1.f, 0.f, 1.f),
                                               "ETA estimer : %.1f sec",
                                               remDist / agent->getSpeed());
                        }

                        // Couleur en fonction de la severite du motif.
                        ImVec4 color;
                        switch (reason) {
                            case core::agent::BlockReason::NONE:
                                color = ImVec4(0.f, 1.f, 0.f, 1.f); break;
                            case core::agent::BlockReason::AT_GOAL:
                                color = ImVec4(0.5f, 0.5f, 0.5f, 1.f); break;
                            case core::agent::BlockReason::NO_PATH:
                                color = ImVec4(1.f, 0.2f, 0.2f, 1.f); break;
                            case core::agent::BlockReason::INTERSECTION_RED:
                                color = ImVec4(1.f, 0.3f, 0.3f, 1.f); break;
                            case core::agent::BlockReason::INTERSECTION_YIELD:
                                color = ImVec4(1.f, 0.8f, 0.f, 1.f); break;
                            case core::agent::BlockReason::LEADER_VEHICLE:
                                color = ImVec4(1.f, 0.5f, 0.f, 1.f); break;
                            case core::agent::BlockReason::CORNERING:
                                color = ImVec4(0.6f, 0.8f, 1.f, 1.f); break;
                            case core::agent::BlockReason::INITIALIZING:
                                color = ImVec4(0.8f, 0.8f, 0.8f, 1.f); break;
                        }
                        ImGui::TextColored(color, "Statut : %s", core::agent::toString(reason));

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