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
#include "core/agent/Personality.hpp"
#include "core/agent/Truck.hpp"
#include "core/agent/Vehicle.hpp"
#include "core/world/World.hpp"
#include "render/Camera.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "io/SceneBuilder.hpp"
#include "io/ScenarioIO.hpp"
#include "render/SfmlRenderer.hpp"
#include "render/SfmlInterop.hpp"

enum class AppState { MAIN_MENU, SIMULATION };
enum class BuildTool {
    ROAD_CITY_50,
    ROAD_HIGHWAY_130,
    INTERSECTION_PRIORITY,
    INTERSECTION_TRAFFIC_LIGHT,
    INTERSECTION_STOP,
    INTERSECTION_ROUNDABOUT,
    ERASE
};

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

    // Curseur OS toujours visible. ImGui-SFML peut le masquer involontairement
    // quand le pointeur est hors widget ; on lui demande de ne plus toucher
    // au curseur et on le force visible cote SFML.
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    ImGui::GetIO().MouseDrawCursor = false;
    window.setMouseCursorVisible(true);

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
    int  roundaboutRadius = 3;   // rayon en tiles (>= 2). 2 => mini, 3 => 5x5, 4 => 7x7.
    bool isPaused = false;
    float simSpeedFactor = 1.0f;

    // --- Mode build interactif ---
    bool draggingRoad  = false;    // trace de route par glisser en cours
    int  dragPrevX = 0, dragPrevY = 0;
    bool pendingRebuild = false;   // map modifiee -> recalcul paths a la fin du geste
    int  hoverX = -1, hoverY = -1; // tile survolee (apercu fantome)
    bool hoverValid = false;

    auto brushRoadType = [&]() {
        return (currentTool == BuildTool::ROAD_HIGHWAY_130) ? RoadType::HIGHWAY_130
                                                            : RoadType::CITY_50;
    };
    // Pose une tile de route en orientant selon 'dir'. N'ecrase jamais un
    // carrefour/rond-point existant (evite les structures orphelines).
    auto paintRoadTile = [&](int gx, int gy, TileDirection dir) {
        if (!world) return;
        if (gx < 0 || gx >= world->getGridWidth() || gy < 0 || gy >= world->getGridHeight()) return;
        if (world->getTile(gx, gy).roadType == RoadType::INTERSECTION) return;
        world->setTile(gx, gy, brushRoadType(), dir);
        pendingRebuild = true;
    };
    // Trace une ligne de tiles entre deux cases avec direction auto (sens du
    // glissement). Axe dominant => evite les diagonales en escalier.
    auto paintRoadLine = [&](int x0, int y0, int x1, int y1) {
        const int adx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
        const int ady = (y1 > y0) ? (y1 - y0) : (y0 - y1);
        if (adx >= ady) {
            const TileDirection dir = (x1 >= x0) ? TileDirection::RIGHT : TileDirection::LEFT;
            const int step = (x1 >= x0) ? 1 : -1;
            for (int x = x0; x != x1 + step; x += step) paintRoadTile(x, y0, dir);
        } else {
            const TileDirection dir = (y1 >= y0) ? TileDirection::DOWN : TileDirection::UP;
            const int step = (y1 >= y0) ? 1 : -1;
            for (int y = y0; y != y1 + step; y += step) paintRoadTile(x0, y, dir);
        }
    };

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

                if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Escape) {
                        currentState = AppState::MAIN_MENU;
                    }
                    // Raccourcis outils (ignores si une saisie ImGui a le focus).
                    else if (!ImGui::GetIO().WantCaptureKeyboard) {
                        switch (event.key.code) {
                            case sf::Keyboard::B:    buildMode = !buildMode; break;
                            case sf::Keyboard::Num1: currentTool = BuildTool::ROAD_CITY_50; break;
                            case sf::Keyboard::Num2: currentTool = BuildTool::ROAD_HIGHWAY_130; break;
                            case sf::Keyboard::Num3: currentTool = BuildTool::INTERSECTION_PRIORITY; break;
                            case sf::Keyboard::Num4: currentTool = BuildTool::INTERSECTION_TRAFFIC_LIGHT; break;
                            case sf::Keyboard::Num5: currentTool = BuildTool::INTERSECTION_STOP; break;
                            case sf::Keyboard::Num6: currentTool = BuildTool::INTERSECTION_ROUNDABOUT; break;
                            case sf::Keyboard::Num7:
                            case sf::Keyboard::Delete: currentTool = BuildTool::ERASE; break;
                            default: break;
                        }
                    }
                }

                const bool inSimArea = sf::Mouse::getPosition(window).x < (int)(winW - PANEL_WIDTH);

                if (event.type == sf::Event::MouseButtonPressed && inSimArea && camera) {
                    sf::Vector2f wPos = camera->screenToWorld(window, sf::Mouse::getPosition(window));
                    int gX = (int)(wPos.x / TILE_SIZE);
                    int gY = (int)(wPos.y / TILE_SIZE);

                    if (event.mouseButton.button == sf::Mouse::Left) {
                        if (buildMode && world) {
                            if (currentTool == BuildTool::ROAD_CITY_50 || currentTool == BuildTool::ROAD_HIGHWAY_130) {
                                // Glisser pour tracer : la direction suit le mouvement.
                                draggingRoad = true;
                                dragPrevX = gX; dragPrevY = gY;
                                const Tile& ex = world->getTile(gX, gY);
                                const TileDirection d0 =
                                    (ex.roadType == RoadType::CITY_50 || ex.roadType == RoadType::HIGHWAY_130)
                                    ? ex.direction : TileDirection::UP;
                                paintRoadTile(gX, gY, d0);
                            }
                            else if (currentTool == BuildTool::INTERSECTION_PRIORITY ||
                                     currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT ||
                                     currentTool == BuildTool::INTERSECTION_STOP ||
                                     currentTool == BuildTool::INTERSECTION_ROUNDABOUT) {
                                if (gX >= 0 && gX < world->getGridWidth() - 1 && gY >= 0 && gY < world->getGridHeight() - 1) {
                                    bool hasRoad = false;
                                    for(int i=0;i<2;++i)
                                        for(int j=0;j<2;++j)
                                            if(world->getTile(gX+i,gY+j).roadType != RoadType::NONE) hasRoad = true;
                                    if (hasRoad) {
                                        const int newId = world->getIntersections().size() + 1;
                                        if (currentTool == BuildTool::INTERSECTION_ROUNDABOUT && roundaboutRadius >= 3) {
                                            scene::buildRoundabout(*world, gX + 1, gY + 1, newId, roundaboutRadius);
                                        } else {
                                            RegulationType reg = RegulationType::PRIORITY_RIGHT;
                                            switch (currentTool) {
                                                case BuildTool::INTERSECTION_TRAFFIC_LIGHT: reg = RegulationType::TRAFFIC_LIGHT; break;
                                                case BuildTool::INTERSECTION_STOP:          reg = RegulationType::STOP; break;
                                                case BuildTool::INTERSECTION_ROUNDABOUT:    reg = RegulationType::ROUNDABOUT; break;
                                                default: break;
                                            }
                                            scene::buildCrossroad(*world, gX + 1, gY + 1, newId, reg);
                                        }
                                        pendingRebuild = true;
                                    }
                                }
                            }
                            else if (currentTool == BuildTool::ERASE) {
                                world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                                pendingRebuild = true;
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
                    else if (event.mouseButton.button == sf::Mouse::Right && buildMode && world) {
                        world->setTile(gX, gY, RoadType::NONE, TileDirection::NONE);
                        pendingRebuild = true;
                    }
                }
                else if (event.type == sf::Event::MouseMoved && draggingRoad && buildMode && world && camera &&
                         (currentTool == BuildTool::ROAD_CITY_50 || currentTool == BuildTool::ROAD_HIGHWAY_130)) {
                    sf::Vector2f wPos = camera->screenToWorld(window, sf::Vector2i(event.mouseMove.x, event.mouseMove.y));
                    int gX = (int)(wPos.x / TILE_SIZE);
                    int gY = (int)(wPos.y / TILE_SIZE);
                    if (gX != dragPrevX || gY != dragPrevY) {
                        paintRoadLine(dragPrevX, dragPrevY, gX, gY);
                        dragPrevX = gX; dragPrevY = gY;
                    }
                }
                else if (event.type == sf::Event::MouseButtonReleased) {
                    if (event.mouseButton.button == sf::Mouse::Left) draggingRoad = false;
                    // Recalcul des chemins une seule fois, a la fin du geste.
                    if (pendingRebuild && world) {
                        renderer.invalidateMapCache();
                        for (auto& agent : agents) agent->recalculatePath(*world);
                        pendingRebuild = false;
                    }
                }
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());
        window.clear(sf::Color(30, 30, 30));

        if (currentState == AppState::MAIN_MENU) {
            window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));

            ImGui::SetNextWindowPos(ImVec2(winW / 2.f - 200.f, winH / 2.f - 180.f));
            ImGui::SetNextWindowSize(ImVec2(400.f, 360.f));
            ImGui::Begin("Menu Principal - MAS", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            ImGui::Text("Simulateur Multi-Agents Auto");
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("Nouvelle Simulation", ImVec2(-1.f, 45.f))) initNewSimulation();
            if (ImGui::Button("Demo Comportements (rond-point / depassement / STOP)", ImVec2(-1.f, 45.f))) {
                scene::buildDemoScenario(world, agents, TILE_SIZE);
                if (world) {
                    camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f, world->getWorldPixelHeight() / 2.f, (float)winW - PANEL_WIDTH, (float)winH);
                    camera->updateViewport((float)winW, (float)winH, PANEL_WIDTH);
                    renderer.invalidateMapCache();
                    currentState = AppState::SIMULATION;
                    clock.restart();
                }
            }
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

            // --- Apercu de construction (fantome + surbrillance de la tile) ---
            hoverValid = false;
            if (buildMode && world && camera) {
                const sf::Vector2i mp = sf::Mouse::getPosition(window);
                if (mp.x < (int)(winW - PANEL_WIDTH)) {
                    const sf::Vector2f wp = camera->screenToWorld(window, mp);
                    hoverX = (int)(wp.x / TILE_SIZE);
                    hoverY = (int)(wp.y / TILE_SIZE);
                    if (hoverX >= 0 && hoverX < world->getGridWidth() &&
                        hoverY >= 0 && hoverY < world->getGridHeight()) {
                        hoverValid = true;
                        int tlx = hoverX, tly = hoverY, gw = 1, gh = 1;
                        core::Color col{200, 200, 200, 70};
                        switch (currentTool) {
                            case BuildTool::ROAD_CITY_50:     col = core::Color{120, 120, 130, 90}; break;
                            case BuildTool::ROAD_HIGHWAY_130: col = core::Color{90, 90, 120, 90};  break;
                            case BuildTool::INTERSECTION_PRIORITY:
                            case BuildTool::INTERSECTION_TRAFFIC_LIGHT:
                            case BuildTool::INTERSECTION_STOP:
                                gw = 2; gh = 2; col = core::Color{255, 210, 40, 70}; break;
                            case BuildTool::INTERSECTION_ROUNDABOUT:
                                if (roundaboutRadius >= 3) {
                                    const int side = 2 * roundaboutRadius - 1;
                                    tlx = hoverX + 2 - roundaboutRadius;
                                    tly = hoverY + 2 - roundaboutRadius;
                                    gw = side; gh = side;
                                } else { gw = 2; gh = 2; }
                                col = core::Color{60, 160, 255, 70};
                                break;
                            case BuildTool::ERASE: col = core::Color{255, 60, 60, 80}; break;
                        }
                        renderer.drawBuildFootprint(tlx, tly, gw, gh, TILE_SIZE, col);
                        renderer.drawHoverHighlight(hoverX, hoverY, TILE_SIZE);
                    }
                }
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
                    if (ImGui::RadioButton("Route standard (50)   [1]", currentTool == BuildTool::ROAD_CITY_50)) currentTool = BuildTool::ROAD_CITY_50;
                    if (ImGui::RadioButton("Autoroute (130)       [2]", currentTool == BuildTool::ROAD_HIGHWAY_130)) currentTool = BuildTool::ROAD_HIGHWAY_130;
                    if (ImGui::RadioButton("Carrefour Priorite    [3]", currentTool == BuildTool::INTERSECTION_PRIORITY)) currentTool = BuildTool::INTERSECTION_PRIORITY;
                    if (ImGui::RadioButton("Carrefour Feux        [4]", currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT)) currentTool = BuildTool::INTERSECTION_TRAFFIC_LIGHT;
                    if (ImGui::RadioButton("Carrefour STOP        [5]", currentTool == BuildTool::INTERSECTION_STOP)) currentTool = BuildTool::INTERSECTION_STOP;
                    if (ImGui::RadioButton("Rond-point            [6]", currentTool == BuildTool::INTERSECTION_ROUNDABOUT)) currentTool = BuildTool::INTERSECTION_ROUNDABOUT;
                    if (currentTool == BuildTool::INTERSECTION_ROUNDABOUT) {
                        ImGui::SliderInt("Rayon RDP (tiles)", &roundaboutRadius, 2, 4);
                        ImGui::TextDisabled("2 = mini 2x2, 3 = 5x5 anneau, 4 = 7x7 anneau");
                    }
                    if (ImGui::RadioButton("Gomme / Effacer       [7]", currentTool == BuildTool::ERASE)) currentTool = BuildTool::ERASE;

                    ImGui::Separator();
                    ImGui::TextDisabled("Glisser = tracer (direction auto)");
                    ImGui::TextDisabled("Clic droit = effacer  |  B = build");

                    // Info de la tile survolee (apercu).
                    if (hoverValid && world) {
                        const Tile& ht = world->getTile(hoverX, hoverY);
                        const char* rt = "vide";
                        switch (ht.roadType) {
                            case RoadType::CITY_50:      rt = "route 50"; break;
                            case RoadType::HIGHWAY_130:  rt = "autoroute"; break;
                            case RoadType::INTERSECTION: rt = "intersection"; break;
                            case RoadType::CITY_30:      rt = "route 30"; break;
                            case RoadType::ROAD_80:      rt = "route 80"; break;
                            default: break;
                        }
                        const char* dir = "-";
                        switch (ht.direction) {
                            case TileDirection::UP:    dir = "haut"; break;
                            case TileDirection::DOWN:  dir = "bas"; break;
                            case TileDirection::LEFT:  dir = "gauche"; break;
                            case TileDirection::RIGHT: dir = "droite"; break;
                            default: break;
                        }
                        ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.f, 1.f),
                                           "Tile [%d,%d] : %s (%s)", hoverX, hoverY, rt, dir);
                    } else {
                        ImGui::TextDisabled("Survolez la carte...");
                    }
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

                        // --- Section Personnalite / Panne (cast vers Vehicle) ---
                        if (auto* veh = dynamic_cast<Vehicle*>(agent.get())) {
                            auto& perso = veh->personality();
                            if (ImGui::TreeNode("Personnalite")) {
                                bool dirty = false;
                                dirty |= ImGui::SliderFloat("Vitesse compliance",
                                    &perso.speedComplianceFactor, 0.5f, 1.5f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Reactivite (1/T)",
                                    &perso.reactionTimeFactor, 0.5f, 2.0f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Distance mini (s0)",
                                    &perso.minGapFactor, 0.5f, 2.0f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Eagerness accel",
                                    &perso.accelEagernessFactor, 0.5f, 1.5f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Decel confort",
                                    &perso.comfortBrakeFactor, 0.5f, 1.5f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Patience inter.",
                                    &perso.intersectionPatience, 0.5f, 2.0f, "%.2fx");
                                dirty |= ImGui::SliderFloat("Envie depasser",
                                    &perso.overtakeWillingness, 0.0f, 1.0f, "%.2f");
                                dirty |= ImGui::SliderFloat("Ratio leader",
                                    &perso.overtakeLeaderRatio, 0.3f, 0.95f, "%.2f");
                                dirty |= ImGui::SliderFloat("Risque panne /min",
                                    &perso.breakdownChancePerMin, 0.0f, 0.1f, "%.3f");
                                dirty |= ImGui::SliderFloat("Duree panne (s)",
                                    &perso.breakdownDurationSec, 1.f, 60.f, "%.0fs");

                                if (dirty) veh->setPersonality(perso);

                                ImGui::Separator();
                                ImGui::Text("Profils preetablis :");
                                if (ImGui::SmallButton("Calme")) {
                                    veh->setPersonality(core::agent::profiles::calmDriver());
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Normal")) {
                                    veh->setPersonality(core::agent::profiles::normalDriver());
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Agressif")) {
                                    veh->setPersonality(core::agent::profiles::aggressiveDriver());
                                }
                                ImGui::TreePop();
                            }

                            // Panne : indicateur + boutons debug.
                            if (veh->brokenDown()) {
                                ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f),
                                                   "PANNE : %.1f sec restantes",
                                                   veh->repairTimeRemaining());
                                if (ImGui::Button("Reparer maintenant", ImVec2(-1.f, 22.f))) {
                                    veh->forceRepair();
                                }
                            } else {
                                if (ImGui::Button("Forcer panne", ImVec2(-1.f, 22.f))) {
                                    veh->forceBreakdown(perso.breakdownDurationSec);
                                }
                            }
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