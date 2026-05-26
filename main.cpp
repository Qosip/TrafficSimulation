// main.cpp
#include <SFML/Graphics.hpp>
#include "imgui.h"
#include "imgui-SFML.h"

#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstdio>

#include "portable-file-dialogs.h"
#include "core/agent/BlockReason.hpp"
#include "core/agent/Car.hpp"
#include "core/agent/Personality.hpp"
#include "core/agent/Truck.hpp"
#include "core/agent/Vehicle.hpp"
#include "core/math/Rng.hpp"
#include "core/metrics/MetricsCollector.hpp"
#include "core/world/World.hpp"
#include "sim/ExperimentRunner.hpp"
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
    INTERSECTION_FIXED_PRIORITY,   // Strategie "recherche" : priorite fixe par axe
    INTERSECTION_P2P,              // Strategie "recherche" : negociation P2P
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

std::string saveCsvDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::save_file("Exporter les metriques (CSV)", "metriques.csv",
                            { "Fichiers CSV (*.csv)", "*.csv" });
    std::string path = f.result();
    if (!path.empty() && path.find(".csv") == std::string::npos) path += ".csv";
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

    // Banc d'essai quantitatif : collecte les metriques de recherche en direct.
    core::metrics::MetricsCollector metrics;

    // Resultats de la derniere experience Monte-Carlo headless (Fixed vs P2P...).
    std::vector<sim::ResultRow> expResults;
    int  expDurationSec = 60;
    int  expRuns        = 1;
    bool expStratFixed  = true;
    bool expStratP2P    = true;
    bool expStratAim    = false;

    // RNG d'UI (heterogeneite gaussienne appliquee a la flotte a la demande).
    core::Rng uiRng(0xA11CEULL);

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
    bool showDecisions = false; // Overlay pedagogique des decisions sur la sim
    BuildTool currentTool = BuildTool::ROAD_CITY_50;
    int  roundaboutSide = 4;     // cote en tiles, PAIR (2,4,6,8). 2 = mini, 4 = anneau 4x4...
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
        metrics.reset();
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
                            case sf::Keyboard::Num7: currentTool = BuildTool::INTERSECTION_FIXED_PRIORITY; break;
                            case sf::Keyboard::Num8: currentTool = BuildTool::INTERSECTION_P2P; break;
                            case sf::Keyboard::Num9:
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
                            else if (currentTool == BuildTool::INTERSECTION_ROUNDABOUT) {
                                // Rond-point : cote PAIR, coin haut-gauche = tile cliquee.
                                int side = roundaboutSide;
                                if (side < 2)      side = 2;
                                if (side % 2 != 0) ++side;
                                if (gX >= 0 && gY >= 0 &&
                                    gX + side <= world->getGridWidth() &&
                                    gY + side <= world->getGridHeight()) {
                                    bool hasRoad = false;
                                    for (int i = 0; i < side; ++i)
                                        for (int j = 0; j < side; ++j)
                                            if (world->getTile(gX+i, gY+j).roadType != RoadType::NONE) hasRoad = true;
                                    if (hasRoad) {
                                        const int newId = world->getIntersections().size() + 1;
                                        scene::buildRoundabout(*world, gX, gY, newId, side);
                                        pendingRebuild = true;
                                    }
                                }
                            }
                            else if (currentTool == BuildTool::INTERSECTION_PRIORITY ||
                                     currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT ||
                                     currentTool == BuildTool::INTERSECTION_STOP ||
                                     currentTool == BuildTool::INTERSECTION_FIXED_PRIORITY ||
                                     currentTool == BuildTool::INTERSECTION_P2P) {
                                if (gX >= 0 && gX < world->getGridWidth() - 1 && gY >= 0 && gY < world->getGridHeight() - 1) {
                                    bool hasRoad = false;
                                    for(int i=0;i<2;++i)
                                        for(int j=0;j<2;++j)
                                            if(world->getTile(gX+i,gY+j).roadType != RoadType::NONE) hasRoad = true;
                                    if (hasRoad) {
                                        const int newId = world->getIntersections().size() + 1;
                                        RegulationType reg = RegulationType::PRIORITY_RIGHT;
                                        switch (currentTool) {
                                            case BuildTool::INTERSECTION_TRAFFIC_LIGHT:  reg = RegulationType::TRAFFIC_LIGHT;  break;
                                            case BuildTool::INTERSECTION_STOP:           reg = RegulationType::STOP;           break;
                                            case BuildTool::INTERSECTION_FIXED_PRIORITY: reg = RegulationType::FIXED_PRIORITY; break;
                                            case BuildTool::INTERSECTION_P2P:            reg = RegulationType::P2P;            break;
                                            default: break;
                                        }
                                        scene::buildCrossroad(*world, gX + 1, gY + 1, newId, reg);
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
                        world->refreshRoundaboutApproaches();   // sorties dynamiques
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
                    metrics.reset();
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
                    metrics.reset();
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
                    metrics.sample(agents, FIXED_DT);   // banc d'essai : echantillonne l'etat a jour
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
                if (showDecisions) renderer.drawAgentDecision(*agent);
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
                            case BuildTool::INTERSECTION_FIXED_PRIORITY:
                                gw = 2; gh = 2; col = core::Color{40, 120, 255, 70}; break;
                            case BuildTool::INTERSECTION_P2P:
                                gw = 2; gh = 2; col = core::Color{60, 220, 180, 70}; break;
                            case BuildTool::INTERSECTION_ROUNDABOUT: {
                                int side = roundaboutSide;
                                if (side < 2)      side = 2;
                                if (side % 2 != 0) ++side;
                                tlx = hoverX; tly = hoverY;   // coin haut-gauche
                                gw = side; gh = side;
                                col = core::Color{60, 160, 255, 70};
                                break;
                            }
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
                    metrics.reset();
                }
                ImGui::Separator();
                ImGui::Checkbox("Pause Generale", &isPaused);
                ImGui::SliderFloat("Vitesse Sim", &simSpeedFactor, 0.2f, 3.0f, "%.1fx");
            }

            // ---- Banc d'essai quantitatif (metriques de recherche) ----
            if (ImGui::CollapsingHeader("Metriques (recherche)", ImGuiTreeNodeFlags_DefaultOpen)) {
                const auto& m = metrics.aggregate();
                ImGui::Text("Temps simule : %.1f s", m.simTime);
                ImGui::Text("Actifs : %d   |   Arrives : %d", m.activeVehicles, m.completedVehicles);
                ImGui::Separator();
                ImGui::Text("Debit       : %.1f veh/min", m.throughputPerMin);
                ImGui::Text("Delay moyen : %.2f s", m.meanDelaySec);
                ImGui::Text("Vitesse moy.: %.0f px/s", m.meanSpeed);
                ImGui::Text("Jerk moyen  : %.0f", m.meanJerkCompleted);
                const ImVec4 ttcCol = (m.minTTC < core::metrics::kCriticalTTC)
                    ? ImVec4(1.f, 0.3f, 0.3f, 1.f) : ImVec4(0.5f, 1.f, 0.5f, 1.f);
                ImGui::TextColored(ttcCol, "TTC min     : %.2f s", m.minTTC);
                ImGui::Text("Incidents TTC<%.1fs : %d", core::metrics::kCriticalTTC, m.ttcViolations);
                ImGui::Text("Arrets totaux : %d", m.totalStops);

                auto plot = [](const char* label, const std::vector<float>& s, const char* fmt) {
                    if (s.empty()) { ImGui::TextDisabled("%s : (collecte...)", label); return; }
                    char overlay[64];
                    std::snprintf(overlay, sizeof(overlay), fmt, s.back());
                    ImGui::PlotLines(label, s.data(), static_cast<int>(s.size()), 0,
                                     overlay, FLT_MAX, FLT_MAX, ImVec2(0.f, 45.f));
                };
                ImGui::Spacing();
                plot("Debit",   metrics.seriesThroughput(), "%.1f/min");
                plot("Delay",   metrics.seriesDelay(),      "%.1f s");
                plot("Vitesse", metrics.seriesSpeed(),      "%.0f");
                plot("TTC min", metrics.seriesMinTTC(),     "%.1f s");

                ImGui::Spacing();
                if (ImGui::Button("Exporter CSV", ImVec2(-1.f, 28.f))) {
                    std::string path = saveCsvDialog();
                    if (!path.empty()) metrics.exportCsv(path);
                }
                if (ImGui::Button("Reset metriques", ImVec2(-1.f, 24.f))) metrics.reset();
            }

            // ---- Choix / comparaison du mode de regulation par carrefour ----
            if (world && ImGui::CollapsingHeader("Carrefours - Mode")) {
                static const RegulationType regTypes[] = {
                    RegulationType::PRIORITY_RIGHT, RegulationType::STOP,
                    RegulationType::YIELD,          RegulationType::TRAFFIC_LIGHT,
                    RegulationType::FIXED_PRIORITY, RegulationType::P2P,
                    RegulationType::AIM
                };
                static const char* regNames[] = {
                    "Priorite a droite", "STOP", "Cedez", "Feux",
                    "Priorite fixe", "P2P (VANET)", "AIM (reservation)"
                };
                const auto& inters = world->getIntersections();
                for (std::size_t i = 0; i < inters.size(); ++i) {
                    const Intersection& it = inters[i];
                    ImGui::PushID(static_cast<int>(i));
                    const std::string lbl = "Carr #" + std::to_string(it.getId());
                    // Rond-point (mini 2x2 ou anneau) : geometrie non convertible.
                    const bool isRoundabout = (it.getType() == RegulationType::ROUNDABOUT);
                    if (isRoundabout) {
                        ImGui::Text("%s : Rond-point", lbl.c_str());
                    } else {
                        int cur = 0;
                        for (int k = 0; k < IM_ARRAYSIZE(regTypes); ++k)
                            if (regTypes[k] == it.getType()) { cur = k; break; }
                        if (ImGui::Combo(lbl.c_str(), &cur, regNames, IM_ARRAYSIZE(regNames))) {
                            world->setIntersectionRegulation(i, regTypes[cur]);
                        }
                    }
                    ImGui::PopID();
                }
                ImGui::TextDisabled("Change le mode en direct (meme geometrie).");
            }

            // ---- Experience Monte-Carlo headless (test d'hypothese du rapport) ----
            if (ImGui::CollapsingHeader("Experience Monte-Carlo")) {
                ImGui::TextWrapped("Compare Priorite Fixe vs P2P sur un carrefour "
                                   "isole, densite 0.1->0.8 v/s. Bloque l'UI le temps "
                                   "du calcul (progression en console).");
                ImGui::SliderInt("Duree mesure (s)", &expDurationSec, 20, 180);
                ImGui::SliderInt("Runs / point", &expRuns, 1, 5);
                ImGui::Checkbox("Prio. Fixe", &expStratFixed); ImGui::SameLine();
                ImGui::Checkbox("P2P", &expStratP2P);          ImGui::SameLine();
                ImGui::Checkbox("AIM", &expStratAim);

                if (ImGui::Button("Lancer l'experience", ImVec2(-1.f, 30.f))) {
                    sim::ExperimentConfig cfg;
                    cfg.durationSec  = static_cast<float>(expDurationSec);
                    cfg.runsPerPoint = expRuns;
                    cfg.strategies.clear();
                    if (expStratFixed) cfg.strategies.push_back(RegulationType::FIXED_PRIORITY);
                    if (expStratP2P)   cfg.strategies.push_back(RegulationType::P2P);
                    if (expStratAim)   cfg.strategies.push_back(RegulationType::AIM);
                    if (!cfg.strategies.empty())
                        expResults = sim::ExperimentRunner::run(cfg, nullptr);
                }

                if (!expResults.empty()) {
                    if (ImGui::BeginTable("expTable", 5,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                        ImGui::TableSetupColumn("Strat");
                        ImGui::TableSetupColumn("Dens");
                        ImGui::TableSetupColumn("Debit");
                        ImGui::TableSetupColumn("Delay");
                        ImGui::TableSetupColumn("TTC");
                        ImGui::TableHeadersRow();
                        for (const auto& r : expResults) {
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn(); ImGui::TextUnformatted(sim::ExperimentRunner::strategyName(r.strategy));
                            ImGui::TableNextColumn(); ImGui::Text("%.2f", r.density);
                            ImGui::TableNextColumn(); ImGui::Text("%.1f", r.throughputPerMin);
                            ImGui::TableNextColumn(); ImGui::Text("%.2f", r.meanDelaySec);
                            ImGui::TableNextColumn(); ImGui::Text("%.2f", r.minTTC);
                        }
                        ImGui::EndTable();
                    }

                    // Courbes par strategie : Delay vs densite + Debit vs densite.
                    // Le croisement attendu materialise le point d'inflexion du rapport.
                    std::vector<RegulationType> strats;
                    for (const auto& r : expResults)
                        if (std::find(strats.begin(), strats.end(), r.strategy) == strats.end())
                            strats.push_back(r.strategy);

                    for (RegulationType st : strats) {
                        std::vector<float> delay, thr;
                        for (const auto& r : expResults) if (r.strategy == st) {
                            delay.push_back(r.meanDelaySec);
                            thr.push_back(r.throughputPerMin);
                        }
                        char lab[64];
                        std::snprintf(lab, sizeof(lab), "Delay %s", sim::ExperimentRunner::strategyName(st));
                        ImGui::PlotLines(lab, delay.data(), static_cast<int>(delay.size()), 0,
                                         nullptr, FLT_MAX, FLT_MAX, ImVec2(0.f, 45.f));
                        std::snprintf(lab, sizeof(lab), "Debit %s", sim::ExperimentRunner::strategyName(st));
                        ImGui::PlotLines(lab, thr.data(), static_cast<int>(thr.size()), 0,
                                         nullptr, FLT_MAX, FLT_MAX, ImVec2(0.f, 45.f));
                    }
                    ImGui::TextDisabled("Axe X = densite croissante (0.1->0.8 v/s)");

                    if (ImGui::Button("Exporter resultats CSV", ImVec2(-1.f, 26.f))) {
                        std::string path = saveCsvDialog();
                        if (!path.empty()) sim::ExperimentRunner::exportCsv(path, expResults);
                    }
                }
            }

            if (ImGui::CollapsingHeader("Palette d'Edition", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Checkbox("Activer Mode Construction (B)", &buildMode);

                // --- NOUVEAU : La Checkbox pour activer la vision cyberpunk ---
                ImGui::Checkbox("Afficher les Flux (Debug)", &showFlowDebug);

                // Overlay pedagogique : montre la decision de CHAQUE vehicule
                // (anneau colore + label CEDE/STOP/P2P/FEU/DOUBLE...) sur la sim.
                ImGui::Checkbox("Afficher les decisions (overlay)", &showDecisions);

                if (buildMode) {
                    ImGui::TextColored(ImVec4(1.f, 1.f, 0.f, 1.f), "Outil de pose selectionne :");
                    if (ImGui::RadioButton("Route standard (50)   [1]", currentTool == BuildTool::ROAD_CITY_50)) currentTool = BuildTool::ROAD_CITY_50;
                    if (ImGui::RadioButton("Autoroute (130)       [2]", currentTool == BuildTool::ROAD_HIGHWAY_130)) currentTool = BuildTool::ROAD_HIGHWAY_130;
                    if (ImGui::RadioButton("Carrefour Priorite    [3]", currentTool == BuildTool::INTERSECTION_PRIORITY)) currentTool = BuildTool::INTERSECTION_PRIORITY;
                    if (ImGui::RadioButton("Carrefour Feux        [4]", currentTool == BuildTool::INTERSECTION_TRAFFIC_LIGHT)) currentTool = BuildTool::INTERSECTION_TRAFFIC_LIGHT;
                    if (ImGui::RadioButton("Carrefour STOP        [5]", currentTool == BuildTool::INTERSECTION_STOP)) currentTool = BuildTool::INTERSECTION_STOP;
                    if (ImGui::RadioButton("Rond-point            [6]", currentTool == BuildTool::INTERSECTION_ROUNDABOUT)) currentTool = BuildTool::INTERSECTION_ROUNDABOUT;
                    if (currentTool == BuildTool::INTERSECTION_ROUNDABOUT) {
                        ImGui::SliderInt("Cote RDP (tiles)", &roundaboutSide, 2, 8);
                        if (roundaboutSide % 2 != 0) ++roundaboutSide;   // toujours PAIR
                        ImGui::TextDisabled("Cote PAIR (2,4,6,8). Coin = tile cliquee.");
                    }

                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.f, 1.f), "Strategies recherche (SMA) :");
                    if (ImGui::RadioButton("Carrefour Priorite Fixe [7]", currentTool == BuildTool::INTERSECTION_FIXED_PRIORITY)) currentTool = BuildTool::INTERSECTION_FIXED_PRIORITY;
                    if (ImGui::RadioButton("Carrefour P2P (VANET)   [8]", currentTool == BuildTool::INTERSECTION_P2P)) currentTool = BuildTool::INTERSECTION_P2P;
                    ImGui::Spacing();

                    if (ImGui::RadioButton("Gomme / Effacer       [9]", currentTool == BuildTool::ERASE)) currentTool = BuildTool::ERASE;

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
                if (ImGui::Button("Conducteurs heterogenes (gaussien)", ImVec2(-1.f, 25.f))) {
                    for (auto& a : agents)
                        if (auto* veh = dynamic_cast<Vehicle*>(a.get()))
                            veh->applyGaussianHeterogeneity(uiRng, 0.18f);
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