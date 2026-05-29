// main.cpp
#include <SFML/Graphics.hpp>
#include "imgui.h"
#include "imgui-SFML.h"

#include <vector>
#include <memory>
#include <functional>
#include <atomic>
#include <thread>
#include <iostream>
#include <string>
#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <cmath>

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
#include "sim/Spawner.hpp"
#include "sim/McLiveSession.hpp"
#include "sim/ThreadPool.hpp"
#include "sim/ParallelDecisions.hpp"
#include "render/Camera.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "io/SceneBuilder.hpp"
#include "io/ScenarioCatalog.hpp"
#include "io/ScenarioIO.hpp"
#include "render/SfmlRenderer.hpp"
#include "render/SfmlInterop.hpp"

enum class AppState { MAIN_MENU, MONTE_CARLO, SIMULATION };
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

std::string saveJsonDialog() {
    if (!pfd::settings::available()) return "";
    auto f = pfd::save_file("Exporter les metriques (JSON)", "metriques.json",
                            { "Fichiers JSON (*.json)", "*.json" });
    std::string path = f.result();
    if (!path.empty() && path.find(".json") == std::string::npos) path += ".json";
    return path;
}

int main() {
    unsigned int winW = 1024;
    unsigned int winH = 800;
    constexpr float PANEL_MIN_WIDTH = 260.f;
    constexpr float PANEL_MAX_WIDTH = 560.f;
    constexpr float PANEL_RESIZER_WIDTH = 8.f;
    float panelWidth = 320.f;
    bool  resizingPanel = false;
    const int GRID_W = 32;
    const int GRID_H = 32;
    const float TILE_SIZE = 50.f;

    auto clampPanelWidth = [&]() {
        const float maxByWindow = std::max(PANEL_MIN_WIDTH, (float)winW - 220.f);
        panelWidth = std::clamp(panelWidth, PANEL_MIN_WIDTH,
                                std::min(PANEL_MAX_WIDTH, maxByWindow));
    };
    auto simAreaWidth = [&]() {
        return std::max(0.f, (float)winW - panelWidth);
    };

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
    bool expStratPlatoon = false;
    bool expStratOrca    = false;

    // --- Monte-Carlo (banc d'essai, ecran dedie) ---
    sim::McLiveSession mcLive;            // mode VISUEL : injection continue rendue
    sim::SpawnProfile  mcSpawn;           // spawner parametrable (types + profils)
    bool  mcVisualMode    = false;        // toggle rendu visuel vs headless
    float mcVisualDensity = 0.3f;         // veh/s pour le mode visuel
    int   mcStopMode      = 0;            // 0=flux continu, 1=nb vehicules, 2=temps
    int   mcMaxSpawns     = 40;           // budget de vehicules (mode 1)
    int   mcTimeLimitSec  = 60;           // duree simulee max (mode 2)
    int   mcRoundaboutSide = 4;           // cote de l'anneau si rond-point (2..8)
    bool  expStratRoundabout = false;     // inclure le rond-point dans le balayage
    // Mode HEADLESS : ExperimentRunner sur un thread dedie (UI reactive).
    std::thread          mcThread;
    std::atomic<float>   mcProgress{0.f};
    std::atomic<bool>    mcRunning{false};
    std::atomic<bool>    mcDone{false};
    std::vector<sim::ResultRow> mcThreadResults;

    // RNG d'UI (heterogeneite gaussienne appliquee a la flotte a la demande).
    core::Rng uiRng(0xA11CEULL);

    // --- Multithreading : pool persistant pour la phase de decision ---
    // Cree une fois (workers idle en attente), reutilise a chaque sous-pas.
    // Au-dela du seuil, computeDecision est reparti sur tous les coeurs ;
    // en-deca, le surcout de synchro depasse le gain -> on reste sequentiel.
    // NB : en parallele, l'attribution FCFS d'AIM n'est plus bit-deterministe
    // (cf. ParallelDecisions.hpp) ; sans effet sur P2P/ORCA/autres (sans etat).
    sim::ThreadPool simPool;
    bool                  useMultithreading  = false;
    constexpr std::size_t kParallelThreshold = 150;

    sf::Clock clock;
    sf::Clock deltaClock;

    // --- NOUVEAU : Horloge pour animer les flux indépendamment de la pause du jeu ---
    sf::Clock globalClock;

    // Wave 5 : pas de simulation fixe. Decouple le solver IDM du framerate.
    constexpr float FIXED_DT = 1.0f / 60.0f;  // 60 Hz logique
    constexpr int   MAX_SUBSTEPS = 12;        // permet l'acceleration jusqu'a ~x10
    float           simAccumulator = 0.f;

    bool buildMode = false;
    bool showFlowDebug = false; // --- NOUVEAU : Variable d'état du bouton ---
    bool showDecisions = false; // Overlay pedagogique des decisions sur la sim
    bool showVehicleIds = false;
    BuildTool currentTool = BuildTool::ROAD_CITY_50;
    int  roundaboutSide = 4;     // cote en tiles, PAIR (2,4,6,8). 2 = mini, 4 = anneau 4x4...
    bool isPaused = false;
    float simSpeedFactor = 1.0f;

    // --- Cycle de vie / fin de simulation ---
    // simFinished : plus aucun agent n'a de route a parcourir -> on fige la sim
    //               ET la collecte de metriques, et on propose la sauvegarde.
    // everHadAgents : evite de declarer "fini" une scene vierge (0 agent).
    // currentRebuild : reconstruit integralement la scene courante (Rejouer /
    //                  Reset), indispensable car les agents arrives sont DETRUITS.
    bool simFinished    = false;
    bool everHadAgents  = false;
    bool endDialogOpened = false;   // le pop-up de fin a-t-il deja ete ouvert ?
    std::function<void()> currentRebuild;

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

    // Entree en simulation a partir d'un 'world' deja construit (camera centree,
    // cache carte invalide, metriques remises a zero, flags de fin reinitialises).
    // Tail commun a tous les points de lancement.
    auto enterSimulationFromCurrentWorld = [&]() {
        if (!world) return;
        clampPanelWidth();
        camera = std::make_unique<Camera>(world->getWorldPixelWidth() / 2.f,
                                          world->getWorldPixelHeight() / 2.f,
                                          simAreaWidth(), (float)winH);
        camera->updateViewport((float)winW, (float)winH, panelWidth);
        renderer.invalidateMapCache();
        metrics.reset();
        simFinished     = false;
        everHadAgents   = !agents.empty();
        endDialogOpened = false;
        currentState    = AppState::SIMULATION;
        clock.restart();
    };

    auto initNewSimulation = [&]() {
        currentRebuild = [&]() {
            mcLive.stop();
            world = std::make_unique<World>(GRID_W, GRID_H, TILE_SIZE);
            agents.clear();
            scene::buildDefaultNetwork(*world);
            scene::spawnDefaultAgents(agents, *world);
            enterSimulationFromCurrentWorld();
        };
        currentRebuild();
    };

    // Construit puis lance un scenario du catalogue en un clic. La fonction de
    // construction est memorisee comme 'currentRebuild' pour Rejouer / Reset
    // (les agents arrives sont detruits, on ne peut pas se contenter de
    // resetToStart sur les survivants).
    auto launchScenario = [&](const scene::ScenarioBuildFn& build) {
        currentRebuild = [&, build]() {
            mcLive.stop();
            build(world, agents);
            enterSimulationFromCurrentWorld();
        };
        currentRebuild();
    };

    // Lance le mode Monte-Carlo VISUEL : carrefour isole alimente en continu,
    // rendu dans la boucle de simulation. Memorise dans currentRebuild (Reset).
    auto launchVisualMc = [&](RegulationType strat) {
        // Fige la condition d'arret + le cote du rond-point au lancement
        // (rejouabilite via Reset).
        const int   maxN = (mcStopMode == 1) ? mcMaxSpawns : 0;
        const float tLim = (mcStopMode == 2) ? static_cast<float>(mcTimeLimitSec) : 0.f;
        const int   side = mcRoundaboutSide;
        currentRebuild = [&, strat, maxN, tLim, side]() {
            mcLive.start(world, agents, strat, mcVisualDensity, mcSpawn, 1337u,
                         maxN, tLim, side);
            enterSimulationFromCurrentWorld();   // everHadAgents=false (0 agent) -> fin geree par mcLive
        };
        currentRebuild();
    };

    // Lance le banc d'essai Monte-Carlo HEADLESS sur un thread dedie (UI fluide).
    auto launchHeadlessMc = [&]() {
        if (mcRunning.load()) return;
        sim::ExperimentConfig cfg;
        cfg.durationSec    = static_cast<float>(expDurationSec);
        cfg.runsPerPoint   = expRuns;
        cfg.spawn          = mcSpawn;
        cfg.roundaboutSide = mcRoundaboutSide;
        cfg.strategies.clear();
        if (expStratFixed)      cfg.strategies.push_back(RegulationType::FIXED_PRIORITY);
        if (expStratP2P)        cfg.strategies.push_back(RegulationType::P2P);
        if (expStratAim)        cfg.strategies.push_back(RegulationType::AIM);
        if (expStratPlatoon)    cfg.strategies.push_back(RegulationType::VIRTUAL_PLATOON);
        if (expStratOrca)       cfg.strategies.push_back(RegulationType::ORCA);
        if (expStratRoundabout) cfg.strategies.push_back(RegulationType::ROUNDABOUT);
        if (cfg.strategies.empty()) return;

        if (mcThread.joinable()) mcThread.join();   // surete : run precedent
        mcThreadResults.clear();
        expResults.clear();
        mcProgress.store(0.f);
        mcDone.store(false);
        mcRunning.store(true);
        mcThread = std::thread([&, cfg]() {
            auto res = sim::ExperimentRunner::run(cfg, &mcProgress);
            mcThreadResults = std::move(res);
            mcDone.store(true, std::memory_order_release);
        });
    };

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::Resized) {
                winW = event.size.width;
                winH = event.size.height;
                clampPanelWidth();
                window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));
                if (camera) camera->updateViewport((float)winW, (float)winH, panelWidth);
            }

            if (currentState == AppState::SIMULATION) {
                const float panelLeft = simAreaWidth();
                if (event.type == sf::Event::MouseButtonPressed &&
                    event.mouseButton.button == sf::Mouse::Left &&
                    std::abs((float)event.mouseButton.x - panelLeft) <= PANEL_RESIZER_WIDTH) {
                    resizingPanel = true;
                }
                if (event.type == sf::Event::MouseMoved && resizingPanel) {
                    panelWidth = (float)winW - (float)event.mouseMove.x;
                    clampPanelWidth();
                    if (camera) camera->updateViewport((float)winW, (float)winH, panelWidth);
                }
                if (event.type == sf::Event::MouseButtonReleased &&
                    event.mouseButton.button == sf::Mouse::Left) {
                    resizingPanel = false;
                }

                if (!resizingPanel && sf::Mouse::getPosition(window).x < (int)simAreaWidth()) {
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
                                currentTool = BuildTool::ERASE; break;
                            case sf::Keyboard::Delete:
                                // En build : raccourci outil ERASE.
                                // Hors build : supprime de la scene tous les
                                // vehicules selectionnes (clic gauche pour selection).
                                if (buildMode) {
                                    currentTool = BuildTool::ERASE;
                                } else {
                                    agents.erase(std::remove_if(
                                        agents.begin(), agents.end(),
                                        [](const std::unique_ptr<IAgent>& a) {
                                            return a && a->isSelected();
                                        }), agents.end());
                                }
                                break;
                            default: break;
                        }
                    }
                }

                const bool inSimArea =
                    !resizingPanel && sf::Mouse::getPosition(window).x < (int)simAreaWidth();

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

            ImGui::SetNextWindowPos(ImVec2(winW / 2.f - 240.f, winH / 2.f - 280.f));
            ImGui::SetNextWindowSize(ImVec2(480.f, 560.f));
            ImGui::Begin("Menu Principal - MAS", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            ImGui::Text("Simulateur Multi-Agents Auto");
            ImGui::Separator();
            ImGui::Spacing();

            // ---- Scenarios de presentation (catalogue, lancement en 1 clic) ----
            ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.f, 1.f), "Scenarios de presentation");
            const auto& catalog = scene::scenarioCatalog();
            static int selScenario = 0;
            if (selScenario >= (int)catalog.size()) selScenario = 0;

            ImGui::SetNextItemWidth(-1.f);
            if (ImGui::BeginCombo("##scenarioCombo", catalog[selScenario].name.c_str())) {
                std::string lastCat;
                for (int i = 0; i < (int)catalog.size(); ++i) {
                    if (catalog[i].category != lastCat) {
                        lastCat = catalog[i].category;
                        if (i != 0) ImGui::Separator();
                        ImGui::TextDisabled("%s", lastCat.c_str());
                    }
                    const bool sel = (i == selScenario);
                    if (ImGui::Selectable(catalog[i].name.c_str(), sel)) selScenario = i;
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::TextWrapped("%s", catalog[selScenario].description.c_str());
            if (ImGui::Button("Lancer le scenario", ImVec2(-1.f, 42.f)))
                launchScenario(catalog[selScenario].build);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // ---- Monte-Carlo : bouton DEDIE (separe des scenarios) ----
            ImGui::TextColored(ImVec4(1.f, 0.8f, 0.3f, 1.f), "Banc d'essai quantitatif");
            if (ImGui::Button("Monte-Carlo (densite x strategie)", ImVec2(-1.f, 40.f)))
                currentState = AppState::MONTE_CARLO;

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // ---- Autres points d'entree ----
            if (ImGui::Button("Nouvelle Simulation (vierge editable)", ImVec2(-1.f, 38.f)))
                initNewSimulation();
            if (ImGui::Button("Charger un Scenario (TXT)", ImVec2(-1.f, 38.f))) {
                std::string path = openFileDialog();
                if (!path.empty() && io::importScenario(path, world, agents)) {
                    currentRebuild = [&, path]() {
                        if (io::importScenario(path, world, agents))
                            enterSimulationFromCurrentWorld();
                    };
                    enterSimulationFromCurrentWorld();
                }
            }
            if (ImGui::Button("Quitter l'Application", ImVec2(-1.f, 38.f))) window.close();

            ImGui::End();
        }
        else if (currentState == AppState::MONTE_CARLO) {
            // Recupere les resultats du thread headless des qu'il a termine.
            if (mcRunning.load() && mcDone.load(std::memory_order_acquire)) {
                if (mcThread.joinable()) mcThread.join();
                expResults = std::move(mcThreadResults);
                mcRunning.store(false);
            }

            window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)winW, (float)winH)));
            ImGui::SetNextWindowPos(ImVec2(winW / 2.f, winH / 2.f), ImGuiCond_Always,
                                    ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowSize(ImVec2(620.f, std::min((float)winH - 30.f, 740.f)));
            ImGui::Begin("Monte-Carlo - Banc d'essai", nullptr,
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

            ImGui::TextWrapped("Compare des strategies de coordination sur un carrefour "
                               "isole alimente a debit croissant. VISUEL = rendu temps "
                               "reel d'un point ; HEADLESS = balayage complet "
                               "(densite 0.1->0.8 v/s) en arriere-plan, puis tableau/courbes.");
            ImGui::Separator();

            ImGui::Checkbox("Rendu visuel de l'execution", &mcVisualMode);
            ImGui::SameLine();
            ImGui::TextDisabled(mcVisualMode ? "(temps reel, 1 point)"
                                             : "(headless, balayage complet)");

            // --- Spawner parametrable (types + profils comportementaux) ---
            if (ImGui::CollapsingHeader("Spawner (trafic stochastique)",
                                        ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::SliderFloat("Proba camion", &mcSpawn.wTruck, 0.f, 1.f, "%.2f");
                mcSpawn.wCar = 1.f - mcSpawn.wTruck;
                ImGui::TextDisabled("Profils conducteurs (poids relatifs) :");
                ImGui::SliderFloat("Normal",   &mcSpawn.wNormal,     0.f, 1.f, "%.2f");
                ImGui::SliderFloat("Agressif", &mcSpawn.wAggressive, 0.f, 1.f, "%.2f");
                ImGui::SliderFloat("Prudent",  &mcSpawn.wCalm,       0.f, 1.f, "%.2f");
                ImGui::Checkbox("Heterogeneite gaussienne", &mcSpawn.gaussianHeterogeneity);
                if (mcSpawn.gaussianHeterogeneity)
                    ImGui::SliderFloat("Sigma (T/aMax/vitesse)", &mcSpawn.driverSigma,
                                       0.f, 0.4f, "%.2f");
            }

            if (mcVisualMode) {
                static const RegulationType vStrats[] = {
                    RegulationType::FIXED_PRIORITY, RegulationType::P2P,
                    RegulationType::AIM,            RegulationType::VIRTUAL_PLATOON,
                    RegulationType::ORCA,
                    RegulationType::PRIORITY_RIGHT, RegulationType::STOP,
                    RegulationType::TRAFFIC_LIGHT,  RegulationType::ROUNDABOUT
                };
                static const char* vNames[] = {
                    "Priorite fixe", "P2P (VANET)", "AIM (reservation)", "Peloton virtuel",
                    "ORCA (espace ouvert)",
                    "Priorite a droite", "STOP", "Feux", "Rond-point"
                };
                static int vIdx = 0;
                ImGui::Combo("Strategie", &vIdx, vNames, IM_ARRAYSIZE(vNames));
                if (vStrats[vIdx] == RegulationType::ROUNDABOUT) {
                    ImGui::SliderInt("Cote anneau", &mcRoundaboutSide, 2, 8, "%d tiles");
                    if (mcRoundaboutSide % 2 != 0) ++mcRoundaboutSide;   // toujours PAIR
                }
                ImGui::SliderFloat("Densite (veh/s)", &mcVisualDensity, 0.05f, 1.0f, "%.2f");

                // Condition d'arret : flux perpetuel, budget de vehicules, ou temps.
                ImGui::TextDisabled("Condition d'arret :");
                ImGui::RadioButton("Flux continu (observation)", &mcStopMode, 0);
                ImGui::RadioButton("Nombre de vehicules", &mcStopMode, 1);
                if (mcStopMode == 1) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(140.f);
                    ImGui::SliderInt("##maxspawn", &mcMaxSpawns, 5, 300, "%d veh");
                }
                ImGui::RadioButton("Limite de temps", &mcStopMode, 2);
                if (mcStopMode == 2) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(140.f);
                    ImGui::SliderInt("##tlim", &mcTimeLimitSec, 10, 300, "%d s");
                }
                ImGui::TextDisabled("A la fin : pop-up + export des metriques (CSV/JSON).");

                if (ImGui::Button("Lancer en visuel", ImVec2(-1.f, 40.f)))
                    launchVisualMc(vStrats[vIdx]);
            } else {
                ImGui::SliderInt("Duree mesure (s)", &expDurationSec, 20, 180);
                ImGui::SliderInt("Runs / point", &expRuns, 1, 5);
                ImGui::TextDisabled("Strategies a comparer :");
                ImGui::Checkbox("Prio. Fixe", &expStratFixed);  ImGui::SameLine();
                ImGui::Checkbox("P2P", &expStratP2P);           ImGui::SameLine();
                ImGui::Checkbox("AIM", &expStratAim);           ImGui::SameLine();
                ImGui::Checkbox("Peloton", &expStratPlatoon);  ImGui::SameLine();
                ImGui::Checkbox("ORCA", &expStratOrca);
                ImGui::Checkbox("Rond-point", &expStratRoundabout);
                if (expStratRoundabout) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(150.f);
                    ImGui::SliderInt("Cote anneau", &mcRoundaboutSide, 2, 8, "%d tiles");
                    if (mcRoundaboutSide % 2 != 0) ++mcRoundaboutSide;
                }

                if (mcRunning.load()) {
                    ImGui::ProgressBar(mcProgress.load(), ImVec2(-1.f, 0.f));
                    ImGui::TextDisabled("Calcul en arriere-plan (UI reactive)...");
                } else if (ImGui::Button("Lancer le balayage", ImVec2(-1.f, 40.f))) {
                    launchHeadlessMc();
                }

                if (!expResults.empty()) {
                    if (ImGui::BeginTable("expTable", 5, ImGuiTableFlags_Borders |
                            ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
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
                    if (ImGui::Button("Exporter resultats CSV", ImVec2(-1.f, 28.f))) {
                        std::string path = saveCsvDialog();
                        if (!path.empty()) sim::ExperimentRunner::exportCsv(path, expResults);
                    }
                }
            }

            ImGui::Separator();
            if (ImGui::Button("Retour au menu", ImVec2(-1.f, 32.f)) && !mcRunning.load())
                currentState = AppState::MAIN_MENU;

            ImGui::End();
        }
        else if (currentState == AppState::SIMULATION) {
            // Wave 5 : pas fixe + accumulateur. Garantit que IDM tourne sur un dt
            // constant quel que soit le framerate (stabilite numerique Euler).
            const float frameTime = isPaused ? 0.f : clock.restart().asSeconds() * simSpeedFactor;
            if (isPaused) { clock.restart(); simAccumulator = 0.f; }

            // Une fois la simulation terminee, tout est fige : ni physique, ni
            // metriques (collecte formellement arretee), seul le pop-up reste.
            if (world && !isPaused && !simFinished) {
                simAccumulator += frameTime;
                int steps = 0;
                while (simAccumulator >= FIXED_DT && steps < MAX_SUBSTEPS) {
                    // Monte-Carlo visuel : injection continue de trafic stochastique.
                    if (mcLive.active()) mcLive.inject(*world, agents, FIXED_DT);
                    world->updateIntersections(FIXED_DT);
                    // Phase 1 : decision. Parallelisee sur les coeurs au-dela du
                    // seuil (grosses scenes type XXXXL) ; sequentielle sinon.
                    if (useMultithreading && agents.size() >= kParallelThreshold)
                        sim::computeDecisionsParallel(agents, *world, simPool);
                    else
                        for (auto& agent : agents) agent->computeDecision(agents, *world);
                    // Phase 2 : integration (sequentielle, deterministe).
                    for (auto& agent : agents) agent->integrate(FIXED_DT);
                    metrics.sample(agents, FIXED_DT);   // banc d'essai : echantillonne l'etat a jour
                    simAccumulator -= FIXED_DT;
                    ++steps;
                }
                // Si le frame a accumule trop de retard, on jette le surplus
                // pour eviter le scenario "spirale de la mort".
                if (steps >= MAX_SUBSTEPS) simAccumulator = 0.f;

                // --- Cycle de vie : DESTRUCTION des agents arrives -------------
                // Un agent a BlockReason::AT_GOAL implique qu'il etait deja
                // hasFinishedPath au computeDecision du dernier sous-pas, donc que
                // metrics.sample l'a vu "complete" dans ce meme sous-pas. On peut
                // le detruire sans perdre sa metrique : il cesse alors d'interagir
                // (perception, policies d'intersection, negociation P2P).
                if (steps > 0) {
                    agents.erase(std::remove_if(agents.begin(), agents.end(),
                        [](const std::unique_ptr<IAgent>& a) {
                            return a && a->getBlockReason() ==
                                   core::agent::BlockReason::AT_GOAL;
                        }), agents.end());

                    // --- Detection de fin de simulation -------------------------
                    // Les agents arrives (AT_GOAL) viennent d'etre detruits ci-dessus
                    // APRES avoir ete comptes. Reste "actif" tout agent dont le motif
                    // n'est pas NO_PATH : un agent qui roule, attend a un feu, ou
                    // vient juste de finir (motif encore NONE, passera AT_GOAL au
                    // prochain pas et sera alors compte). Aucun vehicule arrive n'est
                    // donc perdu dans le comptage.
                    bool anyActive = false;
                    for (const auto& a : agents)
                        if (a && a->getBlockReason() !=
                                 core::agent::BlockReason::NO_PATH) {
                            anyActive = true;
                            break;
                        }

                    if (mcLive.active()) {
                        // Monte-Carlo borne : fin par limite de TEMPS (immediate),
                        // ou par BUDGET de vehicules epuise puis trafic draine.
                        // Sans limite (flux perpetuel) -> ne se termine jamais.
                        if (mcLive.timeLimitReached() ||
                            (mcLive.budgetExhausted() && !anyActive)) {
                            mcLive.stop();
                            simFinished = true;
                        }
                    } else if (everHadAgents && !anyActive) {
                        simFinished = true;
                    }
                }
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
                if (showVehicleIds) renderer.drawAgentId(*agent);
            }

            // --- Apercu de construction (fantome + surbrillance de la tile) ---
            hoverValid = false;
            if (buildMode && world && camera) {
                const sf::Vector2i mp = sf::Mouse::getPosition(window);
                if (mp.x < (int)simAreaWidth()) {
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

            const float panelLeft = simAreaWidth();
            ImGui::SetNextWindowPos(ImVec2(panelLeft - PANEL_RESIZER_WIDTH * 0.5f, 0.f));
            ImGui::SetNextWindowSize(ImVec2(PANEL_RESIZER_WIDTH, (float)winH));
            ImGui::Begin("##DashboardResizeHandle", nullptr,
                         ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
            ImGui::InvisibleButton("##resize", ImVec2(PANEL_RESIZER_WIDTH, (float)winH));
            if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            }
            if (ImGui::IsItemActive()) {
                panelWidth = (float)winW - ImGui::GetIO().MousePos.x;
                clampPanelWidth();
                if (camera) camera->updateViewport((float)winW, (float)winH, panelWidth);
            }
            const ImU32 handleColor = ImGui::GetColorU32(
                (ImGui::IsItemHovered() || ImGui::IsItemActive())
                    ? ImVec4(0.45f, 0.68f, 1.f, 0.9f)
                    : ImVec4(1.f, 1.f, 1.f, 0.22f));
            ImGui::GetWindowDrawList()->AddRectFilled(
                ImVec2(panelLeft - 1.f, 0.f),
                ImVec2(panelLeft + 1.f, (float)winH),
                handleColor);
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2((float)winW - panelWidth, 0.f));
            ImGui::SetNextWindowSize(ImVec2(panelWidth, (float)winH));
            ImGui::Begin("Dashboard de Controle", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

            if (ImGui::CollapsingHeader("Fichiers & Options", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("Sauvegarder (S)", ImVec2(-1.f, 30.f))) {
                    std::string path = saveFileDialog();
                    if (!path.empty()) io::exportScenario(path, *world, agents);
                }
                if (ImGui::Button("Reset Simulation (R)", ImVec2(-1.f, 30.f))) {
                    // Reconstruit la scene complete (les agents arrives ont ete
                    // detruits) ; repli sur resetToStart si pas de constructeur.
                    if (currentRebuild) currentRebuild();
                    else { for (auto& a : agents) a->resetToStart(*world); metrics.reset(); }
                }
                ImGui::Separator();
                ImGui::Checkbox("Pause Generale", &isPaused);
                ImGui::SliderFloat("Vitesse Sim", &simSpeedFactor, 0.2f, 10.0f, "%.1fx");
                ImGui::Separator();
                ImGui::Checkbox("Multithreading (decisions)", &useMultithreading);
                ImGui::TextDisabled("Pool : %u threads. Actif si >= %d agents.",
                                    simPool.parallelism(), (int)kParallelThreshold);
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
                // --- Diagnostic des blocages : snapshot par-vehicule a l'instant
                //     du clic (mettre en PAUSE d'abord pour un etat stable). Donne,
                //     pour chaque agent : son motif, et la SOURCE + le CAP RELATIF de
                //     son leader -> permet de voir si on "suit" un oncoming/croise. ---
                if (ImGui::Button("Exporter diagnostic blocages (CSV)", ImVec2(-1.f, 28.f))) {
                    std::string path = saveCsvDialog();
                    if (!path.empty()) {
                        std::ofstream os(path);
                        os << "simTime," << metrics.aggregate().simTime << "\n";
                        os << "vin,x,y,heading,speed,block,leaderSrc,leaderVin,"
                              "leaderRelHead,leaderClass,leaderGap,leaderSpeed,onInter\n";
                        for (auto& a : agents) {
                            auto* v = dynamic_cast<Vehicle*>(a.get());
                            if (!v) continue;
                            const auto d = v->getDecisionDiagnostic();
                            const char* src = (d.leaderSource == 1) ? "perception"
                                            : (d.leaderSource == 2) ? "filet"
                                            : (d.leaderSource == 3) ? "policy" : "-";
                            const char* cls = "-";
                            if (d.leaderRelHeadingDeg < 900.f) {
                                const float h = std::fabs(d.leaderRelHeadingDeg);
                                cls = (h < 45.f) ? "SAME" : (h <= 135.f ? "CROSS" : "ONCOMING");
                            }
                            os << d.vin << ',' << d.x << ',' << d.y << ',' << d.headingDeg
                               << ',' << d.speed << ','
                               << core::agent::toString(static_cast<core::agent::BlockReason>(d.blockReason))
                               << ',' << src << ',' << d.leaderVin << ','
                               << d.leaderRelHeadingDeg << ',' << cls << ','
                               << d.leaderGap << ',' << d.leaderSpeed << ','
                               << (d.onIntersection ? 1 : 0) << '\n';
                        }
                    }
                }
                if (ImGui::Button("Reset metriques", ImVec2(-1.f, 24.f))) metrics.reset();
            }

            // ---- Choix / comparaison du mode de regulation par carrefour ----
            if (world && ImGui::CollapsingHeader("Carrefours - Mode")) {
                static const RegulationType regTypes[] = {
                    RegulationType::PRIORITY_RIGHT, RegulationType::STOP,
                    RegulationType::YIELD,          RegulationType::TRAFFIC_LIGHT,
                    RegulationType::FIXED_PRIORITY, RegulationType::P2P,
                    RegulationType::AIM,            RegulationType::VIRTUAL_PLATOON,
                    RegulationType::ORCA
                };
                static const char* regNames[] = {
                    "Priorite a droite", "STOP", "Cedez", "Feux",
                    "Priorite fixe", "P2P (VANET)", "AIM (reservation)", "Peloton virtuel",
                    "ORCA (espace ouvert)"
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

            // Le banc d'essai Monte-Carlo a desormais son propre ecran dedie,
            // accessible depuis le menu principal (visuel ou headless).
            if (ImGui::CollapsingHeader("Experience Monte-Carlo")) {
                ImGui::TextWrapped("Le banc d'essai Monte-Carlo (densite x strategie, "
                                   "visuel ou headless) est accessible depuis le MENU "
                                   "PRINCIPAL via le bouton dedie.");
            }

            if (ImGui::CollapsingHeader("Palette d'Edition", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Checkbox("Activer Mode Construction (B)", &buildMode);

                // --- NOUVEAU : La Checkbox pour activer la vision cyberpunk ---
                ImGui::Checkbox("Afficher les Flux (Debug)", &showFlowDebug);

                // Overlay pedagogique : montre la decision de CHAQUE vehicule
                // (anneau colore + label CEDE/STOP/P2P/FEU/DOUBLE...) sur la sim.
                ImGui::Checkbox("Afficher les decisions (overlay)", &showDecisions);

                if (ImGui::Button(showVehicleIds ? "Masquer les IDs vehicules"
                                                 : "Afficher les IDs vehicules",
                                  ImVec2(-1.f, 25.f))) {
                    showVehicleIds = !showVehicleIds;
                }

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
                            case core::agent::BlockReason::INTERSECTION_STOP:
                                color = ImVec4(1.f, 0.65f, 0.15f, 1.f); break;
                            case core::agent::BlockReason::NEGOTIATING:
                            case core::agent::BlockReason::PLATOONING:
                                color = ImVec4(0.4f, 0.85f, 1.f, 1.f); break;
                            case core::agent::BlockReason::LEADER_VEHICLE:
                                color = ImVec4(1.f, 0.5f, 0.f, 1.f); break;
                            case core::agent::BlockReason::CORNERING:
                                color = ImVec4(0.6f, 0.8f, 1.f, 1.f); break;
                            case core::agent::BlockReason::INITIALIZING:
                                color = ImVec4(0.8f, 0.8f, 0.8f, 1.f); break;
                            case core::agent::BlockReason::BREAKDOWN:
                                color = ImVec4(1.f, 0.2f, 0.2f, 1.f); break;
                            case core::agent::BlockReason::OVERTAKING:
                                color = ImVec4(0.8f, 0.5f, 1.f, 1.f); break;
                            case core::agent::BlockReason::KEEP_CLEAR:
                                color = ImVec4(1.f, 0.75f, 0.25f, 1.f); break;
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

            // ---- Pop-up de fin de simulation -------------------------------
            // Ouvert une seule fois a la transition simFinished. Propose la
            // sauvegarde des metriques (CSV/JSON), de rejouer ou de revenir.
            if (simFinished && !endDialogOpened) {
                ImGui::OpenPopup("Simulation terminee");
                endDialogOpened = true;
            }
            ImGui::SetNextWindowPos(ImVec2(winW / 2.f, winH / 2.f), ImGuiCond_Appearing,
                                    ImVec2(0.5f, 0.5f));
            if (ImGui::BeginPopupModal("Simulation terminee", nullptr,
                                       ImGuiWindowFlags_AlwaysAutoResize)) {
                const auto& m = metrics.aggregate();
                ImGui::Text("Simulation terminee");
                ImGui::Separator();
                ImGui::TextWrapped("Collecte des metriques arretee (tous arrives, "
                                   "budget de vehicules epuise, ou limite de temps).");
                ImGui::Spacing();
                ImGui::Text("Vehicules arrives : %d", m.completedVehicles);
                ImGui::Text("Temps simule      : %.1f s", m.simTime);
                ImGui::Text("Debit             : %.1f veh/min", m.throughputPerMin);
                ImGui::Text("Delay moyen       : %.2f s", m.meanDelaySec);
                ImGui::Text("TTC min           : %.2f s", m.minTTC);
                ImGui::Text("Arrets totaux     : %d", m.totalStops);
                ImGui::Separator();
                ImGui::TextDisabled("Sauvegarder les metriques generees :");

                if (ImGui::Button("Exporter CSV", ImVec2(180.f, 32.f))) {
                    std::string path = saveCsvDialog();
                    if (!path.empty()) metrics.exportCsv(path);
                }
                ImGui::SameLine();
                if (ImGui::Button("Exporter JSON", ImVec2(180.f, 32.f))) {
                    std::string path = saveJsonDialog();
                    if (!path.empty()) metrics.exportJson(path);
                }

                ImGui::Spacing();
                if (ImGui::Button("Rejouer", ImVec2(180.f, 32.f))) {
                    ImGui::CloseCurrentPopup();
                    if (currentRebuild) currentRebuild();   // remet simFinished a false
                }
                ImGui::SameLine();
                if (ImGui::Button("Retour au menu", ImVec2(180.f, 32.f))) {
                    ImGui::CloseCurrentPopup();
                    currentState = AppState::MAIN_MENU;
                }
                ImGui::Spacing();
                if (ImGui::Button("Continuer l'observation", ImVec2(-1.f, 26.f))) {
                    // Ferme le pop-up sans relancer : la scene reste figee, on
                    // peut la consulter (camera, panneau metriques) librement.
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }

        ImGui::SFML::Render(window);
        window.display();
    }

    // Le thread Monte-Carlo capture des locals par reference : on l'attend avant
    // de detruire la pile de main (sinon acces a de la memoire liberee).
    if (mcThread.joinable()) mcThread.join();

    ImGui::SFML::Shutdown();
    return 0;
}
