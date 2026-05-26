// src/io/ScenarioCatalog.cpp
#include "io/ScenarioCatalog.hpp"

#include <algorithm>
#include <unordered_set>

#include "core/agent/Car.hpp"
#include "core/agent/Personality.hpp"
#include "core/agent/Truck.hpp"
#include "core/agent/Vehicle.hpp"
#include "core/math/Rng.hpp"
#include "core/math/Vec2.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "io/SceneBuilder.hpp"

namespace scene {
namespace {

namespace prof = core::agent::profiles;
using core::agent::Personality;
using AgentVec = std::vector<std::unique_ptr<IAgent>>;

// Car/Truck figent leur tileSize interne a 50 px : tout scenario garde TS = 50
// pour que la geometrie monde et les corps de vehicules restent coherents.
constexpr float TS = 50.f;

core::Vec2 tileCenter(int tx, int ty) {
    return { tx * TS + TS / 2.f, ty * TS + TS / 2.f };
}

void addCar(AgentVec& agents, World& w, int sx, int sy, int gx, int gy,
            const Personality& p) {
    auto c = std::make_unique<Car>(tileCenter(sx, sy).x, tileCenter(sx, sy).y);
    c->setPersonality(p);
    c->setPath(AStarPlanner::findPath(w, {sx, sy}, {gx, gy}), &w);
    agents.push_back(std::move(c));
}

void addTruck(AgentVec& agents, World& w, int sx, int sy, int gx, int gy,
              const Personality& p) {
    auto t = std::make_unique<Truck>(tileCenter(sx, sy).x, tileCenter(sx, sy).y);
    t->setPersonality(p);
    t->setPath(AStarPlanner::findPath(w, {sx, sy}, {gx, gy}), &w);
    agents.push_back(std::move(t));
}

// =============================================================================
// Geometrie : carrefour 2x2 isole avec routes H + V traversantes.
//
//   Apres makeCrossroad, les voies sont :
//     row cy   = RIGHT (EST,  +x)      col cx   = UP   (NORD, -y)
//     row cy-1 = LEFT  (OUEST,-x)      col cx-1 = DOWN (SUD,  +y)
// =============================================================================
struct Cross { int cx, cy, W, H; };

Cross makeCrossroad(std::unique_ptr<World>& world, RegulationType reg,
                    int W = 30, int H = 30) {
    const int cx = W / 2, cy = H / 2;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    buildHRoad(w, cy, 0, W - 1);
    buildVRoad(w, cx, 0, H - 1);
    buildCrossroad(w, cx, cy, 1, reg);
    return {cx, cy, W, H};
}

// Une voiture par branche, vers la branche opposee, a distance D de la ligne.
// stagger = arrivees etalees (flux naturel) ; sinon les 4 arrivent ensemble.
void spawnFourWay(AgentVec& agents, World& w, const Cross& c,
                  const Personality& p, int D, bool stagger) {
    const int oW = stagger ? 2 : 0;   // ouest
    const int oN = stagger ? 1 : 0;   // nord
    const int oS = stagger ? 3 : 0;   // sud
    addCar(agents, w, c.cx - D,      c.cy,          c.W - 2, c.cy,     p); // EST
    addCar(agents, w, c.cx + D + oW, c.cy - 1,      1,       c.cy - 1, p); // OUEST
    addCar(agents, w, c.cx,          c.cy + D + oN, c.cx,    1,        p); // NORD
    addCar(agents, w, c.cx - 1,      c.cy - D - oS, c.cx - 1, c.H - 2, p); // SUD
}

// =============================================================================
// Constructeurs de scenarios
// =============================================================================

// --- Bases -------------------------------------------------------------------

void buildFreeRoad(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    world = std::make_unique<World>(40, 12, TS);
    buildHRoad(*world, 6, 0, 39);
    addCar(agents, *world, 1, 6, 38, 6, prof::normalDriver());   // trajectoire libre
}

void buildSmoothCrossing(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    Cross c = makeCrossroad(world, RegulationType::PRIORITY_RIGHT);
    // Une voiture EST (axe horizontal) + une voiture SUD legerement plus loin :
    // la verticale arrive apres et cede proprement (pas d'arret brusque).
    addCar(agents, *world, c.cx - 8, c.cy,     c.W - 2, c.cy,     prof::normalDriver());
    addCar(agents, *world, c.cx - 1, c.cy - 11, c.cx - 1, c.H - 2, prof::normalDriver());
}

void buildCarFollowing(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    world = std::make_unique<World>(52, 12, TS);
    buildHRoad(*world, 6, 0, 51);
    // Camion lent en tete : la file de voitures se cale derriere (IDM platoon).
    Personality slowTruck = prof::truckDriver();
    slowTruck.speedComplianceFactor = 0.55f;
    addTruck(agents, *world, 24, 6, 50, 6, slowTruck);
    for (int k = 0; k < 5; ++k)
        addCar(agents, *world, 18 - k * 3, 6, 50, 6, prof::calmDriver());
}

// --- Un scenario par mode de regulation (4 voitures, une par branche) --------

void buildModeDemo(std::unique_ptr<World>& world, AgentVec& agents,
                   RegulationType reg) {
    agents.clear();
    Cross c = makeCrossroad(world, reg);
    spawnFourWay(agents, *world, c, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildRoundaboutDemo(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 30, H = 30;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    // Routes traversantes puis rond-point 4x4 centre (coin haut-gauche 13,13).
    buildHRoad(w, 15, 0, W - 1);
    buildVRoad(w, 15, 0, H - 1);
    buildRoundabout(w, 13, 13, 1, 4);
    w.refreshRoundaboutApproaches();
    // 4 voitures convergentes : sens unique legal, pas de contre-sens.
    addCar(agents, w, 1,  15, W - 2, 15, prof::normalDriver());   // EST
    addCar(agents, w, W - 2, 14, 1,  14, prof::normalDriver());   // OUEST
    addCar(agents, w, 15, H - 2, 15, 1,  prof::normalDriver());   // NORD
    addCar(agents, w, 14, 1, 14, H - 2, prof::normalDriver());    // SUD
}

// --- Cas limites / degrades --------------------------------------------------

// Arrivee SIMULTANEE parfaite des 4 vehicules : teste l'evitement de collision
// en "impasse temporelle". Le mode (P2P / AIM) determine comment le conflit est
// resolu (negociation decentralisee vs reservation centralisee).
void buildSimultaneousArrival(std::unique_ptr<World>& world, AgentVec& agents,
                              RegulationType reg) {
    agents.clear();
    Cross c = makeCrossroad(world, reg);
    spawnFourWay(agents, *world, c, prof::normalDriver(), 10, /*stagger=*/false);
}

// Forte densite sur un carrefour P2P : 3 vehicules par branche convergent. Le
// protocole pair-a-pair se degrade alors en un comportement facon "four-way
// stop" (chacun son tour) -> debit qui s'effondre, file d'attente visible.
void buildHighDensityP2P(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    Cross c = makeCrossroad(world, RegulationType::P2P);
    const int rings[3] = {11, 8, 5};
    for (int D : rings)
        spawnFourWay(agents, *world, c, prof::normalDriver(), D, /*stagger=*/false);
}

void buildOvertake(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    world = std::make_unique<World>(60, 12, TS);
    buildHRoad(*world, 6, 0, 59);
    Personality slowTruck = prof::truckDriver();
    slowTruck.speedComplianceFactor = 0.5f;
    // Voie d'en face LIBRE -> le depassement se finalise.
    addCar  (agents, *world, 2,  6, 58, 6, prof::aggressiveDriver());
    addTruck(agents, *world, 12, 6, 58, 6, slowTruck);
}

void buildConvoyBlocked(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    world = std::make_unique<World>(70, 12, TS);
    buildHRoad(*world, 6, 0, 69);
    Personality slowTruck = prof::truckDriver();
    slowTruck.speedComplianceFactor = 0.5f;
    // Camion lent + voitures pressees derriere, MAIS flux continu en sens
    // inverse -> tout depassement est refuse (voie d'en face occupee).
    addTruck(agents, *world, 22, 6, 68, 6, slowTruck);
    addCar  (agents, *world, 14, 6, 68, 6, prof::aggressiveDriver());
    addCar  (agents, *world, 10, 6, 68, 6, prof::aggressiveDriver());
    for (int k = 0; k < 6; ++k)
        addCar(agents, *world, 60 - k * 8, 5, 1, 5, prof::normalDriver()); // sens inverse
}

void buildBreakdown(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    world = std::make_unique<World>(60, 12, TS);
    buildHRoad(*world, 6, 0, 59);
    // Camion en panne au milieu de la voie : la file derriere doit s'arreter
    // (et ne peut pas doubler car flux inverse). Demontre le cycle de vie panne.
    addTruck(agents, *world, 28, 6, 58, 6, prof::truckDriver());
    if (auto* veh = dynamic_cast<Vehicle*>(agents.back().get()))
        veh->forceBreakdown(18.f);
    for (int k = 0; k < 4; ++k)
        addCar(agents, *world, 22 - k * 4, 6, 58, 6, prof::normalDriver());
    for (int k = 0; k < 4; ++k)
        addCar(agents, *world, 55 - k * 8, 5, 1, 5, prof::normalDriver()); // sens inverse
}

// --- Massif : ville XXXXL ----------------------------------------------------

void buildCityXXXXL(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int N = 64;
    world = std::make_unique<World>(N, N, TS);
    World& w = *world;

    // Maillage regulier : 8 axes H + 8 axes V (espacement 8 tiles) -> 64
    // carrefours, tous mode confondus pour montrer l'heterogeneite.
    const int lines[8] = {4, 12, 20, 28, 36, 44, 52, 60};
    for (int r : lines) buildHRoad(w, r, 0, N - 1);
    for (int c : lines) buildVRoad(w, c, 0, N - 1);

    const RegulationType cyc[6] = {
        RegulationType::PRIORITY_RIGHT, RegulationType::TRAFFIC_LIGHT,
        RegulationType::STOP,           RegulationType::FIXED_PRIORITY,
        RegulationType::P2P,            RegulationType::AIM
    };
    int id = 1, k = 0;
    for (int r : lines)
        for (int c : lines)
            buildCrossroad(w, c, r, id++, cyc[k++ % 6]);

    // Points de spawn = toutes les tiles de route (hors carrefour). On tire des
    // paires (depart, arrivee) reproductibles et on garde les trajets non
    // triviaux. Tirage seede -> scene identique a chaque lancement.
    std::vector<core::TileCoord> roadTiles;
    for (int x = 0; x < N; ++x)
        for (int y = 0; y < N; ++y) {
            const RoadType rt = w.getTile(x, y).roadType;
            if (rt != RoadType::NONE && rt != RoadType::INTERSECTION)
                roadTiles.push_back({x, y});
        }

    core::Rng rng(0xC17C0DE5ULL);
    std::unordered_set<long long> usedStart;
    auto key = [N](core::TileCoord t) { return (long long)t.y * N + t.x; };

    constexpr int kTargetVehicles = 260;
    int spawned = 0, guard = 0;
    while (spawned < kTargetVehicles && guard < kTargetVehicles * 12) {
        ++guard;
        const auto a = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        const auto b = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        if (usedStart.count(key(a))) continue;                 // pas 2 spawns sur la meme tile
        auto path = AStarPlanner::findPath(w, a, b);
        if ((int)path.size() < 10) continue;                   // trajet trop court -> skip

        usedStart.insert(key(a));
        // ~10% de camions ; melange de profils pour un trafic vivant.
        const int dice = rng.uniformInt(0, 99);
        if (dice < 10) {
            addTruck(agents, w, a.x, a.y, b.x, b.y, prof::truckDriver());
        } else {
            const Personality p = (dice < 40) ? prof::aggressiveDriver()
                                : (dice < 70) ? prof::calmDriver()
                                              : prof::normalDriver();
            addCar(agents, w, a.x, a.y, b.x, b.y, p);
        }
        ++spawned;
    }
}

// =============================================================================
// Catalogue (ordre = ordre d'affichage)
// =============================================================================

std::vector<ScenarioDef> makeCatalog() {
    std::vector<ScenarioDef> cat;
    auto add = [&](const char* name, const char* category, const char* desc,
                   ScenarioBuildFn fn) {
        cat.push_back({name, category, desc, std::move(fn)});
    };
    auto mode = [](RegulationType r) {
        return [r](std::unique_ptr<World>& w, AgentVec& a) { buildModeDemo(w, a, r); };
    };

    // --- Bases ---
    add("Route libre (1 voiture)", "Bases",
        "Trajectoire libre, aucune interaction : l'IDM converge vers la vitesse desiree.",
        buildFreeRoad);
    add("Croisement fluide (2 voitures)", "Bases",
        "Priorite a droite, arrivees decalees : la seconde cede sans a-coup.",
        buildSmoothCrossing);
    add("File / car-following (peloton IDM)", "Bases",
        "Camion lent en tete : la file se cale derriere a distance de securite (IDM).",
        buildCarFollowing);

    // --- Carrefour : modes de regulation ---
    add("Priorite a droite", "Carrefour - modes",
        "4 vehicules, une branche chacun. Regle classique de priorite a droite.",
        mode(RegulationType::PRIORITY_RIGHT));
    add("STOP (2 voies)", "Carrefour - modes",
        "Axe principal prioritaire + axe secondaire au STOP : arret net puis insertion.",
        mode(RegulationType::STOP));
    add("Cedez le passage (YIELD)", "Carrefour - modes",
        "L'axe secondaire cede sans s'arreter si la voie est libre.",
        mode(RegulationType::YIELD));
    add("Feux tricolores", "Carrefour - modes",
        "Phases vert/orange/rouge alternees entre les deux axes.",
        mode(RegulationType::TRAFFIC_LIGHT));
    add("Priorite fixe (par axe)", "Carrefour - modes",
        "Strategie recherche : un axe garde la priorite en permanence.",
        mode(RegulationType::FIXED_PRIORITY));
    add("P2P (VANET)", "Carrefour - modes",
        "Negociation pair-a-pair decentralisee : claim/priorite emergente.",
        mode(RegulationType::P2P));
    add("AIM (reservation)", "Carrefour - modes",
        "Gestionnaire centralise par reservation de creneaux (Dresner & Stone).",
        mode(RegulationType::AIM));
    add("Peloton virtuel", "Carrefour - modes",
        "Projection 1D + paires meneur-suiveur : passage entrelace sans arret.",
        mode(RegulationType::VIRTUAL_PLATOON));
    add("Rond-point", "Carrefour - modes",
        "Anneau a sens unique legal : insertion en cedant aux vehicules sur l'anneau.",
        buildRoundaboutDemo);

    // --- Cas limites / degrades ---
    add("Arrivee simultanee x4 - P2P", "Cas limites",
        "Impasse temporelle : 4 vehicules arrivent ensemble. La negociation P2P "
        "departage et evite la collision.",
        [](std::unique_ptr<World>& w, AgentVec& a) {
            buildSimultaneousArrival(w, a, RegulationType::P2P);
        });
    add("Arrivee simultanee x4 - AIM", "Cas limites",
        "Meme impasse, resolue par reservation centralisee (comparaison avec P2P).",
        [](std::unique_ptr<World>& w, AgentVec& a) {
            buildSimultaneousArrival(w, a, RegulationType::AIM);
        });
    add("Forte densite P2P (degrade ~four-way stop)", "Cas limites",
        "12 vehicules convergents : le P2P se degrade en passage chacun-son-tour, "
        "le debit s'effondre.",
        buildHighDensityP2P);
    add("Depassement reussi", "Cas limites",
        "Voiture agressive derriere un camion lent, voie d'en face libre : double.",
        buildOvertake);
    add("Convoi non depassable", "Cas limites",
        "Flux inverse continu : tout depassement est refuse, la file suit le camion.",
        buildConvoyBlocked);
    add("Panne au milieu de la voie", "Cas limites",
        "Un camion tombe en panne : la file s'arrete et attend la reparation.",
        buildBreakdown);

    // --- Massif ---
    add("Ville mixte (demo comportements)", "Massif",
        "16 carrefours (feux / priorite / STOP) + grand rond-point central, "
        "depassements et convois : panorama general des comportements.",
        [](std::unique_ptr<World>& w, AgentVec& a) { buildDemoScenario(w, a, TS); });
    add("Ville XXXXL (~260 vehicules)", "Massif",
        "64 carrefours tous modes confondus, plusieurs centaines de vehicules : "
        "demonstration de scalabilite (le chargement peut prendre 1-2 s).",
        buildCityXXXXL);

    return cat;
}

} // namespace

const std::vector<ScenarioDef>& scenarioCatalog() {
    static const std::vector<ScenarioDef> kCatalog = makeCatalog();
    return kCatalog;
}

} // namespace scene
