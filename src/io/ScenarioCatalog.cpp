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
// SOUS-CARREFOUR : memes regles que Cross + spawnFourWay, mais avec des bornes
// locales (xMin..xMax, yMin..yMax). Permet d'empiler plusieurs carrefours
// INDEPENDANTS dans la meme scene (comparaisons visuelles cote a cote).
// =============================================================================
struct SubCross {
    int cx, cy;              // centre du carrefour
    int xMin, xMax;          // bornes x de la H-road locale (incluses)
    int yMin, yMax;          // bornes y de la V-road locale (incluses)
};

void spawnFourWaySub(AgentVec& agents, World& w, const SubCross& c,
                     const Personality& p, int D, bool stagger) {
    const int oW = stagger ? 2 : 0;
    const int oN = stagger ? 1 : 0;
    const int oS = stagger ? 3 : 0;
    addCar(agents, w, c.cx - D,      c.cy,          c.xMax - 1, c.cy,       p); // EST
    addCar(agents, w, c.cx + D + oW, c.cy - 1,      c.xMin + 1, c.cy - 1,   p); // OUEST
    addCar(agents, w, c.cx,          c.cy + D + oN, c.cx,        c.yMin + 1, p); // NORD
    addCar(agents, w, c.cx - 1,      c.cy - D - oS, c.cx - 1,    c.yMax - 1, p); // SUD
}

struct DoubleCross { SubCross top, bot; };

// Deux carrefours empiles VERTICALEMENT, separes par 'gap' tuiles vides : ce
// trou casse la connexite V-road -> A* ne peut JAMAIS tisser de path qui passe
// par les deux scenes. Chaque sous-monde est strictement isole.
DoubleCross makeTwoCrossroads(std::unique_ptr<World>& world,
                              RegulationType regTop, RegulationType regBot,
                              int W = 30, int halfH = 30, int gap = 4) {
    const int H = 2 * halfH + gap;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    const int cx = W / 2;
    const int cyTop = halfH / 2;
    const int cyBot = halfH + gap + halfH / 2;
    // HAUT : H-road row cyTop, V-road LOCALE [0..halfH-1].
    buildHRoad(w, cyTop, 0, W - 1);
    buildVRoad(w, cx, 0, halfH - 1);
    buildCrossroad(w, cx, cyTop, 1, regTop);
    // BAS  : H-road row cyBot, V-road LOCALE [halfH+gap..H-1].
    buildHRoad(w, cyBot, 0, W - 1);
    buildVRoad(w, cx, halfH + gap, H - 1);
    buildCrossroad(w, cx, cyBot, 2, regBot);
    SubCross top{cx, cyTop, 0, W - 1, 0, halfH - 1};
    SubCross bot{cx, cyBot, 0, W - 1, halfH + gap, H - 1};
    return {top, bot};
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

// Convoi de camions : demonstration du PLATOONING REEL (file rapprochee facon
// CACC -- Cooperative Adaptive Cruise Control) face a un trafic conventionnel.
//
// Deux autoroutes paralleles, MEME nombre de camions, MEME depart, MEME
// espacement initial. Seul change le couplage inter-vehiculaire :
//   * voie HAUTE  : platoon CACC. Temps inter-vehiculaire court (camions
//     connectes qui anticipent le freinage du leader) -> la file reste COMPACTE
//     et avance en bloc. Interet : debit eleve (plus de camions/km), sillage
//     aerodynamique (moins de trainee -> economie de carburant/CO2), flux lisse.
//   * voie BASSE  : camions conventionnels. Grand intervalle de securite (temps
//     de reaction humain) -> la file s'ETALE des le demarrage, debit plus faible.
//
// A observer : l'ecart entre vehicules et la longueur totale de chaque file ;
// le panneau "Metriques" chiffre debit / vitesse moyenne sur l'ensemble.
void buildTruckPlatoon(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 60, H = 14;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;

    // Deux routes 2 voies (CITY_50) : seule la voie EST (RIGHT) est peuplee.
    constexpr int yPlatoon = 4;     // voie HAUTE  (platoon CACC)
    constexpr int yNormal  = 9;     // voie BASSE  (conventionnel)
    buildHRoad(w, yPlatoon, 0, W - 1);
    buildHRoad(w, yNormal,  0, W - 1);

    // Platoon CACC : intervalle TRES court (T ~0.5 s), lancement synchronise
    // (acceleration vive, vehicules connectes) -> la file demarre EN BLOC et
    // reste compacte. Fiable (pas de panne dans la demo).
    Personality platoon = prof::truckDriver();
    platoon.cooperative           = true;    // CACC : unisson + distance constante
    platoon.reactionTimeFactor    = 0.30f;   // (repli IDM court si CACC desactive)
    platoon.minGapFactor          = 0.40f;   // s0 reduit -> distance constante courte
    platoon.accelEagernessFactor  = 1.80f;   // capacite d'accel du leader (surge)
    platoon.comfortBrakeFactor    = 1.20f;
    platoon.speedComplianceFactor = 1.00f;
    platoon.overtakeWillingness   = 0.0f;    // on ne rompt jamais la formation
    platoon.breakdownChancePerMin = 0.0f;

    // Conventionnel : profil camion standard -> accel LENTE + grand intervalle
    // de securite. Au demarrage, chaque camion attend que celui de devant degage
    // (effet "temps de reaction") -> la file s'ETALE en accordeon.
    Personality normal = prof::truckDriver();
    normal.breakdownChancePerMin  = 0.0f;

    // CLE de la demo : les DEUX files demarrent COLLEES et A L'ARRET (espacement
    // 2 tuiles ~ 20 px pare-chocs, comme a un feu rouge). On observe ensuite le
    // LANCEMENT : le platoon part en bloc et reste serre ; la file classique
    // s'etire (accel lente + grand intervalle) facon "feu qui passe au vert".
    constexpr int kCount = 8;
    for (int i = 0; i < kCount; ++i) {
        const int sx = 2 + i * 2;        // 2 tuiles -> ~20 px entre pare-chocs (colle)
        addTruck(agents, w, sx, yPlatoon, W - 2, yPlatoon, platoon);
        addTruck(agents, w, sx, yNormal,  W - 2, yNormal,  normal);
    }
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

// ORCA / espace ouvert : un large carrefour SANS lignes directrices strictes
// regule en evitement continu reciproque. ~10 vehicules convergent des quatre
// branches a des distances etagees : ils ne s'arretent pas (sauf conflit
// imminent) mais modulent leur vitesse pour s'entrelacer en continu. Met en
// evidence le comportement de la policy ORCA face a une densite moderee.
void buildOpenSpaceOrca(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    Cross c = makeCrossroad(world, RegulationType::ORCA);
    World& w = *world;
    const Personality p = prof::normalDriver();

    // EST (row cy, +x) : 3 vehicules etages, tout droit.
    for (int D : {12, 8, 5}) addCar(agents, w, c.cx - D, c.cy, c.W - 2, c.cy, p);
    // OUEST (row cy-1, -x) : 3 vehicules etages, tout droit.
    for (int D : {12, 8, 5}) addCar(agents, w, c.cx + D, c.cy - 1, 1, c.cy - 1, p);
    // NORD (col cx, -y) : 2 vehicules etages, tout droit.
    for (int D : {10, 6}) addCar(agents, w, c.cx, c.cy + D, c.cx, 1, p);
    // SUD (col cx-1, +y) : 2 vehicules etages, tout droit.
    for (int D : {10, 6}) addCar(agents, w, c.cx - 1, c.cy - D, c.cx - 1, c.H - 2, p);
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

// --- Comparaisons visuelles : deux (ou trois) scenes COTE A COTE -------------
//
// Memes spawns, memes profils : seul change le mode ou le parametrage compare.
// On voit DIRECTEMENT les comportements en parallele -> utile en presentation.

void buildComparePRvsSTOP(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::PRIORITY_RIGHT, RegulationType::STOP);
    spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), 10, /*stagger=*/true);
    spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildComparePRvsYIELD(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::PRIORITY_RIGHT, RegulationType::YIELD);
    spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), 10, /*stagger=*/true);
    spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildCompareP2PvsAIM(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::P2P, RegulationType::AIM);
    // Arrivee SIMULTANEE (stagger=false) : impasse temporelle maximale.
    spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), 10, /*stagger=*/false);
    spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), 10, /*stagger=*/false);
}

void buildCompareDensityP2PvsAIM(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::P2P, RegulationType::AIM);
    // 3 anneaux concentriques -> 12 vehicules par carrefour.
    const int rings[3] = {11, 8, 5};
    for (int D : rings) {
        spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), D, /*stagger=*/false);
        spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), D, /*stagger=*/false);
    }
}

void buildCompareLightsVsPlatoon(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::TRAFFIC_LIGHT, RegulationType::VIRTUAL_PLATOON);
    spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), 10, /*stagger=*/true);
    spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildCompareFixedVsP2P(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    const DoubleCross dc = makeTwoCrossroads(world,
        RegulationType::FIXED_PRIORITY, RegulationType::P2P);
    spawnFourWaySub(agents, *world, dc.top, prof::normalDriver(), 10, /*stagger=*/true);
    spawnFourWaySub(agents, *world, dc.bot, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildCompareLightsVsRoundabout(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 30, halfH = 30, gap = 4;
    const int H = 2 * halfH + gap;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    const int cx = W / 2;
    const int cyTop = halfH / 2;
    const int cyBot = halfH + gap + halfH / 2;
    // HAUT : carrefour a feux.
    buildHRoad(w, cyTop, 0, W - 1);
    buildVRoad(w, cx, 0, halfH - 1);
    buildCrossroad(w, cx, cyTop, 1, RegulationType::TRAFFIC_LIGHT);
    // BAS : rond-point cote 4 centre approximativement sur (cx, cyBot).
    buildHRoad(w, cyBot, 0, W - 1);
    buildVRoad(w, cx, halfH + gap, H - 1);
    buildRoundabout(w, cx - 2, cyBot - 2, 2, 4);
    w.refreshRoundaboutApproaches();
    SubCross top{cx, cyTop, 0, W - 1, 0, halfH - 1};
    SubCross bot{cx, cyBot, 0, W - 1, halfH + gap, H - 1};
    spawnFourWaySub(agents, w, top, prof::normalDriver(), 10, /*stagger=*/true);
    spawnFourWaySub(agents, w, bot, prof::normalDriver(), 10, /*stagger=*/true);
}

void buildCompareProfiles(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 60, H = 18;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    // 3 H-roads independantes (chaque buildHRoad cree row y RIGHT + row y-1 LEFT).
    const int rows[3] = {4, 9, 14};
    for (int y : rows) buildHRoad(w, y, 0, W - 1);

    Personality slowTruck = prof::truckDriver();
    slowTruck.speedComplianceFactor = 0.5f;
    const Personality profilesArr[3] = {
        prof::aggressiveDriver(), prof::normalDriver(), prof::calmDriver()
    };
    // HAUT = agressif | MILIEU = normal | BAS = calme : camion lent + 4 suiveurs.
    for (int i = 0; i < 3; ++i) {
        const int y = rows[i];
        addTruck(agents, w, 20, y, W - 2, y, slowTruck);
        for (int k = 0; k < 4; ++k)
            addCar(agents, w, 16 - k * 4, y, W - 2, y, profilesArr[i]);
    }
}

void buildCompareOvertake(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 60, H = 14;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;
    const int yTop = 4, yBot = 9;
    buildHRoad(w, yTop, 0, W - 1);
    buildHRoad(w, yBot, 0, W - 1);

    Personality slowTruck = prof::truckDriver();
    slowTruck.speedComplianceFactor = 0.5f;
    // HAUT : agressifs (depassent). BAS : calmes (restent derriere).
    addTruck(agents, w, 20, yTop, W - 2, yTop, slowTruck);
    addCar  (agents, w, 12, yTop, W - 2, yTop, prof::aggressiveDriver());
    addCar  (agents, w,  6, yTop, W - 2, yTop, prof::aggressiveDriver());
    addTruck(agents, w, 20, yBot, W - 2, yBot, slowTruck);
    addCar  (agents, w, 12, yBot, W - 2, yBot, prof::calmDriver());
    addCar  (agents, w,  6, yBot, W - 2, yBot, prof::calmDriver());
}

// --- Massif : ville XXXXL ----------------------------------------------------

void buildCityXXXXL(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    // Carte RECTANGULAIRE (pas carree) -> deja moins "grille parfaite".
    constexpr int W = 92, H = 72;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;

    // --- Reseau IRREGULIER ---------------------------------------------------
    // Espacement NON uniforme des axes -> blocs de tailles variees (quartiers).
    // Toutes les artères traversent (réseau pleinement connecté pour l'A*), mais
    // l'irrégularité + les ronds-points cassent l'aspect "damier".
    const int rows[7] = {5, 14, 22, 33, 45, 55, 65};
    const int cols[8] = {6, 15, 25, 38, 50, 60, 72, 84};
    const int nR = 7, nC = 8;
    for (int i = 0; i < nR; ++i) buildHRoad(w, rows[i], 0, W - 1);
    for (int j = 0; j < nC; ++j) buildVRoad(w, cols[j], 0, H - 1);

    // --- Ronds-points de tailles VARIEES (côté 4 / 6 / 8) --------------------
    // Placés sur certains croisements (indices col, ligne) ; centre (C,R) ->
    // coin haut-gauche (C - side/2, R - side/2). Le reste = carrefours, modes
    // alternés (priorité / feux / STOP / fixe / P2P / AIM) pour l'hétérogénéité.
    struct RB { int ci, ri, side; };
    const RB rbs[6] = {
        {1, 1, 6}, {4, 0, 4}, {2, 3, 8},
        {5, 4, 6}, {6, 2, 4}, {3, 5, 6}
    };
    auto roundaboutSide = [&](int ci, int ri) -> int {
        for (const auto& rb : rbs) if (rb.ci == ci && rb.ri == ri) return rb.side;
        return 0;
    };

    const RegulationType cyc[6] = {
        RegulationType::PRIORITY_RIGHT, RegulationType::TRAFFIC_LIGHT,
        RegulationType::STOP,           RegulationType::FIXED_PRIORITY,
        RegulationType::P2P,            RegulationType::AIM
    };
    int id = 1, k = 0;
    for (int ri = 0; ri < nR; ++ri)
        for (int ci = 0; ci < nC; ++ci) {
            const int C = cols[ci], R = rows[ri];
            const int side = roundaboutSide(ci, ri);
            if (side > 0) buildRoundabout(w, C - side / 2, R - side / 2, id++, side);
            else          buildCrossroad(w, C, R, id++, cyc[k++ % 6]);
        }
    w.refreshRoundaboutApproaches();   // (idempotent) recalcule les branches

    // Points de spawn = toutes les tiles de route (hors carrefour). On tire des
    // paires (depart, arrivee) reproductibles et on garde les trajets non
    // triviaux. Tirage seede -> scene identique a chaque lancement.
    std::vector<core::TileCoord> roadTiles;
    for (int x = 0; x < W; ++x)
        for (int y = 0; y < H; ++y) {
            const RoadType rt = w.getTile(x, y).roadType;
            if (rt != RoadType::NONE && rt != RoadType::INTERSECTION)
                roadTiles.push_back({x, y});
        }

    core::Rng rng(0xC17C0DE5ULL);
    std::unordered_set<long long> usedStart;
    auto key = [W](core::TileCoord t) { return (long long)t.y * W + t.x; };

    constexpr int kTargetVehicles = 500;
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

// Variante de la ville XXXXL : VOITURES UNIQUEMENT, ~700 vehicules. Reseau
// IDENTIQUE a buildCityXXXXL (memes axes, memes ronds-points, memes regulations,
// meme seed RNG) -> scenes superposables pour A/B mental. Differences :
//   * aucun camion (pas de carrosserie longue qui bouchonne en virage),
//   * profils repartis 40 agressif / 30 normal / 30 calme (densite + dynamique),
//   * budget de spawn plus haut (700 vs 500).
// Stress-test de la logique d'intersection densifie : plus de paires en conflit
// simultane -> revele wedges/cycles que les fixes anti-deadlock doivent absorber.
void buildCityXXXXLCarsOnly(std::unique_ptr<World>& world, AgentVec& agents) {
    agents.clear();
    constexpr int W = 92, H = 72;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;

    const int rows[7] = {5, 14, 22, 33, 45, 55, 65};
    const int cols[8] = {6, 15, 25, 38, 50, 60, 72, 84};
    const int nR = 7, nC = 8;
    for (int i = 0; i < nR; ++i) buildHRoad(w, rows[i], 0, W - 1);
    for (int j = 0; j < nC; ++j) buildVRoad(w, cols[j], 0, H - 1);

    struct RB { int ci, ri, side; };
    const RB rbs[6] = {
        {1, 1, 6}, {4, 0, 4}, {2, 3, 8},
        {5, 4, 6}, {6, 2, 4}, {3, 5, 6}
    };
    auto roundaboutSide = [&](int ci, int ri) -> int {
        for (const auto& rb : rbs) if (rb.ci == ci && rb.ri == ri) return rb.side;
        return 0;
    };

    const RegulationType cyc[6] = {
        RegulationType::PRIORITY_RIGHT, RegulationType::TRAFFIC_LIGHT,
        RegulationType::STOP,           RegulationType::FIXED_PRIORITY,
        RegulationType::P2P,            RegulationType::AIM
    };
    int id = 1, k = 0;
    for (int ri = 0; ri < nR; ++ri)
        for (int ci = 0; ci < nC; ++ci) {
            const int C = cols[ci], R = rows[ri];
            const int side = roundaboutSide(ci, ri);
            if (side > 0) buildRoundabout(w, C - side / 2, R - side / 2, id++, side);
            else          buildCrossroad(w, C, R, id++, cyc[k++ % 6]);
        }
    w.refreshRoundaboutApproaches();

    std::vector<core::TileCoord> roadTiles;
    for (int x = 0; x < W; ++x)
        for (int y = 0; y < H; ++y) {
            const RoadType rt = w.getTile(x, y).roadType;
            if (rt != RoadType::NONE && rt != RoadType::INTERSECTION)
                roadTiles.push_back({x, y});
        }

    // Seed DIFFERENT de buildCityXXXXL : evite de tomber sur les memes 500
    // trajets puis d'en ajouter 200 -- on veut un melange neuf pour cette variante.
    core::Rng rng(0xCA75CA75ULL);
    std::unordered_set<long long> usedStart;
    auto key = [W](core::TileCoord t) { return (long long)t.y * W + t.x; };

    constexpr int kTargetVehicles = 700;
    int spawned = 0, guard = 0;
    while (spawned < kTargetVehicles && guard < kTargetVehicles * 12) {
        ++guard;
        const auto a = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        const auto b = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        if (usedStart.count(key(a))) continue;
        auto path = AStarPlanner::findPath(w, a, b);
        if ((int)path.size() < 10) continue;

        usedStart.insert(key(a));
        const int dice = rng.uniformInt(0, 99);
        const Personality p = (dice < 40) ? prof::aggressiveDriver()
                            : (dice < 70) ? prof::calmDriver()
                                          : prof::normalDriver();
        addCar(agents, w, a.x, a.y, b.x, b.y, p);
        ++spawned;
    }
}

// Grande ville "realiste simple" : uniquement des voitures et des carrefours
// classiques 2x2. Aucun rond-point, camion, P2P/AIM/ORCA/peloton : seulement les
// modes qu'on retrouve dans la conduite ordinaire (feux, STOP, cedez, priorites).
void buildHugeRealCarsSimpleIntersections(std::unique_ptr<World>& world,
                                           AgentVec& agents) {
    agents.clear();
    constexpr int W = 112, H = 84;
    world = std::make_unique<World>(W, H, TS);
    World& w = *world;

    const int rows[9] = {6, 14, 23, 31, 41, 50, 60, 69, 78};
    const int cols[10] = {7, 17, 28, 39, 51, 62, 73, 84, 96, 105};
    constexpr int nR = 9, nC = 10;
    for (int i = 0; i < nR; ++i) buildHRoad(w, rows[i], 0, W - 1);
    for (int j = 0; j < nC; ++j) buildVRoad(w, cols[j], 0, H - 1);

    const RegulationType realRegs[5] = {
        RegulationType::TRAFFIC_LIGHT,
        RegulationType::PRIORITY_RIGHT,
        RegulationType::STOP,
        RegulationType::YIELD,
        RegulationType::FIXED_PRIORITY
    };
    int id = 1;
    for (int ri = 0; ri < nR; ++ri) {
        for (int ci = 0; ci < nC; ++ci) {
            const int pattern = (ri * 3 + ci * 2 + (ri + ci) / 3) % 5;
            buildCrossroad(w, cols[ci], rows[ri], id++, realRegs[pattern]);
        }
    }

    std::vector<core::TileCoord> roadTiles;
    for (int x = 0; x < W; ++x)
        for (int y = 0; y < H; ++y) {
            const RoadType rt = w.getTile(x, y).roadType;
            if (rt != RoadType::NONE && rt != RoadType::INTERSECTION)
                roadTiles.push_back({x, y});
        }

    core::Rng rng(0xB16C4175ULL);
    std::unordered_set<long long> usedStart;
    auto key = [W](core::TileCoord t) { return (long long)t.y * W + t.x; };

    constexpr int kTargetVehicles = 850;
    int spawned = 0, guard = 0;
    while (spawned < kTargetVehicles && guard < kTargetVehicles * 14) {
        ++guard;
        const auto a = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        const auto b = roadTiles[rng.uniformInt(0, (int)roadTiles.size() - 1)];
        if (usedStart.count(key(a))) continue;

        const auto path = AStarPlanner::findPath(w, a, b);
        if ((int)path.size() < 14) continue;

        usedStart.insert(key(a));
        const int dice = rng.uniformInt(0, 99);
        const Personality p = (dice < 20) ? prof::aggressiveDriver()
                            : (dice < 50) ? prof::calmDriver()
                                          : prof::normalDriver();
        addCar(agents, w, a.x, a.y, b.x, b.y, p);
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
    add("Convoi de camions : platoon CACC vs conventionnel", "Bases",
        "Deux files de 8 camions demarrent COLLEES et a l'arret (comme a un feu). "
        "Au vert : en HAUT le platoon CACC s'elance EN BLOC et reste serre ; en "
        "BAS les camions classiques s'etalent en accordeon (accel lente + grand "
        "intervalle de securite). Montre concretement l'interet du platooning.",
        buildTruckPlatoon);

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
    add("ORCA / espace ouvert (~10 vehicules)", "Carrefour - modes",
        "Large carrefour sans lignes directrices : ~10 vehicules naviguent par "
        "evitement de collision CONTINU et reciproque, modulant leur vitesse pour "
        "s'entrelacer plutot que de s'arreter.",
        buildOpenSpaceOrca);
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

    // --- Comparaisons visuelles : deux scenes empilees ---
    add("Compare carrefour : Priorite a droite vs STOP", "Comparaisons visuelles",
        "Deux carrefours identiques empiles. EN HAUT : priorite a droite. EN BAS : "
        "STOP. Memes vehicules, memes arrivees decalees. Differences de debit et de "
        "fluidite visibles d'un coup d'oeil.",
        buildComparePRvsSTOP);
    add("Compare carrefour : Priorite a droite vs Cedez (YIELD)", "Comparaisons visuelles",
        "Priorite a droite (HAUT) vs cedez le passage (BAS). Le YIELD ne s'arrete "
        "que si necessaire ; ailleurs il passe sans toucher au frein.",
        buildComparePRvsYIELD);
    add("Compare carrefour : P2P (VANET) vs AIM (reservation)", "Comparaisons visuelles",
        "Arrivee SIMULTANEE des 4 vehicules sur chaque carrefour. HAUT : negociation "
        "pair-a-pair decentralisee. BAS : reservation centralisee. Comparaison "
        "directe des deux approches recherche.",
        buildCompareP2PvsAIM);
    add("Compare densite : P2P vs AIM (12 vehicules / carrefour)", "Comparaisons visuelles",
        "Forte densite : 12 vehicules par carrefour (3 anneaux concentriques x 4 "
        "branches). P2P degrade en four-way stop / AIM ordonnance les creneaux : "
        "l'ecart de debit est tres marque.",
        buildCompareDensityP2PvsAIM);
    add("Compare carrefour : Feux vs Peloton virtuel", "Comparaisons visuelles",
        "Memes flux des deux cotes. Feux tricolores (HAUT) imposent des arrets ; "
        "le peloton virtuel (BAS) entrelace les passages sans s'arreter.",
        buildCompareLightsVsPlatoon);
    add("Compare carrefour : Priorite fixe vs P2P", "Comparaisons visuelles",
        "Strategie 'un axe garde la main' (HAUT) vs negociation P2P egalitaire (BAS). "
        "Le bras prioritaire ne s'arrete jamais ; le P2P decide cas par cas.",
        buildCompareFixedVsP2P);
    add("Compare carrefour : Feux vs Rond-point", "Comparaisons visuelles",
        "Feux tricolores (HAUT) vs rond-point cote 4 (BAS). Meme flux : on voit la "
        "phase d'arret aux feux face a la continuite du rond-point.",
        buildCompareLightsVsRoundabout);
    add("Compare profils : agressif / normal / calme", "Comparaisons visuelles",
        "Trois voies paralleles, meme camion lent en tete + 4 suiveurs. "
        "HAUT : conducteurs agressifs (gap court, depassements). MILIEU : normaux. "
        "BAS : calmes (gap large, pas de depassement). Differences d'IDM et de "
        "willingness en un coup d'oeil.",
        buildCompareProfiles);
    add("Compare depassement : agressif vs calme", "Comparaisons visuelles",
        "Meme camion lent + 2 suiveurs sur chaque voie. HAUT : agressifs qui "
        "doublent. BAS : calmes qui restent derriere. Demontre overtakeWillingness.",
        buildCompareOvertake);

    // --- Massif ---
    add("Ville mixte (demo comportements)", "Massif",
        "16 carrefours (feux / priorite / STOP) + grand rond-point central, "
        "depassements et convois : panorama general des comportements.",
        [](std::unique_ptr<World>& w, AgentVec& a) { buildDemoScenario(w, a, TS); });
    add("Ville XXXXL (~500 vehicules, reseau irregulier)", "Massif",
        "Ville 92x72 a maillage IRREGULIER (blocs de tailles variees) avec 6 "
        "ronds-points de tailles differentes (cote 4/6/8) parmi ~50 carrefours "
        "tous modes confondus, ~500 vehicules actifs : scalabilite "
        "(multithreading + partitionnement spatial) et anti-gridlock 'keep clear'. "
        "Le chargement (trajets A*) peut prendre quelques secondes ; dezoomez a la "
        "molette pour embrasser toute la ville.",
        buildCityXXXXL);
    add("Ville XXXXL VOITURES uniquement (~700 vehicules)", "Massif",
        "Meme reseau XXXXL (92x72, 6 ronds-points, tous modes de carrefour) mais "
        "AUCUN camion : 700 voitures, profils repartis 40/30/30 (agressif/normal/"
        "calme). Stress-test des fixes anti-wedge en haute densite -- aucun "
        "blocage mutuel ne doit subsister plus de ~1 sec.",
        buildCityXXXXLCarsOnly);
    add("GROSSE ville reelle : voitures + carrefours simples (~850)", "Massif",
        "Grande ville 112x84 avec uniquement des voitures et des carrefours 2x2 "
        "classiques : feux, STOP, cedez-le-passage, priorite a droite et axe "
        "prioritaire. Aucun camion, rond-point, P2P, AIM, ORCA ou peloton.",
        buildHugeRealCarsSimpleIntersections);

    return cat;
}

} // namespace

const std::vector<ScenarioDef>& scenarioCatalog() {
    static const std::vector<ScenarioDef> kCatalog = makeCatalog();
    return kCatalog;
}

} // namespace scene
