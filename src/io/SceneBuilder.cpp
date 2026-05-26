// src/SceneBuilder.cpp
#include "io/SceneBuilder.hpp"

#include "core/agent/Car.hpp"
#include "core/agent/Personality.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/agent/Truck.hpp"

namespace scene {

void buildCrossroad(World& w, int cx, int cy, int id, RegulationType regType) {
    w.setTile(cx - 1, cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx - 1, cy,     RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy,     RoadType::INTERSECTION, TileDirection::NONE);

    Intersection inter(id, regType);
    inter.addCoveredTile({cx - 1, cy - 1});
    inter.addCoveredTile({cx,     cy - 1});
    inter.addCoveredTile({cx - 1, cy});
    inter.addCoveredTile({cx,     cy});

    Approach north; north.direction = Approach::Direction::NORTH; north.entryTile = {cx,     cy - 2}; inter.addApproach(north);
    Approach south; south.direction = Approach::Direction::SOUTH; south.entryTile = {cx - 1, cy + 1}; inter.addApproach(south);
    Approach east;  east.direction  = Approach::Direction::EAST;  east.entryTile  = {cx + 1, cy - 1}; inter.addApproach(east);
    Approach west;  west.direction  = Approach::Direction::WEST;  west.entryTile  = {cx - 2, cy};     inter.addApproach(west);

    // STOP 2 voies (jamais all-way) : l'axe le plus RAPIDE est la route
    // PRINCIPALE prioritaire ; l'autre porte le panneau STOP. Egalite de
    // vitesse -> axe horizontal prioritaire par convention.
    if (regType == RegulationType::STOP) {
        const float hE = getRoadSpeedLimit(w.getTile(cx + 1, cy - 1).roadType);
        const float hW = getRoadSpeedLimit(w.getTile(cx - 2, cy).roadType);
        const float vN = getRoadSpeedLimit(w.getTile(cx, cy - 2).roadType);
        const float vS = getRoadSpeedLimit(w.getTile(cx - 1, cy + 1).roadType);
        const float hSpeed = (hE > hW) ? hE : hW;
        const float vSpeed = (vN > vS) ? vN : vS;
        inter.setStopMajorAxisHorizontal(hSpeed >= vSpeed);
    }

    w.addIntersection(inter);
}

void buildRoundabout(World& w, int x0, int y0, int id, int sideTiles) {
    if (sideTiles < 2)       sideTiles = 2;
    if (sideTiles % 2 != 0)  ++sideTiles;       // cote PAIR impose

    // Bloc carre [x0, x1] x [y0, y1] de cote sideTiles. Anneau = bloc - ilot
    // central creux (pelouse) de cote (sideTiles-2), centre.
    const int x1 = x0 + sideTiles - 1;
    const int y1 = y0 + sideTiles - 1;

    const int hole = sideTiles - 2;             // 0 pour un mini 2x2 (pas d'ilot)
    const int hx0 = x0 + 1, hx1 = x1 - 1;
    const int hy0 = y0 + 1, hy1 = y1 - 1;

    Intersection inter(id, RegulationType::ROUNDABOUT);

    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            if (hole > 0 && x >= hx0 && x <= hx1 && y >= hy0 && y <= hy1) {
                w.setTile(x, y, RoadType::NONE, TileDirection::NONE);
                continue;
            }
            w.setTile(x, y, RoadType::INTERSECTION, TileDirection::NONE);
            inter.addCoveredTile({x, y});
        }
    }

    w.addIntersection(inter);

    // Branches dynamiques : 2, 3 ou 4 sorties selon les routes raccordees au
    // pourtour. Recalcule a chaque edition de route (cf. World::refresh...).
    w.refreshRoundaboutApproaches();
}

void buildHRoad(World& world, int y, int xStart, int xEnd) {
    for (int x = xStart; x <= xEnd; ++x) {
        world.setTile(x, y,     RoadType::CITY_50, TileDirection::RIGHT);
        world.setTile(x, y - 1, RoadType::CITY_50, TileDirection::LEFT);
    }
}

void buildVRoad(World& world, int x, int yStart, int yEnd) {
    for (int y = yStart; y <= yEnd; ++y) {
        world.setTile(x,     y, RoadType::CITY_50, TileDirection::UP);
        world.setTile(x - 1, y, RoadType::CITY_50, TileDirection::DOWN);
    }
}

void buildDefaultNetwork(World& world) {
    buildHRoad(world, 11, 0, 31);
    buildHRoad(world, 22, 0, 31);
    buildVRoad(world, 11, 0, 31);
    buildVRoad(world, 22, 0, 31);

    buildCrossroad(world, 11, 11, 0, RegulationType::PRIORITY_RIGHT);
    buildCrossroad(world, 22, 11, 1, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(world, 11, 22, 2, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(world, 22, 22, 3, RegulationType::PRIORITY_RIGHT);
}

void spawnDefaultAgents(std::vector<std::unique_ptr<IAgent>>& agents, World& world) {
    const float tileSize = world.getTileSize();

    auto addCar = [&](int sx, int sy, int gx, int gy) {
        auto c = std::make_unique<Car>(sx * tileSize + tileSize / 2.f,
                                       sy * tileSize + tileSize / 2.f);
        c->setPath(AStarPlanner::findPath(world, {sx, sy}, {gx, gy}), &world);
        agents.push_back(std::move(c));
    };
    auto addTruck = [&](int sx, int sy, int gx, int gy) {
        auto t = std::make_unique<Truck>(sx * tileSize + tileSize / 2.f,
                                         sy * tileSize + tileSize / 2.f);
        t->setPath(AStarPlanner::findPath(world, {sx, sy}, {gx, gy}), &world);
        agents.push_back(std::move(t));
    };

    addCar(1, 11, 21, 30);
    addTruck(30, 10, 10, 30);
    addCar(30, 10, 1, 10);
    addCar(11, 30, 11, 1);
    addCar(10, 1, 10, 30);
    addTruck(1, 22, 28, 22);
    addCar(21, 1, 21, 30);
    addCar(11, 14, 10, 21);
}

void buildDemoScenario(std::unique_ptr<World>& world,
                       std::vector<std::unique_ptr<IAgent>>& agents,
                       float tileSize)
{
    using core::agent::profiles::normalDriver;
    using core::agent::profiles::aggressiveDriver;
    using core::agent::profiles::truckDriver;

    using core::agent::profiles::calmDriver;

    // Car/Truck figent leur tileSize interne a 50 : on garde le monde coherent.
    const float ts = 50.f;
    (void)tileSize;

    constexpr int W = 72, H = 48;
    world = std::make_unique<World>(W, H, ts);
    agents.clear();
    World& w = *world;

    // --- Reseau maille, entierement connecte -----------------------------
    // 4 axes horizontaux + 4 axes verticaux -> 16 carrefours. Le centre
    // (28,18) est un GRAND rond-point ; les autres alternent feux/priorite/stop.
    const int Hrows[4] = {6, 18, 30, 42};
    const int Vcols[4] = {10, 28, 46, 64};
    for (int r = 0; r < 4; ++r) buildHRoad(w, Hrows[r], 0, W - 1);
    for (int c = 0; c < 4; ++c) buildVRoad(w, Vcols[c], 0, H - 1);

    const RegulationType cyc[3] = {
        RegulationType::TRAFFIC_LIGHT,
        RegulationType::PRIORITY_RIGHT,
        RegulationType::STOP
    };
    int id = 1, ci = 0;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            const int C = Vcols[c], R = Hrows[r];
            if (C == 28 && R == 18) {
                // Rond-point central, cote PAIR = 4 tiles, centre sur (28,18)
                // -> coin haut-gauche (26,16). Ilot central 2x2.
                buildRoundabout(w, 26, 16, id++, 4);
            } else {
                buildCrossroad(w, C, R, id++, cyc[ci++ % 3]);
            }
        }
    }

    // --- Agents ----------------------------------------------------------
    auto addCar = [&](int sx, int sy, int gx, int gy,
                      const core::agent::Personality& p) {
        auto c = std::make_unique<Car>(sx * ts + ts / 2.f, sy * ts + ts / 2.f);
        c->setPersonality(p);
        c->setPath(AStarPlanner::findPath(w, {sx, sy}, {gx, gy}), &w);
        agents.push_back(std::move(c));
    };
    auto addTruck = [&](int sx, int sy, int gx, int gy,
                        const core::agent::Personality& p) {
        auto t = std::make_unique<Truck>(sx * ts + ts / 2.f, sy * ts + ts / 2.f);
        t->setPersonality(p);
        t->setPath(AStarPlanner::findPath(w, {sx, sy}, {gx, gy}), &w);
        agents.push_back(std::move(t));
    };

    // Camion LENT (roule bien sous la limite) -> ecart de vitesse suffisant
    // pour qu'un depassement puisse etre finalise avant le carrefour suivant.
    core::agent::Personality slowTruck = truckDriver();
    slowTruck.speedComplianceFactor = 0.5f;

    // Vers l'EST (voie RIGHT, ligne R). Paires camion-lent + agressive = depassement.
    // Le camion est place juste apres le 1er carrefour (col 10) : la voiture le
    // rattrape tot et a ~12 tiles de droit jusqu'au carrefour suivant (col 28),
    // assez pour finaliser le depassement (sinon il serait refuse).
    addCar  ( 2,  6, 70,  6, aggressiveDriver());   // rattrape le camion -> double
    addTruck(13,  6, 70,  6, slowTruck);            // leader lent
    addCar  ( 2, 18, 70, 18, normalDriver());       // traverse le rond-point E->O
    addCar  ( 2, 30, 70, 30, normalDriver());
    addCar  ( 2, 42, 70, 42, aggressiveDriver());   // double le camion ci-dessous
    addTruck(13, 42, 70, 42, slowTruck);
    addCar  ( 4, 18, 28,  1, normalDriver());       // rond-point : ouest -> nord (tourne)

    // Vers l'OUEST (voie LEFT, ligne R-1).
    addCar  (70,  5,  1,  5, calmDriver());
    addCar  (70, 17,  1, 17, normalDriver());        // rond-point ouest
    addCar  (70, 29,  1, 29, aggressiveDriver());
    addCar  (70, 41,  1, 41, normalDriver());

    // Vers le NORD (voie UP, colonne C).
    addCar  (10, 47, 10,  1, normalDriver());
    addCar  (28, 47, 28,  1, normalDriver());        // rond-point sud -> nord
    addCar  (46, 47, 46,  1, aggressiveDriver());
    addCar  (64, 47, 64,  1, calmDriver());

    // Vers le SUD (voie DOWN, colonne C-1).
    addCar  ( 9,  1,  9, 46, normalDriver());
    addCar  (27,  1, 27, 46, normalDriver());        // rond-point nord -> sud
    addTruck(45,  1, 45, 46, truckDriver());
    addCar  (63,  1, 63, 46, normalDriver());

    // Trajet long en travers de toute la carte (plusieurs intersections).
    addCar  ( 4, 30, 64,  1, aggressiveDriver());
}

} // namespace scene
