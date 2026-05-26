// src/sim/McGeometry.cpp
#include "sim/McGeometry.hpp"

#include "core/world/World.hpp"
#include "io/SceneBuilder.hpp"

namespace sim {

// Convention SceneBuilder :
//   ligne y   = voie RIGHT (vers E),  ligne y-1   = voie LEFT (vers O)
//   colonne x = voie UP    (vers N),  colonne x-1 = voie DOWN (vers S)
std::vector<OriginDef> makeCrossroadOrigins(int G) {
    const int R = G / 2;
    const int C = G / 2;
    const core::TileCoord gN{ C,     0     };   // sortie NORD  (col C, UP)
    const core::TileCoord gS{ C - 1, G - 1 };   // sortie SUD   (col C-1, DOWN)
    const core::TileCoord gE{ G - 1, R     };   // sortie EST   (ligne R, RIGHT)
    const core::TileCoord gW{ 0,     R - 1 };   // sortie OUEST (ligne R-1, LEFT)

    std::vector<OriginDef> o;
    o.push_back({ { 0,     R     }, { gE, gN, gS } });  // venant de l'OUEST -> E
    o.push_back({ { G - 1, R - 1 }, { gW, gS, gN } });  // venant de l'EST   -> O
    o.push_back({ { C,     G - 1 }, { gN, gE, gW } });  // venant du SUD     -> N
    o.push_back({ { C - 1, 0     }, { gS, gW, gE } });  // venant du NORD    -> S
    return o;
}

void buildIsolatedCrossroad(World& world, RegulationType strat, int G) {
    const int R = G / 2;
    const int C = G / 2;
    scene::buildHRoad(world, R, 0, G - 1);
    scene::buildVRoad(world, C, 0, G - 1);
    scene::buildCrossroad(world, C, R, 1, strat);
}

void buildIsolatedRoundabout(World& world, int sideTiles, int G) {
    if (sideTiles < 2)      sideTiles = 2;
    if (sideTiles % 2 != 0) ++sideTiles;          // cote PAIR impose

    const int R = G / 2;
    const int C = G / 2;
    scene::buildHRoad(world, R, 0, G - 1);
    scene::buildVRoad(world, C, 0, G - 1);

    // Anneau centre sur le croisement (C-0.5, R-0.5) -> coin = (C, R) - cote/2.
    // Les axes traversants (lignes R-1/R, colonnes C-1/C) coupent le bloc :
    // refreshRoundaboutApproaches (appele par buildRoundabout) en deduit les 4
    // branches.
    scene::buildRoundabout(world, C - sideTiles / 2, R - sideTiles / 2, 1, sideTiles);
}

} // namespace sim
