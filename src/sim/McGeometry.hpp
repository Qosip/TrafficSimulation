// src/sim/McGeometry.hpp
//
// Geometrie partagee du banc d'essai Monte-Carlo : un carrefour 2x2 isole au
// centre d'une grille carree, avec ses quatre points d'apparition (un par
// branche) et leurs destinations possibles (tout droit + deux virages).
// Utilise par ExperimentRunner (headless) et McLiveSession (visuel).
#pragma once

#include <vector>

#include "core/intersection/IntersectionTypes.hpp"   // RegulationType
#include "core/math/TileCoord.hpp"

class World;

namespace sim {

struct OriginDef {
    core::TileCoord start;
    core::TileCoord goals[3];   // [0] = tout droit, [1]/[2] = virages
};

// Quatre origines (O, E, S, N) vers la grille de cote 'gridSize'. Identiques
// pour un carrefour ou un rond-point (les tiles d'entree/sortie sont les memes).
std::vector<OriginDef> makeCrossroadOrigins(int gridSize);

// Pose routes H+V traversantes + un carrefour central de strategie 'strat'.
void buildIsolatedCrossroad(World& world, RegulationType strat, int gridSize);

// Variante rond-point : meme axes traversants, anneau central de cote 'sideTiles'
// (PAIR, 2..8), centre sur le croisement. Branches recalculees automatiquement.
void buildIsolatedRoundabout(World& world, int sideTiles, int gridSize);

} // namespace sim
