// src/SceneBuilder.hpp
//
// Helpers pour construire des scenarios reproductibles.
// Extraits depuis main.cpp lors de l'Etape 0 du refactor.
// Comportement strictement identique - aucune modification semantique.
#pragma once

#include "core/agent/IAgent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/world/World.hpp"

#include <memory>
#include <vector>

namespace scene {

// Pose une intersection 2x2 a partir de (cx-1, cy-1) jusqu'a (cx, cy).
void buildCrossroad(World& world, int cx, int cy, int id, RegulationType regType);

// Pose un rond-point dont le coin haut-gauche est (x0, y0) et le cote mesure
// 'sideTiles' cellules. sideTiles est FORCE PAIR (>= 2) : un anneau carre dont
// l'ilot central creux (pelouse) fait (sideTiles-2) de cote. Les branches
// (entrees/sorties) sont recalculees dynamiquement selon les routes raccordees
// au pourtour (cf. World::refreshRoundaboutApproaches), donc 2, 3 ou 4 sorties.
void buildRoundabout(World& world, int x0, int y0, int id, int sideTiles);

// Pose une route horizontale 2 voies (LEFT/RIGHT) sur deux lignes y et y-1.
void buildHRoad(World& world, int y, int xStart, int xEnd);

// Pose une route verticale 2 voies (UP/DOWN) sur deux colonnes x et x-1.
void buildVRoad(World& world, int x, int yStart, int yEnd);

// Construit le reseau urbain par defaut (4 carrefours, 4 axes) sur une grille 32x32.
// Utilise par main.cpp et par les snapshot tests.
void buildDefaultNetwork(World& world);

// Place les 7 agents par defaut (5 voitures + 2 camions) et calcule leurs paths.
// L'ordre d'insertion est determinant pour la reproductibilite des tests.
void spawnDefaultAgents(std::vector<std::unique_ptr<IAgent>>& agents, World& world);

// Scenario de demonstration : reconstruit world + agents pour donner a voir,
// cote a cote, les comportements a verifier :
//   * grand rond-point (trajectoire courbe, sens unique legal, pas de contre-sens),
//   * depassement reussi (creneau de rabattement libre, manoeuvre en courbe),
//   * convoi non depassable (creneau pris -> le suiveur reste derriere),
//   * carrefour STOP (chacun son tour, pas d'inter-blocage).
// 'world' est entierement reconstruit a la taille requise.
void buildDemoScenario(std::unique_ptr<World>& world,
                       std::vector<std::unique_ptr<IAgent>>& agents,
                       float tileSize);

} // namespace scene
