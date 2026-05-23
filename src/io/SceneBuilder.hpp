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

} // namespace scene
