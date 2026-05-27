// src/core/intersection/IntersectionTypes.hpp
//
// Types de donnees partages entre Intersection et ses policies.
// Extraction pour casser le cycle d'include Intersection <-> IIntersectionPolicy.
#pragma once

#include "core/math/TileCoord.hpp"

enum class RegulationType {
    PRIORITY_RIGHT,
    STOP,         // Prevu
    YIELD,        // Prevu
    TRAFFIC_LIGHT,
    ROUNDABOUT,   // Prevu
    // --- Strategies de coordination "recherche" (etat de l'art SMA) ---
    // NB : ajoutees EN FIN d'enum pour preserver la compatibilite des
    // scenarios serialises (RegulationType est ecrit en int dans les .txt).
    FIXED_PRIORITY,   // Priorite fixe par axe (route principale vs secondaire)
    P2P,              // Negociation pair-a-pair decentralisee (VanMiddlesworth)
    AIM,              // Gestion centralisee par reservation (Dresner & Stone)
    VIRTUAL_PLATOON,  // Peloton virtuel : projection 1D + paires meneur-suiveur
    ORCA,             // Espace ouvert : evitement continu reciproque (van den Berg)
};

struct Approach {
    enum class Direction { NORTH, SOUTH, EAST, WEST };

    Direction       direction;
    core::TileCoord entryTile;
    bool            hasGreen = false;
};

enum class LightState { GREEN, ORANGE, RED };
