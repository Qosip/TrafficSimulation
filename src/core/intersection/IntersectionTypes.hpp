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
};

struct Approach {
    enum class Direction { NORTH, SOUTH, EAST, WEST };

    Direction       direction;
    core::TileCoord entryTile;
    bool            hasGreen = false;
};

enum class LightState { GREEN, ORANGE, RED };
