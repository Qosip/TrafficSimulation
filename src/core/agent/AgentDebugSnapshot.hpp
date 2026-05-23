// src/AgentDebugSnapshot.hpp
//
// Donnees agregees qu'un agent expose pour les overlays de debogage.
// Sert d'interface entre IAgent (producteur) et SfmlRenderer (consommateur)
// sans coupler le core a SFML.
//
// Cette structure est volontairement plate et "read-only" : c'est un
// snapshot, le renderer ne doit pas le modifier.
#pragma once

#include <vector>

#include "core/math/Vec2.hpp"

struct AgentDebugSnapshot {
    // Vision cone parameters.
    float visionRange = 0.f;
    float visionHalfAngleDeg = 0.f;
    float visionDirectHalfAngleDeg = 0.f;

    // Resultat de la derniere perception.
    bool  hasDirectObstacle = false;
    float directObstacleDistance = 0.f;
    bool  approachingIntersection = false;

    // Positions monde des objets detectes + distance pour colorisation.
    std::vector<core::Vec2> detectionPositions;
    std::vector<float>      detectionDistances;

    // Points de la trajectoire courante (currentLane->getPoints()).
    std::vector<core::Vec2> pathPoints;

    // Visible / actif uniquement quand l'agent est selectionne UI.
    bool selected = false;
    bool active   = false;   // false si l'agent a fini son path
};
