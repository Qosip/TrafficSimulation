// src/core/perception/Perception.hpp
//
// Etape 4 + Wave 5 + Wave 7 :
//   - SFML-free
//   - PerceptionResult traque le leader direct (pointeur + vitesse) pour
//     alimenter l'IDM (delta-v exact).
//   - Wave 7 : selection du leader par PROJECTION DE FRENET sur la trajectoire
//     du suiveur (repere curvilineaire). Le gap est mesure LE LONG du tracé et
//     un vehicule dont le pied de projection s'ecarte lateralement (contre-sens,
//     voie croisee) est rejete. Resout l'accrochage en virage / a l'intersection
//     (le cone angulaire brut accrochait le mauvais vehicule).
#pragma once

#include <memory>
#include <vector>

#include "core/math/Vec2.hpp"

class World;
class IAgent;
class Lane;

enum class DetectedType {
    VEHICLE,
    // Extensible : PEDESTRIAN, SIGNAL, OBSTACLE, EMERGENCY...
};

struct DetectedObject {
    const IAgent* agent;
    DetectedType  type;
    float         distance;
    float         relativeAngle;
    core::Vec2    position;
};

struct PerceptionResult {
    std::vector<DetectedObject> detected;

    // Leader direct sur la meme voie (same-direction).
    bool          hasDirectObstacle      = false;
    float         directObstacleDistance = 0.f;   // gap bumper-to-bumper (px)
    float         directObstacleSpeed    = 0.f;   // px/s -- pour delta-v IDM
    const IAgent* directObstacleAgent    = nullptr;

    bool approachingIntersection = false;
};

struct VisionParams {
    float range                 = 150.f;
    float halfAngleDeg          = 30.f;
    float directHalfAngle       = 8.f;   // cone "detected[]" (visualisation debug)
    float intersectionLookAhead = 120.f;
    float sameLaneHeadingTol    = 45.f;  // garde-fou same-direction (secondaire)
    // Wave 7 : demi-largeur du couloir de Frenet. Un vehicule dont le pied de
    // projection sur MA trajectoire s'ecarte de plus de cette valeur n'est PAS
    // sur ma voie -> jamais un leader. La voie opposee est a ~1 tuile (~50 px) :
    // 22 px isole proprement ma voie sans rejeter un suiveur legerement decentre.
    float laneCorridorHalf      = 22.f;
};

class Perception {
public:
    // 'myLane' / 'myS' : trajectoire curvilineaire du suiveur et son abscisse
    // courante. Si myLane == nullptr, on retombe sur l'ancien filtrage angulaire
    // (cone + cap). Sinon, le leader est choisi par projection de Frenet.
    static PerceptionResult scan(
        core::Vec2 myPosition,
        float      myHeadingDeg,
        const IAgent* myself,
        const std::vector<std::unique_ptr<IAgent>>& agents,
        const World& world,
        const VisionParams& params,
        const Lane* myLane = nullptr,
        float       myS    = 0.f);
};
