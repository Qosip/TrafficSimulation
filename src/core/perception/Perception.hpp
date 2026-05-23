// src/core/perception/Perception.hpp
//
// Etape 4 + Wave 5 :
//   - SFML-free
//   - PerceptionResult traque le leader direct (pointeur + vitesse) pour
//     alimenter l'IDM (delta-v exact).
//   - Filtrage same-direction : un vehicule en contre-sens n'est PAS
//     considere comme leader.
#pragma once

#include <memory>
#include <vector>

#include "core/math/Vec2.hpp"

class World;
class IAgent;

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
    float directHalfAngle       = 8.f;
    float intersectionLookAhead = 120.f;
    float sameLaneHeadingTol    = 45.f;  // Wave 5 : seuil same-direction
};

class Perception {
public:
    static PerceptionResult scan(
        core::Vec2 myPosition,
        float      myHeadingDeg,
        const IAgent* myself,
        const std::vector<std::unique_ptr<IAgent>>& agents,
        const World& world,
        const VisionParams& params);
};
