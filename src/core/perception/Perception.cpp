// src/core/perception/Perception.cpp
#include "core/perception/Perception.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/math/Constants.hpp"
#include "core/world/World.hpp"

PerceptionResult Perception::scan(
    core::Vec2 myPosition,
    float      myHeadingDeg,
    const IAgent* myself,
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const World& world,
    const VisionParams& params)
{
    PerceptionResult result;

    // --- 1. Scan des agents ---
    for (const auto& agent : agents) {
        if (agent.get() == myself) continue;

        const core::Vec2 otherPos{ agent->getPosition().x, agent->getPosition().y };
        const core::Vec2 diff = otherPos - myPosition;
        const float      dist = diff.length();
        if (dist > params.range || dist < 1.f) continue;

        const float angleToOther = std::atan2(diff.y, diff.x) * core::math::RAD2DEG;
        const float relAngle     = core::math::wrapDeg180(angleToOther - myHeadingDeg);
        if (std::abs(relAngle) > params.halfAngleDeg) continue;

        DetectedObject obj;
        obj.agent         = agent.get();
        obj.type          = DetectedType::VEHICLE;
        obj.distance      = dist;
        obj.relativeAngle = relAngle;
        obj.position      = otherPos;
        result.detected.push_back(obj);

        // Cone etroit : candidat "obstacle direct devant".
        if (std::abs(relAngle) <= params.directHalfAngle) {
            // Filtre same-direction : on n'accepte comme leader qu'un vehicule
            // dont le heading est proche du notre (memes lanes / meme sens).
            const float otherHeading = agent->getHeading();
            const float headingDiff  = std::abs(core::math::wrapDeg180(otherHeading - myHeadingDeg));
            if (headingDiff > params.sameLaneHeadingTol) continue;

            const float myHalfLength    = myself->getLength() / 2.f;
            const float otherHalfLength = agent->getLength()  / 2.f;
            const float bumperDist      = std::max(0.f, dist - myHalfLength - otherHalfLength);

            if (!result.hasDirectObstacle || bumperDist < result.directObstacleDistance) {
                result.hasDirectObstacle      = true;
                result.directObstacleDistance = bumperDist;
                result.directObstacleSpeed    = agent->getSpeed();
                result.directObstacleAgent    = agent.get();
            }
        }
    }

    // --- 2. Detection d'intersection devant nous ---
    const float headRad = myHeadingDeg * core::math::DEG2RAD;
    const core::Vec2 lookDir{ std::cos(headRad), std::sin(headRad) };

    const float tileSize = world.getTileSize();
    for (float d = 0.f; d < params.intersectionLookAhead; d += tileSize * 0.5f) {
        const core::Vec2 checkPos = myPosition + lookDir * d;
        const int gridX = static_cast<int>(checkPos.x / tileSize);
        const int gridY = static_cast<int>(checkPos.y / tileSize);

        if (world.getTile(gridX, gridY).roadType == RoadType::INTERSECTION) {
            result.approachingIntersection = true;
            break;
        }
    }

    return result;
}
