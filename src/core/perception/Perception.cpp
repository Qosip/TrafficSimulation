// src/core/perception/Perception.cpp
#include "core/perception/Perception.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/math/Constants.hpp"
#include "core/world/Lane.hpp"
#include "core/world/World.hpp"

PerceptionResult Perception::scan(
    core::Vec2 myPosition,
    float      myHeadingDeg,
    const IAgent* myself,
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const World& world,
    const VisionParams& params,
    const Lane* myLane,
    float       myS)
{
    PerceptionResult result;

    const float myHalfLength = myself->getLength() / 2.f;
    const float tileSize = world.getTileSize();

    // --- 1. Scan des agents ---
    for (const auto& agent : agents) {
        if (agent.get() == myself) continue;

        const core::Vec2 otherPos{ agent->getPosition().x, agent->getPosition().y };
        const core::Vec2 diff = otherPos - myPosition;
        const float      dist = diff.length();
        if (dist > params.range || dist < 1.f) continue;

        const float angleToOther = std::atan2(diff.y, diff.x) * core::math::RAD2DEG;
        const float relAngle     = core::math::wrapDeg180(angleToOther - myHeadingDeg);

        // Cone large : alimente la liste "detected" (visualisation debug).
        if (std::abs(relAngle) <= params.halfAngleDeg) {
            DetectedObject obj;
            obj.agent         = agent.get();
            obj.type          = DetectedType::VEHICLE;
            obj.distance      = dist;
            obj.relativeAngle = relAngle;
            obj.position      = otherPos;
            result.detected.push_back(obj);
        }

        const float otherHalfLength = agent->getLength() / 2.f;

        if (myLane) {
            // --- Selection par PROJECTION DE FRENET (Wave 7) ---
            // On projette l'autre sur MA trajectoire dans la fenetre devant moi.
            // Leader valide ssi : pied AHEAD (s > myS), DANS le couloir de ma voie
            // (|lateral| petit -> rejette contre-sens / voie croisee), gap mesure
            // le long du tracé. Independant de tout cone angulaire -> robuste en
            // virage et a l'approche d'intersection.
            const LaneProjection proj =
                myLane->project(otherPos, myS, myS + params.range);
            if (!proj.valid) continue;
            if (std::abs(proj.lateral) > params.laneCorridorHalf) continue;

            const core::Vec2 foot = myLane->getPositionAt(proj.s);
            const Intersection* footInter =
                world.getIntersectionAt(foot.x, foot.y);
            const Intersection* otherInter =
                world.getIntersectionAt(otherPos.x, otherPos.y);
            if (footInter != otherInter && (footInter || otherInter)) continue;
            if (!footInter && !otherInter) {
                const int fgx = static_cast<int>(foot.x / tileSize);
                const int fgy = static_cast<int>(foot.y / tileSize);
                const int ogx = static_cast<int>(otherPos.x / tileSize);
                const int ogy = static_cast<int>(otherPos.y / tileSize);
                const Tile& fTile = world.getTile(fgx, fgy);
                const Tile& oTile = world.getTile(ogx, ogy);
                if (fTile.roadType == RoadType::NONE ||
                    oTile.roadType == RoadType::NONE) {
                    continue;
                }
                if (fTile.direction != TileDirection::NONE &&
                    oTile.direction != TileDirection::NONE &&
                    fTile.direction != oTile.direction) {
                    continue;
                }
            }
            const float along = proj.s - myS;          // distance le long du trace
            if (along <= 0.f) continue;                // derriere moi sur la voie

            // Garde-fou same-direction COMPARE AU CAP DE LA VOIE au point de
            // projection (pas a MON cap courant) : sur une courbe / un ROND-POINT,
            // mon cap differe de celui du leader plus loin sur l'arc. Un vrai
            // leader qui circule a un cap ~= tangente de la voie -> accepte ; un
            // vehicule qui ATTEND d'entrer (cap radial, ~90° de la tangente) ou en
            // contre-sens -> rejete. Corrige le faux "SUIT" sur un rond-point.
            const float laneHeading = myLane->getHeadingAt(proj.s);
            const float hDiff =
                std::abs(core::math::wrapDeg180(agent->getHeading() - laneHeading));
            if (hDiff > params.sameLaneHeadingTol) continue;

            // Garde-fou ANTI CONTRE-SENS, compare a MON cap COURANT, SANS condition
            // de vitesse. Le garde precedent (vs cap de la VOIE au point projete) a
            // un trou en virage / intersection straight : le cap de ma voie PLUS LOIN
            // sur l'arc peut coincider avec celui d'un vehicule d'EN FACE -> hDiff
            // petit -> il etait pris comme leader -> faux "SUIT" sur un oncoming.
            // Mais : sur un ROND-POINT serre (rayon ~25 px, 2x2), un leader legitime
            // a demi-tour devant a un cap a ~180 du mien (tangentes opposees du cercle)
            // alors qu'il est SUR ma voie. L'anti-oncoming le rejetterait a tort.
            // Resolution : appliquer l'anti-oncoming SEULEMENT quand le trace est
            // ~droit (le cap de la voie au point projete colle a mon cap courant).
            // Sur une courbe serree (curveDiff > 45 deg dans la fenetre), on fait
            // confiance au garde cap-voie qui gere correctement les rond-points.
            const float curveDiff =
                std::abs(core::math::wrapDeg180(laneHeading - myHeadingDeg));
            if (curveDiff < 45.f) {
                const float hDiffMe =
                    std::abs(core::math::wrapDeg180(agent->getHeading() - myHeadingDeg));
                if (hDiffMe > 120.f) continue;
            }

            const float bumperDist =
                std::max(0.f, along - myHalfLength - otherHalfLength);
            if (!result.hasDirectObstacle || bumperDist < result.directObstacleDistance) {
                result.hasDirectObstacle      = true;
                result.directObstacleDistance = bumperDist;
                result.directObstacleSpeed    = agent->getSpeed();
                result.directObstacleAgent    = agent.get();
            }
        } else {
            // --- Repli : ancien cone etroit (pas de trajectoire fournie) ---
            const float headingDiff =
                std::abs(core::math::wrapDeg180(agent->getHeading() - myHeadingDeg));
            if (headingDiff > params.sameLaneHeadingTol) continue;
            if (std::abs(relAngle) > params.directHalfAngle) continue;
            const float bumperDist =
                std::max(0.f, dist - myHalfLength - otherHalfLength);
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
