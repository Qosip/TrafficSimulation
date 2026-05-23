// src/core/agent/Vehicle.cpp
//
// Wave 5 : pipeline strict en deux phases.
//
//   computeDecision()
//     1. Perception (cone same-direction + scan intersection).
//     2. Choix du v0 IDM = min(maxSpeed, road_limit, virage_factor).
//     3. Construction du LeaderInfo final :
//         - leader reel issu de Perception
//         - leader virtuel issu de IIntersectionPolicy.request() si shouldStop
//         - on retient le plus contraignant (plus petit gap).
//     4. pendingAccel = IDM.computeAcceleration(v, v0, leader).
//
//   integrate(dt)
//     - v += pendingAccel * dt (clamp >= 0 et <= v0).
//     - s += v * dt
//     - position + heading lus depuis Lane (parametrisation curviligne).
#include "core/agent/Vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/perception/Perception.hpp"
#include "core/world/World.hpp"

Vehicle::Vehicle(float startX, float startY, float tSize) :
    position{startX, startY}, velocity{0.f, 0.f},
    maxSpeed(0.f), maxAcceleration(0.f),
    bodySize{0.f, 0.f}, bodyColor{255, 255, 255, 255},
    s(0.f),
    hasFinishedPath(false), tileSize(tSize),
    currentAngle(0.f), currentSpeed(0.f), isHeadingInitialized(false)
{}

// -----------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------

void Vehicle::rebuildLaneFromPath(const std::vector<core::TileCoord>& tilePath) {
    std::vector<core::Vec2> waypoints;
    waypoints.reserve(tilePath.size());
    for (const auto& tile : tilePath) {
        waypoints.push_back({
            (tile.x * tileSize) + tileSize / 2.f,
            (tile.y * tileSize) + tileSize / 2.f
        });
    }
    if (waypoints.size() < 2) {
        currentLane = nullptr;
        return;
    }

    std::vector<core::Vec2> localDensePath;
    const float cornerRadius = tileSize;
    localDensePath.push_back(waypoints.front());

    for (std::size_t i = 0; i < waypoints.size() - 1; ++i) {
        const core::Vec2 p1 = waypoints[i];
        const core::Vec2 p2 = waypoints[i + 1];

        if (i + 2 < waypoints.size()) {
            const core::Vec2 p3 = waypoints[i + 2];
            core::Vec2 dir1 = p2 - p1;
            core::Vec2 dir2 = p3 - p2;
            const float len1 = dir1.length();
            const float len2 = dir2.length();
            if (len1 > 0.1f && len2 > 0.1f) {
                dir1 /= len1;
                dir2 /= len2;
                if (std::abs(core::dot(dir1, dir2)) < 0.1f) {
                    const core::Vec2 entryPoint = p2 - dir1 * cornerRadius;
                    const core::Vec2 exitPoint  = p2 + dir2 * cornerRadius;

                    const core::Vec2 toEntry = entryPoint - localDensePath.back();
                    const float distToEntry = toEntry.length();
                    if (distToEntry > 4.f) {
                        const core::Vec2 dirToEntry = toEntry / distToEntry;
                        for (float dE = 4.f; dE < distToEntry; dE += 4.f) {
                            localDensePath.push_back(localDensePath.back() + dirToEntry * 4.f);
                        }
                    }

                    core::Vec2 circleCenter = entryPoint + core::Vec2{-dir1.y, dir1.x} * cornerRadius;
                    const core::Vec2 testRadius = exitPoint - circleCenter;
                    if (std::abs(testRadius.length() - cornerRadius) > 1.f) {
                        circleCenter = entryPoint + core::Vec2{dir1.y, -dir1.x} * cornerRadius;
                    }

                    const float startAngle = std::atan2(entryPoint.y - circleCenter.y,
                                                        entryPoint.x - circleCenter.x);
                    const float endAngle   = std::atan2(exitPoint.y - circleCenter.y,
                                                        exitPoint.x - circleCenter.x);
                    float angleDiff = endAngle - startAngle;
                    while (angleDiff < -core::math::PI) angleDiff += core::math::TWO_PI;
                    while (angleDiff >  core::math::PI) angleDiff -= core::math::TWO_PI;

                    constexpr int numSegments = 15;
                    for (int j = 1; j <= numSegments; ++j) {
                        const float t = static_cast<float>(j) / numSegments;
                        const float a = startAngle + angleDiff * t;
                        localDensePath.push_back({
                            circleCenter.x + std::cos(a) * cornerRadius,
                            circleCenter.y + std::sin(a) * cornerRadius
                        });
                    }
                    i++;
                    continue;
                }
            }
        }

        const core::Vec2 diff = p2 - localDensePath.back();
        const float dist = diff.length();
        if (dist > 4.f) {
            const core::Vec2 direction = diff / dist;
            for (float dd = 4.f; dd < dist; dd += 4.f) {
                localDensePath.push_back(localDensePath.back() + direction * 4.f);
            }
        }
    }
    localDensePath.push_back(waypoints.back());

    currentLane          = std::make_shared<Lane>(localDensePath);
    s                    = 0.f;
    hasFinishedPath      = false;
    isHeadingInitialized = false;
}

void Vehicle::setPath(const std::vector<core::TileCoord>& newPath) {
    if (newPath.empty()) {
        currentLane = nullptr;
        return;
    }
    startTile = newPath.front();
    goalTile  = newPath.back();
    rebuildLaneFromPath(newPath);
}

// -----------------------------------------------------------------
// PHASE 1 : computeDecision
// -----------------------------------------------------------------

void Vehicle::computeDecision(const std::vector<std::unique_ptr<IAgent>>& agents,
                              const World& world)
{
    using core::agent::BlockReason;

    if (!currentLane) {
        pendingAccel        = 0.f;
        pendingDesiredSpeed = 0.f;
        currentBlockReason  = BlockReason::NO_PATH;
        return;
    }
    if (hasFinishedPath) {
        pendingAccel        = 0.f;
        pendingDesiredSpeed = 0.f;
        currentBlockReason  = BlockReason::AT_GOAL;
        return;
    }

    lastPerception = Perception::scan(position, currentAngle, this, agents, world, visionParams);

    // --- v0 : desired speed clamp ---
    const float roadLimit = world.getSpeedLimitAt(position.x, position.y);
    float v0 = (roadLimit > 0.f) ? std::min(maxSpeed, roadLimit) : maxSpeed;

    // Anticipation virage.
    bool corneringClamp = false;
    const float lookAheadS    = std::min(s + 35.f, currentLane->getLength());
    const float futureHeading = currentLane->getHeadingAt(lookAheadS);
    const float angleDiff     = core::math::wrapDeg180(futureHeading - currentAngle);
    if (std::abs(angleDiff) > 15.f) {
        v0 = std::min(v0, maxSpeed * 0.35f);
        corneringClamp = true;
    }
    pendingDesiredSpeed = v0;

    // --- Leader reel ---
    core::behavior::LeaderInfo leader;
    bool leaderIsVehicle  = false;
    if (lastPerception.hasDirectObstacle) {
        leader.present = true;
        leader.gap     = lastPerception.directObstacleDistance;
        leader.speed   = lastPerception.directObstacleSpeed;
        leaderIsVehicle = true;
    }

    // --- Leader virtuel issu de la policy d'intersection ---
    bool leaderFromYield = false;
    bool leaderFromRed   = false;

    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);
    if (interOn) {
        isCommittedToPass       = true;
        committedIntersectionId = interOn->getId();
    } else if (isCommittedToPass) {
        isCommittedToPass       = false;
        committedIntersectionId = -1;
    }

    if (!interOn) {
        const float headRad = currentAngle * core::math::DEG2RAD;
        const core::Vec2 lookDir{ std::cos(headRad), std::sin(headRad) };

        const Intersection* interAhead = nullptr;
        float distToInter = std::numeric_limits<float>::infinity();
        for (float d = 10.f; d < 250.f; d += tileSize * 0.4f) {
            const core::Vec2 checkPos = position + lookDir * d;
            const Intersection* found = world.getIntersectionAt(checkPos.x, checkPos.y);
            if (found) { interAhead = found; distToInter = d; break; }
        }

        if (interAhead && !(isCommittedToPass && committedIntersectionId == interAhead->getId())) {
            const Approach::Direction myDir = world.getApproachDirection(currentAngle);

            core::intersection::PolicyContext ctx;
            ctx.self.position = position;
            ctx.self.speed    = currentSpeed;
            ctx.self.heading  = currentAngle;
            ctx.self.length   = getLength();
            ctx.self.from     = myDir;
            ctx.selfAgent     = this;
            ctx.tileSize      = tileSize;
            ctx.others        = &agents;

            // Si l'agent est physiquement engage (carrosserie deja sur la
            // ligne d'arret), on force le passage : freiner ici provoquerait
            // un arret AU MILIEU du carrefour et bloquerait les flux croises.
            const float engagedThreshold = ctx.self.length / 2.f + 10.f;
            if (distToInter < engagedThreshold) {
                isCommittedToPass       = true;
                committedIntersectionId = interAhead->getId();
            }

            const auto decision = interAhead->request(ctx);

            if (decision.canEnter ||
                (isCommittedToPass && committedIntersectionId == interAhead->getId())) {
                if (distToInter < 60.f) {
                    isCommittedToPass       = true;
                    committedIntersectionId = interAhead->getId();
                }
            } else if (decision.shouldStop) {
                core::behavior::LeaderInfo virtualLeader;
                virtualLeader.present = true;
                virtualLeader.gap     = std::max(decision.stopLineGap, 0.f);
                virtualLeader.speed   = 0.f;

                if (!leader.present || virtualLeader.gap < leader.gap) {
                    leader = virtualLeader;
                    leaderIsVehicle = false;
                    if (interAhead->getType() == RegulationType::TRAFFIC_LIGHT) {
                        leaderFromRed = true;
                    } else {
                        leaderFromYield = true;
                    }
                }
            }
        }
    }

    // --- IDM ---
    pendingAccel = idm.computeAcceleration(currentSpeed, v0, leader);

    // --- Diagnostic du motif de blocage ---
    // Hierarchie : intersection (raison la plus contraignante) > leader reel
    //              > cornering > demarrage > libre.
    if (leaderFromRed) {
        currentBlockReason = BlockReason::INTERSECTION_RED;
    } else if (leaderFromYield) {
        currentBlockReason = BlockReason::INTERSECTION_YIELD;
    } else if (leaderIsVehicle && pendingAccel < 0.f) {
        currentBlockReason = BlockReason::LEADER_VEHICLE;
    } else if (corneringClamp && currentSpeed > v0 - 1.f) {
        currentBlockReason = BlockReason::CORNERING;
    } else if (currentSpeed < 5.f && pendingAccel > 0.f) {
        currentBlockReason = BlockReason::INITIALIZING;
    } else {
        currentBlockReason = BlockReason::NONE;
    }
}

// -----------------------------------------------------------------
// PHASE 2 : integrate (Euler, dt fixe)
// -----------------------------------------------------------------

void Vehicle::integrate(float dt) {
    if (!currentLane || hasFinishedPath) return;

    if (s >= currentLane->getLength() - 1.f) {
        hasFinishedPath = true;
        currentSpeed    = 0.f;
        velocity        = {0.f, 0.f};
        return;
    }

    currentSpeed += pendingAccel * dt;
    currentSpeed  = std::max(0.f, currentSpeed);
    const float cap = pendingDesiredSpeed > 0.f ? pendingDesiredSpeed : maxSpeed;
    currentSpeed  = std::min(currentSpeed, cap);

    s += currentSpeed * dt;
    if (s > currentLane->getLength()) s = currentLane->getLength();

    position     = currentLane->getPositionAt(s);
    currentAngle = currentLane->getHeadingAt(s);

    const float rad = currentAngle * core::math::DEG2RAD;
    velocity.x = std::cos(rad) * currentSpeed;
    velocity.y = std::sin(rad) * currentSpeed;
}

// -----------------------------------------------------------------
// UI / debug / path management
// -----------------------------------------------------------------

bool Vehicle::contains(core::Vec2 point) const {
    const core::Vec2 local = point - position;
    const float rad = -currentAngle * core::math::DEG2RAD;
    const float c   = std::cos(rad);
    const float si  = std::sin(rad);
    const core::Vec2 r{ local.x * c - local.y * si,
                        local.x * si + local.y * c };
    return std::abs(r.x) <= bodySize.x / 2.f &&
           std::abs(r.y) <= bodySize.y / 2.f;
}

AgentDebugSnapshot Vehicle::getDebugSnapshot() const {
    AgentDebugSnapshot snap;
    snap.selected = isSelectedFlag;
    snap.active   = (currentLane != nullptr) && !hasFinishedPath;

    snap.visionRange              = visionParams.range;
    snap.visionHalfAngleDeg       = visionParams.halfAngleDeg;
    snap.visionDirectHalfAngleDeg = visionParams.directHalfAngle;

    snap.hasDirectObstacle       = lastPerception.hasDirectObstacle;
    snap.directObstacleDistance  = lastPerception.directObstacleDistance;
    snap.approachingIntersection = lastPerception.approachingIntersection;

    snap.detectionPositions.reserve(lastPerception.detected.size());
    snap.detectionDistances.reserve(lastPerception.detected.size());
    for (const auto& d : lastPerception.detected) {
        snap.detectionPositions.push_back(d.position);
        snap.detectionDistances.push_back(d.distance);
    }
    if (currentLane) snap.pathPoints = currentLane->getPoints();
    return snap;
}

core::TileCoord Vehicle::getCurrentTile() const {
    return { static_cast<int>(position.x / tileSize),
             static_cast<int>(position.y / tileSize) };
}

void Vehicle::recalculatePath(const World& world) {
    if (hasFinishedPath) return;
    const core::TileCoord current = getCurrentTile();
    const auto newPath = AStarPlanner::findPath(world, current, goalTile);
    if (!newPath.empty()) {
        const core::TileCoord origStart = startTile;
        const core::TileCoord origGoal  = goalTile;
        setPath(newPath);
        startTile = origStart;
        goalTile  = origGoal;
    } else {
        currentLane  = nullptr;
        currentSpeed = 0.f;
    }
}

void Vehicle::resetToStart(const World& world) {
    position             = { startTile.x * tileSize + tileSize / 2.f,
                             startTile.y * tileSize + tileSize / 2.f };
    velocity             = {0.f, 0.f};
    currentSpeed         = 0.f;
    currentAngle         = 0.f;
    isHeadingInitialized = false;
    hasFinishedPath      = false;
    s                    = 0.f;
    pendingAccel         = 0.f;
    pendingDesiredSpeed  = 0.f;
    isCommittedToPass       = false;
    committedIntersectionId = -1;
    currentBlockReason   = core::agent::BlockReason::INITIALIZING;

    const auto fullPath = AStarPlanner::findPath(world, startTile, goalTile);
    if (!fullPath.empty()) {
        const core::TileCoord origStart = startTile;
        const core::TileCoord origGoal  = goalTile;
        setPath(fullPath);
        startTile = origStart;
        goalTile  = origGoal;
    } else {
        currentLane = nullptr;
    }
}

float Vehicle::getRemainingDistance() const {
    if (!currentLane || hasFinishedPath) return 0.f;
    return std::max(0.f, currentLane->getLength() - s);
}
