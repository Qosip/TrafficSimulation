// src/Vehicle.cpp
#include "Vehicle.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

#include "Pathfinder.hpp"
#include "Perception.hpp"
#include "World.hpp"

Vehicle::Vehicle(float startX, float startY, float tSize) :
    position(startX, startY), velocity(0.f, 0.f),
    maxSpeed(0.f), maxAcceleration(0.f), s(0.f),
    hasFinishedPath(false), tileSize(tSize), currentAngle(0.f),
    currentSpeed(0.f), isHeadingInitialized(false)
{}

void Vehicle::setPath(const std::vector<sf::Vector2i>& newPath) {
    if (newPath.empty()) {
        currentLane = nullptr;
        return;
    }

    startTile = newPath.front();
    goalTile = newPath.back();

    std::vector<sf::Vector2f> waypoints;
    for (const auto& tile : newPath) {
        waypoints.push_back(sf::Vector2f(
            (tile.x * tileSize) + tileSize / 2.f,
            (tile.y * tileSize) + tileSize / 2.f
        ));
    }

    if (waypoints.size() < 2) return;

    // On génère la liste de points locaux (anciennement densePath)
    std::vector<sf::Vector2f> localDensePath;
    float cornerRadius = tileSize;
    localDensePath.push_back(waypoints.front());

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        sf::Vector2f p1 = waypoints[i];
        sf::Vector2f p2 = waypoints[i+1];

        if (i + 2 < waypoints.size()) {
            sf::Vector2f p3 = waypoints[i+2];
            sf::Vector2f dir1 = p2 - p1;
            sf::Vector2f dir2 = p3 - p2;

            float len1 = std::sqrt(dir1.x * dir1.x + dir1.y * dir1.y);
            float len2 = std::sqrt(dir2.x * dir2.x + dir2.y * dir2.y);

            if (len1 > 0.1f && len2 > 0.1f) {
                dir1 /= len1;
                dir2 /= len2;

                float dot = (dir1.x * dir2.x) + (dir1.y * dir2.y);
                if (std::abs(dot) < 0.1f) {
                    sf::Vector2f entryPoint = p2 - dir1 * cornerRadius;
                    sf::Vector2f exitPoint = p2 + dir2 * cornerRadius;

                    sf::Vector2f toEntry = entryPoint - localDensePath.back();
                    float distToEntry = std::sqrt(toEntry.x * toEntry.x + toEntry.y * toEntry.y);
                    if (distToEntry > 4.f) {
                        sf::Vector2f dirToEntry = toEntry / distToEntry;
                        for (float d = 4.f; d < distToEntry; d += 4.f) {
                            localDensePath.push_back(localDensePath.back() + dirToEntry * 4.f);
                        }
                    }

                    sf::Vector2f circleCenter = entryPoint + sf::Vector2f(-dir1.y, dir1.x) * cornerRadius;
                    sf::Vector2f testRadius = exitPoint - circleCenter;
                    if (std::abs(std::sqrt(testRadius.x * testRadius.x + testRadius.y * testRadius.y) - cornerRadius) > 1.f) {
                        circleCenter = entryPoint + sf::Vector2f(dir1.y, -dir1.x) * cornerRadius;
                    }

                    float startAngle = std::atan2(entryPoint.y - circleCenter.y, entryPoint.x - circleCenter.x);
                    float endAngle = std::atan2(exitPoint.y - circleCenter.y, exitPoint.x - circleCenter.x);

                    float angleDiff = endAngle - startAngle;
                    while (angleDiff < -3.14159265f) angleDiff += 2.f * 3.14159265f;
                    while (angleDiff > 3.14159265f) angleDiff -= 2.f * 3.14159265f;

                    int numSegments = 15;
                    for (int j = 1; j <= numSegments; ++j) {
                        float t = (float)j / numSegments;
                        float currentArcAngle = startAngle + angleDiff * t;
                        localDensePath.push_back(sf::Vector2f(
                            circleCenter.x + std::cos(currentArcAngle) * cornerRadius,
                            circleCenter.y + std::sin(currentArcAngle) * cornerRadius
                        ));
                    }
                    i++;
                    continue;
                }
            }
        }

        sf::Vector2f diff = p2 - localDensePath.back();
        float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist > 4.f) {
            sf::Vector2f direction = diff / dist;
            for (float d = 4.f; d < dist; d += 4.f) {
                localDensePath.push_back(localDensePath.back() + direction * 4.f);
            }
        }
    }
    localDensePath.push_back(waypoints.back());

    // --- MAGIE 1D : On compile la trajectoire dans un objet Lane paramétrique ---
    currentLane = std::make_shared<Lane>(localDensePath);
    s = 0.f; // On remet le pointeur de distance à 0
    hasFinishedPath = false;
    isHeadingInitialized = false;
}

void Vehicle::update(float dt, const std::vector<std::unique_ptr<IAgent>>& agents, const World& world) {
    if (!currentLane || hasFinishedPath) return;

    lastPerception = Perception::scan(position, currentAngle, this, agents, world, visionParams);

    // Arrêt en fin de voie
    if (s >= currentLane->getLength() - 1.f) {
        hasFinishedPath = true;
        currentSpeed = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        return;
    }

    // Calcul simplifié du target speed avant le passage à l'IDM
    float roadLimit = world.getSpeedLimitAt(position.x, position.y);
    float targetSpeed;
    if (isCommittedToPass) {
        targetSpeed = std::min(maxSpeed, lastRoadSpeedLimit);
    } else {
        targetSpeed = (roadLimit > 0.f) ? std::min(maxSpeed, roadLimit) : maxSpeed;
        if (roadLimit > 0.f) lastRoadSpeedLimit = roadLimit;
    }

    // Anticipation des virages en 1D (on regarde le heading futur)
    float lookAheadS = std::min(s + 35.f, currentLane->getLength());
    float futureHeading = currentLane->getHeadingAt(lookAheadS);
    float angleDiff = futureHeading - currentAngle;
    while (angleDiff < -180.f) angleDiff += 360.f;
    while (angleDiff > 180.f) angleDiff -= 360.f;
    if (std::abs(angleDiff) > 15.f) {
        targetSpeed = std::min(targetSpeed, maxSpeed * 0.35f);
    }

    // Gestion de l'obstacle devant (Restée identique avant IDM)
    if (lastPerception.hasDirectObstacle) {
        float gap = lastPerception.directObstacleDistance;
        float desiredGap = (getLength() / 2.f) + 5.f + (currentSpeed * 0.4f);

        if (gap <= desiredGap) {
            targetSpeed = 0.f;
            if (currentSpeed > 0.f) {
                currentSpeed = std::max(0.f, currentSpeed - (maxAcceleration * 5.f) * dt);
            }
        } else {
            float safeDeceleration = maxAcceleration * 1.5f;
            float availableDist = gap - desiredGap;
            float safeSpeed = std::sqrt(2.f * safeDeceleration * std::max(0.1f, availableDist));
            if (safeSpeed < targetSpeed) targetSpeed = safeSpeed;
        }
    }

    // Logique géométrique des carrefours conservée pour cette étape
    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);
    if (interOn) {
        isCommittedToPass = true;
        committedIntersectionId = interOn->getId();
    } else if (isCommittedToPass) {
        isCommittedToPass = false;
        committedIntersectionId = -1;
    }

    if (interOn && isCommittedToPass) {
        // Avance
    } else if (!interOn) {
        float headRad = currentAngle * 3.14159265f / 180.f;
        sf::Vector2f lookDir(std::cos(headRad), std::sin(headRad));

        const Intersection* interAhead = nullptr;
        float distToInter = 999.f;

        for (float d = 10.f; d < 250.f; d += tileSize * 0.4f) {
            sf::Vector2f checkPos = position + lookDir * d;
            const Intersection* found = world.getIntersectionAt(checkPos.x, checkPos.y);
            if (found) {
                interAhead = found;
                distToInter = d;
                break;
            }
        }

        if (interAhead) {
            if (isCommittedToPass && committedIntersectionId == interAhead->getId()) {
                // Avance
            } else {
                Approach::Direction myDir = world.getApproachDirection(currentAngle);
                bool allowed = interAhead->canPass(myDir, agents);
                float brakingDistance = (currentSpeed * currentSpeed) / (2.f * maxAcceleration * 1.8f);

                if (allowed) {
                    if (distToInter < 60.f) {
                        isCommittedToPass = true;
                        committedIntersectionId = interAhead->getId();
                    }
                } else if (distToInter > brakingDistance) {
                    isCommittedToPass = false;
                    committedIntersectionId = -1;

                    if (interAhead->getType() == RegulationType::TRAFFIC_LIGHT) {
                        float stopDist = 25.f + (getLength() / 2.f);
                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                            if (currentSpeed > 5.f) currentSpeed = std::max(0.f, currentSpeed - (maxAcceleration * 5.f) * dt);
                        } else if (distToInter < stopDist + 80.f) {
                            float factor = (distToInter - stopDist) / 80.f;
                            targetSpeed = std::min(targetSpeed, targetSpeed * factor);
                        }
                    } else if (interAhead->getType() == RegulationType::PRIORITY_RIGHT) {
                        float stopDist = 25.f + (getLength() / 2.f);
                        float brakeDist = 120.f;

                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                        } else if (distToInter < brakeDist) {
                            float factor = (distToInter - stopDist) / (brakeDist - stopDist);
                            float interSpeed = (maxSpeed * 0.35f) + (targetSpeed - (maxSpeed * 0.35f)) * factor;
                            if (interSpeed < targetSpeed) targetSpeed = interSpeed;
                        }
                    }
                } else {
                    isCommittedToPass = true;
                    committedIntersectionId = interAhead->getId();
                }
            }
        } else {
            isCommittedToPass = false;
            committedIntersectionId = -1;
        }
    }

    // --- INTEGRATION KINÉMATIQUE 1D ---
    if (currentSpeed < targetSpeed) {
        currentSpeed += maxAcceleration * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= (maxAcceleration * 1.8f) * dt;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    // On avance de v*dt le long de la courbe paramétrique
    s += currentSpeed * dt;
    if (s > currentLane->getLength()) s = currentLane->getLength();

    // On actualise les données spatiales PUREMENT depuis la Lane (fini le braquage approximatif)
    position = currentLane->getPositionAt(s);
    currentAngle = currentLane->getHeadingAt(s);

    // Pour la forme, on met à jour le vecteur vitesse pour les autres agents
    float rad = currentAngle * 3.14159265f / 180.f;
    velocity.x = std::cos(rad) * currentSpeed;
    velocity.y = std::sin(rad) * currentSpeed;

    shape.setPosition(position);
    shape.setRotation(currentAngle);
}

void Vehicle::draw(sf::RenderWindow& window) {
    window.draw(shape);
}

bool Vehicle::contains(sf::Vector2f point) const {
    return shape.getGlobalBounds().contains(point);
}

void Vehicle::drawDebug(sf::RenderWindow& window) {
    if (!isSelectedFlag || !currentLane || hasFinishedPath) return;

    Perception::drawDebugCone(window, position, currentAngle, visionParams, lastPerception);

    const auto& points = currentLane->getPoints();
    if(points.empty()) return;

    sf::VertexArray pathLine(sf::LineStrip, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pathLine[i].position = points[i];
        pathLine[i].color = sf::Color(147, 112, 219, 220);
    }
    window.draw(pathLine);
}

sf::Vector2i Vehicle::getCurrentTile() const {
    return sf::Vector2i((int)(position.x / tileSize), (int)(position.y / tileSize));
}

void Vehicle::recalculatePath(const World& world) {
    if (hasFinishedPath) return;
    sf::Vector2i current = getCurrentTile();
    std::vector<sf::Vector2i> newPath = Pathfinder::findPath(world, current, goalTile);

    if (!newPath.empty()) {
        sf::Vector2i originalStart = startTile;
        sf::Vector2i originalGoal = goalTile;
        setPath(newPath);
        startTile = originalStart;
        goalTile = originalGoal;
    } else {
        currentLane = nullptr;
        currentSpeed = 0.f;
    }
}

void Vehicle::resetToStart(const World& world) {
    position = sf::Vector2f(startTile.x * tileSize + tileSize / 2.f, startTile.y * tileSize + tileSize / 2.f);
    shape.setPosition(position);
    shape.setRotation(0.f);

    velocity = sf::Vector2f(0.f, 0.f);
    currentSpeed = 0.f;
    currentAngle = 0.f;
    isHeadingInitialized = false;
    hasFinishedPath = false;
    s = 0.f; // NOUVEAU : Reset de la position paramétrique

    isCommittedToPass = false;
    committedIntersectionId = -1;

    std::vector<sf::Vector2i> fullPath = Pathfinder::findPath(world, startTile, goalTile);
    if (!fullPath.empty()) {
        sf::Vector2i originalStart = startTile;
        sf::Vector2i originalGoal = goalTile;
        setPath(fullPath);
        startTile = originalStart;
        goalTile = originalGoal;
    } else {
        currentLane = nullptr;
    }
}

float Vehicle::getRemainingDistance() const {
    if (!currentLane || hasFinishedPath) return 0.f;
    return std::max(0.f, currentLane->getLength() - s);
}