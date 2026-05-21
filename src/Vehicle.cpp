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
    maxSpeed(0.f), maxAcceleration(0.f), currentPathIndex(0),
    hasFinishedPath(false), tileSize(tSize), currentAngle(0.f),
    currentSpeed(0.f), isHeadingInitialized(false)
{}

void Vehicle::setPath(const std::vector<sf::Vector2i>& newPath) {
    densePath.clear();
    if (newPath.empty()) return;

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

    float cornerRadius = tileSize;
    densePath.push_back(waypoints.front());

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

                    sf::Vector2f toEntry = entryPoint - densePath.back();
                    float distToEntry = std::sqrt(toEntry.x * toEntry.x + toEntry.y * toEntry.y);
                    if (distToEntry > 4.f) {
                        sf::Vector2f dirToEntry = toEntry / distToEntry;
                        for (float d = 4.f; d < distToEntry; d += 4.f) {
                            densePath.push_back(densePath.back() + dirToEntry * 4.f);
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
                        densePath.push_back(sf::Vector2f(
                            circleCenter.x + std::cos(currentArcAngle) * cornerRadius,
                            circleCenter.y + std::sin(currentArcAngle) * cornerRadius
                        ));
                    }

                    i++;
                    continue;
                }
            }
        }

        sf::Vector2f diff = p2 - densePath.back();
        float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist > 4.f) {
            sf::Vector2f direction = diff / dist;
            for (float d = 4.f; d < dist; d += 4.f) {
                densePath.push_back(densePath.back() + direction * 4.f);
            }
        }
    }

    densePath.push_back(waypoints.back());
    currentPathIndex = 0;
    hasFinishedPath = false;
    isHeadingInitialized = false;
}

void Vehicle::update(float dt, const std::vector<std::unique_ptr<IAgent>>& agents, const World& world) {
    if (densePath.empty() || hasFinishedPath) return;

    size_t closestIdx = currentPathIndex;
    float minDist = 999999.f;
    size_t searchWindow = std::min(currentPathIndex + 30, densePath.size());

    for (size_t i = currentPathIndex; i < searchWindow; ++i) {
        sf::Vector2f diff = densePath[i] - position;
        float d = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (d < minDist) {
            minDist = d;
            closestIdx = i;
        }
    }
    currentPathIndex = closestIdx;

    lastPerception = Perception::scan(position, currentAngle, this, agents, world, visionParams);

    if (currentPathIndex >= densePath.size() - 2) {
        hasFinishedPath = true;
        currentSpeed = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        return;
    }

    size_t steeringLookAheadIdx = std::min(currentPathIndex + 3, densePath.size() - 1);
    sf::Vector2f targetDir = densePath[steeringLookAheadIdx] - position;
    float targetAngleRad = std::atan2(targetDir.y, targetDir.x);
    float targetAngleDeg = targetAngleRad * 180.f / 3.14159265f;

    if (!isHeadingInitialized) {
        currentAngle = targetAngleDeg;
        isHeadingInitialized = true;
    }

    float angleDiff = targetAngleDeg - currentAngle;
    while (angleDiff < -180.f) angleDiff += 360.f;
    while (angleDiff > 180.f) angleDiff -= 360.f;

    float maxTurnSpeed = 300.f;
    float steering = angleDiff * 15.0f;
    steering = std::clamp(steering, -maxTurnSpeed, maxTurnSpeed);
    currentAngle += steering * dt;

    float roadLimit = world.getSpeedLimitAt(position.x, position.y);
    float targetSpeed;
    if (isCommittedToPass) {
        targetSpeed = std::min(maxSpeed, lastRoadSpeedLimit);
    } else {
        targetSpeed = (roadLimit > 0.f) ? std::min(maxSpeed, roadLimit) : maxSpeed;
        if (roadLimit > 0.f) lastRoadSpeedLimit = roadLimit;
    }
    float corneringSpeed = maxSpeed * 0.35f;

    if (std::abs(angleDiff) > 4.f) {
        targetSpeed = corneringSpeed;
    } else {
        size_t farIdx = std::min(currentPathIndex + 20, densePath.size() - 1);
        size_t farNextIdx = std::min(farIdx + 2, densePath.size() - 1);

        sf::Vector2f currentRoadDir = densePath[std::min(currentPathIndex + 2, densePath.size() - 1)] - densePath[currentPathIndex];
        sf::Vector2f futureRoadDir = densePath[farNextIdx] - densePath[farIdx];

        float lenC = std::sqrt(currentRoadDir.x * currentRoadDir.x + currentRoadDir.y * currentRoadDir.y);
        float lenF = std::sqrt(futureRoadDir.x * futureRoadDir.x + futureRoadDir.y * futureRoadDir.y);

        if (lenC > 0.1f && lenF > 0.1f) {
            currentRoadDir /= lenC;
            futureRoadDir /= lenF;

            float dot = (currentRoadDir.x * futureRoadDir.x) + (currentRoadDir.y * futureRoadDir.y);
            if (dot < 0.98f) targetSpeed = corneringSpeed;
        }
    }

    // --- FIX 1 : HITBOX ET ESPACEMENT DYNAMIQUE ---
    if (lastPerception.hasDirectObstacle) {
        float gap = lastPerception.directObstacleDistance;

        // La marge de sécurité est maintenant intelligente !
        // Moitié de MA taille + Marge de sécurité (15px) + Distance d'arrêt proportionnelle à ma vitesse (règle des 2 secondes)
        float desiredGap = (getLength() / 2.f) + 15.f + (currentSpeed * 0.8f);

        if (gap <= desiredGap) {
            targetSpeed = 0.f;
            if (currentSpeed > 0.f) {
                // Freinage d'urgence puissant (évite les frottements)
                currentSpeed = std::max(0.f, currentSpeed - (maxAcceleration * 5.f) * dt);
            }
        } else {
            float safeDeceleration = maxAcceleration * 1.5f;
            float availableDist = gap - desiredGap;
            float safeSpeed = std::sqrt(2.f * safeDeceleration * std::max(0.1f, availableDist));
            if (safeSpeed < targetSpeed) targetSpeed = safeSpeed;
        }
    }

    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);
    if (interOn) {
        isCommittedToPass = true;
        committedIntersectionId = interOn->getId();
    } else if (isCommittedToPass) {
        isCommittedToPass = false;
        committedIntersectionId = -1;
    }

    if (interOn && isCommittedToPass) {
        // Déjà dans le carrefour, on avance !
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
                // Fonce
            } else {
                Approach::Direction myDir = world.getApproachDirection(currentAngle);
                bool allowed = interAhead->canPass(myDir, agents);
                float brakingDistance = (currentSpeed * currentSpeed) / (2.f * maxAcceleration * 1.8f);

                // --- FIX 2 : IA TIME-TO-COLLISION POUR PRIORITÉ À DROITE ---
                if (allowed && interAhead->getType() == RegulationType::PRIORITY_RIGHT) {
                    // Calcul du temps qu'il NOUS faut pour traverser (Distance + Taille Carrefour + Notre Longueur)
                    float myTTC = (distToInter + 80.f + getLength()) / std::max(10.f, currentSpeed);

                    // Création des vecteurs Avant et Droite pour repérer géométriquement les menaces
                    float radForward = currentAngle * 3.14159265f / 180.f;
                    float radRight = (currentAngle + 90.f) * 3.14159265f / 180.f;
                    sf::Vector2f forwardVec(std::cos(radForward), std::sin(radForward));
                    sf::Vector2f rightVec(std::cos(radRight), std::sin(radRight));

                    // Quel est l'angle de base des véhicules venant de droite ?
                    float threatHeading = currentAngle - 90.f;
                    while (threatHeading < -180.f) threatHeading += 360.f;
                    while (threatHeading > 180.f) threatHeading -= 360.f;

                    for (const auto& other : agents) {
                        if (other.get() == this) continue;

                        float diff = other->getHeading() - threatHeading;
                        while (diff < -180.f) diff += 360.f;
                        while (diff > 180.f) diff -= 360.f;

                        // Si l'autre véhicule regarde dans la direction de la menace (+/- 45 degrés)
                        if (std::abs(diff) < 45.f) {
                            sf::Vector2f toOther = other->getPosition() - position;
                            float dotForward = toOther.x * forwardVec.x + toOther.y * forwardVec.y;
                            float dotRight = toOther.x * rightVec.x + toOther.y * rightVec.y;

                            // S'il est physiquement devant nous et sur notre droite (dans un rayon de 150px)
                            if (dotForward > -20.f && dotForward < 150.f && dotRight > 0.f && dotRight < 150.f) {

                                // On calcule dans combien de temps il percutera notre axe central
                                float otherTTC = dotRight / std::max(10.f, other->getSpeed());

                                // Si son temps de collision arrive AVANT qu'on ait fini de traverser (avec marge de sécurité)
                                if (otherTTC < myTTC + 1.2f) {
                                    allowed = false; // Danger ! On cède le passage !
                                    break;
                                }
                            }
                        }
                    }
                }
                // -----------------------------------------------------------

                if (allowed) {
                    if (distToInter < 60.f) {
                        isCommittedToPass = true;
                        committedIntersectionId = interAhead->getId();
                    }
                } else if (distToInter > brakingDistance) {
                    isCommittedToPass = false;
                    committedIntersectionId = -1;

                    if (interAhead->getType() == RegulationType::TRAFFIC_LIGHT) {
                        float stopDist = 25.f + (getLength() / 2.f); // Arrêt parfait au trait
                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                            if (currentSpeed > 5.f) currentSpeed = std::max(0.f, currentSpeed - (maxAcceleration * 5.f) * dt);
                        } else if (distToInter < stopDist + 80.f) {
                            float factor = (distToInter - stopDist) / 80.f;
                            targetSpeed = std::min(targetSpeed, targetSpeed * factor);
                        }
                    } else if (interAhead->getType() == RegulationType::PRIORITY_RIGHT) {
                        float stopDist = 25.f + (getLength() / 2.f); // Arrêt parfait au trait
                        float brakeDist = 120.f;

                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                        } else if (distToInter < brakeDist) {
                            float factor = (distToInter - stopDist) / (brakeDist - stopDist);
                            float interSpeed = corneringSpeed + (targetSpeed - corneringSpeed) * factor;
                            if (interSpeed < targetSpeed) targetSpeed = interSpeed;
                        }
                    }
                } else {
                    // Si on n'a plus le temps de freiner en sécurité, on force le passage
                    isCommittedToPass = true;
                    committedIntersectionId = interAhead->getId();
                }
            }
        } else {
            isCommittedToPass = false;
            committedIntersectionId = -1;
        }
    }

    if (currentSpeed < targetSpeed) {
        currentSpeed += maxAcceleration * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= (maxAcceleration * 1.8f) * dt;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    float rad = currentAngle * 3.14159265f / 180.f;
    velocity.x = std::cos(rad) * currentSpeed;
    velocity.y = std::sin(rad) * currentSpeed;

    position += velocity * dt;
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
    if (!isSelectedFlag || densePath.empty() || hasFinishedPath) return;

    Perception::drawDebugCone(window, position, currentAngle, visionParams, lastPerception);

    sf::VertexArray pathLine(sf::LineStrip, densePath.size() - currentPathIndex);
    for (size_t i = currentPathIndex; i < densePath.size(); ++i) {
        pathLine[i - currentPathIndex].position = densePath[i];
        pathLine[i - currentPathIndex].color = sf::Color(147, 112, 219, 220);
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
        densePath.clear();
        currentSpeed = 0.f;
    }
}

void Vehicle::resetToStart(const World& world) {
    // 1. Téléportation mathématique
    position = sf::Vector2f(startTile.x * tileSize + tileSize / 2.f, startTile.y * tileSize + tileSize / 2.f);

    // --- LE FIX VISUEL ---
    // On force la mise à jour graphique instantanément pour tuer le "fantôme"
    // qui restait bloqué au milieu de la route détruite.
    shape.setPosition(position);
    shape.setRotation(0.f);
    // -----------------------------

    velocity = sf::Vector2f(0.f, 0.f);
    currentSpeed = 0.f;
    currentAngle = 0.f;
    isHeadingInitialized = false;
    hasFinishedPath = false;
    currentPathIndex = 0;

    isCommittedToPass = false;
    committedIntersectionId = -1;

    // 2. On tente de recalculer la route vers l'objectif
    std::vector<sf::Vector2i> fullPath = Pathfinder::findPath(world, startTile, goalTile);

    if (!fullPath.empty()) {
        // --- LE FIX DE SÉCURITÉ ---
        // On protège le startTile pour qu'il ne soit pas écrasé
        sf::Vector2i originalStart = startTile;
        sf::Vector2i originalGoal = goalTile;

        setPath(fullPath);

        startTile = originalStart;
        goalTile = originalGoal;
    } else {
        // La route est coupée, on le laisse à sa place de spawn (déjà set plus haut)
        densePath.clear();
    }
}

float Vehicle::getRemainingDistance() const {
    if (densePath.empty() || hasFinishedPath) return 0.f;

    float distance = 0.f;
    sf::Vector2f prevPos = position;

    for(size_t i = currentPathIndex; i < densePath.size(); ++i) {
        sf::Vector2f currentPos = densePath[i];
        float dx = currentPos.x - prevPos.x;
        float dy = currentPos.y - prevPos.y;
        distance += std::sqrt(dx * dx + dy * dy);
        prevPos = currentPos;
    }

    return distance;
}