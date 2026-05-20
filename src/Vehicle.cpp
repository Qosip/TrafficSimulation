// src/Vehicle.cpp
#include "Vehicle.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
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

    // Convertir les coordonnées de la grille en positions réelles (pixels) au centre des tuiles
    std::vector<sf::Vector2f> waypoints;
    for (const auto& tile : newPath) {
        waypoints.push_back(sf::Vector2f(
            (tile.x * tileSize) + tileSize / 2.f,
            (tile.y * tileSize) + tileSize / 2.f
        ));
    }

    if (waypoints.size() < 2) return;

    // Rayon de braquage de l'arc de cercle (ici, la taille d'une tuile, soit 50 pixels)
    float cornerRadius = tileSize;

    densePath.push_back(waypoints.front());

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        sf::Vector2f p1 = waypoints[i];
        sf::Vector2f p2 = waypoints[i+1];

        // S'il y a un troisième point, on vérifie si ça forme un virage à 90°
        if (i + 2 < waypoints.size()) {
            sf::Vector2f p3 = waypoints[i+2];

            sf::Vector2f dir1 = p2 - p1;
            sf::Vector2f dir2 = p3 - p2;

            float len1 = std::sqrt(dir1.x * dir1.x + dir1.y * dir1.y);
            float len2 = std::sqrt(dir2.x * dir2.x + dir2.y * dir2.y);

            if (len1 > 0.1f && len2 > 0.1f) {
                dir1 /= len1;
                dir2 /= len2;

                // Produit scalaire pour vérifier si les deux segments sont perpendiculaires (virage à 90°)
                float dot = (dir1.x * dir2.x) + (dir1.y * dir2.y);
                if (std::abs(dot) < 0.1f) {

                    // --- CALCUL DE L'ARC DE CERCLE PARFAIT ---
                    // On recule le point d'entrée du virage et on avance le point de sortie
                    sf::Vector2f entryPoint = p2 - dir1 * cornerRadius;
                    sf::Vector2f exitPoint = p2 + dir2 * cornerRadius;

                    // Génération de la ligne droite jusqu'à l'entrée du virage
                    sf::Vector2f toEntry = entryPoint - densePath.back();
                    float distToEntry = std::sqrt(toEntry.x * toEntry.x + toEntry.y * toEntry.y);
                    if (distToEntry > 4.f) {
                        sf::Vector2f dirToEntry = toEntry / distToEntry;
                        for (float d = 4.f; d < distToEntry; d += 4.f) {
                            densePath.push_back(densePath.back() + dirToEntry * 4.f);
                        }
                    }

                    // Calcul du centre mathématique du cercle de braquage
                    sf::Vector2f circleCenter = entryPoint + sf::Vector2f(-dir1.y, dir1.x) * cornerRadius;
                    // Sécurité : on vérifie si le centre est du bon côté, sinon on inverse
                    sf::Vector2f testRadius = exitPoint - circleCenter;
                    if (std::abs(std::sqrt(testRadius.x * testRadius.x + testRadius.y * testRadius.y) - cornerRadius) > 1.f) {
                        circleCenter = entryPoint + sf::Vector2f(dir1.y, -dir1.x) * cornerRadius;
                    }

                    // Calcul des angles de départ et d'arrivée sur le cercle
                    float startAngle = std::atan2(entryPoint.y - circleCenter.y, entryPoint.x - circleCenter.x);
                    float endAngle = std::atan2(exitPoint.y - circleCenter.y, exitPoint.x - circleCenter.x);

                    // Gestion de la continuité des angles pour interpoler dans le bon sens
                    float angleDiff = endAngle - startAngle;
                    while (angleDiff < -3.14159265f) angleDiff += 2.f * 3.14159265f;
                    while (angleDiff > 3.14159265f) angleDiff -= 2.f * 3.14159265f;

                    // On échantillonne l'arc de cercle en 15 micro-points pour une courbe ultra-lisse
                    int numSegments = 15;
                    for (int j = 1; j <= numSegments; ++j) {
                        float t = (float)j / numSegments;
                        float currentArcAngle = startAngle + angleDiff * t;
                        densePath.push_back(sf::Vector2f(
                            circleCenter.x + std::cos(currentArcAngle) * cornerRadius,
                            circleCenter.y + std::sin(currentArcAngle) * cornerRadius
                        ));
                    }

                    i++; // On saute le point du coin (p2) puisqu'on l'a contourné par un arc de cercle
                    continue;
                }
            }
        }

        // Génération standard en ligne droite si ce n'est pas un virage
        sf::Vector2f diff = p2 - densePath.back();
        float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist > 4.f) {
            sf::Vector2f direction = diff / dist;
            for (float d = 4.f; d < dist; d += 4.f) {
                densePath.push_back(densePath.back() + direction * 4.f);
            }
        }
    }

    // Ajout du point final exact pour terminer proprement la route
    densePath.push_back(waypoints.back());

    currentPathIndex = 0;
    hasFinishedPath = false;
    isHeadingInitialized = false;
}

void Vehicle::update(float dt,
                     const std::vector<std::unique_ptr<IAgent>>& agents,
                     const World& world) {
    if (densePath.empty() || hasFinishedPath) return;

    // --- 1. ANCRAGE SUR LA ROUTE ---
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

    // --- PERCEPTION ---
    lastPerception = Perception::scan(position, currentAngle, this, agents, world, visionParams);

    if (currentPathIndex >= densePath.size() - 2) {
        hasFinishedPath = true;
        currentSpeed = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        return;
    }

    // --- 2. GESTION DU VOLANT (Look-Ahead ultra-court pour coller à la ligne) ---
    // On regarde seulement 3 points devant (~12 pixels) au lieu de 10
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

    // Direction beaucoup plus réactive pour suivre la courbe instantanément
    float maxTurnSpeed = 300.f;
    float steering = angleDiff * 15.0f;
    steering = std::clamp(steering, -maxTurnSpeed, maxTurnSpeed);
    currentAngle += steering * dt;

    // --- 3. LES 3 PHASES DE CONDUITE (Le Cerveau) ---
    float roadLimit = world.getSpeedLimitAt(position.x, position.y);
    float targetSpeed;
    if (isCommittedToPass) {
        // Dans l'intersection → on utilise la limite de la route autour, pas celle de la tuile intersection
        // On cherche la limite de la route d'avant (la dernière limite non-intersection)
        targetSpeed = std::min(maxSpeed, lastRoadSpeedLimit);
    } else {
        targetSpeed = (roadLimit > 0.f) ? std::min(maxSpeed, roadLimit) : maxSpeed;
        // Sauvegarder la dernière limite de route (pas intersection) pour la traversée
        if (roadLimit > 0.f) {
            lastRoadSpeedLimit = roadLimit;
        }
    }
    float corneringSpeed = maxSpeed * 0.35f;

    // A. Phase de Virage : Si le volant est tourné, vitesse basse
    if (std::abs(angleDiff) > 4.f) {
        targetSpeed = corneringSpeed;
    }
    // B. Phase d'Anticipation (Freinage doux en ligne droite)
    else {
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
            if (dot < 0.98f) {
                targetSpeed = corneringSpeed;
            }
        }
    }

    // C. Phase de Perception : réaction aux obstacles détectés
    if (lastPerception.hasDirectObstacle) {
        float dist = lastPerception.directObstacleDistance;
        float stopDistance = 40.f;    // Distance d'arrêt total
        float brakeDistance = 120.f;  // Distance où on commence à freiner

        if (dist <= stopDistance) {
            // Trop proche → arrêt complet
            targetSpeed = 0.f;
        } else if (dist < brakeDistance) {
            // Freinage proportionnel : plus c'est proche, plus on freine
            float factor = (dist - stopDistance) / (brakeDistance - stopDistance);
            float perceptionSpeed = targetSpeed * factor;
            if (perceptionSpeed < targetSpeed) {
                targetSpeed = perceptionSpeed;
            }
        }
    }
    // D. Phase Intersection : respecter la régulation
    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);

    if (interOn) {
        isCommittedToPass = true;
        committedIntersectionId = interOn->getId();
    } else if (isCommittedToPass) {
        isCommittedToPass = false;
        committedIntersectionId = -1;
    }

    if (interOn && isCommittedToPass) {
        // Pas de freinage, on traverse
    } else if (!interOn) {
        float headRad = currentAngle * 3.14159265f / 180.f;
        sf::Vector2f lookDir(std::cos(headRad), std::sin(headRad));

        const Intersection* interAhead = nullptr;
        float distToInter = 999.f;

        for (float d = 10.f; d < 130.f; d += tileSize * 0.4f) {
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
                // Déjà engagé → on fonce
            } else {
                Approach::Direction myDir = world.getApproachDirection(currentAngle);
                bool allowed = interAhead->canPass(myDir, agents);

                // Distance nécessaire pour freiner à cette vitesse
                float brakingDistance = (currentSpeed * currentSpeed) / (2.f * maxAcceleration * 1.8f);

                if (allowed) {
                    if (distToInter < 60.f) {
                        isCommittedToPass = true;
                        committedIntersectionId = interAhead->getId();
                    }
                } else if (distToInter > brakingDistance) {
                    // On peut encore s'arrêter → on freine
                    isCommittedToPass = false;
                    committedIntersectionId = -1;

                    if (interAhead->getType() == RegulationType::TRAFFIC_LIGHT) {
                        float stopDist = 30.f;
                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                        } else if (distToInter < 90.f) {
                            float factor = (distToInter - stopDist) / 40.f;
                            targetSpeed = std::min(targetSpeed, targetSpeed * factor);
                        }
                    } else if (interAhead->getType() == RegulationType::PRIORITY_RIGHT) {
                        float stopDist = 35.f;
                        float brakeDist = 120.f;

                        if (distToInter <= stopDist) {
                            targetSpeed = 0.f;
                        } else if (distToInter < brakeDist) {
                            float factor = (distToInter - stopDist) / (brakeDist - stopDist);
                            float interSpeed = corneringSpeed + (targetSpeed - corneringSpeed) * factor;
                            if (interSpeed < targetSpeed) {
                                targetSpeed = interSpeed;
                            }
                        }
                    }
                } else {
                    // Trop proche pour freiner → on s'engage et on passe
                    isCommittedToPass = true;
                    committedIntersectionId = interAhead->getId();
                }
            }
        } else {
            isCommittedToPass = false;
            committedIntersectionId = -1;
        }
    }

    // --- 4. ACCÉLÉRATEUR ET FREINS ---
    if (currentSpeed < targetSpeed) {
        // Accélération normale
        currentSpeed += maxAcceleration * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else if (currentSpeed > targetSpeed) {
        // Freinage de confort : 1.8x le moteur au lieu de 5.0x
        currentSpeed -= (maxAcceleration * 1.8f) * dt;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    // --- 5. APPLICATION DU MOUVEMENT ---
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

sf::Vector2f Vehicle::getPosition() const {
    return position;
}

bool Vehicle::isFinished() const {
    return hasFinishedPath;
}

// --- MODE DEBUG ---

bool Vehicle::contains(sf::Vector2f point) const {
    // getGlobalBounds() crée un rectangle autour de la voiture et check si la souris est dedans
    return shape.getGlobalBounds().contains(point);
}

void Vehicle::setSelected(bool selected) {
    isSelected = selected;
}

void Vehicle::drawDebug(sf::RenderWindow& window) {
    if (!isSelected || densePath.empty() || hasFinishedPath) return;

    // Dessin du cône de vision
    Perception::drawDebugCone(window, position, currentAngle, visionParams, lastPerception);

    // Dessin du chemin restant
    sf::VertexArray pathLine(sf::LineStrip, densePath.size() - currentPathIndex);
    for (size_t i = currentPathIndex; i < densePath.size(); ++i) {
        pathLine[i - currentPathIndex].position = densePath[i];
        pathLine[i - currentPathIndex].color = sf::Color::Magenta;
    }
    window.draw(pathLine);
}

float Vehicle::getHeading() const {
    return currentAngle;
}

float Vehicle::getSpeed() const {
    return currentSpeed;
}