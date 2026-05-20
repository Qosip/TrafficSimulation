// src/Perception.cpp
#include "Perception.hpp"
#include "IAgent.hpp"
#include "World.hpp"
#include <cmath>

static const float PI = 3.14159265f;
static const float DEG2RAD = PI / 180.f;
static const float RAD2DEG = 180.f / PI;

// Normalise un angle entre -180 et 180
static float normalizeAngle(float deg) {
    while (deg < -180.f) deg += 360.f;
    while (deg > 180.f) deg -= 360.f;
    return deg;
}

PerceptionResult Perception::scan(
    sf::Vector2f myPosition,
    float myHeadingDeg,
    const IAgent* myself,
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const World& world,
    const VisionParams& params)
{
    PerceptionResult result;

    // --- 1. SCAN DES AGENTS ---
    for (const auto& agent : agents) {
        // On ne se détecte pas soi-même
        if (agent.get() == myself) continue;

        sf::Vector2f otherPos = agent->getPosition();
        sf::Vector2f diff = otherPos - myPosition;
        float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);

        // Hors de portée → ignoré
        if (dist > params.range || dist < 1.f) continue;

        // Angle vers l'autre agent
        float angleToOther = std::atan2(diff.y, diff.x) * RAD2DEG;
        float relAngle = normalizeAngle(angleToOther - myHeadingDeg);

        // Hors du cône large → ignoré
        if (std::abs(relAngle) > params.halfAngleDeg) continue;

        // L'agent est dans notre cône de vision !
        DetectedObject obj;
        obj.agent = agent.get();
        obj.type = DetectedType::VEHICLE; // Pour l'instant tout est véhicule
        obj.distance = dist;
        obj.relativeAngle = relAngle;
        obj.position = otherPos;
        result.detected.push_back(obj);

        // Check du cône étroit (obstacle direct devant)
        if (std::abs(relAngle) <= params.directHalfAngle) {
            if (!result.hasDirectObstacle || dist < result.directObstacleDistance) {
                result.hasDirectObstacle = true;
                result.directObstacleDistance = dist;
            }
        }
    }

    // --- 2. DÉTECTION D'INTERSECTION DEVANT NOUS ---
    // On projette un point devant le véhicule et on regarde le type de tuile
    float headRad = myHeadingDeg * DEG2RAD;
    sf::Vector2f lookDir(std::cos(headRad), std::sin(headRad));

    float tileSize = world.getTileSize();
    for (float d = 0.f; d < params.intersectionLookAhead; d += tileSize * 0.5f) {
        sf::Vector2f checkPos = myPosition + lookDir * d;
        int gridX = (int)(checkPos.x / tileSize);
        int gridY = (int)(checkPos.y / tileSize);

        if (world.getTile(gridX, gridY).roadType == RoadType::INTERSECTION) {
            result.approachingIntersection = true;
            break;
        }
    }

    return result;
}

void Perception::drawDebugCone(
    sf::RenderWindow& window,
    sf::Vector2f position,
    float headingDeg,
    const VisionParams& params,
    const PerceptionResult& result)
{
    // Couleur du cône selon l'état
    sf::Color coneColor;
    if (result.hasDirectObstacle && result.directObstacleDistance < 50.f) {
        coneColor = sf::Color(255, 0, 0, 50);      // Rouge = arrêt imminent
    } else if (result.hasDirectObstacle) {
        coneColor = sf::Color(255, 165, 0, 50);     // Orange = obstacle détecté
    } else if (result.approachingIntersection) {
        coneColor = sf::Color(255, 255, 0, 50);     // Jaune = intersection ahead
    } else {
        coneColor = sf::Color(0, 255, 0, 50);       // Vert = libre
    }

    // Dessin du cône comme un triangle fan
    int segments = 20;
    sf::VertexArray cone(sf::TriangleFan, segments + 2);

    // Point central = position du véhicule
    cone[0].position = position;
    cone[0].color = coneColor;

    float startAngle = (headingDeg - params.halfAngleDeg) * DEG2RAD;
    float endAngle = (headingDeg + params.halfAngleDeg) * DEG2RAD;

    for (int i = 0; i <= segments; ++i) {
        float t = (float)i / segments;
        float angle = startAngle + t * (endAngle - startAngle);
        cone[i + 1].position = position + sf::Vector2f(
            std::cos(angle) * params.range,
            std::sin(angle) * params.range
        );
        cone[i + 1].color = coneColor;
    }
    window.draw(cone);

    // Dessin du cône étroit (direct) en plus contrasté
    sf::Color directColor = coneColor;
    directColor.a = 80;
    sf::VertexArray directCone(sf::TriangleFan, segments + 2);
    directCone[0].position = position;
    directCone[0].color = directColor;

    float dStart = (headingDeg - params.directHalfAngle) * DEG2RAD;
    float dEnd = (headingDeg + params.directHalfAngle) * DEG2RAD;

    for (int i = 0; i <= segments; ++i) {
        float t = (float)i / segments;
        float angle = dStart + t * (dEnd - dStart);
        directCone[i + 1].position = position + sf::Vector2f(
            std::cos(angle) * params.range,
            std::sin(angle) * params.range
        );
        directCone[i + 1].color = directColor;
    }
    window.draw(directCone);

    // Cercles sur les objets détectés
    for (const auto& obj : result.detected) {
        sf::CircleShape marker(5.f);
        marker.setOrigin(5.f, 5.f);
        marker.setPosition(obj.position);

        if (obj.distance < 50.f) {
            marker.setFillColor(sf::Color::Red);
        } else {
            marker.setFillColor(sf::Color(255, 165, 0));
        }
        window.draw(marker);
    }
}