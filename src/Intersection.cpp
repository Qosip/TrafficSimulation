// src/Intersection.cpp
#include "Intersection.hpp"
#include "IAgent.hpp"
#include <cmath>

Intersection::Intersection(int id, RegulationType type)
    : id(id), type(type)
{}

void Intersection::addCoveredTile(sf::Vector2i tile) {
    coveredTiles.push_back(tile);
}

void Intersection::addApproach(const Approach& approach) {
    approaches.push_back(approach);
}

void Intersection::update(float dt) {
    if (type == RegulationType::TRAFFIC_LIGHT) {
        updateTrafficLight(dt);
    }
}

void Intersection::updateTrafficLight(float dt) {
    lightTimer += dt;

    float phaseDuration = (currentPhase == 0 || currentPhase == 2) ? greenDuration : orangeDuration;

    if (lightTimer >= phaseDuration) {
        lightTimer = 0.f;
        currentPhase = (currentPhase + 1) % 4;

        for (auto& app : approaches) {
            bool isNS = (app.direction == Approach::Direction::NORTH ||
                         app.direction == Approach::Direction::SOUTH);

            if (currentPhase == 0)      app.hasGreen = isNS;
            else if (currentPhase == 2) app.hasGreen = !isNS;
            else                        app.hasGreen = false;
        }
    }
}

LightState Intersection::getLightState(Approach::Direction dir) const {
    if (type != RegulationType::TRAFFIC_LIGHT) return LightState::GREEN;

    bool isNS = (dir == Approach::Direction::NORTH || dir == Approach::Direction::SOUTH);

    if (currentPhase == 0) return isNS ? LightState::GREEN : LightState::RED;
    if (currentPhase == 1) return isNS ? LightState::ORANGE : LightState::RED;
    if (currentPhase == 2) return isNS ? LightState::RED : LightState::GREEN;
    return isNS ? LightState::RED : LightState::ORANGE;
}

bool Intersection::canPass(Approach::Direction fromDirection,
                            const std::vector<std::unique_ptr<IAgent>>& agents) const
{
    switch (type) {
        case RegulationType::TRAFFIC_LIGHT: {
            LightState state = getLightState(fromDirection);
            return (state == LightState::GREEN || state == LightState::ORANGE);
        }

        case RegulationType::PRIORITY_RIGHT: {
            Approach::Direction rightDir;
            switch (fromDirection) {
                case Approach::Direction::NORTH: rightDir = Approach::Direction::WEST;  break;
                case Approach::Direction::SOUTH: rightDir = Approach::Direction::EAST;  break;
                case Approach::Direction::EAST:  rightDir = Approach::Direction::NORTH; break;
                case Approach::Direction::WEST:  rightDir = Approach::Direction::SOUTH; break;
            }

            const Approach* rightApproach = nullptr;
            for (const auto& app : approaches) {
                if (app.direction == rightDir) {
                    rightApproach = &app;
                    break;
                }
            }

            if (!rightApproach) return true;

            // On check une zone plus large le long de la voie de droite
            // pas juste la tuile d'entrée
            sf::Vector2f entryCenter(
                rightApproach->entryTile.x * 50.f + 25.f,
                rightApproach->entryTile.y * 50.f + 25.f
            );

            // Direction d'où vient la voie de droite (vers le carrefour)
            sf::Vector2f interCenter(0.f, 0.f);
            for (const auto& t : coveredTiles) {
                interCenter.x += t.x * 50.f + 25.f;
                interCenter.y += t.y * 50.f + 25.f;
            }
            interCenter.x /= coveredTiles.size();
            interCenter.y /= coveredTiles.size();

            // On scanne les agents sur la voie de droite dans un rayon généreux
            for (const auto& agent : agents) {
                sf::Vector2f pos = agent->getPosition();

                // Distance à la tuile d'entrée de droite
                sf::Vector2f diff = pos - entryCenter;
                float distToEntry = std::sqrt(diff.x * diff.x + diff.y * diff.y);

                // Distance au centre de l'intersection
                sf::Vector2f diffInter = pos - interCenter;
                float distToInter = std::sqrt(diffInter.x * diffInter.x + diffInter.y * diffInter.y);

                // L'agent est proche de la voie de droite ET roule vers le carrefour
                if ((distToEntry < 120.f || distToInter < 80.f) && agent->getSpeed() > 5.f) {
                    // Vérifier que l'agent se dirige bien vers l'intersection (pas qu'il s'éloigne)
                    float agentHeading = agent->getHeading();
                    float agentRad = agentHeading * 3.14159265f / 180.f;
                    sf::Vector2f agentDir(std::cos(agentRad), std::sin(agentRad));

                    sf::Vector2f toInter = interCenter - pos;
                    float lenToInter = std::sqrt(toInter.x * toInter.x + toInter.y * toInter.y);
                    if (lenToInter > 1.f) {
                        toInter /= lenToInter;
                        float dot = agentDir.x * toInter.x + agentDir.y * toInter.y;
                        // dot > 0 = l'agent se dirige vers l'intersection
                        if (dot > 0.3f) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        default:
            return true;
    }
}

bool Intersection::coversTile(int gridX, int gridY) const {
    for (const auto& tile : coveredTiles) {
        if (tile.x == gridX && tile.y == gridY) return true;
    }
    return false;
}

int Intersection::getId() const { return id; }
RegulationType Intersection::getType() const { return type; }
const std::vector<sf::Vector2i>& Intersection::getCoveredTiles() const { return coveredTiles; }
const std::vector<Approach>& Intersection::getApproaches() const { return approaches; }

void Intersection::draw(sf::RenderWindow& window, float tileSize) const {
    if (type == RegulationType::TRAFFIC_LIGHT) {
        for (const auto& app : approaches) {
            sf::CircleShape light(6.f);
            light.setOrigin(6.f, 6.f);
            light.setOutlineThickness(1.f);
            light.setOutlineColor(sf::Color::Black);

            float px = app.entryTile.x * tileSize + tileSize / 2.f;
            float py = app.entryTile.y * tileSize + tileSize / 2.f;

            switch (app.direction) {
                case Approach::Direction::NORTH: px += tileSize * 0.35f; break;
                case Approach::Direction::SOUTH: px -= tileSize * 0.35f; break;
                case Approach::Direction::EAST:  py += tileSize * 0.35f; break;
                case Approach::Direction::WEST:  py -= tileSize * 0.35f; break;
            }

            light.setPosition(px, py);

            LightState state = getLightState(app.direction);
            switch (state) {
                case LightState::GREEN:  light.setFillColor(sf::Color::Green); break;
                case LightState::ORANGE: light.setFillColor(sf::Color(255, 165, 0)); break;
                case LightState::RED:    light.setFillColor(sf::Color::Red); break;
            }
            window.draw(light);
        }
    }

    if (type == RegulationType::PRIORITY_RIGHT) {
        sf::Vector2f center(0.f, 0.f);
        for (const auto& tile : coveredTiles) {
            center.x += tile.x * tileSize + tileSize / 2.f;
            center.y += tile.y * tileSize + tileSize / 2.f;
        }
        center.x /= coveredTiles.size();
        center.y /= coveredTiles.size();

        sf::CircleShape diamond(8.f, 4);
        diamond.setOrigin(8.f, 8.f);
        diamond.setPosition(center);
        diamond.setFillColor(sf::Color(255, 204, 0));
        diamond.setRotation(45.f);
        window.draw(diamond);
    }
}