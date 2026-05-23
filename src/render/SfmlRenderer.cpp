// src/render/SfmlRenderer.cpp
//
// Etape 4 : tous les types simulation arrivent en core::Vec2/Color/TileCoord
// et sont convertis ICI vers leur equivalent SFML.
#include "render/SfmlRenderer.hpp"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/VertexArray.hpp>

#include <algorithm>
#include <cmath>

#include "core/agent/AgentDebugSnapshot.hpp"
#include "core/agent/IAgent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/world/World.hpp"
#include "core/math/Constants.hpp"
#include "render/SfmlInterop.hpp"

namespace render {

namespace {
constexpr float DEG2RAD = core::math::DEG2RAD;
}

SfmlRenderer::SfmlRenderer(sf::RenderTarget& target) : target_(target) {}

void SfmlRenderer::rebuildMapCache(const World& world) {
    const int   gw = world.getGridWidth();
    const int   gh = world.getGridHeight();
    const float ts = world.getTileSize();

    if (mapTexture_.getSize().x != static_cast<unsigned>(gw * ts) ||
        mapTexture_.getSize().y != static_cast<unsigned>(gh * ts))
    {
        mapTexture_.create(static_cast<unsigned>(gw * ts),
                           static_cast<unsigned>(gh * ts));
    }

    mapTexture_.clear(sf::Color(34, 73, 41)); // Tonalite pelouse

    sf::RectangleShape tileShape(sf::Vector2f(ts, ts));
    tileShape.setOutlineThickness(-1.f);
    tileShape.setOutlineColor(sf::Color(0, 0, 0, 35));

    for (int x = 0; x < gw; ++x) {
        for (int y = 0; y < gh; ++y) {
            const Tile& tile = world.getTile(x, y);
            if (tile.roadType == RoadType::NONE) continue;

            const float posX = x * ts;
            const float posY = y * ts;
            tileShape.setPosition(posX, posY);
            tileShape.setFillColor(toSfml(getRoadColor(tile.roadType)));
            mapTexture_.draw(tileShape);

            if (tile.roadType == RoadType::CITY_50 || tile.roadType == RoadType::HIGHWAY_130) {
                sf::RectangleShape dash(sf::Vector2f(ts * 0.35f, 2.f));
                dash.setFillColor(sf::Color(255, 255, 255, 140));
                dash.setOrigin(dash.getSize().x / 2.f, dash.getSize().y / 2.f);
                dash.setPosition(posX + ts / 2.f, posY + ts / 2.f);

                if (tile.direction == TileDirection::UP || tile.direction == TileDirection::DOWN) {
                    dash.setRotation(90.f);
                }
                mapTexture_.draw(dash);
            }
        }
    }
    mapTexture_.display();
    mapSprite_.setTexture(mapTexture_.getTexture(), true);
    mapCacheValid_ = true;
}

void SfmlRenderer::drawWorldMap(World& world) {
    if (world.consumeMapDirty() || !mapCacheValid_) {
        rebuildMapCache(world);
    }
    target_.draw(mapSprite_);
}

void SfmlRenderer::drawIntersections(const World& world) {
    const float ts = world.getTileSize();
    for (const auto& inter : world.getIntersections()) {

        if (inter.getType() == RegulationType::TRAFFIC_LIGHT) {
            for (const auto& app : inter.getApproaches()) {
                sf::CircleShape light(6.f);
                light.setOrigin(6.f, 6.f);
                light.setOutlineThickness(1.f);
                light.setOutlineColor(sf::Color::Black);

                float px = app.entryTile.x * ts + ts / 2.f;
                float py = app.entryTile.y * ts + ts / 2.f;

                switch (app.direction) {
                    case Approach::Direction::NORTH: px += ts * 0.35f; break;
                    case Approach::Direction::SOUTH: px -= ts * 0.35f; break;
                    case Approach::Direction::EAST:  py += ts * 0.35f; break;
                    case Approach::Direction::WEST:  py -= ts * 0.35f; break;
                }
                light.setPosition(px, py);

                switch (inter.getLightState(app.direction)) {
                    case LightState::GREEN:  light.setFillColor(sf::Color::Green); break;
                    case LightState::ORANGE: light.setFillColor(sf::Color(255, 165, 0)); break;
                    case LightState::RED:    light.setFillColor(sf::Color::Red); break;
                }
                target_.draw(light);
            }
        }

        if (inter.getType() == RegulationType::PRIORITY_RIGHT) {
            const auto& tiles = inter.getCoveredTiles();
            if (tiles.empty()) continue;
            sf::Vector2f center(0.f, 0.f);
            for (const auto& tile : tiles) {
                center.x += tile.x * ts + ts / 2.f;
                center.y += tile.y * ts + ts / 2.f;
            }
            center.x /= static_cast<float>(tiles.size());
            center.y /= static_cast<float>(tiles.size());

            sf::CircleShape diamond(8.f, 4);
            diamond.setOrigin(8.f, 8.f);
            diamond.setPosition(center);
            diamond.setFillColor(sf::Color(255, 204, 0));
            diamond.setRotation(45.f);
            target_.draw(diamond);
        }
    }
}

void SfmlRenderer::drawFlowDebug(const World& world, float time) {
    const float speed        = 80.f;
    const float length       = 180.f;
    const float pulseSpacing = 350.f;

    const sf::Color coreColor(255, 230, 150);
    const sf::Color tailColor(200,  20,   0);

    const float maxThickness = 2.5f;
    const float minThickness = 0.2f;

    const float ts = world.getTileSize();
    const int   gw = world.getGridWidth();
    const int   gh = world.getGridHeight();

    sf::VertexArray trail(sf::Quads);

    for (int x = 0; x < gw; ++x) {
        for (int y = 0; y < gh; ++y) {
            const Tile& tile = world.getTile(x, y);
            if (tile.roadType == RoadType::NONE || tile.roadType == RoadType::INTERSECTION) continue;

            float globalPhase = 0.f;
            if      (tile.direction == TileDirection::RIGHT) globalPhase = time * speed - x * ts;
            else if (tile.direction == TileDirection::LEFT)  globalPhase = time * speed + x * ts;
            else if (tile.direction == TileDirection::DOWN)  globalPhase = time * speed - y * ts;
            else if (tile.direction == TileDirection::UP)    globalPhase = time * speed + y * ts;

            float H = std::fmod(globalPhase, pulseSpacing);
            if (H < 0) H += pulseSpacing;

            const float startClip = std::max(0.f, H - length);
            const float endClip   = std::min(ts, H);

            if (startClip < endClip) {
                const float prog1 = (H - startClip) / length;
                const float prog2 = (H - endClip)   / length;
                const float inv1  = 1.f - prog1;
                const float inv2  = 1.f - prog2;

                const sf::Uint8 a1 = static_cast<sf::Uint8>(255 * std::pow(inv1, 2.0f));
                const sf::Uint8 a2 = static_cast<sf::Uint8>(255 * std::pow(inv2, 2.0f));

                const sf::Color c1(
                    static_cast<sf::Uint8>(coreColor.r * inv1 + tailColor.r * prog1),
                    static_cast<sf::Uint8>(coreColor.g * inv1 + tailColor.g * prog1),
                    static_cast<sf::Uint8>(coreColor.b * inv1 + tailColor.b * prog1),
                    a1);
                const sf::Color c2(
                    static_cast<sf::Uint8>(coreColor.r * inv2 + tailColor.r * prog2),
                    static_cast<sf::Uint8>(coreColor.g * inv2 + tailColor.g * prog2),
                    static_cast<sf::Uint8>(coreColor.b * inv2 + tailColor.b * prog2),
                    a2);

                const float thick1 = minThickness + (maxThickness - minThickness) * inv1;
                const float thick2 = minThickness + (maxThickness - minThickness) * inv2;

                sf::Vector2f pos1, pos2, norm1, norm2;
                if (tile.direction == TileDirection::RIGHT) {
                    pos1 = {x * ts + startClip,     y * ts + ts / 2.f};
                    pos2 = {x * ts + endClip,       y * ts + ts / 2.f};
                    norm1 = {0.f, thick1};
                    norm2 = {0.f, thick2};
                } else if (tile.direction == TileDirection::LEFT) {
                    pos1 = {(x + 1) * ts - startClip, y * ts + ts / 2.f};
                    pos2 = {(x + 1) * ts - endClip,   y * ts + ts / 2.f};
                    norm1 = {0.f, thick1};
                    norm2 = {0.f, thick2};
                } else if (tile.direction == TileDirection::DOWN) {
                    pos1 = {x * ts + ts / 2.f, y * ts + startClip};
                    pos2 = {x * ts + ts / 2.f, y * ts + endClip};
                    norm1 = {thick1, 0.f};
                    norm2 = {thick2, 0.f};
                } else if (tile.direction == TileDirection::UP) {
                    pos1 = {x * ts + ts / 2.f, (y + 1) * ts - startClip};
                    pos2 = {x * ts + ts / 2.f, (y + 1) * ts - endClip};
                    norm1 = {thick1, 0.f};
                    norm2 = {thick2, 0.f};
                }

                trail.append(sf::Vertex(pos1 - norm1, c1));
                trail.append(sf::Vertex(pos1 + norm1, c1));
                trail.append(sf::Vertex(pos2 + norm2, c2));
                trail.append(sf::Vertex(pos2 - norm2, c2));
            }
        }
    }

    target_.draw(trail, sf::BlendAdd);
}

void SfmlRenderer::drawAgent(const IAgent& agent) {
    const core::Vec2  size = agent.getBodySize();
    const core::Color col  = agent.getBodyColor();

    sf::RectangleShape body(sf::Vector2f(size.x, size.y));
    body.setOrigin(size.x / 2.f, size.y / 2.f);
    body.setFillColor(toSfml(col));
    body.setPosition(toSfml(agent.getPosition()));
    body.setRotation(agent.getHeading());
    target_.draw(body);
}

void SfmlRenderer::drawAgentDebug(const IAgent& agent) {
    const AgentDebugSnapshot snap = agent.getDebugSnapshot();
    if (!snap.selected || !snap.active) return;

    const sf::Vector2f position   = toSfml(agent.getPosition());
    const float        headingDeg = agent.getHeading();

    sf::Color coneColor;
    if (snap.hasDirectObstacle && snap.directObstacleDistance < 50.f) {
        coneColor = sf::Color(255, 0, 0, 50);
    } else if (snap.hasDirectObstacle) {
        coneColor = sf::Color(255, 165, 0, 50);
    } else if (snap.approachingIntersection) {
        coneColor = sf::Color(255, 255, 0, 50);
    } else {
        coneColor = sf::Color(0, 255, 0, 50);
    }

    const int segments = 20;

    // Cone large
    sf::VertexArray cone(sf::TriangleFan, segments + 2);
    cone[0].position = position;
    cone[0].color    = coneColor;
    {
        const float startAngle = (headingDeg - snap.visionHalfAngleDeg) * DEG2RAD;
        const float endAngle   = (headingDeg + snap.visionHalfAngleDeg) * DEG2RAD;
        for (int i = 0; i <= segments; ++i) {
            const float t = static_cast<float>(i) / segments;
            const float a = startAngle + t * (endAngle - startAngle);
            cone[i + 1].position = position + sf::Vector2f(std::cos(a) * snap.visionRange,
                                                           std::sin(a) * snap.visionRange);
            cone[i + 1].color = coneColor;
        }
    }
    target_.draw(cone);

    // Cone etroit
    sf::Color directColor = coneColor;
    directColor.a = 80;
    sf::VertexArray directCone(sf::TriangleFan, segments + 2);
    directCone[0].position = position;
    directCone[0].color    = directColor;
    {
        const float dStart = (headingDeg - snap.visionDirectHalfAngleDeg) * DEG2RAD;
        const float dEnd   = (headingDeg + snap.visionDirectHalfAngleDeg) * DEG2RAD;
        for (int i = 0; i <= segments; ++i) {
            const float t = static_cast<float>(i) / segments;
            const float a = dStart + t * (dEnd - dStart);
            directCone[i + 1].position = position + sf::Vector2f(std::cos(a) * snap.visionRange,
                                                                 std::sin(a) * snap.visionRange);
            directCone[i + 1].color = directColor;
        }
    }
    target_.draw(directCone);

    // Marqueurs des objets detectes
    for (std::size_t i = 0; i < snap.detectionPositions.size(); ++i) {
        sf::CircleShape marker(5.f);
        marker.setOrigin(5.f, 5.f);
        marker.setPosition(toSfml(snap.detectionPositions[i]));
        if (snap.detectionDistances[i] < 50.f) marker.setFillColor(sf::Color::Red);
        else                                   marker.setFillColor(sf::Color(255, 165, 0));
        target_.draw(marker);
    }

    // Trajectoire courante
    if (!snap.pathPoints.empty()) {
        sf::VertexArray pathLine(sf::LineStrip, snap.pathPoints.size());
        for (std::size_t i = 0; i < snap.pathPoints.size(); ++i) {
            pathLine[i].position = toSfml(snap.pathPoints[i]);
            pathLine[i].color    = sf::Color(147, 112, 219, 220);
        }
        target_.draw(pathLine);
    }
}

} // namespace render
