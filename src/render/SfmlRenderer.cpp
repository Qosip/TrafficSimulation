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

        // Calcul centre commun pour les panneaux centres.
        const auto& tiles = inter.getCoveredTiles();
        if (tiles.empty()) continue;
        sf::Vector2f center(0.f, 0.f);
        for (const auto& tile : tiles) {
            center.x += tile.x * ts + ts / 2.f;
            center.y += tile.y * ts + ts / 2.f;
        }
        center.x /= static_cast<float>(tiles.size());
        center.y /= static_cast<float>(tiles.size());

        switch (inter.getType()) {
            case RegulationType::TRAFFIC_LIGHT: {
                for (const auto& app : inter.getApproaches()) {
                    // Boitier noir 3 LEDs verticales (R/O/V) + LED active en grand.
                    float px = app.entryTile.x * ts + ts / 2.f;
                    float py = app.entryTile.y * ts + ts / 2.f;
                    switch (app.direction) {
                        case Approach::Direction::NORTH: px += ts * 0.35f; break;
                        case Approach::Direction::SOUTH: px -= ts * 0.35f; break;
                        case Approach::Direction::EAST:  py += ts * 0.35f; break;
                        case Approach::Direction::WEST:  py -= ts * 0.35f; break;
                    }
                    sf::RectangleShape box(sf::Vector2f(8.f, 20.f));
                    box.setOrigin(4.f, 10.f);
                    box.setPosition(px, py);
                    box.setFillColor(sf::Color(20, 20, 20, 230));
                    box.setOutlineColor(sf::Color(0, 0, 0));
                    box.setOutlineThickness(1.f);
                    target_.draw(box);

                    const LightState st = inter.getLightState(app.direction);
                    auto drawLed = [&](float oy, sf::Color on, sf::Color off, bool active) {
                        sf::CircleShape led(2.5f);
                        led.setOrigin(2.5f, 2.5f);
                        led.setPosition(px, py + oy);
                        led.setFillColor(active ? on : off);
                        target_.draw(led);
                    };
                    drawLed(-6.f, sf::Color::Red,           sf::Color(60,20,20), st == LightState::RED);
                    drawLed( 0.f, sf::Color(255, 165, 0),   sf::Color(60,40,20), st == LightState::ORANGE);
                    drawLed( 6.f, sf::Color::Green,         sf::Color(20,60,20), st == LightState::GREEN);
                }
                break;
            }

            case RegulationType::PRIORITY_RIGHT: {
                // Losange jaune (panneau "Vous avez la priorite").
                sf::CircleShape diamond(10.f, 4);
                diamond.setOrigin(10.f, 10.f);
                diamond.setPosition(center);
                diamond.setFillColor(sf::Color(255, 204, 0));
                diamond.setOutlineColor(sf::Color::Black);
                diamond.setOutlineThickness(1.f);
                diamond.setRotation(45.f);
                target_.draw(diamond);
                break;
            }

            case RegulationType::STOP: {
                // STOP 2 voies : le panneau octogonal rouge n'est pose QUE sur
                // les branches SECONDAIRES (celles qui doivent ceder), pas sur
                // l'axe principal prioritaire. On le place a l'entree de chaque
                // approche concernee (comme un vrai panneau au bord de la voie).
                const bool majorH = inter.isStopMajorAxisHorizontal();
                for (const auto& app : inter.getApproaches()) {
                    const bool appHoriz = (app.direction == Approach::Direction::EAST ||
                                           app.direction == Approach::Direction::WEST);
                    if (appHoriz == majorH) continue;   // axe principal -> pas de STOP

                    float px = app.entryTile.x * ts + ts / 2.f;
                    float py = app.entryTile.y * ts + ts / 2.f;
                    switch (app.direction) {
                        case Approach::Direction::NORTH: px += ts * 0.35f; break;
                        case Approach::Direction::SOUTH: px -= ts * 0.35f; break;
                        case Approach::Direction::EAST:  py += ts * 0.35f; break;
                        case Approach::Direction::WEST:  py -= ts * 0.35f; break;
                    }

                    sf::CircleShape octa(11.f, 8);
                    octa.setOrigin(11.f, 11.f);
                    octa.setPosition(px, py);
                    octa.setFillColor(sf::Color(200, 30, 30));
                    octa.setOutlineColor(sf::Color::White);
                    octa.setOutlineThickness(2.f);
                    octa.setRotation(22.5f);
                    target_.draw(octa);

                    sf::RectangleShape bar(sf::Vector2f(9.f, 2.f));
                    bar.setOrigin(4.5f, 1.f);
                    bar.setPosition(px, py);
                    bar.setFillColor(sf::Color::White);
                    target_.draw(bar);
                }
                break;
            }

            case RegulationType::ROUNDABOUT: {
                // Taille dynamique : on derive le rayon des coveredTiles.
                // BBOX -> outerRadius = max(largeur, hauteur) / 2.
                float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
                for (const auto& t : tiles) {
                    const float cx = t.x * ts + ts / 2.f;
                    const float cy = t.y * ts + ts / 2.f;
                    if (cx < minX) minX = cx;
                    if (cy < minY) minY = cy;
                    if (cx > maxX) maxX = cx;
                    if (cy > maxY) maxY = cy;
                }
                const float bw = (maxX - minX) + ts;
                const float bh = (maxY - minY) + ts;
                const float outerR = std::max(bw, bh) / 2.f;
                // Interior : pelouse (couleur fond) si rond-point >= 4x4.
                const bool  isLarge = (tiles.size() > 4);
                const float innerR  = isLarge ? std::max(0.f, outerR - ts) : 0.f;

                // Cercle externe = bordure visible.
                sf::CircleShape outer(outerR);
                outer.setOrigin(outerR, outerR);
                outer.setPosition(center);
                outer.setFillColor(sf::Color(0, 0, 0, 0));    // transparent : laisse voir la chaussee
                outer.setOutlineColor(sf::Color(220, 220, 220));
                outer.setOutlineThickness(2.f);
                target_.draw(outer);

                if (innerR > 4.f) {
                    sf::CircleShape ilot(innerR);
                    ilot.setOrigin(innerR, innerR);
                    ilot.setPosition(center);
                    ilot.setFillColor(sf::Color(38, 77, 44));   // pelouse
                    ilot.setOutlineColor(sf::Color(220, 220, 220));
                    ilot.setOutlineThickness(2.f);
                    target_.draw(ilot);
                } else {
                    // Rond-point compact : petit marqueur central.
                    sf::CircleShape dot(6.f);
                    dot.setOrigin(6.f, 6.f);
                    dot.setPosition(center);
                    dot.setFillColor(sf::Color(38, 77, 44));
                    target_.draw(dot);
                }

                const core::Vec2 Cc = inter.getWorldCenter(ts);
                const sf::Vector2f Cv(Cc.x, Cc.y);

                // Marquage d'entree/sortie INTERACTIF : de simples liseres blancs
                // (pas de fleche) n'apparaissent qu'aux approches reellement
                // reliees a une route. Effacer la route -> le marquage disparait.
                for (const auto& app : inter.getApproaches()) {
                    const Tile& at = world.getTile(app.entryTile.x, app.entryTile.y);
                    if (at.roadType == RoadType::NONE) continue;  // pas de route -> rien

                    const sf::Vector2f ec(app.entryTile.x * ts + ts / 2.f,
                                          app.entryTile.y * ts + ts / 2.f);
                    sf::Vector2f d(ec.x - Cv.x, ec.y - Cv.y);
                    const float dl = std::sqrt(d.x * d.x + d.y * d.y);
                    if (dl < 1e-3f) continue;
                    d.x /= dl; d.y /= dl;

                    const sf::Vector2f tang(-d.y, d.x);
                    for (int sgn = -1; sgn <= 1; sgn += 2) {
                        sf::RectangleShape edge(sf::Vector2f(14.f, 3.f));
                        edge.setOrigin(7.f, 1.5f);
                        edge.setPosition(Cv.x + d.x * outerR + tang.x * sgn * (ts * 0.45f),
                                         Cv.y + d.y * outerR + tang.y * sgn * (ts * 0.45f));
                        edge.setFillColor(sf::Color(230, 230, 230, 200));
                        edge.setRotation(std::atan2(d.y, d.x) * core::math::RAD2DEG);
                        target_.draw(edge);
                    }
                }
                break;
            }

            case RegulationType::YIELD: {
                // Triangle blanc pointe vers le bas avec contour rouge.
                sf::CircleShape tri(12.f, 3);
                tri.setOrigin(12.f, 12.f);
                tri.setPosition(center);
                tri.setFillColor(sf::Color::White);
                tri.setOutlineColor(sf::Color(200, 30, 30));
                tri.setOutlineThickness(2.f);
                tri.setRotation(180.f);
                target_.draw(tri);
                break;
            }

            case RegulationType::FIXED_PRIORITY: {
                // Bande bleue le long de l'axe PRINCIPAL (route prioritaire),
                // + losange jaune central (panneau "priorite").
                const bool majorH = inter.isStopMajorAxisHorizontal();
                sf::RectangleShape band(majorH ? sf::Vector2f(ts * 1.8f, 5.f)
                                               : sf::Vector2f(5.f, ts * 1.8f));
                band.setOrigin(band.getSize().x / 2.f, band.getSize().y / 2.f);
                band.setPosition(center);
                band.setFillColor(sf::Color(40, 120, 255, 200));
                target_.draw(band);

                sf::CircleShape diamond(9.f, 4);
                diamond.setOrigin(9.f, 9.f);
                diamond.setPosition(center);
                diamond.setFillColor(sf::Color(255, 204, 0));
                diamond.setOutlineColor(sf::Color::Black);
                diamond.setOutlineThickness(1.f);
                diamond.setRotation(45.f);
                target_.draw(diamond);
                break;
            }

            case RegulationType::P2P: {
                // Antennes V2V : arcs concentriques evoquant le broadcast VANET.
                for (int k = 1; k <= 3; ++k) {
                    sf::CircleShape ring(4.f + k * 5.f);
                    ring.setOrigin(ring.getRadius(), ring.getRadius());
                    ring.setPosition(center);
                    ring.setFillColor(sf::Color(0, 0, 0, 0));
                    ring.setOutlineColor(sf::Color(60, 220, 180,
                                                   static_cast<sf::Uint8>(220 - k * 40)));
                    ring.setOutlineThickness(2.f);
                    target_.draw(ring);
                }
                sf::CircleShape dot(3.f);
                dot.setOrigin(3.f, 3.f);
                dot.setPosition(center);
                dot.setFillColor(sf::Color(60, 220, 180));
                target_.draw(dot);
                break;
            }
        }
    }
}

void SfmlRenderer::drawBuildFootprint(int gridX, int gridY, int wTiles, int hTiles,
                                      float tileSize, core::Color fill) {
    sf::RectangleShape r(sf::Vector2f(wTiles * tileSize, hTiles * tileSize));
    r.setPosition(gridX * tileSize, gridY * tileSize);
    r.setFillColor(toSfml(fill));
    r.setOutlineColor(sf::Color(255, 255, 255, 200));
    r.setOutlineThickness(2.f);
    target_.draw(r);
}

void SfmlRenderer::drawHoverHighlight(int gridX, int gridY, float tileSize) {
    sf::RectangleShape r(sf::Vector2f(tileSize, tileSize));
    r.setPosition(gridX * tileSize, gridY * tileSize);
    r.setFillColor(sf::Color(255, 255, 255, 28));
    r.setOutlineColor(sf::Color(255, 220, 0, 220));
    r.setOutlineThickness(2.f);
    target_.draw(r);
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
