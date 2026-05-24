// src/core/intersection/TrafficLightPolicy.cpp
#include "core/intersection/TrafficLightPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"  // (transitif pour pointeur)
#include "core/intersection/Intersection.hpp"
#include "core/math/Vec2.hpp"

namespace core::intersection {

namespace {

// Centre geometrique de l'intersection en coordonnees monde.
Vec2 computeCenter(const Intersection& inter, float tileSize) {
    const auto& tiles = inter.getCoveredTiles();
    if (tiles.empty()) return {0.f, 0.f};
    Vec2 c{0.f, 0.f};
    for (const auto& t : tiles) {
        c.x += t.x * tileSize + tileSize / 2.f;
        c.y += t.y * tileSize + tileSize / 2.f;
    }
    return c / static_cast<float>(tiles.size());
}

} // namespace

Decision TrafficLightPolicy::request(const PolicyContext& ctx,
                                     const Intersection& inter) const
{
    Decision d;
    const LightState state = inter.getLightState(ctx.self.from);

    const Vec2 center      = computeCenter(inter, ctx.tileSize);
    const Vec2 toCenter    = center - ctx.self.position;
    const float distCenter = toCenter.length();

    // Distance approximative de la "ligne d'arret" : bord proche de l'intersection.
    const float interHalf = ctx.tileSize;            // 2x2 tiles -> demi-cote = tileSize
    const float buffer    = params_.buffer + ctx.self.length / 2.f;
    const float stopGap   = std::max(0.f, distCenter - interHalf - buffer);

    if (state == LightState::GREEN) {
        d.canEnter   = true;
        d.shouldStop = false;
        return d;
    }

    if (state == LightState::ORANGE) {
        // Distance de freinage confortable a la vitesse courante : v^2 / (2 b).
        const float v          = std::max(0.f, ctx.self.speed);
        const float brakingDist = (v * v) / (2.f * std::max(1.f, params_.comfortDecel));
        // Si le freinage confortable ne tient pas dans le gap restant, on passe
        // (sinon arret d'urgence = blocage des suivants).
        if (brakingDist > stopGap) {
            d.canEnter   = true;
            d.shouldStop = false;
            return d;
        }
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopGap;
        return d;
    }

    // RED.
    d.canEnter    = false;
    d.shouldStop  = true;
    d.stopLineGap = stopGap;
    return d;
}

} // namespace core::intersection
