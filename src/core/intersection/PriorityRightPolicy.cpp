// src/core/intersection/PriorityRightPolicy.cpp
#include "core/intersection/PriorityRightPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"
#include "core/math/Vec2.hpp"

namespace core::intersection {

namespace {

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

// La direction "a ma droite" depuis MA BRANCHE D'ORIGINE.
// Convention ecran : x+ = EST, y+ = SUD. Un vehicule arrivant DEPUIS le NORD
// roule vers le SUD : son cote droit pointe vers l'OUEST (rotation +90deg du
// heading en coordonnees ecran). Idem pour les autres approches.
Approach::Direction rightOf(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::NORTH: return Approach::Direction::WEST;
        case Approach::Direction::SOUTH: return Approach::Direction::EAST;
        case Approach::Direction::EAST:  return Approach::Direction::NORTH;
        case Approach::Direction::WEST:  return Approach::Direction::SOUTH;
    }
    return Approach::Direction::NORTH;
}

// Heading attendu (degres) pour un vehicule arrivant DEPUIS cette direction.
// Convention : axe x positif = EST, axe y positif = SUD (ecran).
float expectedHeadingFor(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::WEST:  return 0.f;    // roule vers l'EST
        case Approach::Direction::EAST:  return 180.f;  // roule vers l'OUEST
        case Approach::Direction::NORTH: return 90.f;   // roule vers le SUD
        case Approach::Direction::SOUTH: return 270.f;  // roule vers le NORD
    }
    return 0.f;
}

} // namespace

Decision PriorityRightPolicy::request(const PolicyContext& ctx,
                                      const Intersection& inter) const
{
    Decision d;
    if (!ctx.others) {
        d.canEnter = true; // sans observation -> on tente le passage
        return d;
    }

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float bufferToLine = 25.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - bufferToLine);

    const Approach::Direction rightDir = rightOf(ctx.self.from);
    const float expectedH = expectedHeadingFor(rightDir);

    // === Fenetre de conflit pour MOI ===
    // [tEnterMe, tClearMe] = intervalle pendant lequel MA carrosserie occupera
    // la zone de conflit.
    const float effSpeed   = std::max(ctx.self.speed, 30.f);
    const float tEnterMe   = stopLineGap / effSpeed;                  // temps pour atteindre la ligne
    const float tCross     = std::max(params_.minCrossingTime,
                                      params_.crossingDistance / effSpeed);
    const float tClearMe   = tEnterMe + tCross + params_.safetyMargin;

    bool  anyInside     = false;
    float worstTArrive  = std::numeric_limits<float>::infinity();
    bool  conflictFound = false;

    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue; // self-skip obligatoire
        const Vec2 oPos{ other->getPosition().x, other->getPosition().y };
        const float distToInter = (oPos - center).length();
        if (distToInter > params_.scanRadius) continue;

        // Heading-match : l'agent vient-il bien de notre droite ?
        const float oHeading = other->getHeading();
        float       hDiff    = math::wrapDeg180(oHeading - expectedH);
        if (std::abs(hDiff) > params_.headingTolerance) continue;

        // Agent deja INTRA-intersection ?
        bool insideInter = false;
        for (const auto& t : inter.getCoveredTiles()) {
            const Vec2 tCenter{ t.x * ctx.tileSize + ctx.tileSize / 2.f,
                                t.y * ctx.tileSize + ctx.tileSize / 2.f };
            if ((oPos - tCenter).length() < params_.insideRadius) {
                insideInter = true;
                break;
            }
        }
        if (insideInter) {
            anyInside = true;
            worstTArrive = 0.f; // bloque immediat
            conflictFound = true;
            break;
        }

        // Loin et arrete : pas une menace immediate.
        const float oSpeed = other->getSpeed();
        if (distToInter > params_.ignoreDistance && oSpeed < 5.f) continue;

        // Verifie qu'il se dirige bien vers l'intersection (dot product).
        const float oRad = oHeading * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        Vec2 toInter = center - oPos;
        const float lenToInter = toInter.length();
        if (lenToInter > 1.f) {
            toInter /= lenToInter;
            if (dot(oDir, toInter) <= 0.2f) continue; // pas dirige vers nous
        }

        // === Fenetre de conflit pour LUI ===
        // tEnterOther    : temps qu'il met pour atteindre la zone de conflit
        // tClearOther    : temps qu'il met pour la liberer
        const float oEffSpeed     = std::max(oSpeed, 1.f);
        const float distToBoundary = std::max(0.f, distToInter - interHalf);
        const float tEnterOther   = distToBoundary / oEffSpeed;
        const float oCross        = std::max(params_.minCrossingTime,
                                             params_.crossingDistance / oEffSpeed);
        const float tClearOther   = tEnterOther + oCross;

        // Recouvrement temporel [tEnterMe, tClearMe] ∩ [tEnterOther, tClearOther].
        const float overlapStart = std::max(tEnterMe, tEnterOther);
        const float overlapEnd   = std::min(tClearMe, tClearOther);
        if (overlapEnd > overlapStart) {
            conflictFound = true;
            worstTArrive = std::min(worstTArrive, tEnterOther);
        }
    }

    if (anyInside || conflictFound) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
        d.yieldUntilT = std::isfinite(worstTArrive) ? worstTArrive : 0.f;
    } else {
        d.canEnter   = true;
        d.shouldStop = false;
    }
    return d;
}

} // namespace core::intersection
