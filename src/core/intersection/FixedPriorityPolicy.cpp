// src/core/intersection/FixedPriorityPolicy.cpp
#include "core/intersection/FixedPriorityPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/agent/TurnIntent.hpp"
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

// Un cap (degres) est-il "horizontal" (E-O) plutot que "vertical" (N-S) ?
bool headingIsHorizontal(float headingDeg) {
    const float r = headingDeg * math::DEG2RAD;
    return std::abs(std::cos(r)) >= std::abs(std::sin(r));
}

bool fromIsHorizontal(Approach::Direction from) {
    return from == Approach::Direction::EAST || from == Approach::Direction::WEST;
}

} // namespace

Decision FixedPriorityPolicy::request(const PolicyContext& ctx,
                                      const Intersection& inter) const
{
    Decision d;

    const bool majorHorizontal = inter.isStopMajorAxisHorizontal();
    const bool selfOnMajorAxis = (fromIsHorizontal(ctx.self.from) == majorHorizontal);

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float bufferToLine = 25.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - bufferToLine);

    // Axe principal : prioritaire, sauf tourne-a-gauche non protege face a un
    // prioritaire venant d'en face tout droit/a droite.
    if (selfOnMajorAxis) {
        const bool turningLeft =
            ctx.selfAgent &&
            ctx.selfAgent->getTurnIntent() == core::agent::TurnIntent::LEFT;
        if (turningLeft && ctx.others) {
            const float myEff   = std::max(ctx.self.speed, 30.f);
            const float myEnter = stopLineGap / myEff;
            for (const auto& other : *ctx.others) {
                if (!other || other.get() == ctx.selfAgent) continue;
                if (headingIsHorizontal(other->getHeading()) != majorHorizontal) continue;
                if (other->getTurnIntent() == core::agent::TurnIntent::LEFT) continue;

                const float dH =
                    std::abs(math::wrapDeg180(other->getHeading() - ctx.self.heading));
                if (dH < 135.f) continue;

                const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
                const float oDist = (oPos - center).length();
                if (oDist > params_.scanRadius) continue;

                const float oRad = other->getHeading() * math::DEG2RAD;
                const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
                Vec2 toInter = center - oPos;
                const float lenToInter = toInter.length();
                if (lenToInter > 1.f) {
                    toInter /= lenToInter;
                    if (dot(oDir, toInter) <= 0.2f) continue;
                }

                const float oEffSpeed      = std::max(other->getSpeed(), 1.f);
                const float distToBoundary = std::max(0.f, oDist - interHalf);
                const float tEnterOther    = distToBoundary / oEffSpeed;
                if (tEnterOther <= myEnter + params_.minCrossingTime + params_.safetyMargin) {
                    d.canEnter    = false;
                    d.shouldStop  = true;
                    d.stopLineGap = stopLineGap;
                    d.yieldUntilT = tEnterOther;
                    return d;
                }
            }
        }

        d.canEnter   = true;
        d.shouldStop = false;
        return d;
    }

    // Axe secondaire : sans observation possible, on tente (degrade prudent).
    if (!ctx.others) { d.canEnter = true; return d; }

    // Fenetre temporelle pendant laquelle MA carrosserie occupera le conflit.
    const float effSpeed = std::max(ctx.self.speed, 30.f);
    const float tEnterMe = stopLineGap / effSpeed;
    const float tCross   = std::max(params_.minCrossingTime,
                                    params_.crossingDistance / effSpeed);
    const float tClearMe = tEnterMe + tCross + params_.safetyMargin;

    bool  conflictFound = false;
    float worstTArrive  = std::numeric_limits<float>::infinity();

    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;          // self-skip obligatoire

        // Ne cede qu'aux vehicules de l'AXE PRINCIPAL.
        if (headingIsHorizontal(other->getHeading()) != majorHorizontal) continue;

        const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
        const float distToInter = (oPos - center).length();
        if (distToInter > params_.scanRadius) continue;

        // Vehicule deja DANS l'intersection -> blocage immediat.
        const bool insideInter = inter.containsWorldPoint(oPos, ctx.tileSize);
        if (insideInter) { conflictFound = true; worstTArrive = 0.f; break; }

        const float oSpeed = other->getSpeed();
        if (distToInter > params_.ignoreDistance && oSpeed < 5.f) continue;

        // Se dirige-t-il vers l'intersection ?
        const float oRad = other->getHeading() * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        Vec2 toInter = center - oPos;
        const float lenToInter = toInter.length();
        if (lenToInter > 1.f) {
            toInter /= lenToInter;
            if (dot(oDir, toInter) <= 0.2f) continue;        // s'eloigne
        }

        // Fenetre de conflit du prioritaire.
        const float oEffSpeed      = std::max(oSpeed, 1.f);
        const float distToBoundary = std::max(0.f, distToInter - interHalf);
        const float tEnterOther    = distToBoundary / oEffSpeed;
        const float oCross         = std::max(params_.minCrossingTime,
                                              params_.crossingDistance / oEffSpeed);
        const float tClearOther    = tEnterOther + oCross;

        const float overlapStart = std::max(tEnterMe, tEnterOther);
        const float overlapEnd   = std::min(tClearMe, tClearOther);
        if (overlapEnd > overlapStart) {
            conflictFound = true;
            worstTArrive  = std::min(worstTArrive, tEnterOther);
        }
    }

    if (conflictFound) {
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
