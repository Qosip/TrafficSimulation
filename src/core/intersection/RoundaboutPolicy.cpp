// src/core/intersection/RoundaboutPolicy.cpp
#include "core/intersection/RoundaboutPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"
#include "core/math/Vec2.hpp"

namespace core::intersection {

Decision RoundaboutPolicy::request(const PolicyContext& ctx,
                                    const Intersection& inter) const
{
    Decision d;

    const Vec2  C        = inter.getWorldCenter(ctx.tileSize);
    const float outerR   = inter.getOuterRadius(ctx.tileSize);
    const float laneR    = std::max(inter.getLaneRadius(ctx.tileSize), 1.f);
    const float distSelf = (C - ctx.self.position).length();

    // Deja dans l'anneau : on degage. Le suivi de file reel gere les vehicules
    // devant moi, pas la regle d'entree.
    const float insideThresh = outerR - ctx.tileSize * 0.4f;
    if (distSelf < insideThresh) { d.canEnter = true; return d; }

    const float buffer      = 18.f + ctx.self.length / 2.f;
    const float stopLineGap = std::max(0.f, distSelf - outerR - buffer);

    if (!ctx.others) { d.canEnter = true; return d; }

    const float thMe = std::atan2(ctx.self.position.y - C.y,
                                  ctx.self.position.x - C.x);

    bool conflict = false;
    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;

        const Vec2  oPos  = other->getPosition();
        const float distO = (oPos - C).length();
        if (distO > outerR + other->getLength() * 0.25f) continue;

        // Un vehicule sur une branche adjacente ou dans une autre zone du grand
        // rond-point ne bloque pas mon insertion.
        const float radialSlack = ctx.tileSize * 0.75f + other->getBodySize().y;
        if (std::abs(distO - laneR) > radialSlack) continue;

        const float thO = std::atan2(oPos.y - C.y, oPos.x - C.x);
        float gap = thO - thMe;                  // amont dans le sens legal
        while (gap < 0.f)           gap += math::TWO_PI;
        while (gap >= math::TWO_PI) gap -= math::TWO_PI;

        const float arcDist = gap * laneR;
        const float closeDist = (ctx.self.length + other->getLength()) * 0.5f + 18.f;
        if (arcDist <= closeDist || gap <= params_.minYieldAngle) {
            conflict = true;
            break;
        }

        const float oRad = other->getHeading() * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        const Vec2  legalTangent{ std::sin(thO), -std::cos(thO) };
        const float alongRing =
            std::max(0.f, core::dot(oDir, legalTangent) * other->getSpeed());
        if (alongRing <= 5.f) continue;          // arrete loin de mon entree

        const float tToEntry = arcDist / alongRing;
        if (tToEntry <= params_.yieldTime + params_.safetyMargin) {
            conflict = true;
            break;
        }
    }

    if (conflict) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
    } else {
        d.canEnter = true;
    }
    return d;
}

} // namespace core::intersection
