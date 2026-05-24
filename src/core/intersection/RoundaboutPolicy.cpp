// src/core/intersection/RoundaboutPolicy.cpp
#include "core/intersection/RoundaboutPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Vec2.hpp"

namespace core::intersection {

Decision RoundaboutPolicy::request(const PolicyContext& ctx,
                                    const Intersection& inter) const
{
    Decision d;

    // Geometrie reelle de l'anneau (source unique partagee avec le rendu).
    const Vec2  C        = inter.getWorldCenter(ctx.tileSize);
    const float outerR   = inter.getOuterRadius(ctx.tileSize);
    const float distSelf = (C - ctx.self.position).length();

    // Deja en train de circuler dans l'anneau -> ne jamais freiner pour
    // "entrer" (le car-following IDM gere la file a l'interieur).
    const float insideThresh = outerR - ctx.tileSize * 0.4f;
    if (distSelf < insideThresh) { d.canEnter = true; return d; }

    const float buffer      = 18.f + ctx.self.length / 2.f;
    const float stopLineGap = std::max(0.f, distSelf - outerR - buffer);

    if (!ctx.others) { d.canEnter = true; return d; }

    // Secteur angulaire d'entree = direction d'ou j'arrive vers le centre.
    const float thMe = std::atan2(ctx.self.position.y - C.y,
                                  ctx.self.position.x - C.x);

    constexpr float TWO_PI   = 6.28318530718f;
    const float     yieldArc = 2.0f;   // rad (~115°) : portion AMONT surveillee

    // Regle francaise : priorite a l'anneau. On cede uniquement aux vehicules
    // deja a l'interieur situes en AMONT immediat de mon point d'entree (dans
    // le sens legal anti-horaire ecran = angle atan2 decroissant). Les
    // vehicules en aval (qui viennent de passer) ne nous concernent pas :
    // evite l'auto-blocage "je cede a tout l'anneau".
    bool conflict = false;
    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;
        const Vec2 oPos{ other->getPosition().x, other->getPosition().y };
        const float distO = (oPos - C).length();
        if (distO > outerR) continue;                       // hors anneau

        const float thO = std::atan2(oPos.y - C.y, oPos.x - C.x);
        // Amont = angle PLUS GRAND que le mien (le flux decroit vers moi).
        float gap = thO - thMe;
        while (gap < 0.f)     gap += TWO_PI;
        while (gap >= TWO_PI) gap -= TWO_PI;
        if (gap < yieldArc) { conflict = true; break; }
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
