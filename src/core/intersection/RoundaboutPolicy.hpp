// src/core/intersection/RoundaboutPolicy.hpp
//
// Policy "Rond-point" : priorite aux vehicules deja engages sur l'anneau,
// mais uniquement s'ils arrivent reellement sur mon point d'entree. Un vehicule
// loin sur une autre portion de l'anneau ne doit pas bloquer mon insertion.
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"

namespace core::intersection {

struct RoundaboutParams {
    float entryGuardRadius  = 80.f;   // px : rayon "approche" autour du centre
    float insideDetectRadius = 70.f;  // px : rayon "interieur" du rond-point
    float safetyMargin       = 0.8f;  // s  : marge sur le temps d'arrivee
    float minCrossingTime    = 1.5f;  // s  : temps mini pour traverser
    float yieldTime          = 1.8f;  // s  : temps max avant qu'un prioritaire arrive a mon entree
    float minYieldAngle      = 0.35f; // rad: garde proche meme a vitesse faible
};

class RoundaboutPolicy : public IIntersectionPolicy {
public:
    explicit RoundaboutPolicy(RoundaboutParams params = {}) : params_(params) {}
    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    RoundaboutParams params_;
};

} // namespace core::intersection
