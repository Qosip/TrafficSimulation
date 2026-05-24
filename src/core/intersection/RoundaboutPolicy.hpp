// src/core/intersection/RoundaboutPolicy.hpp
//
// Policy "Rond-point" : priorite TOTALE aux vehicules deja a l'interieur
// (regle francaise standard depuis 1984). L'agent entrant cede a tout
// vehicule dont la position se trouve dans le disque central.
//
// Pas de gestion fine de la direction de rotation : on considere tous les
// vehicules a l'interieur comme prioritaires.
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"

namespace core::intersection {

struct RoundaboutParams {
    float entryGuardRadius  = 80.f;   // px : rayon "approche" autour du centre
    float insideDetectRadius = 70.f;  // px : rayon "interieur" du rond-point
    float safetyMargin       = 0.8f;  // s  : marge sur le temps d'arrivee
    float minCrossingTime    = 1.5f;  // s  : temps mini pour traverser
};

class RoundaboutPolicy : public IIntersectionPolicy {
public:
    explicit RoundaboutPolicy(RoundaboutParams params = {}) : params_(params) {}
    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    RoundaboutParams params_;
};

} // namespace core::intersection
