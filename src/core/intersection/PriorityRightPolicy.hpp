// src/core/intersection/PriorityRightPolicy.hpp
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"

namespace core::intersection {

// Policy "priorite a droite" implementee par Gap Acceptance :
// l'agent ne s'engage que si la fenetre temporelle de conflit avec
// les vehicules venant de sa droite est superieure a un seuil T_required.
//
// Parametres ajustables (futur : profile psychologique).
struct GapAcceptanceParams {
    float scanRadius        = 200.f;  // px : zone d'observation autour du centre
    float safetyMargin      = 1.5f;   // s  : marge de securite ajoutee au temps de traverse
    float minCrossingTime   = 2.0f;   // s  : temps minimum de traverse (cas vitesse faible)
    float crossingDistance  = 100.f;  // px : distance approximative traversee (2 tiles)
    float headingTolerance  = 60.f;   // degres : tolerance heading vs direction attendue
    float ignoreDistance    = 80.f;   // px : au-dela, on ignore les vehicules a l'arret
    float insideRadius      = 35.f;   // px : agent considere INTRA-intersection
};

class PriorityRightPolicy : public IIntersectionPolicy {
public:
    explicit PriorityRightPolicy(GapAcceptanceParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    GapAcceptanceParams params_;
};

} // namespace core::intersection
