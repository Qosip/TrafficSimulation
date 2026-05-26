// src/core/intersection/FixedPriorityPolicy.hpp
//
// Strategie de coordination "Priorite Fixe" (cf. rapport SMA, architecture 1).
//
// Un AXE est declare prioritaire de maniere statique (route principale) ;
// l'autre axe est subordonne (route secondaire). Les vehicules de l'axe
// principal ne s'arretent JAMAIS. Les vehicules de l'axe secondaire cedent
// le passage tant qu'un vehicule prioritaire occupe (ou va occuper) la zone
// de conflit -- exactement comme une perte de priorite envoyee a l'IDM sous
// forme d'obstacle virtuel a la ligne d'arret.
//
// L'axe principal est lu via Intersection::isStopMajorAxisHorizontal()
// (true = horizontal E-O principal). Cette strategie sert de reference
// "centralisee a priorite fixe" dans la comparaison de l'hypothese du rapport
// (P2P decentralise vs priorite fixe, point d'inflexion lie a la densite).
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"   // GapAcceptanceParams

namespace core::intersection {

class FixedPriorityPolicy : public IIntersectionPolicy {
public:
    explicit FixedPriorityPolicy(GapAcceptanceParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    GapAcceptanceParams params_;
};

} // namespace core::intersection
