// src/core/intersection/StopPolicy.hpp
//
// Policy "STOP" : l'agent doit marquer un arret complet a la ligne d'arret,
// puis appliquer une gap-acceptance comme une priorite-a-droite mais avec
// observation 360 (tous les autres ont la priorite).
//
// L'etat "arret marque" vit dans le Vehicle (stopHeldTime + stopReleased)
// car il est par-agent et par-intersection.
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"

namespace core::intersection {

struct StopParams {
    GapAcceptanceParams gap;        // re-use des fenetres de conflit
    float requiredHaltTime = 0.8f;  // secondes d'arret complet exigees
    float haltSpeedEpsilon = 5.f;   // px/s en-dessous = "arrete"
};

class StopPolicy : public IIntersectionPolicy {
public:
    explicit StopPolicy(StopParams params = {}) : params_(params) {}
    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    StopParams params_;
};

} // namespace core::intersection
