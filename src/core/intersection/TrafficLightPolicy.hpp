// src/core/intersection/TrafficLightPolicy.hpp
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"

namespace core::intersection {

// Policy de carrefour a feux tricolores.
// - GREEN  -> canEnter = true
// - ORANGE -> shouldStop si on a la distance de freinage suffisante, sinon
//             on engage le passage (eviterait un freinage d'urgence dangereux).
// - RED    -> shouldStop sur la ligne d'arret
struct TrafficLightParams {
    float comfortDecel = 80.f;   // px/s^2  -- aligne sur IdmParams::bComf typique
    float buffer       = 25.f;   // px      -- marge avant la ligne d'arret
};

class TrafficLightPolicy : public IIntersectionPolicy {
public:
    explicit TrafficLightPolicy(TrafficLightParams params = {}) : params_(params) {}
    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    TrafficLightParams params_;
};

} // namespace core::intersection
