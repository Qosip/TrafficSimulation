// src/core/behavior/IdmModel.hpp
//
// Implementation du modele Intelligent Driver Model (Treiber, Hennecke, Helbing 2000).
//
//   a = a_max * [ 1 - (v / v0)^delta - (s* / s)^2 ]
//   s* = s0 + max(0, v*T + v*Δv / (2 * sqrt(a_max * b)))
//
// Reference: Treiber et al., "Congested traffic states in empirical observations
// and microscopic simulations", Phys. Rev. E 62, 2000.
#pragma once

#include "core/behavior/ICarFollowingModel.hpp"

namespace core::behavior {

struct IdmParams {
    float T     = 1.5f;   // safe time headway (s)
    float s0    = 5.f;    // minimum gap (px)
    float aMax  = 150.f;  // max acceleration (px/s^2)
    float bComf = 80.f;   // comfortable deceleration (px/s^2)
    float delta = 4.f;    // acceleration exponent
};

class IdmModel : public ICarFollowingModel {
public:
    explicit IdmModel(IdmParams p = {}) : p_(p) {}

    void setParams(const IdmParams& p) { p_ = p; }
    const IdmParams& params() const    { return p_; }

    float computeAcceleration(float selfSpeed,
                               float desiredSpeed,
                               const LeaderInfo& leader) const override;

private:
    IdmParams p_;
};

} // namespace core::behavior
