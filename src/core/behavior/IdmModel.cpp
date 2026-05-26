// src/core/behavior/IdmModel.cpp
#include "core/behavior/IdmModel.hpp"

#include <algorithm>
#include <cmath>

namespace core::behavior {

float IdmModel::computeAcceleration(float selfSpeed,
                                     float desiredSpeed,
                                     const LeaderInfo& leader) const
{
    const float v  = std::max(0.f, selfSpeed);
    const float v0 = std::max(1.0f, desiredSpeed); // evite division par 0

    // Terme "free-flow" : tendance a converger vers v0.
    const float freeRatio = v / v0;
    const float freeTerm  = std::pow(freeRatio, p_.delta);

    if (!leader.present) {
        return p_.aMax * (1.f - freeTerm);
    }

    // Terme "interaction" : freinage en fonction du leader.
    const float dv     = v - leader.speed;

    // Time-headway : on veut rester v*T metres derriere un MOBILE. Pour un point
    // d'arret FIXE (stopTarget) ce terme est non physique -- on s'arrete pile a
    // la ligne -- et le conserver fait freiner beaucoup trop tot (sStar gonfle de
    // v*T) puis fluer jusqu'a la ligne. On ne garde alors que le terme cinematique
    // (distance de freinage confortable) + s0.
    const float headway = leader.stopTarget ? 0.f : (v * p_.T);
    const float sStar0  = headway + (v * dv) / (2.f * std::sqrt(p_.aMax * p_.bComf));
    const float sStar   = p_.s0 + std::max(0.f, sStar0);

    // gap minimal pour eviter division par 0 (collision imminente -> freinage maximal)
    const float s        = std::max(0.5f, leader.gap);
    const float intRatio = sStar / s;
    const float intTerm  = intRatio * intRatio;

    return p_.aMax * (1.f - freeTerm - intTerm);
}

} // namespace core::behavior
