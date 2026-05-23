// tests/test_idm.cpp
//
// Sanity checks sur le modele IDM (formule Treiber).
// On verifie les limites analytiques connues :
//   - voie libre, v = 0  ->  a = aMax  (acceleration maximale au demarrage)
//   - voie libre, v = v0 ->  a = 0     (vitesse cible atteinte)
//   - leader stationnaire au minimum gap -> forte deceleration negative
#include <gtest/gtest.h>

#include "core/behavior/IdmModel.hpp"

using core::behavior::IdmModel;
using core::behavior::IdmParams;
using core::behavior::LeaderInfo;

TEST(IdmModel, FreeFlowFromRestEqualsAmax) {
    IdmParams p;
    p.aMax  = 150.f;
    p.delta = 4.f;
    IdmModel idm(p);

    LeaderInfo none{};  // present = false
    const float a = idm.computeAcceleration(0.f, 200.f, none);
    EXPECT_NEAR(a, p.aMax, 1e-3f);
}

TEST(IdmModel, FreeFlowAtDesiredSpeedIsZero) {
    IdmParams p;
    p.aMax  = 150.f;
    p.delta = 4.f;
    IdmModel idm(p);

    LeaderInfo none{};
    const float a = idm.computeAcceleration(200.f, 200.f, none);
    EXPECT_NEAR(a, 0.f, 1e-3f);
}

TEST(IdmModel, StoppedLeaderAtMinimumGapProducesStrongBrake) {
    IdmParams p;
    p.aMax  = 150.f;
    p.bComf = 80.f;
    p.T     = 1.5f;
    p.s0    = 5.f;
    IdmModel idm(p);

    LeaderInfo leader;
    leader.present = true;
    leader.gap     = 4.f;  // sous s0 -> sStar/s > 1 -> forte deceleration
    leader.speed   = 0.f;

    const float a = idm.computeAcceleration(50.f, 200.f, leader);
    EXPECT_LT(a, -p.bComf) << "Doit freiner plus fort que la decel confortable (a=" << a << ")";
}

TEST(IdmModel, LargeGapWithSameSpeedLeaderApproximatesFreeFlow) {
    IdmParams p;
    p.aMax  = 150.f;
    p.bComf = 80.f;
    p.T     = 1.5f;
    p.s0    = 5.f;
    IdmModel idm(p);

    LeaderInfo leader;
    leader.present = true;
    leader.gap     = 1000.f;
    leader.speed   = 100.f;

    // a v=100 et v0=200 sans leader -> a = 150 * (1 - 0.5^4) = 140.625
    const float aFree = idm.computeAcceleration(100.f, 200.f, LeaderInfo{});
    const float a     = idm.computeAcceleration(100.f, 200.f, leader);
    EXPECT_NEAR(a, aFree, 5.f); // tolerance large : on veut juste que ce soit proche
    EXPECT_GT(a, 0.f);
}
