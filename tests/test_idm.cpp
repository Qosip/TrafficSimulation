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

// --- stopTarget : leader virtuel FIXE (ligne de STOP / feu / yield) ----------
//
// Regression du "freine trop tot puis flue jusqu'a la ligne" : pour un point
// d'arret fixe on retire le terme de time-headway v*T. A gap moyen, le freinage
// doit donc etre NETTEMENT plus doux qu'avec l'IDM complet, tout en restant
// franc une fois la ligne proche.

TEST(IdmModel, StopTargetBrakesLaterThanFullIdmAtMidGap) {
    IdmParams p;          // profil voiture
    p.T     = 1.0f;
    p.s0    = 5.f;
    p.aMax  = 150.f;
    p.bComf = 120.f;
    p.delta = 4.f;
    IdmModel idm(p);

    // En croisiere (v = v0), a 150 px de la ligne.
    LeaderInfo full;
    full.present = true;
    full.gap     = 150.f;
    full.speed   = 0.f;
    full.stopTarget = false;

    LeaderInfo stop = full;
    stop.stopTarget = true;

    const float aFull = idm.computeAcceleration(150.f, 150.f, full);
    const float aStop = idm.computeAcceleration(150.f, 150.f, stop);

    // Le terme v*T (=150 px) gonfle sStar -> l'IDM complet freine fort tres tot.
    EXPECT_LT(aFull, -p.bComf) << "IDM complet doit freiner fort a 150px (a=" << aFull << ")";
    // Sans time-headway, a cette distance on ne fait que lever le pied.
    EXPECT_GT(aStop, aFull) << "stopTarget doit freiner BIEN moins (aStop=" << aStop
                            << " vs aFull=" << aFull << ")";
    EXPECT_GT(aStop, -p.bComf) << "stopTarget : freinage doux a mi-distance (a=" << aStop << ")";
}

TEST(IdmModel, StopTargetStillBrakesHardNearLine) {
    IdmParams p;
    p.T     = 1.0f;
    p.s0    = 5.f;
    p.aMax  = 150.f;
    p.bComf = 120.f;
    p.delta = 4.f;
    IdmModel idm(p);

    // Tout proche de la ligne : la securite prime, freinage franc malgre stopTarget.
    LeaderInfo stop;
    stop.present    = true;
    stop.gap        = 18.f;
    stop.speed      = 0.f;
    stop.stopTarget = true;

    const float a = idm.computeAcceleration(80.f, 150.f, stop);
    EXPECT_LT(a, -p.bComf) << "Pres de la ligne, doit freiner fort (a=" << a << ")";
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
