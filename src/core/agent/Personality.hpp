// src/core/agent/Personality.hpp
//
// Profil psychologique d'un agent. Influence directement :
//   - les parametres IDM (T, s0, aMax, bComf) -> distance + reactivite
//   - le respect des limitations (speedComplianceFactor > 1 = "speede")
//   - les seuils de gap-acceptance (patience)
//   - la decision de depassement (aggressiveness, riskTolerance)
//   - la probabilite de panne (breakdownChancePerMin)
//
// Toutes les valeurs sont expose-ees comme champs publics pour permettre
// au shell UI (panneau ImGui) de les editer en live.
#pragma once

#include <algorithm>

namespace core::agent {

struct Personality {
    // --- Conduite (IDM modulators) ---
    float reactionTimeFactor   = 1.f;   // [0.5..2.0]   multiplie T IDM
    float minGapFactor         = 1.f;   // [0.5..2.0]   multiplie s0 IDM
    float comfortBrakeFactor   = 1.f;   // [0.5..1.5]   multiplie bComf IDM
    float accelEagernessFactor = 1.f;   // [0.5..1.5]   multiplie aMax IDM

    // --- Respect de la vitesse ---
    // 1.0 = conforme, 1.2 = roule a 120 quand limite = 100, 0.8 = prudent.
    float speedComplianceFactor = 1.f;  // [0.5..1.5]

    // --- Comportement aux intersections (gap-acceptance) ---
    // > 1 : accepte des gaps plus petits ; < 1 : attend plus longtemps.
    float intersectionPatience = 1.f;   // [0.5..2.0]

    // --- Depassement ---
    // Probabilite d'envisager un depassement quand toutes les conditions
    // techniques sont reunies. 0 = ne depasse jamais.
    float overtakeWillingness = 0.f;    // [0..1]
    // Vitesse-leader / vitesse-self minimum pour declencher.
    // 0.7 = ne depasse que si leader < 70% de ma vitesse.
    float overtakeLeaderRatio = 0.7f;   // [0.3..0.95]

    // --- Distraction / fiabilite mecanique ---
    // Probabilite par MINUTE simulee de tomber en panne. 0.01 = 1%/min.
    float breakdownChancePerMin = 0.f;  // [0..0.1]
    float breakdownDurationSec  = 8.f;  // duree moyenne d'arret (s)

    // --- Coordination de peloton (CACC) ---
    // true : poursuite COOPERATIVE a distance constante avec feed-forward de
    // l'acceleration du predecesseur (vehicules connectes). Demarrage en unisson
    // (aucun delai de reaction) et intervalle court CONSTANT (pas d'ouverture de
    // gap avec la vitesse, contrairement a l'IDM en temps-inter constant).
    bool cooperative = false;

    // Helpers de clamp pour proteger les sliders UI.
    void clamp() {
        auto C = [](float v, float lo, float hi) { return std::clamp(v, lo, hi); };
        reactionTimeFactor    = C(reactionTimeFactor,    0.3f, 3.0f);
        minGapFactor          = C(minGapFactor,          0.3f, 3.0f);
        comfortBrakeFactor    = C(comfortBrakeFactor,    0.3f, 2.0f);
        accelEagernessFactor  = C(accelEagernessFactor,  0.3f, 2.0f);
        speedComplianceFactor = C(speedComplianceFactor, 0.3f, 2.0f);
        intersectionPatience  = C(intersectionPatience,  0.3f, 3.0f);
        overtakeWillingness   = C(overtakeWillingness,   0.0f, 1.0f);
        overtakeLeaderRatio   = C(overtakeLeaderRatio,   0.3f, 0.95f);
        breakdownChancePerMin = C(breakdownChancePerMin, 0.0f, 0.5f);
        breakdownDurationSec  = C(breakdownDurationSec,  1.0f, 120.f);
    }
};

// Profils preetablis utilises par Car/Truck/scenarios.
namespace profiles {

inline Personality calmDriver() {
    Personality p;
    p.reactionTimeFactor    = 1.3f;
    p.minGapFactor          = 1.3f;
    p.comfortBrakeFactor    = 0.8f;
    p.accelEagernessFactor  = 0.8f;
    p.speedComplianceFactor = 0.95f;
    p.intersectionPatience  = 1.4f;
    p.overtakeWillingness   = 0.05f;
    p.overtakeLeaderRatio   = 0.55f;
    p.breakdownChancePerMin = 0.005f;
    return p;
}

inline Personality normalDriver() {
    Personality p; // valeurs par defaut
    p.overtakeWillingness   = 0.25f;
    p.breakdownChancePerMin = 0.01f;
    return p;
}

inline Personality aggressiveDriver() {
    Personality p;
    p.reactionTimeFactor    = 0.7f;
    p.minGapFactor          = 0.7f;
    p.comfortBrakeFactor    = 1.3f;
    p.accelEagernessFactor  = 1.3f;
    p.speedComplianceFactor = 1.15f;     // roule 15% au-dessus
    p.intersectionPatience  = 0.7f;
    p.overtakeWillingness   = 0.7f;
    p.overtakeLeaderRatio   = 0.85f;
    p.breakdownChancePerMin = 0.015f;
    return p;
}

inline Personality truckDriver() {
    Personality p;
    p.reactionTimeFactor    = 1.4f;
    p.minGapFactor          = 1.2f;
    p.comfortBrakeFactor    = 0.7f;
    p.accelEagernessFactor  = 0.5f;
    p.speedComplianceFactor = 0.9f;
    p.intersectionPatience  = 1.3f;
    p.overtakeWillingness   = 0.0f;      // les camions ne depassent pas
    p.breakdownChancePerMin = 0.02f;     // un peu plus fragile
    return p;
}

} // namespace profiles
} // namespace core::agent
