// src/core/agent/TurnIntent.hpp
//
// Intention de manoeuvre d'un agent a la prochaine intersection.
// Utilise par les protocoles de coordination decentralisee (P2P
// VanMiddlesworth) pour departager les conflits : une trajectoire
// rectiligne l'emporte sur un virage (regle de dominance 3).
//
// Volontairement minimaliste : la regle ne distingue PAS gauche/droite,
// seulement "tout droit" vs "tourne".
#pragma once

namespace core::agent {

enum class TurnIntent {
    UNKNOWN,    // pas de chemin / indeterminable
    STRAIGHT,   // traversee rectiligne
    TURNING,    // virage (gauche ou droite)
};

inline const char* toString(TurnIntent t) {
    switch (t) {
        case TurnIntent::UNKNOWN:  return "?";
        case TurnIntent::STRAIGHT: return "tout droit";
        case TurnIntent::TURNING:  return "virage";
    }
    return "?";
}

} // namespace core::agent
