// src/core/agent/TurnIntent.hpp
//
// Intention de manoeuvre d'un agent a la prochaine intersection.
// Utilise par :
//   - les protocoles de coordination decentralisee (P2P VanMiddlesworth) pour
//     departager les conflits : une trajectoire rectiligne l'emporte sur un
//     virage (regle de dominance 3). Ces regles ne distinguent PAS le sens du
//     virage -> elles agregent LEFT|RIGHT via isTurning().
//   - la regulation a feux : un tourne-a-GAUCHE croise le flux tout-droit d'en
//     face (meme phase verte) et doit lui ceder ; un tourne-a-DROITE non. D'ou
//     la distinction gauche/droite.
#pragma once

namespace core::agent {

enum class TurnIntent {
    UNKNOWN,    // pas de chemin / indeterminable
    STRAIGHT,   // traversee rectiligne
    LEFT,       // virage a gauche  (croise l'oncoming -> doit ceder aux feux)
    RIGHT,      // virage a droite  (ne croise pas l'oncoming)
};

// "Tourne" agrege, pour les regles indifferentes au sens (dominance P2P).
inline bool isTurning(TurnIntent t) {
    return t == TurnIntent::LEFT || t == TurnIntent::RIGHT;
}

inline const char* toString(TurnIntent t) {
    switch (t) {
        case TurnIntent::UNKNOWN:  return "?";
        case TurnIntent::STRAIGHT: return "tout droit";
        case TurnIntent::LEFT:     return "gauche";
        case TurnIntent::RIGHT:    return "droite";
    }
    return "?";
}

} // namespace core::agent
