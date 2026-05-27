// src/core/agent/BlockReason.hpp
//
// Diagnostic enumere de l'etat decisionnel d'un agent.
// Sert exclusivement aux overlays de debug (panneau de controle, dashboard).
// Pas de logique metier ne doit brancher dessus.
#pragma once

namespace core::agent {

enum class BlockReason {
    NONE,                 // Agent roule librement
    NO_PATH,              // Pas de route calculee
    AT_GOAL,              // Destination atteinte
    INITIALIZING,         // Acceleration depuis arret, voie libre
    CORNERING,            // v0 reduit par anticipation d'un virage
    LEADER_VEHICLE,       // Vehicule reel en face, IDM freine
    INTERSECTION_RED,     // Feu rouge / orange
    INTERSECTION_YIELD,   // Gap acceptance refuse (priorite a droite / fixe)
    INTERSECTION_STOP,    // Arret obligatoire panneau STOP
    NEGOTIATING,          // P2P : Claim domine, cede le passage (VanMiddlesworth)
    PLATOONING,           // Peloton virtuel : suit un meneur projete (zipping)
    BREAKDOWN,            // En panne (timer)
    OVERTAKING,           // En manoeuvre de depassement
    KEEP_CLEAR,           // Anti-gridlock : sortie occupee -> on ne s'engage pas
};

inline const char* toString(BlockReason r) {
    switch (r) {
        case BlockReason::NONE:               return "Libre";
        case BlockReason::NO_PATH:            return "PERDU (route coupee)";
        case BlockReason::AT_GOAL:            return "Arrive a destination";
        case BlockReason::INITIALIZING:       return "Demarrage";
        case BlockReason::CORNERING:          return "Ralentit pour virage";
        case BlockReason::LEADER_VEHICLE:     return "Bloque par vehicule";
        case BlockReason::INTERSECTION_RED:   return "Feu rouge / orange";
        case BlockReason::INTERSECTION_YIELD: return "Cede priorite";
        case BlockReason::INTERSECTION_STOP:  return "Marque l'arret (STOP)";
        case BlockReason::NEGOTIATING:        return "Negocie (P2P)";
        case BlockReason::PLATOONING:         return "Peloton virtuel";
        case BlockReason::BREAKDOWN:          return "En panne";
        case BlockReason::OVERTAKING:         return "Depassement en cours";
        case BlockReason::KEEP_CLEAR:         return "Sortie bloquee (anti-gridlock)";
    }
    return "?";
}

} // namespace core::agent
