// src/core/behavior/ICarFollowingModel.hpp
//
// Abstraction Strategy d'un modele de car-following.
// Permet de brancher IDM, Krauss, Wiedemann, OVM, Gipps... sans toucher
// au moteur Vehicle.
//
// Convention : la fonction retourne une acceleration signee (px/s^2).
//   - valeur > 0 : agent veut accelerer
//   - valeur < 0 : agent veut freiner
//
// Wave 4 du refactor.
#pragma once

#include <limits>

namespace core::behavior {

// Info sur le leader (vehicule devant) au moment de la decision.
// Pour un "leader virtuel" (point d'arret fixe), on regle :
//   present = true, gap = distance_to_stop, speed = 0, stopTarget = true
struct LeaderInfo {
    bool  present = false;
    float gap     = std::numeric_limits<float>::infinity(); // px, bumper-to-bumper
    float speed   = 0.f;                                    // px/s

    // true = "leader" virtuel FIXE (ligne de STOP, feu rouge, cede-le-passage).
    // On peut s'arreter pile a la ligne : le terme de time-headway (v*T) de l'IDM
    // n'a alors aucun sens (il sert a rester T secondes DERRIERE un mobile) et
    // ferait freiner ~2-3x trop tot puis fluer interminablement. Cf. IdmModel.
    // false (defaut) = leader REEL ou peloton mobile -> IDM complet.
    bool  stopTarget = false;
};

class ICarFollowingModel {
public:
    virtual ~ICarFollowingModel() = default;

    // selfSpeed     : vitesse courante (px/s)
    // desiredSpeed  : v0 IDM (peut etre clip par limite routiere ou virage)
    // leader        : meilleur leader (reel ou virtuel) - non-present = voie libre
    virtual float computeAcceleration(float selfSpeed,
                                       float desiredSpeed,
                                       const LeaderInfo& leader) const = 0;
};

} // namespace core::behavior
