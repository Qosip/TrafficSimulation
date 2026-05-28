// src/core/intersection/AimPolicy.hpp
//
// Strategie "AIM" (Autonomous Intersection Management), Dresner & Stone.
//
// Un gestionnaire centralise (Intersection Manager) arbitre l'acces par un
// systeme de RESERVATIONS spatio-temporelles, en politique "Premier Arrive,
// Premier Servi" (FCFS). Chaque vehicule approchant demande a reserver la
// fenetre temporelle [tEnter, tExit] pendant laquelle il occupera le
// carrefour. Le gestionnaire accorde si aucune reservation conflictuelle
// (trajectoire croisee se chevauchant dans le temps) n'existe ; sinon il
// refuse, et le vehicule ralentit (via son IDM) puis re-soumet une requete
// modifiee jusqu'a obtenir une clairance.
//
// Le carrefour 2x2 est modelise comme une boite unique (granularite "1 tuile") :
// deux trajectoires PARALLELES (meme axe) peuvent coexister (voies separees /
// suivi de file), seules les trajectoires PERPENDICULAIRES se disputent la boite.
//
// La table de reservations est un etat MUTABLE (cache) propre a chaque
// intersection (chaque Intersection possede sa propre instance de policy).
// Le temps absolu provient de Intersection::now().
#pragma once

#include <unordered_map>

#include "core/agent/TurnIntent.hpp"
#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"   // GapAcceptanceParams

namespace core::intersection {

class AimPolicy : public IIntersectionPolicy {
public:
    explicit AimPolicy(GapAcceptanceParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    struct Slot {
        float tEnterAbs   = 0.f;
        float tExitAbs    = 0.f;
        Approach::Direction from = Approach::Direction::NORTH;
        core::agent::TurnIntent intent = core::agent::TurnIntent::UNKNOWN;
        float lastSeenAbs = 0.f;     // pour purger les reservations orphelines
    };

    GapAcceptanceParams params_;
    mutable std::unordered_map<int, Slot> reservations_;   // VIN -> creneau accorde
};

} // namespace core::intersection
