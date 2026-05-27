// src/core/intersection/OrcaPolicy.hpp
//
// Strategie "ORCA" / espace ouvert : evitement de collision CONTINU et
// RECIPROQUE, inspire de van den Berg et al., "Reciprocal n-body Collision
// Avoidance" (ORCA, 2011), adapte a l'architecture voie+IDM de ce simulateur.
//
// Contrairement a AIM (reservation discrete de creneaux) ou STOP (arret franc),
// l'idee ORCA est qu'AUCUNE ligne directrice stricte ne regit le carrefour :
// chaque vehicule module CONTINUMENT sa vitesse pour s'inserer entre les
// trajectoires croisees, ne s'arretant FRANCHEMENT qu'en dernier recours.
//
// Adaptation au moteur (mouvement 1D le long d'une Lane curviligne) :
//   * On raisonne en "temps d'arrivee au centre" t = distance / vitesse.
//   * Reciprocite : sur deux trajectoires en conflit, celui qui arriverait le
//     PLUS TARD cede la moitie de la responsabilite -> il ralentit (leader
//     virtuel MOBILE), l'autre garde sa vitesse. A egalite, bris par VIN
//     (le plus grand VIN cede) pour eviter le double-cede (collision) et le
//     double-passage (deadlock).
//   * Le cede est SOUPLE par defaut (followVirtualLeader : on continue a rouler
//     en se calant derriere un meneur projete) -> entrelacement facon "zip".
//     Il ne devient un arret FERME (shouldStop) que si le conflit est imminent
//     (l'autre est deja dans la boite, ou quasi a l'arret en travers).
//
// Policy SANS etat persistant : la decision derive entierement du contexte
// cinematique observe a chaque tick (donc thread-safe par construction).
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"   // GapAcceptanceParams

namespace core::intersection {

struct OrcaParams {
    GapAcceptanceParams gap;                 // rayons de scan / fenetres de conflit
    float claimDistance   = 170.f;           // px : au-dela, approche libre (pas d'arbitrage)
    float stoppedEps      = 8.f;             // px/s : en dessous = "arrete"
    float tieMarginSec    = 0.25f;           // s  : egalite des temps d'arrivee
    float hardStopArrival = 0.6f;            // s  : si l'autre arrive avant -> arret FERME
    float softSlowFactor  = 0.55f;           // fraction du v0 conservee en cede souple
};

class OrcaPolicy : public IIntersectionPolicy {
public:
    explicit OrcaPolicy(OrcaParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    OrcaParams params_;
};

} // namespace core::intersection
