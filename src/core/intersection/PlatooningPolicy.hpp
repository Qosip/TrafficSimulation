// src/core/intersection/PlatooningPolicy.hpp
//
// Strategie "Peloton Virtuel" (Virtual Platooning), Medina et al.
//
// Au lieu de s'arreter, les vehicules de voies PHYSIQUES distinctes sont
// projetes sur une VOIE VIRTUELLE 1D traversant le carrefour. La coordonnee
// de projection retenue ici est le TEMPS-JUSQU'AU-POINT-DE-CONFLIT : sur cet
// axe, le vehicule croise qui arrive JUSTE AVANT moi devient mon "meneur".
// J'adopte alors un meneur virtuel MOBILE (sa vitesse, a une distance projetee
// sur ma propre trajectoire) et je le suis via mon IDM -> insertion fluide en
// "fermeture eclair" (zipping), a vitesse quasi constante, sans arret total.
//
// La securite reste garantie par l'IDM : si le meneur projete est lent ou
// arrete, l'IDM me ralentit (voire m'arrete) automatiquement.
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"   // GapAcceptanceParams

namespace core::intersection {

struct PlatoonParams {
    GapAcceptanceParams gap;
    float engageDistance = 220.f;   // px : distance d'amorce du peloton virtuel
    float clearancePx    = 30.f;    // marge spatiale minimale derriere le meneur
};

class PlatooningPolicy : public IIntersectionPolicy {
public:
    explicit PlatooningPolicy(PlatoonParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

private:
    PlatoonParams params_;
};

} // namespace core::intersection
