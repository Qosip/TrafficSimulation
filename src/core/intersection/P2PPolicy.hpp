// src/core/intersection/P2PPolicy.hpp
//
// Strategie de coordination "Pair-a-Pair" decentralisee, inspiree de
// VanMiddlesworth, Dresner & Stone, "Replacing the Stop Sign: Unmanaged
// Intersection Control for Autonomous Vehicles" (AAMAS 2008).
//
// AUCUNE autorite centrale : chaque agent execute localement, a chaque tick,
// le meme algorithme de resolution de conflits en observant les "Claims" de
// ses pairs (ici : leur etat cinematique observable + VIN + intention).
//
// Machine a etats (derivee de la distance a l'intersection, donc sans etat
// persistant cote policy) :
//   - LURKING   : loin (> claimDistance) -> observe, aucune revendication.
//   - CLAIMING  : proche -> revendique et applique la hierarchie de dominance.
//   - TRAVERSAL : engage (gere par le commit-to-pass du Vehicle).
//
// Hierarchie de dominance (un Claim en conflit me DOMINE -> je cede) :
//   Regle 1 : si les deux roulent, le plus petit estimated_exit_time gagne.
//   Regle 2 : si les deux sont arretes, le vehicule a DROITE gagne.
//   Regle 3 : a egalite, une trajectoire rectiligne gagne sur un virage.
//   Regle 4 : ultime bris d'egalite, le plus petit VIN gagne.
// (cas mixte arrete/roulant : le vehicule arrete cede au roulant en conflit.)
#pragma once

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"   // GapAcceptanceParams

namespace core::intersection {

// Etat de negociation, expose pour la visualisation (overlay).
enum class P2PState { LURKING, CLAIMING, TRAVERSAL };

struct P2PParams {
    GapAcceptanceParams gap;               // fenetres de conflit spatio-temporelles
    float claimDistance = 160.f;           // px : seuil LURKING -> CLAIMING
    float stoppedEps    = 8.f;             // px/s : en dessous = "arrete"
    float exitTimeTol   = 0.3f;            // s  : egalite de temps de sortie
};

class P2PPolicy : public IIntersectionPolicy {
public:
    explicit P2PPolicy(P2PParams params = {}) : params_(params) {}

    Decision request(const PolicyContext& ctx, const Intersection& inter) const override;

    // Etat de negociation pour un contexte donne (purement diagnostique /
    // visualisation : derive de la distance, ne modifie pas la decision).
    P2PState stateFor(const PolicyContext& ctx, const Intersection& inter) const;

private:
    P2PParams params_;
};

} // namespace core::intersection
