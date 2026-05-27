// src/sim/ParallelDecisions.hpp
//
// Phase 1 du pas de simulation (computeDecision) parallelisee sur les agents.
//
// SURETE : le pipeline en deux phases (computeDecision puis integrate) rend la
// parallelisation correcte par construction --
//   * computeDecision LIT l'etat des autres agents (position, vitesse, cap)
//     mais ces champs ne sont ECRITS que dans integrate(), qui s'execute dans
//     une phase SEPAREE (sequentielle). Pendant la phase de decision, l'etat
//     observable de la flotte est donc fige -> lectures concurrentes sures.
//   * Chaque agent n'ECRIT que ses propres membres (accel/vitesse desiree,
//     drapeaux d'etat, RNG par-agent) -> aucun partage en ecriture.
//   * Le SEUL etat mutable reellement partage est la table de reservations des
//     policies d'intersection (AIM), protegee par un mutex par-intersection
//     dans Intersection::request.
//
// DETERMINISME : les policies sans etat (P2P, ORCA, priorite, STOP...) restent
// bit-deterministes en parallele. SEUL AIM, dont l'attribution FCFS depend de
// l'ORDRE des requetes, perd la reproductibilite exacte sous parallelisme.
// C'est pourquoi le banc d'essai Monte-Carlo et les tests snapshot tournent
// TOUJOURS en sequentiel ; le parallelisme est reserve a l'affichage temps reel.
#pragma once

#include <memory>
#include <vector>

class World;
class IAgent;

namespace sim {

class ThreadPool;

// Execute agents[i]->computeDecision(agents, world) pour tous les agents, en
// parallele via 'pool'. Equivalent fonctionnel de la boucle sequentielle.
void computeDecisionsParallel(std::vector<std::unique_ptr<IAgent>>& agents,
                              const World& world, ThreadPool& pool);

} // namespace sim
