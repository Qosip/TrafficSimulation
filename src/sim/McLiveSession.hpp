// src/sim/McLiveSession.hpp
//
// Monte-Carlo "visuel" : un carrefour isole alimente EN CONTINU par un Spawner
// stochastique, destine a etre rendu dans la boucle principale (l'utilisateur
// observe le flux). Le recyclage des vehicules arrives est assure par le moteur
// hote (destruction des AT_GOAL). Flux perpetuel -> il n'y a pas de "fin".
//
// Pendant : seul l'etat d'injection vit ici (origines, accumulateur, Spawner).
// L'integration physique et la collecte de metriques restent geres par la
// boucle hote, exactement comme un scenario normal.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"   // RegulationType
#include "sim/McGeometry.hpp"
#include "sim/Spawner.hpp"

class World;
class IAgent;

namespace sim {

class McLiveSession {
public:
    // (Re)construit le monde (carrefour isole) + reinitialise l'injection.
    //   maxSpawns    : 0 = illimite, sinon on cesse d'injecter apres N vehicules
    //                  (la sim se termine quand ils ont tous evacue -> metriques).
    //   timeLimitSec : 0 = illimite, sinon la sim se termine a cet instant
    //                  (temps SIMULE), pour recuperer le flux mesure.
    // Les deux a 0 -> flux PERPETUEL (observation libre).
    // 'roundaboutSide' n'est utilise que si strat == ROUNDABOUT (cote de l'anneau).
    void start(std::unique_ptr<World>& world,
               std::vector<std::unique_ptr<IAgent>>& agents,
               RegulationType strat, float densityVehPerSec,
               const SpawnProfile& profile, std::uint64_t seed,
               int maxSpawns = 0, float timeLimitSec = 0.f,
               int roundaboutSide = 4, int gridSize = 31);

    // A appeler une fois par pas FIXE, AVANT computeDecision : injecte les
    // vehicules dus selon la densite (respecte un espacement minimal a l'entree,
    // le plafond maxAgents et le budget/temps), et avance l'horloge interne.
    void inject(World& world, std::vector<std::unique_ptr<IAgent>>& agents, float dt);

    bool  active()  const { return active_; }
    void  stop()          { active_ = false; }

    // Conditions d'arret (interrogees par le moteur hote).
    bool  timeLimitReached() const { return timeLimit_ > 0.f && elapsed_ >= timeLimit_; }
    bool  budgetExhausted()  const { return maxSpawns_ > 0 && spawnedCount_ >= maxSpawns_; }

    float density()  const { return density_; }
    int   spawned()  const { return spawnedCount_; }
    float elapsed()  const { return elapsed_; }
    int   maxSpawns()    const { return maxSpawns_; }
    float timeLimit()    const { return timeLimit_; }
    RegulationType strategy() const { return strategy_; }

private:
    bool           active_       = false;
    RegulationType strategy_     = RegulationType::FIXED_PRIORITY;
    float          density_      = 0.3f;   // veh/s injectes
    float          injectAccum_  = 0.f;
    int            maxAgents_     = 90;
    float          tileSize_      = 50.f;
    int            maxSpawns_     = 0;     // 0 = illimite
    float          timeLimit_     = 0.f;   // s simulees, 0 = illimite
    int            spawnedCount_  = 0;
    float          elapsed_       = 0.f;
    std::vector<OriginDef>   origins_;
    std::unique_ptr<Spawner> spawner_;
};

} // namespace sim
