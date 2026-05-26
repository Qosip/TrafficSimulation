// src/sim/ExperimentRunner.hpp
//
// Pipeline experimental stochastique (Monte-Carlo) du rapport SMA.
//
// Teste l'hypothese centrale : la performance relative d'une coordination
// decentralisee (P2P) vs une priorite fixe presente un point d'inflexion
// correle a la densite du flux. Pour chaque (strategie x densite x run), on
// instancie un carrefour isole, on injecte des vehicules a un debit donne, on
// recycle ceux arrives, et on agrege les metriques sur une fenetre de mesure
// (apres warm-up). 100% headless : aucune dependance rendu/SFML.
#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"   // RegulationType
#include "sim/Spawner.hpp"                            // SpawnProfile

namespace sim {

struct ExperimentConfig {
    std::vector<RegulationType> strategies{ RegulationType::FIXED_PRIORITY,
                                            RegulationType::P2P };
    std::vector<float> densities{ 0.1f, 0.2f, 0.3f, 0.4f,
                                  0.5f, 0.6f, 0.7f, 0.8f };   // veh/s injectes
    float    durationSec   = 60.f;   // fenetre de MESURE
    float    warmupSec     = 10.f;   // transitoire ignore
    int      runsPerPoint  = 1;      // repetitions Monte-Carlo (moyennees)
    unsigned baseSeed      = 1337u;
    int      gridSize      = 31;     // cote de la grille (impair -> centre net)
    int      maxAgents     = 60;     // garde-fou anti-explosion (et reactivite UI)
    int      roundaboutSide = 4;     // cote de l'anneau si strategie == ROUNDABOUT

    // Generation de trafic : type de vehicule + profils comportementaux + bruit.
    SpawnProfile spawn{};
};

// Une ligne de resultat agregee par point (strategie, densite).
struct ResultRow {
    RegulationType strategy = RegulationType::FIXED_PRIORITY;
    float density           = 0.f;
    float throughputPerMin  = 0.f;
    float meanDelaySec      = 0.f;
    float meanSpeed         = 0.f;
    float minTTC            = 0.f;
    float ttcViolations     = 0.f;   // moyenne sur les runs (float)
    float totalStops        = 0.f;
    float completed         = 0.f;
};

class ExperimentRunner {
public:
    // Execute toute la grille. Synchrone (peut tourner sur un thread dedie pour
    // ne pas figer l'UI). Trace la progression sur std::cout. Retourne une ligne
    // par (strategie, densite), moyennee sur les runs. 'progressFraction'
    // (optionnel, atomique) est mis a jour dans [0,1] -> lisible par l'UI.
    static std::vector<ResultRow> run(const ExperimentConfig& cfg,
                                      std::atomic<float>* progressFraction = nullptr);

    static bool exportCsv(const std::string& path,
                          const std::vector<ResultRow>& rows);

    static const char* strategyName(RegulationType t);

private:
    static ResultRow runOne(RegulationType strat, float density,
                            unsigned seed, const ExperimentConfig& cfg);
};

} // namespace sim
