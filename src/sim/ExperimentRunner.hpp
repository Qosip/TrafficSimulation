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

#include <string>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"   // RegulationType

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
    bool     stochasticDrivers = true;  // heterogeneite gaussienne des conducteurs
    float    driverSigma   = 0.15f;  // ecart-type du bruit sur T / aMax / vitesse
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
    // Execute toute la grille. Bloquant (synchrone). Trace la progression sur
    // std::cout. Retourne une ligne par (strategie, densite), moyennee sur les
    // runs. 'progressFraction' (optionnel) est mis a jour dans [0,1].
    static std::vector<ResultRow> run(const ExperimentConfig& cfg,
                                      float* progressFraction = nullptr);

    static bool exportCsv(const std::string& path,
                          const std::vector<ResultRow>& rows);

    static const char* strategyName(RegulationType t);

private:
    static ResultRow runOne(RegulationType strat, float density,
                            unsigned seed, const ExperimentConfig& cfg);
};

} // namespace sim
