// tools/benchmark.cpp
//
// Banc de PERFORMANCE headless (aucune dependance rendu). Mesure le debit du
// moteur de simulation -- combien de secondes SIMULEES le solveur traite par
// seconde de temps reel -- en rejouant un balayage Monte-Carlo deterministe.
//
// Sert deux buts :
//   1. Detecter une regression de performance en CI (le chiffre est exporte
//      dans le recapitulatif de build via .github/workflows/main.yml).
//   2. Donner une reference locale ("est-ce que mon optimisation accelere ?").
//
// Sortie : lignes lisibles par l'humain + lignes "BENCH key=value" faciles a
// parser cote CI. Code de retour 0 si le run aboutit.
//
// Usage : simbench [duree_s] [runs_par_point]
//   ex.  simbench 15 1
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"
#include "sim/ExperimentRunner.hpp"

int main(int argc, char** argv) {
    sim::ExperimentConfig cfg;
    cfg.durationSec  = (argc > 1) ? std::stof(argv[1]) : 15.f;
    cfg.warmupSec    = 5.f;
    cfg.runsPerPoint = (argc > 2) ? std::atoi(argv[2]) : 1;

    // Charge representative : deux strategies x balayage de densite complet.
    cfg.strategies = { RegulationType::FIXED_PRIORITY, RegulationType::P2P };
    // cfg.densities garde le balayage par defaut (0.1 -> 0.8 v/s).

    // Secondes SIMULEES totales = (warmup + mesure) x densites x strategies x runs.
    const double simSecondsPerPoint = cfg.warmupSec + cfg.durationSec;
    const double totalSimSeconds =
        simSecondsPerPoint *
        static_cast<double>(cfg.densities.size()) *
        static_cast<double>(cfg.strategies.size()) *
        static_cast<double>(cfg.runsPerPoint);

    std::cout << "== TrafficSimulation : benchmark headless ==\n";
    std::cout << "strategies=" << cfg.strategies.size()
              << " densites="  << cfg.densities.size()
              << " runs="      << cfg.runsPerPoint
              << " mesure="    << cfg.durationSec << "s"
              << " warmup="    << cfg.warmupSec   << "s\n";

    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<sim::ResultRow> rows = sim::ExperimentRunner::run(cfg, nullptr);
    const auto t1 = std::chrono::steady_clock::now();

    const double wallSeconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    const double speedup = (wallSeconds > 1e-9) ? (totalSimSeconds / wallSeconds) : 0.0;

    // Agregats lisibles.
    std::cout << "\n-- Resultats --\n";
    for (const auto& r : rows) {
        std::cout << sim::ExperimentRunner::strategyName(r.strategy)
                  << "  d=" << r.density
                  << "  debit=" << r.throughputPerMin << "/min"
                  << "  delay=" << r.meanDelaySec << "s"
                  << "  minTTC=" << r.minTTC << "s\n";
    }

    // Lignes machine (parsees par la CI -> GitHub Step Summary).
    std::cout << "\nBENCH wall_seconds=" << wallSeconds << "\n";
    std::cout << "BENCH sim_seconds="  << totalSimSeconds << "\n";
    std::cout << "BENCH speedup_x_realtime=" << speedup << "\n";
    std::cout << "BENCH points=" << rows.size() << "\n";

    return rows.empty() ? 1 : 0;
}
