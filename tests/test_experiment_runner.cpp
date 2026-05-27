// tests/test_experiment_runner.cpp
//
// Couvre le banc d'essai Monte-Carlo headless (sim::ExperimentRunner) : un petit
// balayage (strategie x densite) doit produire une ligne de resultat par point,
// avec des metriques plausibles ; l'export CSV doit reussir ; et la table des
// noms de strategies doit couvrir tous les modes (regression si un cas manque).
#include <gtest/gtest.h>

#include <cstdio>     // std::remove
#include <string>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"
#include "sim/ExperimentRunner.hpp"

TEST(ExperimentRunner, SmallSweepProducesOneRowPerPoint) {
    sim::ExperimentConfig cfg;
    cfg.strategies   = { RegulationType::FIXED_PRIORITY, RegulationType::P2P };
    cfg.densities    = { 0.2f, 0.5f };
    cfg.warmupSec    = 2.f;
    cfg.durationSec  = 4.f;
    cfg.runsPerPoint = 1;
    cfg.maxAgents    = 30;

    const auto rows = sim::ExperimentRunner::run(cfg, nullptr);
    ASSERT_EQ(rows.size(), cfg.strategies.size() * cfg.densities.size());
    for (const auto& r : rows) {
        EXPECT_GE(r.throughputPerMin, 0.f);
        EXPECT_GE(r.meanDelaySec,     0.f);
        EXPECT_GT(r.density,          0.f);
        EXPECT_GT(r.minTTC,           0.f);
    }

    const std::string path = "experiment_rows.csv";
    EXPECT_TRUE(sim::ExperimentRunner::exportCsv(path, rows));
    std::remove(path.c_str());
}

TEST(ExperimentRunner, StrategyNameCoversAllModes) {
    using RT = RegulationType;
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::PRIORITY_RIGHT),  "PriorityRight");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::STOP),            "Stop");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::YIELD),           "Yield");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::TRAFFIC_LIGHT),   "TrafficLight");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::ROUNDABOUT),      "Roundabout");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::FIXED_PRIORITY),  "FixedPriority");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::P2P),             "P2P");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::AIM),             "AIM");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::VIRTUAL_PLATOON), "VirtualPlatoon");
    EXPECT_STREQ(sim::ExperimentRunner::strategyName(RT::ORCA),            "ORCA");
}

TEST(ExperimentRunner, RoundaboutStrategyRuns) {
    // Branche geometrie rond-point du runner (buildIsolatedRoundabout).
    sim::ExperimentConfig cfg;
    cfg.strategies    = { RegulationType::ROUNDABOUT };
    cfg.densities     = { 0.3f };
    cfg.warmupSec     = 2.f;
    cfg.durationSec   = 4.f;
    cfg.runsPerPoint  = 1;
    cfg.maxAgents     = 30;
    cfg.roundaboutSide = 4;

    const auto rows = sim::ExperimentRunner::run(cfg, nullptr);
    ASSERT_EQ(rows.size(), 1u);
    EXPECT_EQ(rows.front().strategy, RegulationType::ROUNDABOUT);
}
