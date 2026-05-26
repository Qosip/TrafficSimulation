// src/sim/ExperimentRunner.cpp
#include "sim/ExperimentRunner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "core/agent/Car.hpp"
#include "core/agent/IAgent.hpp"
#include "core/agent/Truck.hpp"
#include "core/agent/Vehicle.hpp"
#include "core/math/Rng.hpp"
#include "core/math/Vec2.hpp"
#include "core/metrics/MetricsCollector.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/world/World.hpp"
#include "io/SceneBuilder.hpp"

namespace sim {

namespace {

constexpr float kDt    = 1.f / 60.f;
constexpr float kTile  = 50.f;

struct OriginDef {
    core::TileCoord start;
    core::TileCoord goals[3];   // [0] = tout droit, [1]/[2] = virages
};

// Geometrie du carrefour isole : routes 2 voies a la ligne R (E-O) et la
// colonne C (N-S), croisement en (C,R). Convention SceneBuilder :
//   ligne y   = voie RIGHT (vers E),  ligne y-1 = voie LEFT  (vers O)
//   colonne x = voie UP    (vers N),  colonne x-1 = voie DOWN (vers S)
std::vector<OriginDef> makeOrigins(int G) {
    const int R = G / 2;
    const int C = G / 2;
    const core::TileCoord gN{ C,     0     };   // sortie NORD  (col C, UP)
    const core::TileCoord gS{ C - 1, G - 1 };   // sortie SUD   (col C-1, DOWN)
    const core::TileCoord gE{ G - 1, R     };   // sortie EST   (ligne R, RIGHT)
    const core::TileCoord gW{ 0,     R - 1 };   // sortie OUEST (ligne R-1, LEFT)

    std::vector<OriginDef> o;
    o.push_back({ { 0,     R     }, { gE, gN, gS } });  // venant de l'OUEST -> E
    o.push_back({ { G - 1, R - 1 }, { gW, gS, gN } });  // venant de l'EST   -> O
    o.push_back({ { C,     G - 1 }, { gN, gE, gW } });  // venant du SUD     -> N
    o.push_back({ { C - 1, 0     }, { gS, gW, gE } });  // venant du NORD    -> S
    return o;
}

void buildScenario(World& world, RegulationType strat, int G) {
    const int R = G / 2;
    const int C = G / 2;
    scene::buildHRoad(world, R, 0, G - 1);
    scene::buildVRoad(world, C, 0, G - 1);
    scene::buildCrossroad(world, C, R, 1, strat);
}

float averageOf(const std::vector<float>& v) {
    if (v.empty()) return 0.f;
    double s = 0.0;
    for (float x : v) s += x;
    return static_cast<float>(s / v.size());
}

float minOf(const std::vector<float>& v) {
    float m = std::numeric_limits<float>::infinity();
    for (float x : v) m = std::min(m, x);
    return std::isfinite(m) ? m : 999.f;
}

} // namespace

const char* ExperimentRunner::strategyName(RegulationType t) {
    switch (t) {
        case RegulationType::PRIORITY_RIGHT: return "PriorityRight";
        case RegulationType::STOP:           return "Stop";
        case RegulationType::YIELD:          return "Yield";
        case RegulationType::TRAFFIC_LIGHT:  return "TrafficLight";
        case RegulationType::ROUNDABOUT:     return "Roundabout";
        case RegulationType::FIXED_PRIORITY: return "FixedPriority";
        case RegulationType::P2P:            return "P2P";
        case RegulationType::AIM:            return "AIM";
        case RegulationType::VIRTUAL_PLATOON:return "VirtualPlatoon";
    }
    return "?";
}

ResultRow ExperimentRunner::runOne(RegulationType strat, float density,
                                   unsigned seed, const ExperimentConfig& cfg) {
    const int G = cfg.gridSize;
    World world(G, G, kTile);
    buildScenario(world, strat, G);

    const std::vector<OriginDef> origins = makeOrigins(G);
    std::vector<std::unique_ptr<IAgent>> agents;
    core::Rng rng(seed);
    core::metrics::MetricsCollector mc;

    const float injectInterval = (density > 1e-4f) ? (1.f / density) : 1e9f;
    float injectAccum = 0.f;

    auto trySpawn = [&]() {
        if (static_cast<int>(agents.size()) >= cfg.maxAgents) return;
        const OriginDef& od = origins[rng.uniformInt(0, 3)];

        // 60% tout droit, 20% / 20% virages.
        const float roll = rng.uniform(0.f, 1.f);
        const core::TileCoord goal = (roll < 0.6f) ? od.goals[0]
                                   : (roll < 0.8f) ? od.goals[1]
                                                   : od.goals[2];

        auto path = AStarPlanner::findPath(world, od.start, goal);
        if (path.empty()) return;

        const core::Vec2 spawn{ od.start.x * kTile + kTile / 2.f,
                                od.start.y * kTile + kTile / 2.f };
        for (const auto& a : agents) {
            if ((a->getPosition() - spawn).length() < kTile * 1.2f) return;  // file -> on saute
        }

        std::unique_ptr<Vehicle> v;
        if (rng.bernoulli(0.15f)) v = std::make_unique<Truck>(spawn.x, spawn.y);
        else                      v = std::make_unique<Car>(spawn.x, spawn.y);
        v->setPath(path, &world);
        // Heterogeneite des conducteurs (methode de Monte-Carlo du rapport).
        if (cfg.stochasticDrivers) v->applyGaussianHeterogeneity(rng, cfg.driverSigma);
        agents.push_back(std::move(v));
    };

    auto recycle = [&]() {
        agents.erase(std::remove_if(agents.begin(), agents.end(),
            [](const std::unique_ptr<IAgent>& a) {
                const auto r = a->getBlockReason();
                return r == core::agent::BlockReason::AT_GOAL ||
                       r == core::agent::BlockReason::NO_PATH;
            }), agents.end());
    };

    auto simulate = [&](float duration, bool sample) {
        float t = 0.f;
        while (t < duration) {
            injectAccum += kDt;
            while (injectAccum >= injectInterval) {
                injectAccum -= injectInterval;
                trySpawn();
            }
            world.updateIntersections(kDt);
            for (auto& a : agents) a->computeDecision(agents, world);
            for (auto& a : agents) a->integrate(kDt);
            if (sample) mc.sample(agents, kDt);
            recycle();
            t += kDt;
        }
    };

    simulate(cfg.warmupSec, false);   // transitoire
    mc.reset();
    simulate(cfg.durationSec, true);  // fenetre de mesure

    const auto& agg = mc.aggregate();
    ResultRow row;
    row.strategy         = strat;
    row.density          = density;
    row.throughputPerMin = agg.completedVehicles * 60.f / std::max(cfg.durationSec, 1.f);
    row.meanDelaySec     = agg.meanDelaySec;
    row.meanSpeed        = mc.seriesSpeed().empty() ? agg.meanSpeed
                                                    : averageOf(mc.seriesSpeed());
    row.minTTC           = minOf(mc.seriesMinTTC());
    row.ttcViolations    = static_cast<float>(agg.ttcViolations);
    row.totalStops       = static_cast<float>(agg.totalStops);
    row.completed        = static_cast<float>(agg.completedVehicles);
    return row;
}

std::vector<ResultRow> ExperimentRunner::run(const ExperimentConfig& cfg,
                                             float* progressFraction) {
    std::vector<ResultRow> rows;
    const std::size_t total = cfg.strategies.size() * cfg.densities.size();
    std::size_t done = 0;

    for (RegulationType strat : cfg.strategies) {
        for (float density : cfg.densities) {
            // Moyenne Monte-Carlo sur runsPerPoint repetitions.
            ResultRow acc;
            acc.strategy = strat;
            acc.density  = density;
            const int reps = std::max(1, cfg.runsPerPoint);
            for (int r = 0; r < reps; ++r) {
                const unsigned seed = cfg.baseSeed
                    + static_cast<unsigned>(std::lround(density * 1000.f)) * 131u
                    + static_cast<unsigned>(strat) * 977u
                    + static_cast<unsigned>(r) * 7u;
                const ResultRow one = runOne(strat, density, seed, cfg);
                acc.throughputPerMin += one.throughputPerMin;
                acc.meanDelaySec     += one.meanDelaySec;
                acc.meanSpeed        += one.meanSpeed;
                acc.minTTC           += one.minTTC;
                acc.ttcViolations    += one.ttcViolations;
                acc.totalStops       += one.totalStops;
                acc.completed        += one.completed;
            }
            const float inv = 1.f / static_cast<float>(reps);
            acc.throughputPerMin *= inv;
            acc.meanDelaySec     *= inv;
            acc.meanSpeed        *= inv;
            acc.minTTC           *= inv;
            acc.ttcViolations    *= inv;
            acc.totalStops       *= inv;
            acc.completed        *= inv;
            rows.push_back(acc);

            ++done;
            if (progressFraction) *progressFraction = static_cast<float>(done) /
                                                      static_cast<float>(std::max<std::size_t>(total, 1));
            std::cout << "[Experience] " << strategyName(strat)
                      << " densite=" << density
                      << " -> debit=" << acc.throughputPerMin
                      << " delay=" << acc.meanDelaySec
                      << " incTTC=" << acc.ttcViolations
                      << " stops=" << acc.totalStops
                      << " minTTC=" << acc.minTTC << "\n";
        }
    }
    if (progressFraction) *progressFraction = 1.f;
    return rows;
}

bool ExperimentRunner::exportCsv(const std::string& path,
                                 const std::vector<ResultRow>& rows) {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << "strategy,density_veh_s,throughput_per_min,mean_delay_s,"
         "mean_speed_px_s,min_ttc_s,ttc_violations,total_stops,completed\n";
    for (const auto& r : rows) {
        f << strategyName(r.strategy) << ","
          << r.density            << ","
          << r.throughputPerMin   << ","
          << r.meanDelaySec       << ","
          << r.meanSpeed          << ","
          << r.minTTC             << ","
          << r.ttcViolations      << ","
          << r.totalStops         << ","
          << r.completed          << "\n";
    }
    return true;
}

} // namespace sim
