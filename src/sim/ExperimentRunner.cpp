// src/sim/ExperimentRunner.cpp
#include "sim/ExperimentRunner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/math/Rng.hpp"
#include "core/math/Vec2.hpp"
#include "core/metrics/MetricsCollector.hpp"
#include "core/world/World.hpp"
#include "sim/McGeometry.hpp"
#include "sim/Spawner.hpp"

namespace sim {

namespace {

constexpr float kDt    = 1.f / 60.f;
constexpr float kTile  = 50.f;

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
    if (strat == RegulationType::ROUNDABOUT)
        buildIsolatedRoundabout(world, cfg.roundaboutSide, G);
    else
        buildIsolatedCrossroad(world, strat, G);

    const std::vector<OriginDef> origins = makeCrossroadOrigins(G);
    std::vector<std::unique_ptr<IAgent>> agents;
    Spawner spawner(cfg.spawn, seed);            // type + profil + bruit, seede
    core::metrics::MetricsCollector mc;

    const float injectInterval = (density > 1e-4f) ? (1.f / density) : 1e9f;
    float injectAccum = 0.f;

    auto trySpawn = [&]() {
        if (static_cast<int>(agents.size()) >= cfg.maxAgents) return;
        const OriginDef& od = origins[spawner.rng().uniformInt(0, 3)];

        // 60% tout droit, 20% / 20% virages.
        const float roll = spawner.rng().uniform(0.f, 1.f);
        const core::TileCoord goal = (roll < 0.6f) ? od.goals[0]
                                   : (roll < 0.8f) ? od.goals[1]
                                                   : od.goals[2];

        const core::Vec2 spawn{ od.start.x * kTile + kTile / 2.f,
                                od.start.y * kTile + kTile / 2.f };
        for (const auto& a : agents) {
            if ((a->getPosition() - spawn).length() < kTile * 1.2f) return;  // file -> on saute
        }

        auto v = spawner.spawn(world, od.start, goal);   // nullptr si pas de chemin
        if (v) agents.push_back(std::move(v));
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
                                             std::atomic<float>* progressFraction) {
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
            if (progressFraction)
                progressFraction->store(static_cast<float>(done) /
                    static_cast<float>(std::max<std::size_t>(total, 1)),
                    std::memory_order_relaxed);
            std::cout << "[Experience] " << strategyName(strat)
                      << " densite=" << density
                      << " -> debit=" << acc.throughputPerMin
                      << " delay=" << acc.meanDelaySec
                      << " incTTC=" << acc.ttcViolations
                      << " stops=" << acc.totalStops
                      << " minTTC=" << acc.minTTC << "\n";
        }
    }
    if (progressFraction) progressFraction->store(1.f, std::memory_order_relaxed);
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
