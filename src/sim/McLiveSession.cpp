// src/sim/McLiveSession.cpp
#include "sim/McLiveSession.hpp"

#include "core/agent/IAgent.hpp"
#include "core/math/Vec2.hpp"
#include "core/world/World.hpp"

namespace sim {

void McLiveSession::start(std::unique_ptr<World>& world,
                          std::vector<std::unique_ptr<IAgent>>& agents,
                          RegulationType strat, float densityVehPerSec,
                          const SpawnProfile& profile, std::uint64_t seed,
                          int maxSpawns, float timeLimitSec,
                          int roundaboutSide, int gridSize) {
    world = std::make_unique<World>(gridSize, gridSize, tileSize_);
    if (strat == RegulationType::ROUNDABOUT)
        buildIsolatedRoundabout(*world, roundaboutSide, gridSize);
    else
        buildIsolatedCrossroad(*world, strat, gridSize);
    agents.clear();

    origins_      = makeCrossroadOrigins(gridSize);
    spawner_      = std::make_unique<Spawner>(profile, seed);
    strategy_     = strat;
    density_      = densityVehPerSec;
    maxSpawns_    = maxSpawns;
    timeLimit_    = timeLimitSec;
    spawnedCount_ = 0;
    elapsed_      = 0.f;
    injectAccum_  = 0.f;
    active_       = true;
}

void McLiveSession::inject(World& world,
                           std::vector<std::unique_ptr<IAgent>>& agents, float dt) {
    if (!active_ || !spawner_ || origins_.empty()) return;

    // Horloge simulee (sert a la limite de temps), avancee meme une fois le
    // budget de vehicules epuise (on attend alors la fin du drainage).
    elapsed_ += dt;

    // Plus d'injection si le budget est atteint ou le temps ecoule : on laisse
    // le trafic restant evacuer (le moteur hote declenchera la fin).
    if (budgetExhausted() || timeLimitReached()) return;

    const float interval = (density_ > 1e-4f) ? (1.f / density_) : 1e9f;
    injectAccum_ += dt;
    while (injectAccum_ >= interval) {
        injectAccum_ -= interval;
        if (budgetExhausted()) break;
        if (static_cast<int>(agents.size()) >= maxAgents_) continue;

        const OriginDef& od = origins_[spawner_->rng().uniformInt(0, 3)];
        // 60% tout droit, 20% / 20% virages (meme repartition que le headless).
        const float roll = spawner_->rng().uniform(0.f, 1.f);
        const core::TileCoord goal = (roll < 0.6f) ? od.goals[0]
                                   : (roll < 0.8f) ? od.goals[1]
                                                   : od.goals[2];

        const core::Vec2 spawnPos{ od.start.x * tileSize_ + tileSize_ / 2.f,
                                   od.start.y * tileSize_ + tileSize_ / 2.f };
        bool blocked = false;
        for (const auto& a : agents)
            if (a && (a->getPosition() - spawnPos).length() < tileSize_ * 1.2f) {
                blocked = true;   // file a l'entree -> on saute cette injection
                break;
            }
        if (blocked) continue;

        auto v = spawner_->spawn(world, od.start, goal);
        if (v) {
            agents.push_back(std::move(v));
            ++spawnedCount_;
        }
    }
}

} // namespace sim
