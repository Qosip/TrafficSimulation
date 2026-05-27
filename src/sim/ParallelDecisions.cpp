// src/sim/ParallelDecisions.cpp
#include "sim/ParallelDecisions.hpp"

#include "core/agent/IAgent.hpp"
#include "sim/ThreadPool.hpp"

namespace sim {

void computeDecisionsParallel(std::vector<std::unique_ptr<IAgent>>& agents,
                              const World& world, ThreadPool& pool) {
    const std::size_t n = agents.size();
    // Capture par pointeur du vecteur : computeDecision a besoin de la flotte
    // complete (perception, arbitrage d'intersection) en LECTURE SEULE.
    std::vector<std::unique_ptr<IAgent>>* a = &agents;
    pool.parallelFor(n, [a, &world](std::size_t i) {
        IAgent* agent = (*a)[i].get();
        if (agent) agent->computeDecision(*a, world);
    });
}

} // namespace sim
