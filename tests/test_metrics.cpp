// tests/test_metrics.cpp
//
// Couvre le collecteur de metriques (testbed) : sur le scenario "Route libre"
// (un seul vehicule, trajectoire libre), il doit comptabiliser exactement UNE
// completion une fois le vehicule arrive, et faire avancer le temps simule.
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "core/agent/BlockReason.hpp"
#include "core/agent/IAgent.hpp"
#include "core/metrics/MetricsCollector.hpp"
#include "core/world/World.hpp"
#include "io/ScenarioCatalog.hpp"

using AgentVec = std::vector<std::unique_ptr<IAgent>>;
using core::agent::BlockReason;

TEST(MetricsCollector, CountsExactlyOneCompletionOnFreeRoad) {
    const scene::ScenarioDef* def = nullptr;
    for (const auto& d : scene::scenarioCatalog())
        if (d.name.find("Route libre") != std::string::npos) { def = &d; break; }
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    const int spawned = static_cast<int>(agents.size());
    ASSERT_GE(spawned, 1);

    core::metrics::MetricsCollector mc;
    const float dt = 1.f / 60.f;
    for (int i = 0; i < 60 * 60 && !agents.empty(); ++i) {   // <= 60 s simulees
        world->updateIntersections(dt);
        for (auto& a : agents) a->computeDecision(agents, *world);
        for (auto& a : agents) a->integrate(dt);
        mc.sample(agents, dt);                               // APRES integrate
        agents.erase(std::remove_if(agents.begin(), agents.end(),
            [](const std::unique_ptr<IAgent>& a) {
                const auto r = a->getBlockReason();
                return r == BlockReason::AT_GOAL || r == BlockReason::NO_PATH;
            }), agents.end());
    }

    const auto& agg = mc.aggregate();
    EXPECT_EQ(agg.completedVehicles, spawned) << "Le vehicule arrive doit etre compte";
    EXPECT_GT(agg.simTime, 0.f);
    EXPECT_TRUE(agents.empty()) << "Le vehicule doit avoir atteint son but en < 60 s";
}
