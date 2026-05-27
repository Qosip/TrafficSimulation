// tests/test_parallel.cpp
//
// Couvre sim::computeDecisionsParallel : la phase de decision parallelisee doit
// donner EXACTEMENT le meme resultat que la boucle sequentielle sur une scene
// SANS etat partage mutable (pas d'intersection AIM). C'est garanti par le
// pipeline en 2 phases : chaque decision ne LIT que l'etat du pas precedent et
// n'ECRIT que ses propres champs -> l'ordre de traitement n'influe pas.
//
// On utilise le scenario "Convoi de camions" (CACC, route droite, aucune
// intersection) : entierement deterministe en parallele.
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"
#include "io/ScenarioCatalog.hpp"
#include "sim/ParallelDecisions.hpp"
#include "sim/ThreadPool.hpp"

using AgentVec = std::vector<std::unique_ptr<IAgent>>;

namespace {

const scene::ScenarioDef* findScenario(const std::string& needle) {
    for (const auto& d : scene::scenarioCatalog())
        if (d.name.find(needle) != std::string::npos) return &d;
    return nullptr;
}

std::vector<float> runConvoy(bool parallel, sim::ThreadPool* pool) {
    const scene::ScenarioDef* def = findScenario("Convoi de camions");
    std::unique_ptr<World> w;
    AgentVec agents;
    def->build(w, agents);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 300; ++i) {
        w->updateIntersections(dt);
        if (parallel) sim::computeDecisionsParallel(agents, *w, *pool);
        else          for (auto& a : agents) a->computeDecision(agents, *w);
        for (auto& a : agents) a->integrate(dt);
    }
    std::vector<float> xs;
    for (const auto& a : agents) xs.push_back(a->getPosition().x);
    return xs;
}

} // namespace

TEST(ParallelDecisions, MatchesSerialOnStatelessScene) {
    ASSERT_NE(findScenario("Convoi de camions"), nullptr);

    sim::ThreadPool pool;
    const std::vector<float> serial   = runConvoy(false, nullptr);
    const std::vector<float> parallel = runConvoy(true,  &pool);

    ASSERT_EQ(serial.size(), parallel.size());
    ASSERT_FALSE(serial.empty());
    for (std::size_t i = 0; i < serial.size(); ++i)
        EXPECT_NEAR(serial[i], parallel[i], 1e-3f) << "agent " << i;
}
