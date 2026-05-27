// tests/test_scenario_io.cpp
//
// Couvre la (de)serialisation texte des scenarios (io::exportScenario /
// importScenario) : un aller-retour export -> import doit reconstruire la meme
// grille et le meme nombre d'agents. Couvre aussi le cas d'erreur (fichier
// absent -> false).
#include <gtest/gtest.h>

#include <cstdio>     // std::remove
#include <memory>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"
#include "io/ScenarioCatalog.hpp"
#include "io/ScenarioIO.hpp"

using AgentVec = std::vector<std::unique_ptr<IAgent>>;

namespace {
const scene::ScenarioDef* findScenario(const std::string& needle) {
    for (const auto& d : scene::scenarioCatalog())
        if (d.name.find(needle) != std::string::npos) return &d;
    return nullptr;
}
}

TEST(ScenarioIO, RoundTripPreservesWorldAndAgentCount) {
    const scene::ScenarioDef* def = findScenario("Croisement fluide");
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    ASSERT_TRUE(world);
    const int gw = world->getGridWidth();
    const int gh = world->getGridHeight();
    const std::size_t nAgents = agents.size();
    ASSERT_GT(nAgents, 0u);

    const std::string path = "scenario_io_roundtrip.txt";
    ASSERT_TRUE(io::exportScenario(path, *world, agents));

    std::unique_ptr<World> w2;
    AgentVec a2;
    ASSERT_TRUE(io::importScenario(path, w2, a2));
    ASSERT_TRUE(w2);
    EXPECT_EQ(w2->getGridWidth(),  gw);
    EXPECT_EQ(w2->getGridHeight(), gh);
    // Les agents sont reconstruits depuis (depart, but) via A* sur la MEME
    // geometrie -> meme effectif.
    EXPECT_EQ(a2.size(), nAgents);

    std::remove(path.c_str());
}

TEST(ScenarioIO, RoundTripPreservesIntersectionMode) {
    // Le mode P2P doit survivre a l'aller-retour (champ serialise en int).
    const scene::ScenarioDef* def = findScenario("P2P (VANET)");
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    ASSERT_FALSE(world->getIntersections().empty());
    const RegulationType mode = world->getIntersections().front().getType();

    const std::string path = "scenario_io_mode.txt";
    ASSERT_TRUE(io::exportScenario(path, *world, agents));

    std::unique_ptr<World> w2;
    AgentVec a2;
    ASSERT_TRUE(io::importScenario(path, w2, a2));
    ASSERT_FALSE(w2->getIntersections().empty());
    EXPECT_EQ(w2->getIntersections().front().getType(), mode);

    std::remove(path.c_str());
}

TEST(ScenarioIO, ImportMissingFileReturnsFalse) {
    std::unique_ptr<World> w;
    AgentVec a;
    EXPECT_FALSE(io::importScenario("fichier_inexistant_8f3a.txt", w, a));
}
