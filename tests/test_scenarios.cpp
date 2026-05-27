// tests/test_scenarios.cpp
//
// Tests d'integration pilotes par le CATALOGUE de scenarios. Couvre :
//   * integrite : chaque scenario se construit sans crash et produit une scene
//     jouable (world non nul, agents presents) ;
//   * presence du scenario ORCA / espace ouvert (cahier des charges) ;
//   * LIVENESS (cas limites) : les scenarios reputes "durs" se DRAINENT --
//       - arrivee simultanee parfaite de 4 vehicules (resolution de deadlock),
//       - ORCA espace ouvert (~10 vehicules, evitement continu),
//       - saturation P2P (degrade facon four-way stop).
//     Invariant verifie : tout vehicule finit par ATTEINDRE son but, aucun ne
//     reste bloque sans chemin (NO_PATH), et la scene se vide entierement.
//
// La boucle de simulation est ici SEQUENTIELLE et a dt FIXE (determinisme),
// comme le banc d'essai Monte-Carlo.
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "core/agent/BlockReason.hpp"
#include "core/agent/IAgent.hpp"
#include "core/math/Vec2.hpp"
#include "core/world/World.hpp"
#include "io/ScenarioCatalog.hpp"

using AgentVec = std::vector<std::unique_ptr<IAgent>>;
using core::agent::BlockReason;

namespace {

constexpr float kDt = 1.f / 60.f;

const scene::ScenarioDef* findScenario(const std::string& needle) {
    for (const auto& s : scene::scenarioCatalog())
        if (s.name.find(needle) != std::string::npos) return &s;
    return nullptr;
}

struct DrainResult {
    int   spawned     = 0;
    int   reachedGoal = 0;   // agents ayant atteint AT_GOAL
    int   noPath      = 0;   // agents bloques sans chemin (echec)
    int   activeLeft  = 0;   // agents encore en circulation au timeout
    float minSep      = 1e9f; // separation minimale observee (sanity collision)
};

// Fait tourner la scene jusqu'a vidage complet ou expiration du budget simule.
// Recycle (compte + retire) les agents AT_GOAL / NO_PATH comme le moteur hote.
DrainResult drain(std::unique_ptr<World>& world, AgentVec& agents, float maxSec) {
    DrainResult r;
    r.spawned = static_cast<int>(agents.size());

    for (float t = 0.f; t < maxSec && !agents.empty(); t += kDt) {
        world->updateIntersections(kDt);
        for (auto& a : agents) if (a) a->computeDecision(agents, *world);
        for (auto& a : agents) if (a) a->integrate(kDt);

        // Sanity collision : separation minimale entre paires d'agents.
        for (std::size_t i = 0; i < agents.size(); ++i)
            for (std::size_t j = i + 1; j < agents.size(); ++j) {
                if (!agents[i] || !agents[j]) continue;
                const float d = (agents[i]->getPosition() - agents[j]->getPosition()).length();
                r.minSep = std::min(r.minSep, d);
            }

        // Comptabilise puis retire les agents termines.
        for (auto& a : agents) {
            if (!a) continue;
            const auto reason = a->getBlockReason();
            if (reason == BlockReason::AT_GOAL) ++r.reachedGoal;
            else if (reason == BlockReason::NO_PATH) ++r.noPath;
        }
        agents.erase(std::remove_if(agents.begin(), agents.end(),
            [](const std::unique_ptr<IAgent>& a) {
                if (!a) return true;
                const auto reason = a->getBlockReason();
                return reason == BlockReason::AT_GOAL ||
                       reason == BlockReason::NO_PATH;
            }), agents.end());
    }
    r.activeLeft = static_cast<int>(agents.size());
    return r;
}

} // namespace

// --- Integrite du catalogue --------------------------------------------------

TEST(ScenarioCatalog, EveryScenarioBuildsAValidScene) {
    const auto& catalog = scene::scenarioCatalog();
    ASSERT_FALSE(catalog.empty());

    for (const auto& def : catalog) {
        std::unique_ptr<World> world;
        AgentVec agents;
        ASSERT_NO_THROW(def.build(world, agents)) << "build crash : " << def.name;
        EXPECT_TRUE(world)            << "world nul : " << def.name;
        EXPECT_FALSE(agents.empty())  << "aucun agent : " << def.name;
        if (world) {
            EXPECT_GT(world->getGridWidth(), 0);
            EXPECT_GT(world->getGridHeight(), 0);
        }
    }
}

// Smoke test : fait TOURNER chaque scenario quelques secondes. Exerce toute la
// chaine decisionnelle (perception, IDM/CACC, depassement, panne, filet
// anti-collision) ET chaque policy de regulation (les scenarios "modes" couvrent
// priorite/STOP/cedez/feux/priorite-fixe/P2P/AIM/peloton/ORCA/rond-point).
// Aucun invariant verifie : on attrape les crash / acces invalides a l'echelle.
TEST(ScenarioCatalog, EveryScenarioRunsBrieflyWithoutCrash) {
    for (const auto& def : scene::scenarioCatalog()) {
        std::unique_ptr<World> world;
        AgentVec agents;
        def.build(world, agents);
        ASSERT_TRUE(world) << def.name;

        for (int i = 0; i < 360 && !agents.empty(); ++i) {   // ~6 s simulees
            world->updateIntersections(kDt);
            for (auto& a : agents) if (a) a->computeDecision(agents, *world);
            for (auto& a : agents) if (a) a->integrate(kDt);
            agents.erase(std::remove_if(agents.begin(), agents.end(),
                [](const std::unique_ptr<IAgent>& a) {
                    if (!a) return true;
                    const auto r = a->getBlockReason();
                    return r == BlockReason::AT_GOAL || r == BlockReason::NO_PATH;
                }), agents.end());
        }
        SUCCEED() << def.name;
    }
}

TEST(ScenarioCatalog, ContainsOrcaOpenSpaceScenario) {
    const scene::ScenarioDef* def = findScenario("ORCA");
    ASSERT_NE(def, nullptr) << "Le scenario ORCA / espace ouvert doit etre au catalogue";

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    EXPECT_GE(static_cast<int>(agents.size()), 8) << "~10 vehicules attendus";
}

// --- Liveness : resolution de deadlock ---------------------------------------

TEST(Liveness, SimultaneousFourWayP2PResolvesDeadlock) {
    const scene::ScenarioDef* def = findScenario("simultanee x4 - P2P");
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    const int spawned = static_cast<int>(agents.size());
    ASSERT_EQ(spawned, 4) << "Quatre vehicules, un par branche";

    const DrainResult r = drain(world, agents, /*maxSec*/150.f);
    EXPECT_EQ(r.noPath, 0)             << "Aucun vehicule ne doit rester sans chemin";
    EXPECT_EQ(r.activeLeft, 0)         << "La scene doit se vider (pas de deadlock)";
    EXPECT_EQ(r.reachedGoal, spawned)  << "Les 4 vehicules doivent franchir et arriver";
    EXPECT_GT(r.minSep, 3.f)           << "Pas d'interpenetration (collision)";
}

// --- Liveness : ORCA espace ouvert -------------------------------------------

TEST(Liveness, OrcaOpenSpaceDrains) {
    const scene::ScenarioDef* def = findScenario("ORCA");
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    const int spawned = static_cast<int>(agents.size());

    const DrainResult r = drain(world, agents, /*maxSec*/200.f);
    EXPECT_EQ(r.noPath, 0);
    EXPECT_EQ(r.activeLeft, 0)         << "Evitement continu -> tout le monde passe";
    EXPECT_EQ(r.reachedGoal, spawned);
    EXPECT_GT(r.minSep, 3.f);
}

// --- Liveness : saturation P2P (degrade four-way stop) -----------------------

TEST(Liveness, SaturationP2PEventuallyDrains) {
    const scene::ScenarioDef* def = findScenario("Forte densite P2P");
    ASSERT_NE(def, nullptr);

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    const int spawned = static_cast<int>(agents.size());
    ASSERT_GE(spawned, 12);

    // Debit effondre mais liveness preservee : budget genereux.
    const DrainResult r = drain(world, agents, /*maxSec*/300.f);
    EXPECT_EQ(r.noPath, 0);
    EXPECT_EQ(r.reachedGoal, spawned)  << "Meme sature, la scene finit par se vider";
    EXPECT_EQ(r.activeLeft, 0);
}
