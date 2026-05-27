// tests/test_platoon.cpp
//
// Couvre le comportement du PLATOON CACC (poursuite cooperative a distance
// constante + feed-forward) face au convoi conventionnel (IDM, temps-inter
// constant). Apres un demarrage commun "feu vert", la file cooperative doit
// rester nettement plus COMPACTE que la file classique qui s'etale en accordeon.
//
// On separe les deux files par leur ordonnee (la voie platoon est au-dessus de
// la voie conventionnelle dans le scenario).
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"
#include "io/ScenarioCatalog.hpp"

using AgentVec = std::vector<std::unique_ptr<IAgent>>;

TEST(Platoon, CaccConvoyStaysTighterThanConventional) {
    const scene::ScenarioDef* def = nullptr;
    for (const auto& d : scene::scenarioCatalog())
        if (d.name.find("Convoi de camions") != std::string::npos) { def = &d; break; }
    ASSERT_NE(def, nullptr) << "Scenario platoon attendu au catalogue";

    std::unique_ptr<World> world;
    AgentVec agents;
    def->build(world, agents);
    ASSERT_GE(agents.size(), 8u);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 600; ++i) {   // 10 s de lancement
        world->updateIntersections(dt);
        for (auto& a : agents) a->computeDecision(agents, *world);
        for (auto& a : agents) a->integrate(dt);
    }

    // La voie platoon est plus haute (y plus petit) que la voie conventionnelle.
    // On coupe a mi-hauteur entre les deux files.
    constexpr float kSplitY = 350.f;
    float pMin =  std::numeric_limits<float>::infinity(), pMax = -pMin;
    float nMin =  std::numeric_limits<float>::infinity(), nMax = -nMin;
    for (const auto& a : agents) {
        const float x = a->getPosition().x;
        const float y = a->getPosition().y;
        if (y < kSplitY) { pMin = std::min(pMin, x); pMax = std::max(pMax, x); }
        else             { nMin = std::min(nMin, x); nMax = std::max(nMax, x); }
    }
    const float platoonSpread = pMax - pMin;
    const float normalSpread  = nMax - nMin;

    EXPECT_GT(platoonSpread, 0.f);
    EXPECT_GT(normalSpread,  0.f);
    EXPECT_LT(platoonSpread, normalSpread)
        << "Le platoon CACC doit rester plus compact (spread platoon="
        << platoonSpread << " px vs conventionnel=" << normalSpread << " px)";
}
