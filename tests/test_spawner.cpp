// tests/test_spawner.cpp
//
// Couvre le generateur de trafic parametrable (Spawner) et le pilotage de la
// session Monte-Carlo visuelle bornee (McLiveSession) :
//   * reproductibilite (meme graine -> meme sequence type/profil) ;
//   * distribution des types respectee aux bornes (100% camion / 100% voiture) ;
//   * trajet impossible -> nullptr (pas de crash) ;
//   * budget de vehicules : injecte EXACTEMENT N puis s'arrete ;
//   * limite de temps : atteinte apres la duree simulee demandee.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"
#include "sim/McGeometry.hpp"
#include "sim/McLiveSession.hpp"
#include "sim/Spawner.hpp"

using sim::SpawnProfile;
using sim::Spawner;

namespace {

constexpr int   kG  = 31;
constexpr float kDt = 1.f / 60.f;

World makeCrossWorld() {
    World w(kG, kG, 50.f);
    sim::buildIsolatedCrossroad(w, RegulationType::FIXED_PRIORITY, kG);
    return w;
}

// Les camions sont longs (80 px) ; les voitures courtes (30). getLength() suffit
// a classer le type sans dependre d'un getType() textuel.
bool isTruck(const IAgent& a) { return a.getLength() > 50.f; }

} // namespace

TEST(Spawner, SameSeedIsReproducible) {
    World w1 = makeCrossWorld();
    World w2 = makeCrossWorld();
    const auto origins = sim::makeCrossroadOrigins(kG);

    SpawnProfile p;                  // distribution mixte par defaut
    Spawner s1(p, 4242u);
    Spawner s2(p, 4242u);

    for (int i = 0; i < 30; ++i) {
        auto a = s1.spawn(w1, origins[0].start, origins[0].goals[0]);
        auto b = s2.spawn(w2, origins[0].start, origins[0].goals[0]);
        ASSERT_TRUE(a);
        ASSERT_TRUE(b);
        EXPECT_EQ(isTruck(*a), isTruck(*b)) << "divergence de type au tirage " << i;
    }
}

TEST(Spawner, AllTruckWeightProducesOnlyTrucks) {
    World w = makeCrossWorld();
    const auto origins = sim::makeCrossroadOrigins(kG);

    SpawnProfile p;
    p.wCar = 0.f; p.wTruck = 1.f;
    Spawner s(p, 1u);
    for (int i = 0; i < 25; ++i) {
        auto v = s.spawn(w, origins[1].start, origins[1].goals[0]);
        ASSERT_TRUE(v);
        EXPECT_TRUE(isTruck(*v));
    }
}

TEST(Spawner, AllCarWeightProducesOnlyCars) {
    World w = makeCrossWorld();
    const auto origins = sim::makeCrossroadOrigins(kG);

    SpawnProfile p;
    p.wCar = 1.f; p.wTruck = 0.f;
    Spawner s(p, 2u);
    for (int i = 0; i < 25; ++i) {
        auto v = s.spawn(w, origins[2].start, origins[2].goals[0]);
        ASSERT_TRUE(v);
        EXPECT_FALSE(isTruck(*v));
    }
}

// Cas d'EXCEPTION : aucun chemin possible (destination hors reseau routier).
// Le spawner doit renvoyer nullptr sans crash ni allocation d'agent orphelin.
TEST(Spawner, UnreachableGoalReturnsNullptr) {
    World w = makeCrossWorld();
    const auto origins = sim::makeCrossroadOrigins(kG);

    SpawnProfile p;
    Spawner s(p, 99u);
    // {0,0} est une tuile vide (hors des axes du carrefour) -> A* echoue.
    auto v = s.spawn(w, origins[0].start, core::TileCoord{0, 0});
    EXPECT_FALSE(v) << "Destination injoignable -> nullptr attendu";
}

// Cas d'EXCEPTION : depart hors reseau (tuile vide) -> egalement nullptr.
TEST(Spawner, UnreachableStartReturnsNullptr) {
    World w = makeCrossWorld();
    Spawner s(SpawnProfile{}, 100u);
    auto v = s.spawn(w, core::TileCoord{0, 0}, core::TileCoord{0, 1});
    EXPECT_FALSE(v);
}

// =============================================================================
// McLiveSession : conditions d'arret bornees (budget de vehicules / temps).
// =============================================================================

// BUDGET : avec maxSpawns = N, la session injecte EXACTEMENT N vehicules puis
// cesse d'en creer (budgetExhausted) -- cas limite "saturation maitrisee".
TEST(McLiveSession, BudgetSpawnsExactlyN) {
    std::unique_ptr<World> w;
    std::vector<std::unique_ptr<IAgent>> agents;
    sim::McLiveSession mc;

    constexpr int kBudget = 6;
    mc.start(w, agents, RegulationType::FIXED_PRIORITY, /*density*/1.0f,
             SpawnProfile{}, /*seed*/7u, /*maxSpawns*/kBudget, /*timeLimit*/0.f);

    // Avance le temps simule jusqu'a epuisement du budget (garde-fou anti-boucle).
    for (int i = 0; i < 200000 && !mc.budgetExhausted(); ++i)
        mc.inject(*w, agents, kDt);

    EXPECT_TRUE(mc.budgetExhausted());
    EXPECT_EQ(mc.spawned(), kBudget) << "Le budget doit injecter EXACTEMENT N vehicules";

    // Au-dela du budget, plus aucune injection.
    const int before = mc.spawned();
    for (int i = 0; i < 1000; ++i) mc.inject(*w, agents, kDt);
    EXPECT_EQ(mc.spawned(), before);
}

// TEMPS : avec timeLimitSec = T, la session se declare terminee une fois T
// secondes SIMULEES ecoulees.
TEST(McLiveSession, TimeLimitReachedAfterDuration) {
    std::unique_ptr<World> w;
    std::vector<std::unique_ptr<IAgent>> agents;
    sim::McLiveSession mc;

    constexpr float kLimit = 2.0f;   // s simulees
    mc.start(w, agents, RegulationType::P2P, /*density*/0.3f,
             SpawnProfile{}, /*seed*/7u, /*maxSpawns*/0, /*timeLimit*/kLimit);

    int steps = 0;
    const int maxSteps = static_cast<int>((kLimit + 1.0f) / kDt);
    while (!mc.timeLimitReached() && steps < maxSteps) {
        mc.inject(*w, agents, kDt);
        ++steps;
    }
    EXPECT_TRUE(mc.timeLimitReached());
    EXPECT_GE(mc.elapsed(), kLimit);
}
