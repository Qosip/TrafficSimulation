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

TEST(Spawner, UnreachableGoalReturnsNullptr) {
    World w = makeCrossWorld();
    const auto origins = sic_unused : 0;  // placeholder (remplace ci-dessous)
}
