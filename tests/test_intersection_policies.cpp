// tests/test_intersection_policies.cpp
//
// Tests UNITAIRES des policies d'intersection au niveau de request(), centres
// sur les CAS D'EXCEPTION exiges par le cahier des charges :
//   * "donnees reseau manquantes" : PolicyContext.others == nullptr
//     (aucun voisinage observable) -> la policy doit decider SUREMENT (laisser
//     passer) sans dereferencer de pointeur nul.
//   * "paquets perdus" : le vecteur de voisins contient des entrees nulles
//     (un pair dont l'etat n'a pas ete recu) -> la policy doit les ignorer.
//   * cas normal : un conflit perpendiculaire reel est bien detecte.
//
// On instancie les policies directement (sans passer par Intersection) pour
// isoler leur logique de la geometrie.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "core/agent/Car.hpp"
#include "core/agent/IAgent.hpp"
#include "core/intersection/AimPolicy.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/intersection/OrcaPolicy.hpp"
#include "core/intersection/P2PPolicy.hpp"
#include "core/math/Vec2.hpp"

using core::intersection::AgentContext;
using core::intersection::AimPolicy;
using core::intersection::Decision;
using core::intersection::OrcaPolicy;
using core::intersection::P2PPolicy;
using core::intersection::PolicyContext;

namespace {

// Carrefour 2x2 isole synthetique centre sur (cx,cy) en coords TUILE, tileSize
// 50 -> centre monde a ((cx+0.5)*50, (cy+0.5)*50)... ici on couvre 4 tuiles.
Intersection makeBox(RegulationType type, int x0 = 10, int y0 = 10) {
    Intersection inter(1, type);
    inter.addCoveredTile({x0,     y0});
    inter.addCoveredTile({x0 + 1, y0});
    inter.addCoveredTile({x0,     y0 + 1});
    inter.addCoveredTile({x0 + 1, y0 + 1});
    return inter;
}

PolicyContext makeContext(core::Vec2 pos, float speed, float heading) {
    PolicyContext ctx;
    ctx.self.position = pos;
    ctx.self.speed    = speed;
    ctx.self.heading  = heading;
    ctx.self.length   = 30.f;
    ctx.self.accel    = 150.f;
    ctx.selfAgent     = nullptr;
    ctx.tileSize      = 50.f;
    ctx.others        = nullptr;   // par defaut : aucune donnee reseau
    return ctx;
}

} // namespace

// --- Donnees reseau manquantes (others == nullptr) ---------------------------

TEST(PolicyExceptions, OrcaNullNeighborhoodPasses) {
    OrcaPolicy pol;
    Intersection inter = makeBox(RegulationType::ORCA);
    // Approche proche du centre, others nul.
    PolicyContext ctx = makeContext({525.f - 120.f, 525.f}, 80.f, 0.f);
    Decision d = pol.request(ctx, inter);
    EXPECT_TRUE(d.canEnter);
    EXPECT_FALSE(d.shouldStop);
}

TEST(PolicyExceptions, P2PNullNeighborhoodPasses) {
    P2PPolicy pol;
    Intersection inter = makeBox(RegulationType::P2P);
    PolicyContext ctx = makeContext({525.f - 120.f, 525.f}, 80.f, 0.f);
    Decision d = pol.request(ctx, inter);
    EXPECT_TRUE(d.canEnter);
}

TEST(PolicyExceptions, AimNullNeighborhoodPasses) {
    AimPolicy pol;
    Intersection inter = makeBox(RegulationType::AIM);
    PolicyContext ctx = makeContext({525.f - 120.f, 525.f}, 80.f, 0.f);
    Decision d = pol.request(ctx, inter);
    EXPECT_TRUE(d.canEnter);
}

// --- Paquets perdus : le voisinage contient une entree NULLE -----------------

TEST(PolicyExceptions, OrcaIgnoresNullPeerEntries) {
    OrcaPolicy pol;
    Intersection inter = makeBox(RegulationType::ORCA);

    std::vector<std::unique_ptr<IAgent>> peers;
    peers.push_back(nullptr);                 // pair dont l'etat n'a pas ete recu
    peers.push_back(nullptr);

    PolicyContext ctx = makeContext({525.f - 120.f, 525.f}, 80.f, 0.f);
    ctx.others = &peers;

    Decision d = pol.request(ctx, inter);     // ne doit PAS crasher
    EXPECT_TRUE(d.canEnter);
    EXPECT_FALSE(d.shouldStop);
}

TEST(PolicyExceptions, P2PIgnoresNullPeerEntries) {
    P2PPolicy pol;
    Intersection inter = makeBox(RegulationType::P2P);
    std::vector<std::unique_ptr<IAgent>> peers;
    peers.push_back(nullptr);
    PolicyContext ctx = makeContext({525.f - 120.f, 525.f}, 80.f, 0.f);
    ctx.others = &peers;
    Decision d = pol.request(ctx, inter);
    EXPECT_TRUE(d.canEnter);
}

// --- Cas NORMAL : ORCA n'arbitre QUE les conflits croises --------------------
// Un voisin sur le MEME axe (parallele) ne doit JAMAIS declencher de cede au
// titre de l'intersection : le suivi de file (car-following) s'en charge cote
// Vehicle. La policy doit laisser passer.
TEST(OrcaPolicy, DoesNotYieldToParallelTraffic) {
    OrcaPolicy pol;
    Intersection inter = makeBox(RegulationType::ORCA);
    const core::Vec2 center{525.f, 525.f};

    // Voisin sans path -> getHeading()==0 (cap horizontal, comme self).
    auto parallel = std::make_unique<Car>(center.x + 40.f, center.y);
    std::vector<std::unique_ptr<IAgent>> peers;
    peers.push_back(std::move(parallel));

    PolicyContext ctx = makeContext({center.x - 100.f, center.y}, 80.f, 0.f);
    ctx.others = &peers;

    Decision d = pol.request(ctx, inter);
    EXPECT_TRUE(d.canEnter)      << "Trafic parallele -> pas de cede d'intersection";
    EXPECT_FALSE(d.shouldStop);
}
