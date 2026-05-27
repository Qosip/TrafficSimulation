// tests/test_world.cpp
//
// Couvre la grille World : limites de vitesse par type de route, pose/lecture de
// tuile + vitesse au point monde, et bascule de regulation d'un carrefour en
// direct (meme geometrie) -- fonctionnalite utilisee par l'UI et le banc d'essai.
#include <gtest/gtest.h>

#include "core/world/World.hpp"
#include "io/SceneBuilder.hpp"

TEST(World, SpeedLimitByRoadType) {
    EXPECT_FLOAT_EQ(getRoadSpeedLimit(RoadType::CITY_50),     100.f);
    EXPECT_FLOAT_EQ(getRoadSpeedLimit(RoadType::HIGHWAY_130), 260.f);
    EXPECT_FLOAT_EQ(getRoadSpeedLimit(RoadType::NONE),        0.f);
}

TEST(World, SetTileAndSpeedAtWorldPoint) {
    World w(10, 10, 50.f);
    w.setTile(3, 3, RoadType::HIGHWAY_130, TileDirection::RIGHT);
    EXPECT_EQ(w.getTile(3, 3).roadType, RoadType::HIGHWAY_130);
    // Centre de la tuile (3,3) en coords monde.
    EXPECT_FLOAT_EQ(w.getSpeedLimitAt(3 * 50.f + 25.f, 3 * 50.f + 25.f), 260.f);
}

TEST(World, IntersectionRegulationSwitchKeepsGeometry) {
    World w(12, 12, 50.f);
    scene::buildHRoad(w, 6, 0, 11);
    scene::buildVRoad(w, 6, 0, 11);
    scene::buildCrossroad(w, 6, 6, 1, RegulationType::P2P);

    ASSERT_FALSE(w.getIntersections().empty());
    EXPECT_EQ(w.getIntersections().front().getType(), RegulationType::P2P);
    const std::size_t tilesBefore = w.getIntersections().front().getCoveredTiles().size();

    w.setIntersectionRegulation(0, RegulationType::AIM);
    EXPECT_EQ(w.getIntersections().front().getType(), RegulationType::AIM);
    // La geometrie (tuiles couvertes) est preservee : seul le mode change.
    EXPECT_EQ(w.getIntersections().front().getCoveredTiles().size(), tilesBefore);

    // Indice hors borne -> no-op (pas de crash).
    w.setIntersectionRegulation(99, RegulationType::STOP);
    EXPECT_EQ(w.getIntersections().front().getType(), RegulationType::AIM);
}
