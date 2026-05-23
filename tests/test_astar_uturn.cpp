// tests/test_astar_uturn.cpp
//
// Verifie que l'A* refuse de produire un chemin qui passerait par un
// demi-tour a l'interieur d'une intersection (Wave 2 du refactor).
#include <gtest/gtest.h>

#include "core/pathfinding/AStarPlanner.hpp"
#include "core/world/World.hpp"
#include "io/SceneBuilder.hpp"

namespace {

// Construit une mini-grille 5x5 avec UN seul carrefour en (2,2).
//   - Route verticale x=2 (UP) / x=1 (DOWN), y in [0..4]
//   - Route horizontale y=2 (RIGHT) / y=1 (LEFT),  x in [0..4]
//   - Carrefour 2x2 couvrant (1,1), (2,1), (1,2), (2,2).
void buildMiniNetwork(World& w) {
    scene::buildVRoad(w, 2, 0, 4);
    scene::buildHRoad(w, 2, 0, 4);
    scene::buildCrossroad(w, 2, 2, 0, RegulationType::PRIORITY_RIGHT);
}

} // namespace

TEST(AStarPlanner, RejectsUTurnThroughIntersection) {
    World world(5, 5, 50.f);
    buildMiniNetwork(world);

    // Depart : (2,4) lane UP. Arrivee : (1,4) lane DOWN (rangee adjacente, sens oppose).
    // Le seul chemin geometrique passe par le carrefour ET requiert un demi-tour.
    const auto path = AStarPlanner::findPath(world, {2, 4}, {1, 4});

    EXPECT_TRUE(path.empty())
        << "Demi-tour interdit, le pathfinder doit retourner un chemin vide. Longueur="
        << path.size();
}

TEST(AStarPlanner, AllowsLegitTraverse) {
    World world(5, 5, 50.f);
    buildMiniNetwork(world);

    // Depart : (2,4) lane UP. Arrivee : (4,2) lane RIGHT.
    // Trajet legitime : montee, traversee intersection en virant a droite, sortie est.
    const auto path = AStarPlanner::findPath(world, {2, 4}, {4, 2});

    ASSERT_FALSE(path.empty()) << "Chemin legitime non trouve";
    EXPECT_EQ(path.front(), core::TileCoord(2, 4));
    EXPECT_EQ(path.back(),  core::TileCoord(4, 2));
}

TEST(AStarPlanner, NoPathTraversesMoreThanThreeIntersectionCells) {
    // Verifie que le plafond cellsInInter <= 3 est respecte sur tous les
    // chemins legitimes. Empeche les spirales internes a l'intersection 2x2.
    World world(7, 7, 50.f);
    scene::buildVRoad(world, 3, 0, 6);
    scene::buildHRoad(world, 3, 0, 6);
    scene::buildCrossroad(world, 3, 3, 0, RegulationType::PRIORITY_RIGHT);

    const auto path = AStarPlanner::findPath(world, {3, 6}, {6, 3});
    ASSERT_FALSE(path.empty());

    int interCells = 0;
    for (const auto& t : path) {
        if (world.getTile(t.x, t.y).roadType == RoadType::INTERSECTION) {
            ++interCells;
        }
    }
    EXPECT_LE(interCells, 3)
        << "Le chemin traverse plus de 3 cellules d'intersection (spirale ?), nb=" << interCells;
}

TEST(AStarPlanner, BlocksConsecutiveOppositeMovesInsideIntersection) {
    // Sanity check : meme si la grille a des lanes paralleles tout autour,
    // une sequence (entree NORD -> sortie SUD) doit etre refusee.
    World world(7, 7, 50.f);
    scene::buildVRoad(world, 3, 0, 6);
    scene::buildHRoad(world, 3, 0, 6);
    scene::buildCrossroad(world, 3, 3, 0, RegulationType::PRIORITY_RIGHT);

    const auto path = AStarPlanner::findPath(world, {3, 6}, {2, 6});
    EXPECT_TRUE(path.empty()) << "U-turn par intersection encore tolere (taille=" << path.size() << ")";
}
