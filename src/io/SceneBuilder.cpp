// src/SceneBuilder.cpp
#include "io/SceneBuilder.hpp"

#include "core/agent/Car.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/agent/Truck.hpp"

namespace scene {

void buildCrossroad(World& w, int cx, int cy, int id, RegulationType regType) {
    w.setTile(cx - 1, cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy - 1, RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx - 1, cy,     RoadType::INTERSECTION, TileDirection::NONE);
    w.setTile(cx,     cy,     RoadType::INTERSECTION, TileDirection::NONE);

    Intersection inter(id, regType);
    inter.addCoveredTile({cx - 1, cy - 1});
    inter.addCoveredTile({cx,     cy - 1});
    inter.addCoveredTile({cx - 1, cy});
    inter.addCoveredTile({cx,     cy});

    Approach north; north.direction = Approach::Direction::NORTH; north.entryTile = {cx,     cy - 2}; inter.addApproach(north);
    Approach south; south.direction = Approach::Direction::SOUTH; south.entryTile = {cx - 1, cy + 1}; inter.addApproach(south);
    Approach east;  east.direction  = Approach::Direction::EAST;  east.entryTile  = {cx + 1, cy - 1}; inter.addApproach(east);
    Approach west;  west.direction  = Approach::Direction::WEST;  west.entryTile  = {cx - 2, cy};     inter.addApproach(west);

    w.addIntersection(inter);
}

void buildHRoad(World& world, int y, int xStart, int xEnd) {
    for (int x = xStart; x <= xEnd; ++x) {
        world.setTile(x, y,     RoadType::CITY_50, TileDirection::RIGHT);
        world.setTile(x, y - 1, RoadType::CITY_50, TileDirection::LEFT);
    }
}

void buildVRoad(World& world, int x, int yStart, int yEnd) {
    for (int y = yStart; y <= yEnd; ++y) {
        world.setTile(x,     y, RoadType::CITY_50, TileDirection::UP);
        world.setTile(x - 1, y, RoadType::CITY_50, TileDirection::DOWN);
    }
}

void buildDefaultNetwork(World& world) {
    buildHRoad(world, 11, 0, 31);
    buildHRoad(world, 22, 0, 31);
    buildVRoad(world, 11, 0, 31);
    buildVRoad(world, 22, 0, 31);

    buildCrossroad(world, 11, 11, 0, RegulationType::PRIORITY_RIGHT);
    buildCrossroad(world, 22, 11, 1, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(world, 11, 22, 2, RegulationType::TRAFFIC_LIGHT);
    buildCrossroad(world, 22, 22, 3, RegulationType::PRIORITY_RIGHT);
}

void spawnDefaultAgents(std::vector<std::unique_ptr<IAgent>>& agents, World& world) {
    const float tileSize = world.getTileSize();

    auto addCar = [&](int sx, int sy, int gx, int gy) {
        auto c = std::make_unique<Car>(sx * tileSize + tileSize / 2.f,
                                       sy * tileSize + tileSize / 2.f);
        c->setPath(AStarPlanner::findPath(world, {sx, sy}, {gx, gy}));
        agents.push_back(std::move(c));
    };
    auto addTruck = [&](int sx, int sy, int gx, int gy) {
        auto t = std::make_unique<Truck>(sx * tileSize + tileSize / 2.f,
                                         sy * tileSize + tileSize / 2.f);
        t->setPath(AStarPlanner::findPath(world, {sx, sy}, {gx, gy}));
        agents.push_back(std::move(t));
    };

    addCar(1, 11, 21, 30);
    addTruck(30, 10, 10, 30);
    addCar(30, 10, 1, 10);
    addCar(11, 30, 11, 1);
    addCar(10, 1, 10, 30);
    addTruck(1, 22, 28, 22);
    addCar(21, 1, 21, 30);
    addCar(11, 14, 10, 21);
}

} // namespace scene
