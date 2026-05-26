// src/sim/Spawner.cpp
#include "sim/Spawner.hpp"

#include <algorithm>

#include "core/agent/Car.hpp"
#include "core/agent/Truck.hpp"
#include "core/agent/Vehicle.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/world/World.hpp"

namespace sim {

namespace prof = core::agent::profiles;

Spawner::Spawner(SpawnProfile profile, std::uint64_t seed)
    : profile_(profile), rng_(seed) {}

bool Spawner::rollTruck() {
    const float wc  = std::max(0.f, profile_.wCar);
    const float wt  = std::max(0.f, profile_.wTruck);
    const float sum = wc + wt;
    if (sum <= 1e-6f) return false;
    return rng_.uniform(0.f, sum) >= wc;   // [0,wc) -> voiture, [wc,sum) -> camion
}

core::agent::Personality Spawner::rollPersonality(bool isTruck) {
    if (isTruck) return prof::truckDriver();

    const float wn  = std::max(0.f, profile_.wNormal);
    const float wa  = std::max(0.f, profile_.wAggressive);
    const float wcm = std::max(0.f, profile_.wCalm);
    const float sum = wn + wa + wcm;
    if (sum <= 1e-6f) return prof::normalDriver();

    const float r = rng_.uniform(0.f, sum);
    if (r < wn)        return prof::normalDriver();
    if (r < wn + wa)   return prof::aggressiveDriver();
    return prof::calmDriver();
}

std::unique_ptr<IAgent> Spawner::spawn(World& world,
                                       core::TileCoord start,
                                       core::TileCoord goal) {
    auto path = AStarPlanner::findPath(world, start, goal);
    if (path.empty()) return nullptr;

    const float ts = world.getTileSize();
    const float sx = start.x * ts + ts / 2.f;
    const float sy = start.y * ts + ts / 2.f;

    const bool truck = rollTruck();
    std::unique_ptr<Vehicle> v;
    if (truck) v = std::make_unique<Truck>(sx, sy);
    else       v = std::make_unique<Car>(sx, sy);

    v->setPersonality(rollPersonality(truck));
    v->setPath(path, &world);
    if (profile_.gaussianHeterogeneity)
        v->applyGaussianHeterogeneity(rng_, profile_.driverSigma);

    return v;
}

} // namespace sim
