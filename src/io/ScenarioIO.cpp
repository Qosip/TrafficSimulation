// src/io/ScenarioIO.cpp
#include "io/ScenarioIO.hpp"

#include <fstream>
#include <sstream>

#include "core/agent/Car.hpp"
#include "core/agent/Truck.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "io/SceneBuilder.hpp"

namespace io {

bool exportScenario(const std::string& filename,
                    const World& world,
                    const std::vector<std::unique_ptr<IAgent>>& agents)
{
    std::ofstream file(filename);
    if (!file.is_open()) return false;

    file << "W " << world.getGridWidth() << " " << world.getGridHeight() << " "
         << world.getTileSize() << "\n";

    for (int x = 0; x < world.getGridWidth(); ++x) {
        for (int y = 0; y < world.getGridHeight(); ++y) {
            const Tile& tile = world.getTile(x, y);
            if (tile.roadType != RoadType::NONE) {
                file << "T " << x << " " << y << " "
                     << static_cast<int>(tile.roadType) << " "
                     << static_cast<int>(tile.direction) << "\n";
            }
        }
    }
    for (const auto& inter : world.getIntersections()) {
        if (!inter.getCoveredTiles().empty()) {
            core::TileCoord firstTile = inter.getCoveredTiles()[0];
            file << "I " << firstTile.x + 1 << " " << firstTile.y + 1 << " "
                 << inter.getId() << " "
                 << static_cast<int>(inter.getType()) << "\n";
        }
    }
    for (const auto& agent : agents) {
        file << "A " << agent->getType() << " "
             << agent->getStartTile().x << " " << agent->getStartTile().y << " "
             << agent->getGoalTile().x  << " " << agent->getGoalTile().y << "\n";
    }
    return true;
}

bool importScenario(const std::string& filename,
                    std::unique_ptr<World>& outWorld,
                    std::vector<std::unique_ptr<IAgent>>& outAgents)
{
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    outAgents.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        char entryType; ss >> entryType;

        if (entryType == 'W') {
            int w, h; float tSize; ss >> w >> h >> tSize;
            outWorld = std::make_unique<World>(w, h, tSize);
        }
        else if (entryType == 'T' && outWorld) {
            int x, y, rType, dir; ss >> x >> y >> rType >> dir;
            outWorld->setTile(x, y,
                              static_cast<RoadType>(rType),
                              static_cast<TileDirection>(dir));
        }
        else if (entryType == 'I' && outWorld) {
            int cx, cy, id, rType; ss >> cx >> cy >> id >> rType;
            scene::buildCrossroad(*outWorld, cx, cy, id,
                                  static_cast<RegulationType>(rType));
        }
        else if (entryType == 'A' && outWorld) {
            std::string type; int sx, sy, gx, gy;
            ss >> type >> sx >> sy >> gx >> gy;
            const float tSize = outWorld->getTileSize();
            std::unique_ptr<Vehicle> v;
            if      (type == "CAR")   v = std::make_unique<Car>  (sx * tSize + tSize / 2.f, sy * tSize + tSize / 2.f);
            else if (type == "TRUCK") v = std::make_unique<Truck>(sx * tSize + tSize / 2.f, sy * tSize + tSize / 2.f);
            if (v) {
                v->setPath(AStarPlanner::findPath(*outWorld, {sx, sy}, {gx, gy}));
                outAgents.push_back(std::move(v));
            }
        }
    }
    return outWorld != nullptr;
}

} // namespace io
