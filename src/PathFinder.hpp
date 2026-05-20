// src/Pathfinder.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "World.hpp"

// Pour utiliser sf::Vector2i comme clé dans une map
struct Vector2iHash {
    std::size_t operator()(const sf::Vector2i& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1);
    }
};

class Pathfinder {
public:
    static std::vector<sf::Vector2i> findPath(const World& world, sf::Vector2i start, sf::Vector2i goal) {
        std::vector<sf::Vector2i> emptyPath;

        if (world.getTile(start.x, start.y).roadType == RoadType::NONE ||
            world.getTile(goal.x, goal.y).roadType == RoadType::NONE) {
            return emptyPath;
        }

        using Node = std::pair<float, sf::Vector2i>;
        auto cmp = [](const Node& a, const Node& b) { return a.first > b.first; };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> openSet(cmp);

        std::unordered_map<sf::Vector2i, sf::Vector2i, Vector2iHash> cameFrom;
        std::unordered_map<sf::Vector2i, float, Vector2iHash> gScore;

        openSet.push({0.f, start});
        gScore[start] = 0.f;

        while (!openSet.empty()) {
            sf::Vector2i current = openSet.top().second;
            openSet.pop();

            if (current == goal) {
                return reconstructPath(cameFrom, current);
            }

            // --- NOUVEAU : Déterminer la direction actuelle du véhicule ---
            sf::Vector2i currentDir(0, 0);
            if (cameFrom.find(current) != cameFrom.end()) {
                currentDir = current - cameFrom[current];
            }

            for (const auto& neighbor : world.getValidNeighbors(current.x, current.y)) {
                sf::Vector2i moveDir = neighbor - current;

                // --- NOUVEAU : Calcul du coût avec pénalité de virage ---
                float moveCost = 1.0f;

                // Si on bougeait déjà (currentDir != 0,0) et qu'on change de direction
                if (currentDir != sf::Vector2i(0, 0) && currentDir != moveDir) {
                    moveCost += 0.5f; // Pénalité pour décourager les zig-zags
                }

                float tentative_gScore = gScore[current] + moveCost;

                if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative_gScore;

                    float hScore = std::abs(neighbor.x - goal.x) + std::abs(neighbor.y - goal.y);
                    openSet.push({tentative_gScore + hScore, neighbor});
                }
            }
        }

        return emptyPath;
    }

private:
    static std::vector<sf::Vector2i> reconstructPath(
        std::unordered_map<sf::Vector2i, sf::Vector2i, Vector2iHash>& cameFrom,
        sf::Vector2i current)
    {
        std::vector<sf::Vector2i> path;
        path.push_back(current);

        while (cameFrom.find(current) != cameFrom.end()) {
            current = cameFrom[current];
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};