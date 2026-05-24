// src/core/pathfinding/AStarPlanner.hpp
//
// A* sur grille a 4-connexite avec :
//   * penalite de virage (decourage les zig-zags),
//   * BLOCAGE strict des demi-tours :
//       1) interdiction d'inversion immediate (lastMoveDir == -moveDir),
//       2) interdiction de sortie d'intersection opposee a l'entree,
//       3) plafond du nombre de cellules visitees dans une meme intersection
//          (max 3 cellules de la zone 2x2) -- ferme la porte aux trajectoires
//          en spirale qui cumuleraient ~270 deg de virage et toucheraient
//          la limite 360 deg sur une fenetre de 4 cellules.
//
// L'etat de recherche est le triplet :
//   (TileCoord pos, TileCoord entryDir, int cellsInInter)
//   - entryDir = {0,0}     : on n'est pas dans une intersection
//   - entryDir != {0,0}    : direction par laquelle on est entre dans l'intersection courante
//   - cellsInInter         : nombre de cellules deja visitees dans l'intersection courante
//
// Etape 4 + Wave 2 + Wave 6 : SFML purge, U-turn ban renforce.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

#include "core/intersection/Intersection.hpp"
#include "core/math/TileCoord.hpp"
#include "core/world/World.hpp"

class AStarPlanner {
public:
    static std::vector<core::TileCoord> findPath(const World& world,
                                                 core::TileCoord start,
                                                 core::TileCoord goal)
    {
        std::vector<core::TileCoord> emptyPath;

        if (world.getTile(start.x, start.y).roadType == RoadType::NONE ||
            world.getTile(goal.x,  goal.y).roadType  == RoadType::NONE) {
            return emptyPath;
        }

        const float tileSizeForLookup = world.getTileSize();
        auto isInter = [&](core::TileCoord t) {
            return world.getTile(t.x, t.y).roadType == RoadType::INTERSECTION;
        };
        // Cap dynamique : carrefour classique 2x2 -> 3 cellules max (anti
        // demi-tour en spirale). Seul un ROND-POINT autorise la traversee de
        // tout son anneau (sinon impossible d'en faire le tour).
        auto interCellCapAt = [&](core::TileCoord t) -> int {
            const Intersection* it = world.getIntersectionAt(
                t.x * tileSizeForLookup + tileSizeForLookup / 2.f,
                t.y * tileSizeForLookup + tileSizeForLookup / 2.f);
            if (!it) return 3;
            if (it->getType() == RegulationType::ROUNDABOUT)
                return std::max<int>(3, static_cast<int>(it->getCoveredTiles().size()));
            return 3;
        };

        struct State {
            core::TileCoord pos;
            core::TileCoord entryDir;
            int             cellsInInter = 0;
            constexpr bool operator==(const State& o) const {
                return pos == o.pos && entryDir == o.entryDir &&
                       cellsInInter == o.cellsInInter;
            }
        };
        struct StateHash {
            std::size_t operator()(const State& s) const noexcept {
                // pos.x, pos.y, entryDir.x, entryDir.y, cellsInInter -> 64 bits + mix.
                std::uint64_t k =
                    (static_cast<std::uint64_t>(static_cast<std::uint16_t>(s.pos.x))      << 48) |
                    (static_cast<std::uint64_t>(static_cast<std::uint16_t>(s.pos.y))      << 32) |
                    (static_cast<std::uint64_t>(static_cast<std::uint16_t>(s.entryDir.x)) << 16) |
                    (static_cast<std::uint64_t>(static_cast<std::uint16_t>(s.entryDir.y)));
                k ^= static_cast<std::uint64_t>(s.cellsInInter) * 0x9E3779B97F4A7C15ULL;
                k ^= k >> 33; k *= 0xff51afd7ed558ccdULL;
                k ^= k >> 33; k *= 0xc4ceb9fe1a85ec53ULL;
                k ^= k >> 33;
                return static_cast<std::size_t>(k);
            }
        };

        // Si le start est deja sur une intersection on tolere toutes les sorties
        // (entryDir = {0,0} == "pas de contrainte").
        State startState{start, {0, 0}, 0};

        using PQNode = std::pair<float, State>;
        auto cmp = [](const PQNode& a, const PQNode& b) { return a.first > b.first; };
        std::priority_queue<PQNode, std::vector<PQNode>, decltype(cmp)> openSet(cmp);

        std::unordered_map<State, State, StateHash> cameFrom;
        std::unordered_map<State, float, StateHash> gScore;

        openSet.push({0.f, startState});
        gScore[startState] = 0.f;

        while (!openSet.empty()) {
            State current = openSet.top().second;
            openSet.pop();

            if (current.pos == goal) {
                return reconstructPath(cameFrom, current);
            }

            // Direction d'arrivee (pour penalite de virage) : positions diffs.
            core::TileCoord arrivalDir{0, 0};
            if (auto it = cameFrom.find(current); it != cameFrom.end()) {
                arrivalDir = current.pos - it->second.pos;
            }

            for (const auto& neighbor : world.getValidNeighbors(current.pos.x, current.pos.y)) {
                const core::TileCoord moveDir = neighbor - current.pos;

                // Filtre 1 : inversion immediate (U-turn sec, hors intersection).
                if (arrivalDir != core::TileCoord{0, 0} &&
                    moveDir == core::TileCoord{-arrivalDir.x, -arrivalDir.y}) {
                    continue;
                }

                const bool curIsInter  = isInter(current.pos);
                const bool nextIsInter = isInter(neighbor);

                // Calcul de l'entryDir + cellsInInter successeurs + filtrage U-turn.
                core::TileCoord nextEntryDir{0, 0};
                int             nextCellsInInter = 0;

                if (nextIsInter) {
                    if (curIsInter) {
                        // On reste dans l'intersection : persiste la direction d'entree.
                        nextEntryDir     = current.entryDir;
                        nextCellsInInter = current.cellsInInter + 1;
                    } else {
                        // On vient d'y entrer.
                        nextEntryDir     = moveDir;
                        nextCellsInInter = 1;
                    }
                    // Filtre 2 : plafond cellules visitees dans la meme intersection.
                    // Cap = max(3, taille de l'anneau) pour permettre la traversee
                    // d'un grand rond-point sans casser le filtre U-turn 2x2.
                    if (nextCellsInInter > interCellCapAt(neighbor)) continue;
                } else {
                    if (curIsInter && current.entryDir != core::TileCoord{0, 0}) {
                        // Filtre 3 : sortie d'intersection opposee a l'entree.
                        if (moveDir == core::TileCoord{-current.entryDir.x, -current.entryDir.y}) {
                            continue;
                        }
                    }
                    nextEntryDir     = {0, 0};
                    nextCellsInInter = 0;
                }

                float moveCost = 1.0f;
                if (arrivalDir != core::TileCoord{0, 0} && arrivalDir != moveDir) {
                    moveCost += 0.5f;
                }

                const State nextState{neighbor, nextEntryDir, nextCellsInInter};
                const float tentativeG = gScore[current] + moveCost;

                auto itG = gScore.find(nextState);
                if (itG == gScore.end() || tentativeG < itG->second) {
                    cameFrom[nextState] = current;
                    gScore[nextState]   = tentativeG;
                    const float hScore = static_cast<float>(
                        std::abs(neighbor.x - goal.x) + std::abs(neighbor.y - goal.y));
                    openSet.push({tentativeG + hScore, nextState});
                }
            }
        }
        return emptyPath;
    }

private:
    template <typename StateT, typename StateHashT>
    static std::vector<core::TileCoord> reconstructPath(
        std::unordered_map<StateT, StateT, StateHashT>& cameFrom,
        StateT current)
    {
        std::vector<core::TileCoord> path;
        path.push_back(current.pos);

        while (true) {
            auto it = cameFrom.find(current);
            if (it == cameFrom.end()) break;
            current = it->second;
            path.push_back(current.pos);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};
