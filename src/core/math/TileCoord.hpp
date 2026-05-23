// src/core/math/TileCoord.hpp
//
// Coordonnees entieres de tuile sur la grille de simulation.
// Remplace sf::Vector2i a l'interieur du core.
#pragma once

#include <cstddef>
#include <functional>

namespace core {

struct TileCoord {
    int x = 0;
    int y = 0;

    constexpr TileCoord() = default;
    constexpr TileCoord(int xx, int yy) : x(xx), y(yy) {}

    constexpr bool operator==(TileCoord o) const { return x == o.x && y == o.y; }
    constexpr bool operator!=(TileCoord o) const { return !(*this == o); }

    constexpr TileCoord operator+(TileCoord o) const { return {x + o.x, y + o.y}; }
    constexpr TileCoord operator-(TileCoord o) const { return {x - o.x, y - o.y}; }
};

struct TileCoordHash {
    std::size_t operator()(TileCoord t) const noexcept {
        // splitmix-style 32+32 → 64 fold
        std::uint64_t k = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t.x)) << 32)
                        | static_cast<std::uint32_t>(t.y);
        k ^= k >> 33; k *= 0xff51afd7ed558ccdULL;
        k ^= k >> 33; k *= 0xc4ceb9fe1a85ec53ULL;
        k ^= k >> 33;
        return static_cast<std::size_t>(k);
    }
};

} // namespace core

namespace std {
template <>
struct hash<core::TileCoord> {
    std::size_t operator()(core::TileCoord t) const noexcept {
        return core::TileCoordHash{}(t);
    }
};
} // namespace std
