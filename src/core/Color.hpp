// src/core/Color.hpp
//
// POD RGBA 8 bits. Substitut SFML-free de sf::Color cote core.
#pragma once

#include <cstdint>

namespace core {

struct Color {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;

    constexpr Color() = default;
    constexpr Color(std::uint8_t rr, std::uint8_t gg, std::uint8_t bb, std::uint8_t aa = 255)
        : r(rr), g(gg), b(bb), a(aa) {}

    static constexpr Color rgb(std::uint8_t rr, std::uint8_t gg, std::uint8_t bb) {
        return {rr, gg, bb, 255};
    }

    constexpr bool operator==(Color o) const {
        return r == o.r && g == o.g && b == o.b && a == o.a;
    }
};

} // namespace core
