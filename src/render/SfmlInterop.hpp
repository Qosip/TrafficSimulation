// src/render/SfmlInterop.hpp
//
// Helpers de conversion 1-pour-1 entre les types du core (Vec2, Color) et
// leurs equivalents SFML. Volontairement confines dans la couche render/
// pour ne PAS polluer le core.
#pragma once

#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Vector2.hpp>

#include "core/Color.hpp"
#include "core/math/Vec2.hpp"

namespace render {

inline sf::Vector2f toSfml(core::Vec2 v)        { return {v.x, v.y}; }
inline core::Vec2   toCore(sf::Vector2f v)      { return {v.x, v.y}; }

inline sf::Color    toSfml(core::Color c)       { return {c.r, c.g, c.b, c.a}; }
inline core::Color  toCore(sf::Color c)         { return {c.r, c.g, c.b, c.a}; }

} // namespace render
