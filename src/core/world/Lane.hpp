// src/Lane.hpp
//
// Trajectoire 1D parametrique (curvilineaire).
// Etape 4 du refactor : sf::Vector2f remplace par core::Vec2.
#pragma once

#include <vector>

#include "core/math/Vec2.hpp"

// Resultat d'une projection d'un point monde sur la trajectoire (repere de
// Frenet) : abscisse curvilineaire du pied de projection + ecart lateral signe.
struct LaneProjection {
    bool  valid   = false;
    float s       = 0.f;   // abscisse curvilineaire du pied (px)
    float lateral = 0.f;   // distance perpendiculaire SIGNEE au tracé (px)
};

class Lane {
private:
    std::vector<core::Vec2> points;
    std::vector<float>      accumulatedDistances;
    float                   totalLength;

public:
    explicit Lane(const std::vector<core::Vec2>& waypoints);

    // Position (x, y) le long de la courbe a la distance curvilineaire s.
    core::Vec2 getPositionAt(float s) const;

    // Heading (en degres) a la distance s.
    float getHeadingAt(float s) const;

    // Projection de Frenet : pied de p sur la polyligne, en ne considerant que
    // l'intervalle d'abscisse [sMin, sMax] (borne la recherche -> O(segments
    // pertinents)). Renvoie l'abscisse du pied le plus PROCHE et l'ecart lateral
    // signe (gauche/droite du sens de parcours). Sert au car-following exact
    // (gap mesure LE LONG du tracé, pas a vol d'oiseau) et au filtrage same-lane :
    // un vehicule en contre-sens / sur une voie croisee projette un |lateral|
    // grand -> rejete comme leader. Indispensable en virage, ou un cone angulaire
    // brut accroche le mauvais vehicule.
    LaneProjection project(core::Vec2 p, float sMin, float sMax) const;

    float getLength() const { return totalLength; }
    const std::vector<core::Vec2>& getPoints() const { return points; }
};
