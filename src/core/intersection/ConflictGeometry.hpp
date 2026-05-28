// src/core/intersection/ConflictGeometry.hpp
//
// Helpers geometriques communs aux policies d'intersection. Ils evitent de
// reduire un conflit a "axes perpendiculaires" : un tourne-a-gauche peut aussi
// couper un vehicule venant d'en face qui continue tout droit ou tourne a droite.
#pragma once

#include <cmath>

#include "core/agent/TurnIntent.hpp"
#include "core/intersection/IntersectionTypes.hpp"
#include "core/math/Constants.hpp"

namespace core::intersection::conflict {

inline bool isHorizontalAxis(Approach::Direction d) {
    return d == Approach::Direction::EAST || d == Approach::Direction::WEST;
}

inline float expectedHeadingFor(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::WEST:  return 0.f;
        case Approach::Direction::EAST:  return 180.f;
        case Approach::Direction::NORTH: return 90.f;
        case Approach::Direction::SOUTH: return 270.f;
    }
    return 0.f;
}

inline Approach::Direction directionFromHeading(float headingDeg) {
    const Approach::Direction dirs[4] = {
        Approach::Direction::NORTH, Approach::Direction::SOUTH,
        Approach::Direction::EAST,  Approach::Direction::WEST
    };
    Approach::Direction best = Approach::Direction::NORTH;
    float bestErr = 1e9f;
    for (auto d : dirs) {
        const float err = std::abs(math::wrapDeg180(headingDeg - expectedHeadingFor(d)));
        if (err < bestErr) { bestErr = err; best = d; }
    }
    return best;
}

inline Approach::Direction rightOf(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::NORTH: return Approach::Direction::WEST;
        case Approach::Direction::SOUTH: return Approach::Direction::EAST;
        case Approach::Direction::EAST:  return Approach::Direction::NORTH;
        case Approach::Direction::WEST:  return Approach::Direction::SOUTH;
    }
    return Approach::Direction::NORTH;
}

inline Approach::Direction leftOf(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::NORTH: return Approach::Direction::EAST;
        case Approach::Direction::SOUTH: return Approach::Direction::WEST;
        case Approach::Direction::EAST:  return Approach::Direction::SOUTH;
        case Approach::Direction::WEST:  return Approach::Direction::NORTH;
    }
    return Approach::Direction::NORTH;
}

inline Approach::Direction oppositeOf(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::NORTH: return Approach::Direction::SOUTH;
        case Approach::Direction::SOUTH: return Approach::Direction::NORTH;
        case Approach::Direction::EAST:  return Approach::Direction::WEST;
        case Approach::Direction::WEST:  return Approach::Direction::EAST;
    }
    return Approach::Direction::NORTH;
}

inline Approach::Direction exitFor(Approach::Direction from,
                                   core::agent::TurnIntent intent) {
    if (intent == core::agent::TurnIntent::LEFT)  return leftOf(from);
    if (intent == core::agent::TurnIntent::RIGHT) return rightOf(from);
    return oppositeOf(from);
}

inline bool sameAxis(Approach::Direction a, Approach::Direction b) {
    return isHorizontalAxis(a) == isHorizontalAxis(b);
}

inline bool oncoming(Approach::Direction a, Approach::Direction b) {
    return sameAxis(a, b) && a != b;
}

inline bool isStraightOrUnknown(core::agent::TurnIntent intent) {
    return intent == core::agent::TurnIntent::STRAIGHT ||
           intent == core::agent::TurnIntent::UNKNOWN;
}

inline bool leftTurnMustYieldTo(Approach::Direction selfFrom,
                                core::agent::TurnIntent selfIntent,
                                Approach::Direction otherFrom,
                                core::agent::TurnIntent otherIntent) {
    if (selfIntent != core::agent::TurnIntent::LEFT) return false;
    if (!oncoming(selfFrom, otherFrom)) return false;
    // Le tourne-a-gauche coupe l'oncoming tout-droit et l'oncoming a droite
    // (sortie partagee). Deux gauches opposees se degagent sans blocage central.
    return otherIntent != core::agent::TurnIntent::LEFT;
}

inline bool movementsConflict(Approach::Direction aFrom,
                              core::agent::TurnIntent aIntent,
                              Approach::Direction bFrom,
                              core::agent::TurnIntent bIntent) {
    if (aFrom == bFrom) return false;

    if (leftTurnMustYieldTo(aFrom, aIntent, bFrom, bIntent) ||
        leftTurnMustYieldTo(bFrom, bIntent, aFrom, aIntent)) {
        return true;
    }

    if (!sameAxis(aFrom, bFrom)) return true;

    const Approach::Direction aExit = exitFor(aFrom, aIntent);
    const Approach::Direction bExit = exitFor(bFrom, bIntent);
    return aExit == bExit;
}

} // namespace core::intersection::conflict
