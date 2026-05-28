// src/core/intersection/PlatooningPolicy.cpp
#include "core/intersection/PlatooningPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/intersection/ConflictGeometry.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"
#include "core/math/Vec2.hpp"

namespace core::intersection {

namespace {

Vec2 computeCenter(const Intersection& inter, float tileSize) {
    const auto& tiles = inter.getCoveredTiles();
    if (tiles.empty()) return {0.f, 0.f};
    Vec2 c{0.f, 0.f};
    for (const auto& t : tiles) {
        c.x += t.x * tileSize + tileSize / 2.f;
        c.y += t.y * tileSize + tileSize / 2.f;
    }
    return c / static_cast<float>(tiles.size());
}

float expectedHeadingFor(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::WEST:  return 0.f;
        case Approach::Direction::EAST:  return 180.f;
        case Approach::Direction::NORTH: return 90.f;
        case Approach::Direction::SOUTH: return 270.f;
    }
    return 0.f;
}

Approach::Direction directionFromHeading(float headingDeg) {
    const Approach::Direction dirs[4] = {
        Approach::Direction::NORTH, Approach::Direction::SOUTH,
        Approach::Direction::EAST,  Approach::Direction::WEST
    };
    Approach::Direction best = Approach::Direction::NORTH;
    float bestErr = 1e9f;
    for (auto dD : dirs) {
        const float err = std::abs(math::wrapDeg180(headingDeg - expectedHeadingFor(dD)));
        if (err < bestErr) { bestErr = err; best = dD; }
    }
    return best;
}

bool isHorizontalAxis(Approach::Direction d) {
    return d == Approach::Direction::EAST || d == Approach::Direction::WEST;
}

bool axesConflict(Approach::Direction a, Approach::Direction b) {
    return isHorizontalAxis(a) != isHorizontalAxis(b);
}

} // namespace

Decision PlatooningPolicy::request(const PolicyContext& ctx, const Intersection& inter) const {
    Decision d;
    d.canEnter = true;   // par defaut on ne s'arrete jamais (peloton fluide)
    if (!ctx.others) return d;

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float distToCenter = (center - ctx.self.position).length();

    // Hors de la zone d'amorce : on roule librement (le car-following reel reste
    // actif cote Vehicle).
    if (distToCenter > params_.engageDistance) return d;

    // Mon temps d'arrivee au point de conflit.
    const float myEff    = std::max(ctx.self.speed, 25.f);
    const float myTEnter = distToCenter / myEff;
    const int   myVin    = ctx.selfAgent ? ctx.selfAgent->getVehicleId() : -1;

    const Approach::Direction myFrom = ctx.self.from;
    const core::agent::TurnIntent myIntent =
        ctx.selfAgent ? ctx.selfAgent->getTurnIntent()
                      : core::agent::TurnIntent::UNKNOWN;

    // Cherche mon predecesseur immediat sur l'axe virtuel : le vehicule croise
    // qui arrive JUSTE AVANT moi (plus grand tEnter encore inferieur au mien).
    const IAgent* leaderAgent = nullptr;
    float bestTEnter   = -std::numeric_limits<float>::infinity();
    float leaderSpeed  = 0.f;
    float leaderTEnter = 0.f;

    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;

        const Approach::Direction oFrom =
            conflict::directionFromHeading(other->getHeading());
        if (!conflict::movementsConflict(myFrom, myIntent,
                                         oFrom, other->getTurnIntent())) {
            continue;                                       // trajectoires compatibles
        }

        const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
        const float oDist = (oPos - center).length();
        if (oDist > params_.gap.scanRadius) continue;

        // Se dirige-t-il vers le carrefour ?
        const float oRad = other->getHeading() * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        Vec2 toInter = center - oPos;
        const float lenToInter = toInter.length();
        if (lenToInter > 1.f) {
            toInter /= lenToInter;
            if (dot(oDir, toInter) <= 0.2f) continue;        // s'eloigne
        }

        const float oEff    = std::max(other->getSpeed(), 1.f);
        const float oTEnter = oDist / oEff;

        // Predecesseur = arrive avant moi (bris d'egalite par VIN en cas
        // d'arrivee quasi simultanee, pour eviter deux "tetes" qui se percutent),
        // et le plus tardif parmi eux (mon voisin immediat sur l'axe virtuel).
        const bool before = (oTEnter < myTEnter - 0.05f) ||
                            (std::abs(oTEnter - myTEnter) <= 0.05f &&
                             other->getVehicleId() >= 0 &&
                             (myVin < 0 || other->getVehicleId() < myVin));
        if (before && oTEnter > bestTEnter) {
            bestTEnter   = oTEnter;
            leaderAgent  = other.get();
            leaderSpeed  = other->getSpeed();
            leaderTEnter = oTEnter;
        }
    }

    if (!leaderAgent) {
        // Je suis en tete sur l'axe virtuel -> je passe sans contrainte.
        return d;
    }

    // Meneur virtuel MOBILE projete sur ma trajectoire. L'ecart temporel
    // (myTEnter - leaderTEnter) est converti en distance le long de ma voie,
    // + une marge de degagement. IDM suit ce meneur -> je m'insere derriere.
    const float dtGap   = std::max(0.f, myTEnter - leaderTEnter);
    const float convSpd = std::max(myEff, std::max(leaderSpeed, 1.f));
    d.followVirtualLeader = true;
    d.virtualLeaderGap    = params_.clearancePx + dtGap * convSpd;
    d.virtualLeaderSpeed  = leaderSpeed;
    return d;
}

} // namespace core::intersection
