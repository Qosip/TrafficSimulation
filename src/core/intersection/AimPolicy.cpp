// src/core/intersection/AimPolicy.cpp
#include "core/intersection/AimPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/intersection/ConflictGeometry.hpp"
#include "core/intersection/Intersection.hpp"
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

} // namespace

Decision AimPolicy::request(const PolicyContext& ctx, const Intersection& inter) const {
    Decision d;
    const float now = inter.now();

    // Purge des reservations expirees (le vehicule a quitte la boite).
    for (auto it = reservations_.begin(); it != reservations_.end(); ) {
        if (it->second.tExitAbs < now) it = reservations_.erase(it);
        else                           ++it;
    }

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float bufferToLine = 25.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - bufferToLine);

    // Fenetre de reservation demandee, en temps ABSOLU.
    const Approach::Direction myFrom = ctx.self.from;
    const core::agent::TurnIntent myIntent =
        ctx.selfAgent ? ctx.selfAgent->getTurnIntent()
                      : core::agent::TurnIntent::UNKNOWN;
    const float eff       = std::max(ctx.self.speed, 30.f);
    const float tEnterRel = stopLineGap / eff;
    const float tCross    = std::max(params_.minCrossingTime,
                                     params_.crossingDistance / eff);
    const float tEnterAbs = now + tEnterRel;
    const float tExitAbs  = now + tEnterRel + tCross + params_.safetyMargin;

    const int myVin = ctx.selfAgent ? ctx.selfAgent->getVehicleId() : -1;

    // Deja titulaire d'une reservation -> on la rafraichit et on passe (FCFS :
    // une fois accorde, le droit est conserve jusqu'a la traversee).
    if (myVin >= 0) {
        auto it = reservations_.find(myVin);
        if (it != reservations_.end()) {
            it->second.tEnterAbs   = tEnterAbs;
            it->second.tExitAbs    = tExitAbs;
            it->second.from        = myFrom;
            it->second.intent      = myIntent;
            it->second.lastSeenAbs = now;
            d.canEnter = true;
            return d;
        }
    }

    // Conflit = trajectoire geometriquement incompatible dont la fenetre
    // chevauche la mienne. Les trajectoires paralleles compatibles coexistent.
    bool conflict = false;
    for (const auto& kv : reservations_) {
        if (kv.first == myVin) continue;
        const Slot& s = kv.second;
        if (!conflict::movementsConflict(myFrom, myIntent, s.from, s.intent))
            continue;
        const float overlapStart = std::max(tEnterAbs, s.tEnterAbs);
        const float overlapEnd   = std::min(tExitAbs,  s.tExitAbs);
        if (overlapEnd > overlapStart) { conflict = true; break; }
    }

    if (!conflict) {
        // FCFS : le premier demandeur d'un creneau libre le reserve.
        if (myVin >= 0)
            reservations_[myVin] = Slot{ tEnterAbs, tExitAbs, myFrom, myIntent, now };
        d.canEnter   = true;
        d.shouldStop = false;
    } else {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
    }
    return d;
}

} // namespace core::intersection
