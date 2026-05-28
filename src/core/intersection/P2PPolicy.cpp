// src/core/intersection/P2PPolicy.cpp
#include "core/intersection/P2PPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/agent/TurnIntent.hpp"
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

// Cap (degres) attendu pour un vehicule arrivant DEPUIS cette direction.
// Convention ecran : x+ = EST, y+ = SUD.
float expectedHeadingFor(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::WEST:  return 0.f;
        case Approach::Direction::EAST:  return 180.f;
        case Approach::Direction::NORTH: return 90.f;
        case Approach::Direction::SOUTH: return 270.f;
    }
    return 0.f;
}

// Direction d'ou vient un vehicule, deduite de son cap (plus proche des 4 axes).
Approach::Direction directionFromHeading(float headingDeg) {
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

// Direction "a ma droite" depuis MA BRANCHE D'ORIGINE.
// Convention ecran : x+ = EST, y+ = SUD. Un vehicule arrivant DEPUIS le NORD
// roule vers le SUD : son cote droit pointe vers l'OUEST (rotation +90deg du
// heading en coordonnees ecran). Idem pour les autres approches.
Approach::Direction rightOf(Approach::Direction from) {
    switch (from) {
        case Approach::Direction::NORTH: return Approach::Direction::WEST;
        case Approach::Direction::SOUTH: return Approach::Direction::EAST;
        case Approach::Direction::EAST:  return Approach::Direction::NORTH;
        case Approach::Direction::WEST:  return Approach::Direction::SOUTH;
    }
    return Approach::Direction::NORTH;
}

bool isHorizontalAxis(Approach::Direction d) {
    return d == Approach::Direction::EAST || d == Approach::Direction::WEST;
}

// Conflit geometrique : axes perpendiculaires (trafic croise). On ignore les
// flux paralleles (meme axe) : voies opposees = couloirs separes, suivi de
// file = gere par le car-following, pas par l'arbitrage d'intersection.
bool axesConflict(Approach::Direction a, Approach::Direction b) {
    return isHorizontalAxis(a) != isHorizontalAxis(b);
}

} // namespace

P2PState P2PPolicy::stateFor(const PolicyContext& ctx, const Intersection& inter) const {
    // Deja physiquement dans l'intersection ? -> TRAVERSAL.
    for (const auto& t : inter.getCoveredTiles()) {
        const Vec2 tc{ t.x * ctx.tileSize + ctx.tileSize / 2.f,
                       t.y * ctx.tileSize + ctx.tileSize / 2.f };
        if ((ctx.self.position - tc).length() < ctx.tileSize * 0.75f) return P2PState::TRAVERSAL;
    }
    const Vec2  center = computeCenter(inter, ctx.tileSize);
    const float dist   = (center - ctx.self.position).length();
    return (dist > params_.claimDistance) ? P2PState::LURKING : P2PState::CLAIMING;
}

Decision P2PPolicy::request(const PolicyContext& ctx, const Intersection& inter) const {
    Decision d;
    if (!ctx.others) { d.canEnter = true; return d; }

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float bufferToLine = 25.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - bufferToLine);

    // LURKING : trop loin pour revendiquer -> on approche librement. Le
    // car-following (leader reel) reste actif cote Vehicle ; on ne fait que
    // ne PAS encore arbitrer le conflit d'intersection.
    if (distToCenter > params_.claimDistance) { d.canEnter = true; return d; }

    // --- Mon "Claim" ---
    const float myEff      = std::max(ctx.self.speed, 30.f);
    const float myEnter    = stopLineGap / myEff;
    const float myCross    = std::max(params_.gap.minCrossingTime,
                                      params_.gap.crossingDistance / myEff);
    const float myClear    = myEnter + myCross + params_.gap.safetyMargin;
    const float myExit     = myEnter + myCross;
    const bool  myStopped  = ctx.self.speed <= params_.stoppedEps;
    const bool  myTurning  = (ctx.selfAgent &&
                              core::agent::isTurning(ctx.selfAgent->getTurnIntent()));
    const int   myVin      = ctx.selfAgent ? ctx.selfAgent->getVehicleId() : -1;
    const Approach::Direction myFrom = ctx.self.from;

    bool  yieldHard      = false;   // domine par un mouvant / intra-inter / regle 1,3,4
    bool  yieldRightHand = false;   // domine UNIQUEMENT par la priorite a droite
    int   minStoppedVin  = std::numeric_limits<int>::max();
    float worstTArrive   = std::numeric_limits<float>::infinity();

    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;          // self-skip obligatoire

        const Approach::Direction oFrom =
            conflict::directionFromHeading(other->getHeading());
        const core::agent::TurnIntent oIntent = other->getTurnIntent();
        if (!conflict::movementsConflict(myFrom, ctx.selfAgent
                                                  ? ctx.selfAgent->getTurnIntent()
                                                  : core::agent::TurnIntent::UNKNOWN,
                                         oFrom, oIntent)) {
            continue;                                       // trajectoires compatibles
        }

        const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
        const float oDist = (oPos - center).length();
        if (oDist > params_.gap.scanRadius) continue;

        const float oSpeed = other->getSpeed();

        // Vehicule deja DANS l'intersection -> il a la priorite de fait.
        bool insideInter = false;
        for (const auto& t : inter.getCoveredTiles()) {
            const Vec2 tc{ t.x * ctx.tileSize + ctx.tileSize / 2.f,
                           t.y * ctx.tileSize + ctx.tileSize / 2.f };
            if ((oPos - tc).length() < params_.gap.insideRadius) { insideInter = true; break; }
        }
        if (insideInter) { yieldHard = true; worstTArrive = 0.f; break; }

        // Loin et arrete -> pas une menace.
        if (oDist > params_.gap.ignoreDistance && oSpeed < 5.f) continue;

        // Se dirige-t-il vers l'intersection ?
        const float oRad = other->getHeading() * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        Vec2 toInter = center - oPos;
        const float lenToInter = toInter.length();
        if (lenToInter > 1.f) {
            toInter /= lenToInter;
            if (dot(oDir, toInter) <= 0.2f) continue;        // s'eloigne
        }

        // --- Son Claim ---
        const float oEff           = std::max(oSpeed, 1.f);
        const float distToBoundary = std::max(0.f, oDist - interHalf);
        const float oEnter         = distToBoundary / oEff;
        const float oCross         = std::max(params_.gap.minCrossingTime,
                                              params_.gap.crossingDistance / oEff);
        const float oClear         = oEnter + oCross;
        const float oExit          = oEnter + oCross;

        // Recouvrement temporel : sans chevauchement, pas de conflit reel.
        const float overlapStart = std::max(myEnter, oEnter);
        const float overlapEnd   = std::min(myClear, oClear);
        if (overlapEnd <= overlapStart) continue;

        // --- Hierarchie de dominance VanMiddlesworth ---
        const bool oStopped = oSpeed <= params_.stoppedEps;
        const bool oTurning = core::agent::isTurning(oIntent);
        const int  oVin     = other->getVehicleId();
        if (oStopped && oVin >= 0) minStoppedVin = std::min(minStoppedVin, oVin);

        bool decided        = false;
        bool otherDominates = false;
        bool viaRightHand   = false;

        if (!myStopped && !oStopped) {
            // Regle 1 : les deux roulent -> plus petit temps de sortie gagne.
            if (std::abs(oExit - myExit) > params_.exitTimeTol) {
                otherDominates = (oExit < myExit);
                decided = true;
            }
        } else if (myStopped && !oStopped) {
            otherDominates = true;  decided = true;  // arrete cede au roulant
        } else if (!myStopped && oStopped) {
            otherDominates = false; decided = true;  // le roulant garde la main
        } else {
            // Regle 2 : les deux arretes -> priorite a droite.
            if (oFrom == conflict::rightOf(myFrom))      { otherDominates = true;  decided = true; viaRightHand = true; }
            else if (myFrom == conflict::rightOf(oFrom)) { otherDominates = false; decided = true; }
        }

        if (!decided && (myTurning != oTurning)) {
            // Regle 3 : rectiligne domine virage.
            otherDominates = (myTurning && !oTurning);
            decided = true;
        }
        if (!decided) {
            // Regle 4 : plus petit VIN gagne (bris d'egalite ultime).
            otherDominates = (oVin >= 0 && (myVin < 0 || oVin < myVin));
            decided = true;
        }

        if (otherDominates) {
            if (viaRightHand) yieldRightHand = true;
            else              yieldHard      = true;
            worstTArrive = std::min(worstTArrive, oEnter);
        }
    }

    // Liveness : la priorite a droite, appliquee a 4 arrivees simultanees,
    // produit un cycle (chacun cede a son voisin de droite -> blocage total).
    // Le rapport signale ce cas comme "regle de la droite inapplicable" et
    // delegue au bris d'egalite. On l'implemente sans information globale :
    // le plus petit VIN parmi les conflictants ARRETES franchit, ce qui libere
    // le cycle et reproduit la liberation sequentielle d'un four-way-stop.
    bool mustYield = yieldHard || yieldRightHand;
    if (mustYield && !yieldHard && myStopped &&
        myVin >= 0 && myVin < minStoppedVin) {
        mustYield = false;
    }

    if (mustYield) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
        d.yieldUntilT = std::isfinite(worstTArrive) ? worstTArrive : 0.f;
    } else {
        d.canEnter   = true;
        d.shouldStop = false;
    }
    return d;
}

} // namespace core::intersection
