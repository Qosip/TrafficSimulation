// src/core/intersection/OrcaPolicy.cpp
#include "core/intersection/OrcaPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/agent/IAgent.hpp"
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

bool isHorizontal(float headingDeg) {
    const float h = std::abs(math::wrapDeg180(headingDeg));
    // ~0 (EST) ou ~180 (OUEST) -> axe horizontal ; ~90/270 -> vertical.
    return (h < 45.f) || (h > 135.f);
}

// Deux trajectoires sont en conflit si elles sont sur des axes perpendiculaires
// (trafic croise). Les flux paralleles (meme axe) sont geres par le car-following.
bool axesConflict(float headingA, float headingB) {
    return isHorizontal(headingA) != isHorizontal(headingB);
}

} // namespace

Decision OrcaPolicy::request(const PolicyContext& ctx, const Intersection& inter) const {
    Decision d;
    // Donnees reseau manquantes (pas de voisinage observable) -> on ne peut
    // arbitrer aucun conflit : repli SUR sur le passage libre (le car-following
    // longitudinal reste actif cote Vehicle). Idem pointeurs nuls plus bas.
    if (!ctx.others) { d.canEnter = true; return d; }

    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float bufferToLine = 25.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - bufferToLine);

    // Loin : approche libre, pas encore d'arbitrage d'evitement.
    if (distToCenter > params_.claimDistance) { d.canEnter = true; return d; }

    const float myEff     = std::max(ctx.self.speed, 1.f);
    const float myArrive  = std::max(0.f, distToCenter - interHalf) / myEff;   // s avant la boite
    const int   myVin     = ctx.selfAgent ? ctx.selfAgent->getVehicleId() : -1;
    const float myHeading = ctx.self.heading;

    bool  mustYield      = false;   // au moins un conflit me domine
    bool  hardStop       = false;   // conflit imminent -> arret FERME
    float dominatingTArr = std::numeric_limits<float>::infinity();  // t d'arrivee du + contraignant

    for (const auto& other : *ctx.others) {
        if (!other) continue;                                // paquet perdu -> ignore
        if (other.get() == ctx.selfAgent) continue;          // self-skip obligatoire

        const float oHeading = other->getHeading();
        if (!axesConflict(myHeading, oHeading)) continue;    // pas un conflit croise

        const Vec2  oPos   = other->getPosition();
        const float oDist  = (oPos - center).length();
        if (oDist > params_.gap.scanRadius) continue;
        const float oSpeed = other->getSpeed();

        // Deja DANS la boite -> il occupe l'espace : on doit l'eviter franchement.
        bool insideInter = false;
        for (const auto& t : inter.getCoveredTiles()) {
            const Vec2 tc{ t.x * ctx.tileSize + ctx.tileSize / 2.f,
                           t.y * ctx.tileSize + ctx.tileSize / 2.f };
            if ((oPos - tc).length() < params_.gap.insideRadius) { insideInter = true; break; }
        }
        if (insideInter) {
            mustYield = true;
            hardStop  = true;
            dominatingTArr = 0.f;
            break;
        }

        // Se dirige-t-il vers le carrefour ? (sinon il degage, pas une menace)
        const float oRad = oHeading * math::DEG2RAD;
        const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
        Vec2 toInter = center - oPos;
        const float lenToInter = toInter.length();
        if (lenToInter > 1.f) {
            toInter /= lenToInter;
            if (dot(oDir, toInter) <= 0.2f) continue;        // s'eloigne
        }

        const float oEff    = std::max(oSpeed, 1.f);
        const float oArrive = std::max(0.f, oDist - interHalf) / oEff;
        const bool  oStopped = oSpeed <= params_.stoppedEps;

        // --- Arbitrage reciproque par temps d'arrivee ---
        bool otherDominates;
        if (std::abs(oArrive - myArrive) > params_.tieMarginSec) {
            otherDominates = (oArrive < myArrive);           // arrive avant -> priorite
        } else {
            // Egalite : bris deterministe par VIN. Le plus GRAND VIN cede (et
            // le plus petit passe) -> jamais de double-cede ni de double-passage.
            otherDominates = (myVin < 0) ||
                             (other->getVehicleId() >= 0 && myVin > other->getVehicleId());
        }
        if (!otherDominates) continue;

        mustYield = true;
        dominatingTArr = std::min(dominatingTArr, oArrive);
        // Conflit imminent : l'autre arrive tres tot, ou est arrete EN TRAVERS
        // tout pres -> on ne peut pas s'entrelacer, arret ferme.
        if (oArrive < params_.hardStopArrival || (oStopped && oDist < interHalf + bufferToLine))
            hardStop = true;
    }

    if (!mustYield) {
        d.canEnter = true;
        return d;
    }

    if (hardStop) {
        // Dernier recours : arret FERME a la ligne (point fixe).
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
        d.yieldUntilT = std::isfinite(dominatingTArr) ? dominatingTArr : 0.f;
        return d;
    }

    // Cede SOUPLE (coeur ORCA) : on ne s'arrete pas, on suit un meneur virtuel
    // MOBILE place a la ligne et avancant a une fraction de notre allure. L'IDM
    // nous fait ralentir juste ce qu'il faut pour ceder le passage puis nous
    // reinsere -> effet d'entrelacement continu. canEnter reste false pour ne
    // pas armer le commit-to-pass, mais aucun arret franc n'est demande.
    d.canEnter            = false;
    d.shouldStop          = false;
    d.followVirtualLeader = true;
    d.virtualLeaderGap    = stopLineGap;
    d.virtualLeaderSpeed  = ctx.self.speed * params_.softSlowFactor;
    d.yieldUntilT         = std::isfinite(dominatingTArr) ? dominatingTArr : 0.f;
    return d;
}

} // namespace core::intersection
