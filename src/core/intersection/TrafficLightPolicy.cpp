// src/core/intersection/TrafficLightPolicy.cpp
#include "core/intersection/TrafficLightPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"  // (transitif pour pointeur)
#include "core/agent/TurnIntent.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"   // wrapDeg180, DEG2RAD
#include "core/math/Vec2.hpp"

namespace core::intersection {

namespace {

// Centre geometrique de l'intersection en coordonnees monde.
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

Decision TrafficLightPolicy::request(const PolicyContext& ctx,
                                     const Intersection& inter) const
{
    Decision d;
    const LightState state = inter.getLightState(ctx.self.from);

    const Vec2 center      = computeCenter(inter, ctx.tileSize);
    const Vec2 toCenter    = center - ctx.self.position;
    const float distCenter = toCenter.length();

    // Distance approximative de la "ligne d'arret" : bord proche de l'intersection.
    const float interHalf = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float buffer    = params_.buffer + ctx.self.length / 2.f;
    const float stopGap   = std::max(0.f, distCenter - interHalf - buffer);

    if (state == LightState::GREEN) {
        // Tourne-a-gauche non protege. Les deux approches OPPOSEES partagent le
        // meme vert -> un tourne-a-GAUCHE croise le flux tout-droit venant d'en
        // face et doit lui ceder. Tout-droit / tourne-a-droite ne croisent pas
        // l'oncoming -> passage libre. Avant, seul le filet anti-collision (VIN)
        // tranchait, par VIN et non par priorite reelle : le mauvais vehicule
        // pouvait ceder. La cession est bornee par le cycle du feu (l'oncoming
        // passe au rouge a la phase suivante) -> jamais de famine permanente.
        const bool turningLeft =
            ctx.selfAgent &&
            ctx.selfAgent->getTurnIntent() == core::agent::TurnIntent::LEFT;
        if (turningLeft && ctx.others) {
            const float myEff   = std::max(ctx.self.speed, 30.f);
            const float myEnter = stopGap / myEff;       // ~temps pour atteindre la ligne
            for (const auto& other : *ctx.others) {
                if (!other || other.get() == ctx.selfAgent) continue;

                // Sens oppose (oncoming) ? cap a ~180 du mien.
                const float dH =
                    std::abs(math::wrapDeg180(other->getHeading() - ctx.self.heading));
                if (dH < 135.f) continue;

                // L'oncoming TOUT-DROIT ou a DROITE barre mon tourne-a-gauche
                // (a droite : sortie partagee). Deux gauches opposees se degagent.
                // UNKNOWN = prudence -> traite comme conflictuel.
                const core::agent::TurnIntent oi = other->getTurnIntent();
                if (oi == core::agent::TurnIntent::LEFT) continue;

                const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
                const float oDist = (oPos - center).length();
                if (oDist > params_.oncomingScan) continue;

                // Se dirige-t-il bien vers le carrefour ?
                const float oRad = other->getHeading() * math::DEG2RAD;
                const Vec2  oDir{ std::cos(oRad), std::sin(oRad) };
                Vec2 toC = center - oPos;
                const float lenC = toC.length();
                if (lenC > 1.f) { toC /= lenC; if (dot(oDir, toC) <= 0.2f) continue; }

                // Atteint-il la zone de conflit avant que j'aie fini de traverser ?
                const float oEff     = std::max(other->getSpeed(), 1.f);
                const float oToBound = std::max(0.f, oDist - interHalf);
                const float oEnter   = oToBound / oEff;
                if (oEnter <= std::max(params_.leftYieldTime, myEnter)) {
                    d.canEnter    = false;
                    d.shouldStop  = true;
                    d.stopLineGap = stopGap;
                    return d;
                }
            }
        }

        d.canEnter   = true;
        d.shouldStop = false;
        return d;
    }

    if (state == LightState::ORANGE) {
        // Distance de freinage confortable a la vitesse courante : v^2 / (2 b).
        const float v          = std::max(0.f, ctx.self.speed);
        const float brakingDist = (v * v) / (2.f * std::max(1.f, params_.comfortDecel));
        // Si le freinage confortable ne tient pas dans le gap restant, on passe
        // (sinon arret d'urgence = blocage des suivants).
        if (brakingDist > stopGap) {
            d.canEnter   = true;
            d.shouldStop = false;
            return d;
        }
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopGap;
        return d;
    }

    // RED.
    d.canEnter    = false;
    d.shouldStop  = true;
    d.stopLineGap = stopGap;
    return d;
}

} // namespace core::intersection
