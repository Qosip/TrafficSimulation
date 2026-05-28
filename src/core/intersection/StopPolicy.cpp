// src/core/intersection/StopPolicy.cpp
//
// STOP 2 voies (et NON all-way). Modele : 1 route PRINCIPALE prioritaire + 1
// route SECONDAIRE qui porte le panneau STOP.
//   * Agent sur l'axe PRINCIPAL  -> ne s'arrete jamais (canEnter = true).
//   * Agent sur l'axe SECONDAIRE -> 3 phases :
//       1) Approche  : freine jusqu'a la ligne d'arret.
//       2) Marquage  : arret complet exige a la ligne.
//       3) Insertion : repart des que l'axe principal est LIBRE (aucun
//          vehicule principal engage ou arrivant dans la fenetre de temps).
//
// Plus de "cede a droite" permanent : quand l'axe principal est vide, le
// vehicule secondaire repart immediatement.
#include "core/intersection/StopPolicy.hpp"

#include <algorithm>
#include <cmath>

#include "core/agent/IAgent.hpp"
#include "core/agent/TurnIntent.hpp"
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

// Vrai si la direction d'arrivee est portee par l'axe horizontal (E-O).
bool isHorizontalApproach(Approach::Direction d) {
    return d == Approach::Direction::EAST || d == Approach::Direction::WEST;
}

// Classe un AUTRE vehicule sur l'axe horizontal d'apres son cap (heading deg).
// cos(cap) ~ ±1 => roule horizontalement ; sin(cap) ~ ±1 => verticalement.
bool isHorizontalHeading(float headingDeg) {
    const float r = headingDeg * core::math::DEG2RAD;
    return std::abs(std::cos(r)) >= std::abs(std::sin(r));
}

} // namespace

Decision StopPolicy::request(const PolicyContext& ctx,
                             const Intersection& inter) const
{
    Decision d;

    const bool selfHorizontal = isHorizontalApproach(ctx.self.from);
    const bool selfOnMajor     = (selfHorizontal == inter.isStopMajorAxisHorizontal());
    const Vec2  center       = computeCenter(inter, ctx.tileSize);
    const float interHalf    = inter.getOuterRadius(ctx.tileSize); // demi-cote reel (== tileSize si 2x2)
    const float distToCenter = (center - ctx.self.position).length();
    const float buffer       = 12.f + ctx.self.length / 2.f;
    const float stopLineGap  = std::max(0.f, distToCenter - interHalf - buffer);

    // --- Axe principal : prioritaire, sauf tourne-a-gauche non protege face
    //     a l'oncoming prioritaire tout droit/a droite. --------------------
    if (selfOnMajor) {
        const bool turningLeft =
            ctx.selfAgent &&
            ctx.selfAgent->getTurnIntent() == core::agent::TurnIntent::LEFT;
        if (turningLeft && ctx.others) {
            const float myEff   = std::max(ctx.self.speed, 30.f);
            const float myEnter = stopLineGap / myEff;
            for (const auto& other : *ctx.others) {
                if (!other || other.get() == ctx.selfAgent) continue;
                if (isHorizontalHeading(other->getHeading()) != inter.isStopMajorAxisHorizontal())
                    continue;
                if (other->getTurnIntent() == core::agent::TurnIntent::LEFT) continue;

                const float dH = std::abs(core::math::wrapDeg180(
                    other->getHeading() - ctx.self.heading));
                if (dH < 135.f) continue;

                const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
                const float dO = (oPos - center).length();
                if (dO > params_.gap.scanRadius) continue;

                const float hr = other->getHeading() * core::math::DEG2RAD;
                const Vec2  vel{ std::cos(hr), std::sin(hr) };
                Vec2 toCenter = center - oPos;
                const float len = toCenter.length();
                if (len > 1.f) {
                    toCenter /= len;
                    if (core::dot(vel, toCenter) <= 0.2f) continue;
                }

                const float oSpeed = std::max(other->getSpeed(), 1.f);
                const float tEnter = std::max(0.f, dO - interHalf) / oSpeed;
                if (tEnter <= myEnter + params_.gap.minCrossingTime +
                              params_.gap.safetyMargin) {
                    d.canEnter    = false;
                    d.shouldStop  = true;
                    d.stopLineGap = stopLineGap;
                    d.yieldUntilT = tEnter;
                    return d;
                }
            }
        }

        d.canEnter   = true;
        d.shouldStop = false;
        return d;
    }

    // --- Axe secondaire : panneau STOP. ----------------------------------

    // Phase 1 : approche -> freine jusqu'a la ligne.
    //
    // IMPORTANT (couplage avec Vehicle::computeDecision) : ce seuil DOIT etre
    // >= kHaltZone (16 px) du Vehicle. Sinon le Vehicle declenche son arret sec
    // (stopForceHalt) ALORS qu'on est encore en phase 1 (canEnter toujours
    // false, sans test de degagement) -> le vehicule se fige et ne repart
    // jamais. En gardant la zone "a la ligne" plus large que kHaltZone, l'arret
    // sec survient dans la phase 3 ci-dessous, qui evalue le degagement et
    // libere des que l'axe principal est sur.
    constexpr float kAtLineZone = 18.f;   // >= Vehicle kHaltZone (16)
    if (stopLineGap > kAtLineZone) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
        return d;
    }

    // Phase 2 : a la ligne mais encore en mouvement -> arret complet exige.
    if (ctx.self.speed > params_.haltSpeedEpsilon) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = 0.f;
        return d;
    }

    // Phase 3 : arret marque -> insertion des que c'est SUR. On ne cede qu'aux
    // vehicules de l'axe PRINCIPAL :
    //   (a) deja dans/au bord de la boite                       -> conflit ;
    //   (b) en approche arrivant AVANT que J'AIE FINI de degager -> conflit.
    // Le temps de degagement tient compte de MA distance a parcourir (entrer +
    // traverser + degager ma longueur) et de MON acceleration depuis l'arret :
    // un vehicule lent/long exige un creneau plus grand -> on passe sans crash.
    // Si l'axe est libre dans cette fenetre, on repart IMMEDIATEMENT.
    const float dangerDist = interHalf + ctx.tileSize * 0.4f;   // boite + marge

    // Temps qu'il me faut pour degager le carrefour depuis l'arret.
    const float egoAccel  = (ctx.self.accel > 1.f) ? ctx.self.accel : 60.f;
    const float crossDist = distToCenter + interHalf + ctx.self.length;  // jusqu'a sortir cote oppose
    const float tClear    = std::sqrt(2.f * crossDist / (egoAccel * 0.85f));
    const float safeTime  = tClear + 0.8f;                      // + marge de securite

    bool conflict = false;
    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;

        // On ignore tout ce qui n'est PAS sur l'axe principal.
        if (isHorizontalHeading(other->getHeading()) != inter.isStopMajorAxisHorizontal())
            continue;

        const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
        const float dO = (oPos - center).length();
        // Borne large : meme rapide (≈350 px/s), au-dela il ne peut pas
        // atteindre la boite pendant mon temps de degagement -> sans risque.
        if (dO > safeTime * 350.f + dangerDist) continue;

        // (a) vehicule principal engage / tout proche de la boite.
        if (dO < dangerDist) { conflict = true; break; }

        // (b) vehicule principal en approche : arrive-t-il avant la fin de mon
        //     degagement ? (sa vitesse propre est prise en compte).
        const Vec2  toCenter = (center - oPos);
        const float len      = toCenter.length();
        if (len < 1.f) { conflict = true; break; }
        const float hr  = other->getHeading() * core::math::DEG2RAD;
        const Vec2  vel { std::cos(hr), std::sin(hr) };
        const float approaching = core::dot(vel, toCenter / len);   // >0 => se rapproche
        const float oSpeed = other->getSpeed();
        if (approaching > 0.25f && oSpeed > params_.haltSpeedEpsilon) {
            const float tEnter = (dO - dangerDist) / oSpeed;
            if (tEnter < safeTime) { conflict = true; break; }
        }
    }

    if (conflict) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = 0.f;
        d.yieldUntilT = 1.0f;
        return d;
    }

    d.canEnter   = true;
    d.shouldStop = false;
    return d;
}

} // namespace core::intersection
