// src/core/intersection/StopPolicy.cpp
//
// STOP (all-way) : 3 phases.
//   1) Approche -> shouldStop tant que stopGap > 5 px.
//   2) Marquage -> shouldStop tant qu'on n'est pas a l'arret complet a la ligne.
//   3) Engagement spatial SANS interblocage : on cede aux vehicules dans la
//      boite ou PLUS PROCHES du centre, tie-break stable a egalite, et on
//      IGNORE le trafic encore loin (il devra s'arreter aussi). Garantit
//      toujours un gagnant unique -> personne ne reste bloque indefiniment.
#include "core/intersection/StopPolicy.hpp"

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

} // namespace

Decision StopPolicy::request(const PolicyContext& ctx,
                              const Intersection& inter) const
{
    Decision d;

    const Vec2  center        = computeCenter(inter, ctx.tileSize);
    const float interHalf     = ctx.tileSize;
    const float distToCenter  = (center - ctx.self.position).length();
    const float buffer        = 25.f + ctx.self.length / 2.f;
    const float stopLineGap   = std::max(0.f, distToCenter - interHalf - buffer);

    // Phase 1 : approche -> brake jusqu'a la ligne.
    if (stopLineGap > 5.f) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = stopLineGap;
        return d;
    }

    // Phase 2 : a la ligne mais encore en mouvement -> force halt.
    if (ctx.self.speed > params_.haltSpeedEpsilon) {
        d.canEnter    = false;
        d.shouldStop  = true;
        d.stopLineGap = 0.f;
        return d;
    }

    // Phase 3 : arret marque -> engagement facon "all-way stop", robuste et
    // SANS interblocage. La regle est purement spatiale (qui est le plus proche
    // de la boite passe), ce qui garantit toujours un gagnant unique :
    //   * on cede a tout vehicule deja DANS le carrefour (ou en approche immediate) ;
    //   * on cede a tout vehicule PLUS PROCHE du centre que nous (il s'engage avant) ;
    //   * a distance ~egale, un seul gagne via tie-break stable (X puis Y) ;
    //   * on IGNORE le trafic encore loin : en all-way stop il devra s'arreter
    //     lui aussi. (C'etait LA cause du blocage : on cedait au flux transversal
    //     encore en mouvement, donc plus personne ne repartait.)
    constexpr float eps        = 1.0f;                       // px : tolerance "egalite"
    const float     insideDist = interHalf + ctx.tileSize * 0.3f;

    bool conflict = false;
    for (const auto& other : *ctx.others) {
        if (!other) continue;
        if (other.get() == ctx.selfAgent) continue;

        const Vec2  oPos{ other->getPosition().x, other->getPosition().y };
        const float dO = (oPos - center).length();
        if (dO > params_.gap.scanRadius) continue;

        if (dO < insideDist)            { conflict = true; break; } // dans la boite
        if (dO < distToCenter - eps)    { conflict = true; break; } // plus proche -> prioritaire

        if (dO <= distToCenter + eps) {                              // ~egalite : tie-break
            const bool iWin =
                (ctx.self.position.x <  oPos.x - eps) ||
                (std::abs(ctx.self.position.x - oPos.x) <= eps &&
                 ctx.self.position.y <  oPos.y - eps);
            if (!iWin) { conflict = true; break; }
        }
        // sinon : plus loin -> ignore (il s'arretera aussi).
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
