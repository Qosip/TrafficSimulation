// src/core/agent/IAgent.hpp
//
// Etape 4 + Wave 5 :
//   - Interface 100% SFML-free.
//   - update() retire au profit du pipeline PERCEPTION -> DECISION -> ACTION
//     en deux phases strictes :
//       1) computeDecision(agents, world)   = lecture seule, calcule pendingAccel
//       2) integrate(dt)                    = mutation, Euler avec dt fixe
//     Cette separation garantit que tous les agents prennent leur decision
//     vis-a-vis du MEME etat global du monde (pas de pollution sequentielle).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/agent/AgentDebugSnapshot.hpp"
#include "core/agent/BlockReason.hpp"
#include "core/Color.hpp"
#include "core/math/TileCoord.hpp"
#include "core/math/Vec2.hpp"

class World;

class IAgent {
public:
    virtual ~IAgent() = default;

    // Phase 1 : perception + decision. Aucune mutation de l'etat global qui
    //           pourrait influencer d'autres agents avant leur propre decision.
    virtual void computeDecision(const std::vector<std::unique_ptr<IAgent>>& agents,
                                  const World& world) = 0;

    // Phase 2 : integration cinematique. dt FIXE (rythme par la boucle simulation).
    virtual void integrate(float dt) = 0;

    // Getters physiques fondamentaux.
    virtual core::Vec2 getPosition() const = 0;
    virtual float      getHeading()  const = 0;
    virtual float      getSpeed()    const = 0;
    virtual float      getLength()   const = 0;

    // Donnees de carrosserie consommees par le renderer.
    virtual core::Vec2  getBodySize() const = 0;
    virtual core::Color getBodyColor() const = 0;

    virtual AgentDebugSnapshot getDebugSnapshot() const = 0;

    // Scenario + edition.
    virtual std::string     getType()        const = 0;
    virtual core::TileCoord getStartTile()   const = 0;
    virtual core::TileCoord getGoalTile()    const = 0;
    virtual core::TileCoord getCurrentTile() const = 0;
    virtual void recalculatePath(const World& world) = 0;
    virtual void resetToStart(const World& world)    = 0;

    // UI : selection + inspection.
    virtual bool  contains(core::Vec2 point) const = 0;
    virtual void  setSelected(bool selected) = 0;
    virtual bool  isSelected() const = 0;
    virtual float getRemainingDistance() const = 0;

    // Diagnostic : pourquoi l'agent freine / est arrete.
    virtual core::agent::BlockReason getBlockReason() const = 0;
};
