// src/core/agent/Vehicle.hpp
//
// Wave 5 :
//   - update() supprime, remplace par computeDecision() + integrate().
//   - Logique car-following deleguee a un ICarFollowingModel (IDM par defaut).
//   - Decision intersection deleguee a IIntersectionPolicy via virtual leader.
#pragma once

#include <memory>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/agent/Personality.hpp"
#include "core/behavior/ICarFollowingModel.hpp"
#include "core/behavior/IdmModel.hpp"
#include "core/math/Rng.hpp"
#include "core/perception/Perception.hpp"
#include "core/world/Lane.hpp"

class Vehicle : public IAgent {
protected:
    core::Vec2  position;
    core::Vec2  velocity;

    float maxSpeed;          // borne superieure du desired speed
    float maxAcceleration;

    core::Vec2  bodySize;
    core::Color bodyColor;

    std::shared_ptr<Lane> currentLane;
    float                 s;

    bool  hasFinishedPath;
    float tileSize;

    float currentAngle;
    float currentSpeed;
    bool  isHeadingInitialized;

    bool             isSelectedFlag = false;
    VisionParams     visionParams;
    PerceptionResult lastPerception;

    // --- Wave 5 : decision/integration split ---
    float pendingAccel        = 0.f;  // sortie de computeDecision
    float pendingDesiredSpeed = 0.f;  // v0 IDM courant (clip limite/virage)
    core::behavior::IdmModel idm;     // modele car-following par defaut

    // Diagnostic (UI uniquement, pas de logique metier branchee dessus).
    core::agent::BlockReason currentBlockReason = core::agent::BlockReason::NONE;

    // Commit de passage : evite qu'un agent freine au milieu de l'intersection
    // si la priorite bascule pendant la traversee.
    bool isCommittedToPass       = false;
    int  committedIntersectionId = -1;
    bool wasOnCommittedInter     = false;  // a-t-on deja ete PHYSIQUEMENT sur l'inter ?

    // --- Wave 6 : Personality / Breakdown / Overtake ---
    core::agent::Personality personality_;
    core::behavior::IdmParams baseIdmParams_;  // IDM "neutre" avant modulation perso
    core::Rng                 rng_;            // RNG par agent, seede par adresse au spawn

    // Etat panne.
    bool  isBroken          = false;
    float breakdownTimer    = 0.f;   // temps restant avant fin de reparation (s)
    float timeSinceLastCheck = 0.f;  // accumule pour cadencer le dice-roll

    // Etat STOP : un agent doit marquer un arret complet (≥ 1s) avant
    // de pouvoir engager la gap-acceptance d'un STOP.
    int   currentStopIntersectionId = -1;
    float stopHeldTime              = 0.f;
    bool  stopReleased              = false;

    // Etat depassement (offset lateral transitoire vs Lane curvilineaire).
    enum class OvertakeState { NONE, OVERTAKING, RETURNING };
    OvertakeState overtakeState  = OvertakeState::NONE;
    float         lateralOffset  = 0.f;          // px, signe (positif = droite du heading)
    float         overtakeTarget = 0.f;          // valeur cible de lateralOffset
    const IAgent* overtakeLeader = nullptr;      // agent qu'on est en train de doubler
    float         overtakeElapsed = 0.f;         // sec depuis debut manoeuvre, anti-eternel
    float         lateralVel_     = 0.f;          // px/s lateral courant -> yaw visuel naturel

    core::TileCoord startTile;
    core::TileCoord goalTile;

    void rebuildLaneFromPath(const std::vector<core::TileCoord>& tilePath,
                             const World* world);
    void applyPersonalityToIdm();
    void updateBreakdown(float dt);
    bool isClearBehindForRestart(const std::vector<std::unique_ptr<IAgent>>& agents) const;
    bool tryStartOvertake(const std::vector<std::unique_ptr<IAgent>>& agents,
                          const World& world,
                          const core::behavior::LeaderInfo& leader);
    // Decision : transitions d'etat (lecture seule du monde).
    void updateOvertakeDecision(const std::vector<std::unique_ptr<IAgent>>& agents,
                                const World& world);
    // Integration : lissage du lateralOffset.
    void updateOvertakeMotion(float dt);
    bool isOncomingLaneFree(const std::vector<std::unique_ptr<IAgent>>& agents,
                            float requiredClearAhead) const;
    // Le creneau de rabattement (devant le leader, sur notre voie) est-il libre ?
    // Evite de doubler quand la place d'arrivee est deja prise (convoi).
    bool isReturnSlotFree(const std::vector<std::unique_ptr<IAgent>>& agents,
                          const core::behavior::LeaderInfo& leader) const;

public:
    Vehicle(float startX, float startY, float tSize = 50.f);
    virtual ~Vehicle() = default;

    void computeDecision(const std::vector<std::unique_ptr<IAgent>>& agents,
                         const World& world) override;
    void integrate(float dt) override;

    core::Vec2 getPosition() const override { return position; }
    float      getHeading()  const override { return currentAngle; }
    float      getSpeed()    const override { return currentSpeed; }
    float      getLength()   const override { return bodySize.x; }

    core::Vec2  getBodySize()  const override { return bodySize; }
    core::Color getBodyColor() const override { return bodyColor; }
    AgentDebugSnapshot getDebugSnapshot() const override;

    std::string     getType()        const override { return "VEHICLE"; }
    core::TileCoord getStartTile()   const override { return startTile; }
    core::TileCoord getGoalTile()    const override { return goalTile; }
    core::TileCoord getCurrentTile() const override;

    void setPath(const std::vector<core::TileCoord>& newPath,
                 const World* world = nullptr);
    void recalculatePath(const World& world) override;
    void resetToStart(const World& world)    override;

    bool  contains(core::Vec2 point) const override;
    void  setSelected(bool selected) override { isSelectedFlag = selected; }
    bool  isSelected() const          override { return isSelectedFlag; }
    float getRemainingDistance() const override;
    core::agent::BlockReason getBlockReason() const override { return currentBlockReason; }

    // Permet a Car/Truck d'injecter leurs parametres IDM specifiques.
    void setIdmParams(const core::behavior::IdmParams& p) {
        baseIdmParams_ = p;
        applyPersonalityToIdm();
    }

    // Personnalite : accesseurs UI + injection programmatique (scenarios, profils).
    core::agent::Personality&       personality()       { return personality_; }
    const core::agent::Personality& personality() const { return personality_; }
    void setPersonality(const core::agent::Personality& p) {
        personality_ = p;
        personality_.clamp();
        applyPersonalityToIdm();
    }

    // Etat panne (UI).
    bool  brokenDown() const { return isBroken; }
    float repairTimeRemaining() const { return breakdownTimer; }
    void  forceBreakdown(float seconds);  // outil debug UI

    // Force la fin de panne immediate (outil debug).
    void  forceRepair() { isBroken = false; breakdownTimer = 0.f; }
};
