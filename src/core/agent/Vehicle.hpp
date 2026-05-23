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
#include "core/behavior/ICarFollowingModel.hpp"
#include "core/behavior/IdmModel.hpp"
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

    core::TileCoord startTile;
    core::TileCoord goalTile;

    void rebuildLaneFromPath(const std::vector<core::TileCoord>& tilePath);

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

    void setPath(const std::vector<core::TileCoord>& newPath);
    void recalculatePath(const World& world) override;
    void resetToStart(const World& world)    override;

    bool  contains(core::Vec2 point) const override;
    void  setSelected(bool selected) override { isSelectedFlag = selected; }
    bool  isSelected() const          override { return isSelectedFlag; }
    float getRemainingDistance() const override;
    core::agent::BlockReason getBlockReason() const override { return currentBlockReason; }

    // Permet a Car/Truck d'injecter leurs parametres IDM specifiques.
    void setIdmParams(const core::behavior::IdmParams& p) { idm.setParams(p); }
};
