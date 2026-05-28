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
    float lastAccel_          = 0.f;  // commande integree au dernier pas (feed-forward CACC)
    core::behavior::IdmModel idm;     // modele car-following par defaut

    // Diagnostic (UI uniquement, pas de logique metier branchee dessus).
    core::agent::BlockReason currentBlockReason = core::agent::BlockReason::NONE;

    // Instrumentation : trace de la decision (export CSV des blocages). Rempli a
    // chaque computeDecision. AUCUNE logique metier ne s'y branche.
    int   dbgLeaderSrc_           = 0;     // 0 aucun, 1 perception, 2 filet, 3 policy
    int   dbgLeaderVin_           = -1;    // VIN du leader retenu (si vehicule)
    float dbgLeaderRelHeadingDeg_ = 999.f; // cap leader - mon cap ; |.|<45 meme sens,
                                           // <=135 croise, >135 contre-sens
    float dbgLeaderGap_           = -1.f;
    float dbgLeaderSpeed_         = 0.f;
    bool  dbgOnInter_             = false; // physiquement sur l'aire d'un carrefour

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

    // Anti-gridlock "keep clear" : temps cumule d'attente a la ligne parce que la
    // SORTIE du carrefour est occupee. Borne (cf. computeDecision) pour garantir
    // la liveness -> passe le seuil, on cede au bris de cycle par VIN.
    float keepClearWaited_          = 0.f;

    // STOP : a-t-on deja, durant CETTE approche, recu un shouldStop de la policy ?
    // Sert a distinguer l'axe MAJEUR (jamais shouldStop -> aucun halt protocol)
    // de l'axe MINEUR (qui doit marquer l'arret). Sans ce flag, l'axe majeur
    // declenchait par erreur stopForceHalt parce que stopLineGap defaut = 0 px
    // < kHaltZone -> "atLine" -> halt requis. Reset par changement d'intersection.
    bool  stopEverYielded_          = false;

    // --- Deadlock auto-recovery -----------------------------------------------
    // Cycle rotatif non resolu (A cede a B, B a C, C a A) : meme apres bris VIN,
    // certains scenarios denses produisent un gel. Filet de DERNIER recours :
    // si je suis immobile depuis > kDeadlockGrace ET ma voie est physiquement
    // libre devant moi (aucun corps projete sur ma trajectoire), on force un
    // creep d'acceleration pour casser le cycle. Non declenche pour les attentes
    // legitimes (feu rouge, STOP a marquer) -- on identifie via BlockReason.
    float deadlockStuckTime_        = 0.f;

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

    // VIN stable assigne a la construction (compteur global deterministe :
    // l'ordre de spawn est fixe -> reproductible). Sert au bris d'egalite P2P
    // et de cle de suivi pour les metriques.
    int vehicleId_ = -1;

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

    // Snapshot de diagnostic decisionnel (export CSV des blocages). Tout est
    // capture a la derniere computeDecision -> aucun pointeur pendant.
    struct DecisionDiagnostic {
        int   vin;
        float x, y, headingDeg, speed;
        int   blockReason;            // (int) core::agent::BlockReason
        int   leaderSource;           // 0 aucun, 1 perception, 2 filet, 3 policy
        int   leaderVin;
        float leaderRelHeadingDeg;    // 999 si pas de leader-vehicule
        float leaderGap, leaderSpeed;
        bool  onIntersection;
    };
    DecisionDiagnostic getDecisionDiagnostic() const;

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

    int getVehicleId() const override { return vehicleId_; }
    core::agent::TurnIntent getTurnIntent() const override;
    float getCurrentAccel() const override { return lastAccel_; }

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

    // Heterogeneite gaussienne (robustesse Monte-Carlo du rapport) : bruite le
    // temps de reaction T, l'envie d'accelerer et la conformite a la limite
    // autour de leur valeur courante (multiplicateur ~ N(1, sigma)).
    void applyGaussianHeterogeneity(core::Rng& rng, float sigma);

    // Etat panne (UI).
    bool  brokenDown() const { return isBroken; }
    float repairTimeRemaining() const { return breakdownTimer; }
    void  forceBreakdown(float seconds);  // outil debug UI

    // Force la fin de panne immediate (outil debug).
    void  forceRepair() { isBroken = false; breakdownTimer = 0.f; }
};
