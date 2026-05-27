// src/core/agent/Vehicle.cpp
//
// Wave 5 : pipeline strict en deux phases.
//
//   computeDecision()
//     1. Perception (cone same-direction + scan intersection).
//     2. Choix du v0 IDM = min(maxSpeed, road_limit, virage_factor).
//     3. Construction du LeaderInfo final :
//         - leader reel issu de Perception
//         - leader virtuel issu de IIntersectionPolicy.request() si shouldStop
//         - on retient le plus contraignant (plus petit gap).
//     4. pendingAccel = IDM.computeAcceleration(v, v0, leader).
//
//   integrate(dt)
//     - v += pendingAccel * dt (clamp >= 0 et <= v0).
//     - s += v * dt
//     - position + heading lus depuis Lane (parametrisation curviligne).
#include "core/agent/Vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/Intersection.hpp"
#include "core/math/Constants.hpp"
#include "core/pathfinding/AStarPlanner.hpp"
#include "core/perception/Perception.hpp"
#include "core/world/World.hpp"

Vehicle::Vehicle(float startX, float startY, float tSize) :
    position{startX, startY}, velocity{0.f, 0.f},
    maxSpeed(0.f), maxAcceleration(0.f),
    bodySize{0.f, 0.f}, bodyColor{255, 255, 255, 255},
    s(0.f),
    hasFinishedPath(false), tileSize(tSize),
    currentAngle(0.f), currentSpeed(0.f), isHeadingInitialized(false),
    // Seed deterministique : derive uniquement de la position de spawn.
    // Garantit la reproductibilite des snapshot tests (baseline.csv) et
    // evite tout ASLR-induced non-determinism (pointeur instable run-to-run).
    rng_(static_cast<std::uint64_t>(
        static_cast<std::uint64_t>(startX * 1000.f) * 0x9E3779B97F4A7C15ULL ^
        static_cast<std::uint64_t>(startY * 1000.f) * 0xBF58476D1CE4E5B9ULL))
{
    // VIN deterministe : l'ordre de construction des agents est fixe
    // (cf. SceneBuilder), donc reproductible run-to-run. N'influence aucune
    // physique sauf le bris d'egalite ultime du protocole P2P.
    static int s_nextVehicleId = 0;
    vehicleId_ = s_nextVehicleId++;
}

// -----------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------

void Vehicle::rebuildLaneFromPath(const std::vector<core::TileCoord>& tilePath,
                                  const World* world) {
    std::vector<core::Vec2> waypoints;
    waypoints.reserve(tilePath.size());
    for (const auto& tile : tilePath) {
        waypoints.push_back({
            (tile.x * tileSize) + tileSize / 2.f,
            (tile.y * tileSize) + tileSize / 2.f
        });
    }
    if (waypoints.size() < 2) {
        currentLane = nullptr;
        return;
    }

    // Intersection ROND-POINT couvrant le waypoint i (sinon nullptr).
    auto roundaboutAt = [&](int i) -> const Intersection* {
        if (!world || i < 0 || i >= static_cast<int>(waypoints.size())) return nullptr;
        const Intersection* it =
            world->getIntersectionAt(waypoints[i].x, waypoints[i].y);
        return (it && it->getType() == RegulationType::ROUNDABOUT) ? it : nullptr;
    };

    std::vector<core::Vec2> localDensePath;
    const float cornerRadius = tileSize;
    localDensePath.push_back(waypoints.front());

    // Bezier quadratique echantillonne, ajoute a la trajectoire dense.
    auto pushQuad = [&](core::Vec2 p0, core::Vec2 p1, core::Vec2 p2, int n) {
        for (int k = 1; k <= n; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(n);
            const float u = 1.f - t;
            localDensePath.push_back({
                u * u * p0.x + 2.f * u * t * p1.x + t * t * p2.x,
                u * u * p0.y + 2.f * u * t * p1.y + t * t * p2.y
            });
        }
    };
    // Intersection des droites (a + s.da) et (b + r.db) ; si quasi paralleles,
    // renvoie 'fallback'.
    auto lineX = [](core::Vec2 a, core::Vec2 da, core::Vec2 b, core::Vec2 db,
                    core::Vec2 fallback) {
        const float denom = core::cross(da, db);
        if (std::abs(denom) < 1e-4f) return fallback;
        const float s = core::cross(b - a, db) / denom;
        return a + da * s;
    };

    // Rond-point : arc circulaire, sens LEGAL francais (angle atan2 DECROISSANT).
    // Au lieu de rejoindre l'anneau au point RADIAL (virage a 90° pile -> le
    // raccord bossait/faisait une vague), on se greffe TANGENTIELLEMENT un peu
    // plus loin sur l'arc (angle decale 'delta'). Le raccord devient un simple
    // conge quadratique : tangente de depart = axe ROUTE (controle sur la droite
    // de la route -> aucune embardee), tangente d'arrivee = tangente de l'ARC.
    // Conge convexe unique -> AUCUNE vague.
    auto appendRoundaboutArc = [&](const Intersection& inter,
                                   core::Vec2 approachCenter,
                                   core::Vec2 exitCenter) {
        const core::Vec2 C = inter.getWorldCenter(tileSize);
        const float      R = inter.getLaneRadius(tileSize);

        const float thEntry = std::atan2(approachCenter.y - C.y, approachCenter.x - C.x);
        const float thExit  = std::atan2(exitCenter.y    - C.y, exitCenter.x    - C.x);

        float sweep = thEntry - thExit;                 // sens legal = decroissant
        while (sweep <= 0.0001f) sweep += core::math::TWO_PI;   // -> (0, 2π]

        // Greffe tangentielle decalee : ~25°, plafonnee a un quart du balayage
        // de chaque cote (l'arc garde au moins la moitie de son etendue).
        const float delta = std::min(0.45f, sweep * 0.25f);
        const float thIn  = thEntry - delta;
        const float thOut = thExit  + delta;

        const core::Vec2 entryPt{ C.x + std::cos(thEntry) * R, C.y + std::sin(thEntry) * R };
        const core::Vec2 Pin    { C.x + std::cos(thIn)  * R, C.y + std::sin(thIn)  * R };
        const core::Vec2 Pout   { C.x + std::cos(thOut) * R, C.y + std::sin(thOut) * R };
        const core::Vec2 tIn { std::sin(thIn),  -std::cos(thIn)  };  // tangente arc (deplacement)
        const core::Vec2 tOut{ std::sin(thOut), -std::cos(thOut) };

        // --- Conge d'ENTREE : A (sur la route) -> Pin (sur l'arc) ---
        const core::Vec2 A = localDensePath.back();
        core::Vec2 dRoadIn = entryPt - A;               // axe reel de la route (radial)
        const float lRin = dRoadIn.length();
        if (lRin > 1e-3f) dRoadIn /= lRin;
        const core::Vec2 ctrlIn = lineX(A, dRoadIn, Pin, tIn, (A + Pin) * 0.5f);
        pushQuad(A, ctrlIn, Pin, 16);

        // --- ARC (courbe validee) de thIn a thOut, decroissant ---
        float sweepArc = thIn - thOut;
        while (sweepArc <= 0.0001f) sweepArc += core::math::TWO_PI;
        const int segs = std::max(8, static_cast<int>(sweepArc * R / 5.f));
        for (int k = 1; k <= segs; ++k) {               // k=segs -> Pout
            const float t = static_cast<float>(k) / static_cast<float>(segs);
            const float a = thIn - sweepArc * t;
            localDensePath.push_back({ C.x + std::cos(a) * R, C.y + std::sin(a) * R });
        }

        // --- Conge de SORTIE : Pout (sur l'arc) -> exitCenter (sur la route) ---
        core::Vec2 dRoadOut = exitCenter - C;           // axe route = radiale sortante
        const float lRout = dRoadOut.length();
        if (lRout > 1e-3f) dRoadOut /= lRout;
        const core::Vec2 ctrlOut = lineX(Pout, tOut, exitCenter, dRoadOut,
                                         (Pout + exitCenter) * 0.5f);
        pushQuad(Pout, ctrlOut, exitCenter, 16);
    };

    std::size_t i = 0;
    while (i + 1 < waypoints.size()) {
        // Le waypoint suivant entre-t-il dans un rond-point pas encore traite ?
        const Intersection* rNext = roundaboutAt(static_cast<int>(i + 1));
        if (rNext && roundaboutAt(static_cast<int>(i)) != rNext) {
            std::size_t j = i + 1;
            while (j + 1 < waypoints.size() &&
                   roundaboutAt(static_cast<int>(j + 1)) == rNext) {
                ++j;
            }
            const core::Vec2 exitApproach =
                (j + 1 < waypoints.size()) ? waypoints[j + 1] : waypoints[j];
            appendRoundaboutArc(*rNext, waypoints[i], exitApproach);
            i = (j + 1 < waypoints.size()) ? (j + 1) : j;
            continue;
        }

        const core::Vec2 p1 = waypoints[i];
        const core::Vec2 p2 = waypoints[i + 1];

        // Lissage de coin (rue 90°), sauf si le coin touche un rond-point.
        if (i + 2 < waypoints.size() &&
            !roundaboutAt(static_cast<int>(i + 1)) &&
            !roundaboutAt(static_cast<int>(i + 2))) {
            const core::Vec2 p3 = waypoints[i + 2];
            core::Vec2 dir1 = p2 - p1;
            core::Vec2 dir2 = p3 - p2;
            const float len1 = dir1.length();
            const float len2 = dir2.length();
            if (len1 > 0.1f && len2 > 0.1f) {
                dir1 /= len1;
                dir2 /= len2;
                if (std::abs(core::dot(dir1, dir2)) < 0.1f) {
                    const core::Vec2 entryPoint = p2 - dir1 * cornerRadius;
                    const core::Vec2 exitPoint  = p2 + dir2 * cornerRadius;

                    const core::Vec2 toEntry = entryPoint - localDensePath.back();
                    const float distToEntry = toEntry.length();
                    if (distToEntry > 4.f) {
                        const core::Vec2 dirToEntry = toEntry / distToEntry;
                        for (float dE = 4.f; dE < distToEntry; dE += 4.f) {
                            localDensePath.push_back(localDensePath.back() + dirToEntry * 4.f);
                        }
                    }

                    core::Vec2 circleCenter = entryPoint + core::Vec2{-dir1.y, dir1.x} * cornerRadius;
                    const core::Vec2 testRadius = exitPoint - circleCenter;
                    if (std::abs(testRadius.length() - cornerRadius) > 1.f) {
                        circleCenter = entryPoint + core::Vec2{dir1.y, -dir1.x} * cornerRadius;
                    }

                    const float startAngle = std::atan2(entryPoint.y - circleCenter.y,
                                                        entryPoint.x - circleCenter.x);
                    const float endAngle   = std::atan2(exitPoint.y - circleCenter.y,
                                                        exitPoint.x - circleCenter.x);
                    float angleDiff = endAngle - startAngle;
                    while (angleDiff < -core::math::PI) angleDiff += core::math::TWO_PI;
                    while (angleDiff >  core::math::PI) angleDiff -= core::math::TWO_PI;

                    constexpr int numSegments = 15;
                    for (int k = 1; k <= numSegments; ++k) {
                        const float t = static_cast<float>(k) / numSegments;
                        const float a = startAngle + angleDiff * t;
                        localDensePath.push_back({
                            circleCenter.x + std::cos(a) * cornerRadius,
                            circleCenter.y + std::sin(a) * cornerRadius
                        });
                    }
                    i += 2;
                    continue;
                }
            }
        }

        // Segment droit standard.
        const core::Vec2 diff = p2 - localDensePath.back();
        const float dist = diff.length();
        if (dist > 4.f) {
            const core::Vec2 direction = diff / dist;
            for (float dd = 4.f; dd < dist; dd += 4.f) {
                localDensePath.push_back(localDensePath.back() + direction * 4.f);
            }
        }
        ++i;
    }
    localDensePath.push_back(waypoints.back());

    currentLane          = std::make_shared<Lane>(localDensePath);
    s                    = 0.f;
    hasFinishedPath      = false;
    isHeadingInitialized = false;
}

void Vehicle::setPath(const std::vector<core::TileCoord>& newPath,
                      const World* world) {
    if (newPath.empty()) {
        currentLane = nullptr;
        return;
    }
    startTile = newPath.front();
    goalTile  = newPath.back();
    rebuildLaneFromPath(newPath, world);
}

// -----------------------------------------------------------------
// PHASE 1 : computeDecision
// -----------------------------------------------------------------

void Vehicle::computeDecision(const std::vector<std::unique_ptr<IAgent>>& agents,
                              const World& world)
{
    using core::agent::BlockReason;

    if (!currentLane) {
        pendingAccel        = 0.f;
        pendingDesiredSpeed = 0.f;
        currentBlockReason  = BlockReason::NO_PATH;
        return;
    }
    if (hasFinishedPath) {
        pendingAccel        = 0.f;
        pendingDesiredSpeed = 0.f;
        currentBlockReason  = BlockReason::AT_GOAL;
        return;
    }

    // Panne : freinage d'urgence puis arret total.
    // Liberation differee : timer expire ET voie arriere claire.
    if (isBroken) {
        if (breakdownTimer <= 0.f && isClearBehindForRestart(agents)) {
            isBroken = false;
            // Reset complet pour repartir proprement.
            breakdownTimer = 0.f;
        } else {
            pendingAccel        = -idm.params().aMax * 2.f;
            pendingDesiredSpeed = 0.f;
            currentBlockReason  = BlockReason::BREAKDOWN;
            return;
        }
    }

    lastPerception = Perception::scan(position, currentAngle, this, agents, world, visionParams);

    // --- v0 : desired speed clamp ---
    // speedComplianceFactor > 1 = agent depasse la limitation (perso pressee).
    const float roadLimit = world.getSpeedLimitAt(position.x, position.y);
    const float compliantLimit = roadLimit * personality_.speedComplianceFactor;
    float v0 = (roadLimit > 0.f) ? std::min(maxSpeed, compliantLimit) : maxSpeed;

    // Anticipation virage.
    bool corneringClamp = false;
    const float lookAheadS    = std::min(s + 35.f, currentLane->getLength());
    const float futureHeading = currentLane->getHeadingAt(lookAheadS);
    const float angleDiff     = core::math::wrapDeg180(futureHeading - currentAngle);
    // En depassement le corps "yaw" volontairement (~15°) sur voie droite :
    // on ne declenche pas le frein de virage, sinon la manoeuvre cale a cote
    // du leader sans pouvoir le depasser.
    if (overtakeState == OvertakeState::NONE && std::abs(angleDiff) > 15.f) {
        v0 = std::min(v0, maxSpeed * 0.35f);
        corneringClamp = true;
    }
    pendingDesiredSpeed = v0;

    // --- Leader reel ---
    core::behavior::LeaderInfo leader;
    bool leaderIsVehicle  = false;
    if (lastPerception.hasDirectObstacle) {
        // Pendant depassement, on ignore le leader qu'on est en train de doubler.
        const bool skipForOvertake =
            (overtakeState == OvertakeState::OVERTAKING &&
             lastPerception.directObstacleAgent == overtakeLeader);
        if (!skipForOvertake) {
            leader.present  = true;
            leader.gap      = lastPerception.directObstacleDistance;
            leader.speed    = lastPerception.directObstacleSpeed;
            leaderIsVehicle = true;
        }
    }

    // --- Depassement : tentative + transitions d'etat ---
    if (overtakeState == OvertakeState::NONE && leader.present) {
        tryStartOvertake(agents, world, leader);
    }
    updateOvertakeDecision(agents, world);

    // Si on est en train de doubler, le leader courant ne nous freine plus
    // (offset lateral, c'est la voie d'en face qui compte).
    if (overtakeState != OvertakeState::NONE) {
        leader = core::behavior::LeaderInfo{};
        leaderIsVehicle = false;
    }

    // --- Leader virtuel issu de la policy d'intersection ---
    bool leaderFromYield = false;
    bool leaderFromRed   = false;
    bool leaderFromStop  = false;
    bool leaderFromP2P   = false;   // negociation P2P : Claim domine
    bool leaderFromPlatoon = false; // peloton virtuel : meneur projete mobile
    bool stopForceHalt   = false;   // arret FERME a la ligne d'un STOP

    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);
    if (interOn) {
        isCommittedToPass       = true;
        committedIntersectionId = interOn->getId();
        wasOnCommittedInter     = true;          // on est PHYSIQUEMENT dessus
    } else if (isCommittedToPass && wasOnCommittedInter) {
        // On ne libere QU'APRES avoir reellement traverse (etre passe SUR
        // l'intersection puis en etre ressorti). Sinon, un vehicule qui vient
        // de s'engager (mais pas encore entre) verrait son commit/STOP efface
        // chaque frame -> oscillation stop/redemarrage a 1 px/s.
        isCommittedToPass       = false;
        committedIntersectionId = -1;
        wasOnCommittedInter     = false;
        // Re-arme l'etat STOP pour une eventuelle future approche.
        currentStopIntersectionId = -1;
        stopHeldTime              = 0.f;
        stopReleased              = false;
    }

    if (!interOn) {
        const float headRad = currentAngle * core::math::DEG2RAD;
        const core::Vec2 lookDir{ std::cos(headRad), std::sin(headRad) };

        const Intersection* interAhead = nullptr;
        float distToInter = std::numeric_limits<float>::infinity();
        for (float d = 10.f; d < 250.f; d += tileSize * 0.4f) {
            const core::Vec2 checkPos = position + lookDir * d;
            const Intersection* found = world.getIntersectionAt(checkPos.x, checkPos.y);
            if (found) { interAhead = found; distToInter = d; break; }
        }

        if (interAhead && !(isCommittedToPass && committedIntersectionId == interAhead->getId())) {
            const Approach::Direction myDir = world.getApproachDirection(currentAngle);

            core::intersection::PolicyContext ctx;
            ctx.self.position = position;
            ctx.self.speed    = currentSpeed;
            ctx.self.heading  = currentAngle;
            ctx.self.length   = getLength();
            ctx.self.accel    = idm.params().aMax;   // pour estimer mon temps de degagement
            ctx.self.from     = myDir;
            ctx.selfAgent     = this;
            ctx.tileSize      = tileSize;
            ctx.others        = &agents;

            // Si l'agent est physiquement engage (carrosserie deja sur la
            // ligne d'arret), on force le passage : freiner ici provoquerait
            // un arret AU MILIEU du carrefour et bloquerait les flux croises.
            // EXCEPTION : a un STOP non encore libere, on ne s'auto-engage PAS
            // (sinon on franchirait le stop sans s'arreter).
            const bool  isStopAhead = (interAhead->getType() == RegulationType::STOP);
            const bool  stopDone     = isStopAhead &&
                                       currentStopIntersectionId == interAhead->getId() &&
                                       stopReleased;
            const float engagedThreshold = ctx.self.length / 2.f + 10.f;
            if (distToInter < engagedThreshold && (!isStopAhead || stopDone)) {
                isCommittedToPass       = true;
                committedIntersectionId = interAhead->getId();
            }

            const auto decision = interAhead->request(ctx);

            // --- STOP : arret FERME a la ligne (pas de rampe a 1 km/h), puis
            //     redemarrage des qu'on peut passer (ou apres temporisation). ---
            // La policy STOP est sans etat ; l'etat "j'ai marque l'arret a CE
            // stop" + la liberation vivent ici (par-agent).
            bool canPass = decision.canEnter;
            if (interAhead->getType() == RegulationType::STOP) {
                constexpr float kFixedDt      = 1.f / 60.f;  // pas de simulation fixe
                constexpr float kHaltZone     = 16.f;        // px : "arrive a la ligne"
                constexpr float kHaltSpeed    = 20.f;        // px/s : on commence a compter
                constexpr float kRequiredHalt = 0.25f;       // s d'arret marque (bref) exige

                const int sid = interAhead->getId();
                if (currentStopIntersectionId != sid) {      // nouveau stop -> reset
                    currentStopIntersectionId = sid;
                    stopHeldTime = 0.f;
                    stopReleased = false;
                }

                if (!stopReleased) {
                    // "A la ligne" = la policy ne me demande plus d'approcher
                    // (gap d'arret quasi nul). Vrai aussi bien quand la voie est
                    // libre (canEnter) que quand on cede (shouldStop, gap 0).
                    const bool atLine = (decision.stopLineGap <= kHaltZone);
                    if (atLine) {
                        // On marque l'arret (compte le temps une fois ~immobile)
                        // et on FIGE l'arret a la ligne tant qu'on n'est pas
                        // libere -> arret net, pas de rampe a 1 km/h.
                        if (currentSpeed <= kHaltSpeed) stopHeldTime += kFixedDt;

                        // Liberation : arret bref marque ET voie SURE (canEnter,
                        // qui tient compte de mon temps de degagement). Evaluee
                        // independamment de shouldStop -> on repart vraiment des
                        // que c'est libre (c'etait LE bug : plus personne ne
                        // passait car canEnter et shouldStop s'excluent).
                        if (stopHeldTime >= kRequiredHalt && decision.canEnter)
                            stopReleased = true;
                        else
                            stopForceHalt = true;
                    }
                    // sinon (encore loin) : approche normale -> leader virtuel
                    // de freinage via la branche shouldStop ci-dessous.
                }
                canPass = stopReleased;
            }

            if (canPass ||
                (isCommittedToPass && committedIntersectionId == interAhead->getId())) {
                if (distToInter < 60.f) {
                    isCommittedToPass       = true;
                    committedIntersectionId = interAhead->getId();
                }
            } else if (stopForceHalt) {
                // Arret ferme PILE a la ligne : leader virtuel colle (gap 0).
                leader            = core::behavior::LeaderInfo{};
                leader.present    = true;
                leader.gap        = 0.f;
                leader.speed      = 0.f;
                leader.stopTarget = true;   // point FIXE -> IDM sans time-headway
                leaderIsVehicle   = false;
                leaderFromStop    = true;
            } else if (decision.shouldStop) {
                core::behavior::LeaderInfo virtualLeader;
                virtualLeader.present    = true;
                virtualLeader.gap        = std::max(decision.stopLineGap, 0.f);
                virtualLeader.speed      = 0.f;
                virtualLeader.stopTarget = true;   // ligne d'arret FIXE

                if (!leader.present || virtualLeader.gap < leader.gap) {
                    leader = virtualLeader;
                    leaderIsVehicle = false;
                    if (interAhead->getType() == RegulationType::TRAFFIC_LIGHT) {
                        leaderFromRed = true;
                    } else if (interAhead->getType() == RegulationType::STOP) {
                        leaderFromStop = true;
                    } else if (interAhead->getType() == RegulationType::P2P) {
                        leaderFromP2P = true;
                    } else {
                        leaderFromYield = true;
                    }
                }
            }

            // --- Virtual Platooning : meneur virtuel MOBILE (voie croisee
            //     projetee). On ne s'arrete pas : on se cale derriere via l'IDM.
            //     Fusion avec le leader courant : le plus contraignant gagne.
            if (decision.followVirtualLeader) {
                core::behavior::LeaderInfo vlead;
                vlead.present = true;
                vlead.gap     = std::max(decision.virtualLeaderGap, 0.f);
                vlead.speed   = std::max(decision.virtualLeaderSpeed, 0.f);
                if (!leader.present || vlead.gap < leader.gap) {
                    leader            = vlead;
                    leaderIsVehicle   = false;
                    leaderFromPlatoon = true;
                }
            }
        }
    }

    // --- Filet de securite anti-collision (DERNIER RECOURS) -----------------
    // Independant de toute policy d'intersection. La Perception ne retient comme
    // leader QUE le trafic de meme sens (car-following) ; le trafic CROISE est
    // gere par les leaders virtuels des policies. Si cette heuristique faillit
    // (projection 1D du peloton, arrivees quasi simultanees...), rien n'empeche
    // deux carrosseries de se chevaucher. Ce filet freine pour TOUT agent
    // physiquement dans mon couloir d'avance quand l'impact est imminent.
    // Declenchement de proximite uniquement -> n'altere pas la conduite normale.
    {
        const float headRad = currentAngle * core::math::DEG2RAD;
        const core::Vec2 fwd{ std::cos(headRad), std::sin(headRad) };
        const float myHalfLen   = bodySize.x / 2.f;
        const float myHalfWidth = bodySize.y / 2.f;

        for (const auto& other : agents) {
            if (!other || other.get() == this) continue;
            const core::Vec2 dp = other->getPosition() - position;
            const float forwardDist = dp.x * fwd.x + dp.y * fwd.y;
            if (forwardDist <= 0.f) continue;                       // derriere moi

            const float lateral      = std::abs(dp.x * (-fwd.y) + dp.y * fwd.x);
            const float otherHalfLen = other->getLength() / 2.f;
            // Couloir = ma demi-largeur + l'empreinte de l'autre (croise => sa
            // longueur barre ma voie) + petite marge laterale.
            if (lateral > myHalfWidth + otherHalfLen + 3.f) continue;

            const float bumper = forwardDist - myHalfLen - otherHalfLen;

            // Composante de vitesse de l'autre le long de MON cap (croise -> ~0).
            const float oRad      = other->getHeading() * core::math::DEG2RAD;
            const core::Vec2 oFwd{ std::cos(oRad), std::sin(oRad) };
            const float oFwdSpeed = (oFwd.x * fwd.x + oFwd.y * fwd.y) * other->getSpeed();
            const float closing   = std::max(currentSpeed - oFwdSpeed, 0.f);

            // Marge proportionnelle a la vitesse de rapprochement (plus de
            // distance de reaction quand ca ferme vite), bornee petit.
            const float margin = 6.f + 0.25f * closing;
            if (bumper >= margin) continue;                         // pas imminent

            // --- Anti-DEADLOCK : conflit reciproque ? -------------------------
            // Si 'other' m'a AUSSI dans son couloir d'avance, on est dans un
            // conflit mutuel ou les DEUX freineraient l'un pour l'autre et se
            // figeraient ("je le suis / il me suit"). On tranche par VIN : le
            // plus PETIT VIN garde la priorite (ne cede pas), le plus grand cede.
            // Ordre total -> aucun cycle possible. Un PUR suiveur (l'autre devant
            // moi mais moi pas devant lui) freine toujours -> suivi de file et
            // anti-collision en courbe (rond-point) preserves.
            {
                const core::Vec2 dpBA = position - other->getPosition();
                const float otherForward = dpBA.x * oFwd.x + dpBA.y * oFwd.y;
                const float otherLateral = std::abs(dpBA.x * (-oFwd.y) + dpBA.y * oFwd.x);
                const float otherHalfWidth = other->getBodySize().y / 2.f;
                const bool  mutual = (otherForward > 0.f) &&
                                     (otherLateral <= otherHalfWidth + myHalfLen + 3.f);
                if (mutual) {
                    const int myV = getVehicleId();
                    const int oV  = other->getVehicleId();
                    const bool iHavePriority = (myV >= 0) && (oV < 0 || myV < oV);
                    if (iHavePriority) continue;   // je passe ; l'autre cedera
                }
            }

            // Leader d'urgence : on retient le plus contraignant (plus petit gap).
            if (!leader.present || bumper < leader.gap) {
                leader            = core::behavior::LeaderInfo{};
                leader.present    = true;
                leader.gap        = std::max(bumper, 0.f);
                leader.speed      = std::max(oFwdSpeed, 0.f);
                leader.stopTarget = (oFwdSpeed < 5.f);   // obstacle ~fixe -> arret net
                leaderIsVehicle   = true;
            }
        }
    }

    // --- IDM ---
    pendingAccel = idm.computeAcceleration(currentSpeed, v0, leader);

    // STOP : marquage a la ligne. Frein FERME mais LISSE. Le leader virtuel
    // colle (gap 0) ferait tendre l'IDM vers une deceleration quasi infinie qui
    // annule la vitesse en UNE frame (arret en a-coup). On impose a la place une
    // deceleration bornee (~0.75x le confort) : l'arret se fait en ~0.5 s,
    // franc mais souple. L'anti-fluage (kCreepSpeed) absorbe le dernier px/s.
    if (stopForceHalt && !stopReleased) {
        pendingAccel = -idm.params().bComf * 0.75f;
    }

    // --- Diagnostic du motif de blocage ---
    if (overtakeState != OvertakeState::NONE) {
        currentBlockReason = BlockReason::OVERTAKING;
    } else if (leaderFromRed) {
        currentBlockReason = BlockReason::INTERSECTION_RED;
    } else if (leaderFromStop) {
        currentBlockReason = BlockReason::INTERSECTION_STOP;
    } else if (leaderFromP2P) {
        currentBlockReason = BlockReason::NEGOTIATING;
    } else if (leaderFromPlatoon) {
        currentBlockReason = BlockReason::PLATOONING;
    } else if (leaderFromYield) {
        currentBlockReason = BlockReason::INTERSECTION_YIELD;
    } else if (leaderIsVehicle && pendingAccel < 0.f) {
        currentBlockReason = BlockReason::LEADER_VEHICLE;
    } else if (corneringClamp && currentSpeed > v0 - 1.f) {
        currentBlockReason = BlockReason::CORNERING;
    } else if (currentSpeed < 5.f && pendingAccel > 0.f) {
        currentBlockReason = BlockReason::INITIALIZING;
    } else {
        currentBlockReason = BlockReason::NONE;
    }
}

// -----------------------------------------------------------------
// PHASE 2 : integrate (Euler, dt fixe)
// -----------------------------------------------------------------

void Vehicle::integrate(float dt) {
    if (!currentLane || hasFinishedPath) return;

    // Panne : agent fige, on decremente le timer ici (computeDecision
    // a deja signale isBroken). A la fin, on attend la voie libre derriere.
    if (isBroken) {
        currentSpeed = 0.f;
        velocity     = {0.f, 0.f};
        updateBreakdown(dt);
        return;
    }

    // Dice-roll de panne possible meme quand on roule.
    updateBreakdown(dt);
    if (isBroken) {
        currentSpeed = 0.f;
        velocity     = {0.f, 0.f};
        return;
    }

    if (s >= currentLane->getLength() - 1.f) {
        hasFinishedPath = true;
        currentSpeed    = 0.f;
        velocity        = {0.f, 0.f};
        return;
    }

    currentSpeed += pendingAccel * dt;
    currentSpeed  = std::max(0.f, currentSpeed);
    const float cap = pendingDesiredSpeed > 0.f ? pendingDesiredSpeed : maxSpeed;
    currentSpeed  = std::min(currentSpeed, cap);

    // Anti-fluage : l'IDM approche l'arret de facon asymptotique (la derniere
    // poignee de px/s s'eternise). Des qu'on est tres lent SANS reelle volonte
    // d'accelerer, on colle a 0 -> arret franc. Le seuil (14 px/s) reste bien
    // sous la vitesse de croisiere la plus basse (>=30 px/s) : aucun vehicule
    // qui roule normalement n'est fige. Un demarrage donne pendingAccel >> 2
    // (l'IDM pousse fort depuis l'arret) -> la condition ne s'y declenche pas.
    constexpr float kCreepSpeed = 14.f;   // px/s
    constexpr float kCreepAccel = 2.f;    // px/s^2
    if (currentSpeed < kCreepSpeed && pendingAccel < kCreepAccel) currentSpeed = 0.f;

    s += currentSpeed * dt;
    if (s > currentLane->getLength()) s = currentLane->getLength();

    updateOvertakeMotion(dt);

    const core::Vec2 basePos     = currentLane->getPositionAt(s);
    const float      laneHeading = currentLane->getHeadingAt(s);

    // Placement lateral : perpendiculaire au heading de la VOIE (pas au yaw).
    const float laneRad = laneHeading * core::math::DEG2RAD;
    const core::Vec2 perp{ -std::sin(laneRad), std::cos(laneRad) };
    position.x = basePos.x + perp.x * lateralOffset;
    position.y = basePos.y + perp.y * lateralOffset;

    // Yaw visuel : le corps pointe le long du mouvement reel (avance +
    // derive laterale). Donne une vraie courbe pendant le depassement plutot
    // qu'une simple translation. Nul des que lateralVel_ revient a 0.
    const float yawRad = std::atan2(lateralVel_, std::max(currentSpeed, 1.f));
    currentAngle = laneHeading + yawRad * core::math::RAD2DEG;

    const float rad = currentAngle * core::math::DEG2RAD;
    velocity.x = std::cos(rad) * currentSpeed;
    velocity.y = std::sin(rad) * currentSpeed;
}

// -----------------------------------------------------------------
// UI / debug / path management
// -----------------------------------------------------------------

bool Vehicle::contains(core::Vec2 point) const {
    const core::Vec2 local = point - position;
    const float rad = -currentAngle * core::math::DEG2RAD;
    const float c   = std::cos(rad);
    const float si  = std::sin(rad);
    const core::Vec2 r{ local.x * c - local.y * si,
                        local.x * si + local.y * c };
    return std::abs(r.x) <= bodySize.x / 2.f &&
           std::abs(r.y) <= bodySize.y / 2.f;
}

AgentDebugSnapshot Vehicle::getDebugSnapshot() const {
    AgentDebugSnapshot snap;
    snap.selected = isSelectedFlag;
    snap.active   = (currentLane != nullptr) && !hasFinishedPath;

    snap.visionRange              = visionParams.range;
    snap.visionHalfAngleDeg       = visionParams.halfAngleDeg;
    snap.visionDirectHalfAngleDeg = visionParams.directHalfAngle;

    snap.hasDirectObstacle       = lastPerception.hasDirectObstacle;
    snap.directObstacleDistance  = lastPerception.directObstacleDistance;
    snap.approachingIntersection = lastPerception.approachingIntersection;

    snap.detectionPositions.reserve(lastPerception.detected.size());
    snap.detectionDistances.reserve(lastPerception.detected.size());
    for (const auto& d : lastPerception.detected) {
        snap.detectionPositions.push_back(d.position);
        snap.detectionDistances.push_back(d.distance);
    }
    if (currentLane) snap.pathPoints = currentLane->getPoints();
    return snap;
}

core::TileCoord Vehicle::getCurrentTile() const {
    return { static_cast<int>(position.x / tileSize),
             static_cast<int>(position.y / tileSize) };
}

void Vehicle::recalculatePath(const World& world) {
    if (hasFinishedPath) return;
    const core::TileCoord current = getCurrentTile();
    const auto newPath = AStarPlanner::findPath(world, current, goalTile);
    if (!newPath.empty()) {
        const core::TileCoord origStart = startTile;
        const core::TileCoord origGoal  = goalTile;
        setPath(newPath, &world);
        startTile = origStart;
        goalTile  = origGoal;
    } else {
        currentLane  = nullptr;
        currentSpeed = 0.f;
    }
}

void Vehicle::resetToStart(const World& world) {
    position             = { startTile.x * tileSize + tileSize / 2.f,
                             startTile.y * tileSize + tileSize / 2.f };
    velocity             = {0.f, 0.f};
    currentSpeed         = 0.f;
    currentAngle         = 0.f;
    isHeadingInitialized = false;
    hasFinishedPath      = false;
    s                    = 0.f;
    pendingAccel         = 0.f;
    pendingDesiredSpeed  = 0.f;
    isCommittedToPass       = false;
    committedIntersectionId = -1;
    wasOnCommittedInter     = false;
    currentBlockReason   = core::agent::BlockReason::INITIALIZING;
    isBroken             = false;
    breakdownTimer       = 0.f;
    timeSinceLastCheck   = 0.f;
    overtakeState        = OvertakeState::NONE;
    lateralOffset        = 0.f;
    overtakeTarget       = 0.f;
    overtakeLeader       = nullptr;
    overtakeElapsed      = 0.f;
    lateralVel_          = 0.f;
    currentStopIntersectionId = -1;
    stopHeldTime         = 0.f;
    stopReleased         = false;

    const auto fullPath = AStarPlanner::findPath(world, startTile, goalTile);
    if (!fullPath.empty()) {
        const core::TileCoord origStart = startTile;
        const core::TileCoord origGoal  = goalTile;
        setPath(fullPath, &world);
        startTile = origStart;
        goalTile  = origGoal;
    } else {
        currentLane = nullptr;
    }
}

float Vehicle::getRemainingDistance() const {
    if (!currentLane || hasFinishedPath) return 0.f;
    return std::max(0.f, currentLane->getLength() - s);
}

core::agent::TurnIntent Vehicle::getTurnIntent() const {
    // Classe la manoeuvre imminente en comparant le cap actuel au cap ~3 tiles
    // plus loin sur la trajectoire (couvre l'arc d'un virage de carrefour).
    // |delta| faible -> tout droit ; sinon -> virage. Pas de distinction
    // gauche/droite : la regle de dominance P2P ne la requiert pas.
    if (!currentLane || hasFinishedPath) return core::agent::TurnIntent::UNKNOWN;
    const float here  = currentLane->getHeadingAt(s);
    const float ahead = currentLane->getHeadingAt(
        std::min(s + tileSize * 3.f, currentLane->getLength()));
    const float delta = std::abs(core::math::wrapDeg180(ahead - here));
    return (delta < 30.f) ? core::agent::TurnIntent::STRAIGHT
                          : core::agent::TurnIntent::TURNING;
}

// =================================================================
// Wave 6 : Personality / Breakdown / Overtake
// =================================================================

void Vehicle::applyPersonalityToIdm() {
    core::behavior::IdmParams p = baseIdmParams_;
    p.T     *= personality_.reactionTimeFactor;
    p.s0    *= personality_.minGapFactor;
    p.aMax  *= personality_.accelEagernessFactor;
    p.bComf *= personality_.comfortBrakeFactor;
    idm.setParams(p);
}

void Vehicle::applyGaussianHeterogeneity(core::Rng& rng, float sigma) {
    personality_.reactionTimeFactor    *= rng.normal(1.f, sigma);
    personality_.accelEagernessFactor  *= rng.normal(1.f, sigma);
    personality_.speedComplianceFactor *= rng.normal(1.f, sigma);
    personality_.clamp();
    applyPersonalityToIdm();
}

void Vehicle::forceBreakdown(float seconds) {
    isBroken       = true;
    breakdownTimer = std::max(0.5f, seconds);
}

void Vehicle::updateBreakdown(float dt) {
    if (isBroken) {
        breakdownTimer -= dt;
        if (breakdownTimer <= 0.f) {
            // Fin de reparation : check derriere avant de redemarrer.
            // L'agent reste BROKEN jusqu'au prochain tick computeDecision
            // qui validera la securite via isClearBehindForRestart.
            breakdownTimer = 0.f;
        }
        return;
    }

    // Dice-roll de panne, cadence a 0.5s pour limiter le bruit RNG.
    timeSinceLastCheck += dt;
    if (timeSinceLastCheck < 0.5f) return;
    const float window = timeSinceLastCheck;
    timeSinceLastCheck = 0.f;

    const float probPerSec = personality_.breakdownChancePerMin / 60.f;
    const float pInWindow  = 1.f - std::pow(1.f - probPerSec, window);
    if (rng_.bernoulli(std::clamp(pInWindow, 0.f, 1.f))) {
        // Duree autour de la moyenne (± 50%).
        const float jitter = rng_.uniform(0.5f, 1.5f);
        forceBreakdown(personality_.breakdownDurationSec * jitter);
    }
}

bool Vehicle::isClearBehindForRestart(
    const std::vector<std::unique_ptr<IAgent>>& agents) const
{
    // Scan derriere = cone arriere de 30° sur ~70px.
    // On ignore les agents A L'ARRET (speed < 5) : ils ne risquent pas
    // de me percuter, et eux-memes sont probablement bloques PAR moi
    // (deadlock potentiel sinon).
    constexpr float scanRange    = 70.f;
    constexpr float halfAngleDeg = 30.f;

    const float backRad = (currentAngle + 180.f) * core::math::DEG2RAD;
    const core::Vec2 backDir{ std::cos(backRad), std::sin(backRad) };

    for (const auto& other : agents) {
        if (!other || other.get() == this) continue;
        if (other->getSpeed() < 5.f) continue;             // arrete -> non bloquant
        const core::Vec2 diff = other->getPosition() - position;
        const float dist = diff.length();
        if (dist > scanRange || dist < 1.f) continue;

        // Angle entre dir arriere et diff.
        const float dotN = (diff.x * backDir.x + diff.y * backDir.y) / dist;
        const float angleDeg = std::acos(std::clamp(dotN, -1.f, 1.f)) * core::math::RAD2DEG;
        if (angleDeg <= halfAngleDeg) return false;
    }
    return true;
}

bool Vehicle::isOncomingLaneFree(
    const std::vector<std::unique_ptr<IAgent>>& agents,
    float requiredClearAhead) const
{
    // Filtre les vehicules en sens INVERSE (heading delta > 135°) qui
    // arrivent par devant. On simule a vitesse relative la fenetre de
    // collision pendant requiredClearAhead / closingSpeed.
    const float headRad = currentAngle * core::math::DEG2RAD;
    const core::Vec2 forwardDir{ std::cos(headRad), std::sin(headRad) };

    for (const auto& other : agents) {
        if (!other || other.get() == this) continue;
        const core::Vec2 diff = other->getPosition() - position;
        const float forwardDist = diff.x * forwardDir.x + diff.y * forwardDir.y;
        if (forwardDist <= 0.f || forwardDist > requiredClearAhead + 60.f) continue;

        // Largeur de couloir = 1.5 tiles (laisse de la marge laterale).
        const float lateral = std::abs(diff.x * (-forwardDir.y) + diff.y * forwardDir.x);
        if (lateral > tileSize * 1.5f) continue;

        // Sens oppose ?
        const float diffHeading = std::abs(
            core::math::wrapDeg180(other->getHeading() - currentAngle));
        if (diffHeading < 135.f) continue;

        // Si oncoming va vite, le temps de collision se reduit.
        const float closingSpeed = std::max(currentSpeed + other->getSpeed(), 30.f);
        const float ttc = forwardDist / closingSpeed;
        // On veut ≥ 4s pour rentrer en securite.
        if (ttc < 4.0f) return false;
    }
    return true;
}

bool Vehicle::isReturnSlotFree(
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const core::behavior::LeaderInfo& leader) const
{
    // On ne double que si l'on pourra se RABATTRE devant le leader. Le creneau
    // requis se situe au-dela du leader, sur notre voie, sur une longueur
    // fonction de la vitesse. S'il est deja occupe (convoi serre), on renonce :
    // sinon le vehicule se decale sur le cote sans pouvoir revenir.
    const float headRad = currentAngle * core::math::DEG2RAD;
    const core::Vec2 fwd{ std::cos(headRad), std::sin(headRad) };

    const float leaderFwd = std::max(leader.gap, 0.f);
    const float slotLen   = bodySize.x * 2.5f + currentSpeed * 1.2f + 30.f;
    const float slotEnd   = leaderFwd + slotLen;

    for (const auto& other : agents) {
        if (!other || other.get() == this) continue;
        if (other.get() == lastPerception.directObstacleAgent) continue; // le leader
        const core::Vec2 diff = other->getPosition() - position;
        const float fwdDist = diff.x * fwd.x + diff.y * fwd.y;
        if (fwdDist <= leaderFwd || fwdDist > slotEnd) continue;          // hors creneau
        const float lateral = std::abs(diff.x * (-fwd.y) + diff.y * fwd.x);
        if (lateral > tileSize * 0.7f) continue;                          // autre voie
        const float dHead = std::abs(
            core::math::wrapDeg180(other->getHeading() - currentAngle));
        if (dHead > 45.f) continue;                                       // sens oppose
        return false;                                                     // creneau pris
    }
    return true;
}

bool Vehicle::tryStartOvertake(
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const World& world,
    const core::behavior::LeaderInfo& leader)
{
    if (overtakeState != OvertakeState::NONE) return false;
    if (personality_.overtakeWillingness <= 0.f) return false;
    if (isBroken) return false;
    if (!leader.present || !lastPerception.directObstacleAgent) return false;

    // Conditions techniques durs.
    if (currentSpeed < 50.f) return false;                       // pas a l'arret
    if (leader.gap > 90.f) return false;                         // trop loin
    if (leader.speed > currentSpeed * personality_.overtakeLeaderRatio) return false;

    // Pas de virage proche : check angle sur 120px lookahead.
    if (currentLane) {
        const float futureH = currentLane->getHeadingAt(
            std::min(s + 120.f, currentLane->getLength()));
        if (std::abs(core::math::wrapDeg180(futureH - currentAngle)) > 6.f) return false;
    }

    // Longueur totale de la manoeuvre (estimation par vitesse RELATIVE) :
    // temps pour rattraper+depasser le leader, converti en distance parcourue,
    // plus la distance de rabattement.
    const float relSpeed    = std::max(currentSpeed - leader.speed, 15.f);
    const float relGap      = leader.gap + bodySize.x * 2.5f + 35.f;  // a combler relativement
    const float passDist    = currentSpeed * (relGap / relSpeed);     // distance longitudinale
    const float returnDist  = currentSpeed * 1.0f + bodySize.x;
    const float maneuverLen = passDist + returnDist;

    // Distance reellement libre devant : jusqu'a la prochaine intersection OU
    // la fin du trajet (point d'arrivee). On REFUSE de se lancer si la
    // manoeuvre ne peut pas etre TERMINEE (rabattement inclus) avant cet
    // obstacle -- sinon le vehicule resterait coince sur la voie d'en face.
    float clearAhead = currentLane ? (currentLane->getLength() - s) : maneuverLen;
    {
        const float headRad = currentAngle * core::math::DEG2RAD;
        const core::Vec2 lookDir{ std::cos(headRad), std::sin(headRad) };
        for (float d = 10.f; d < maneuverLen + tileSize; d += tileSize * 0.4f) {
            const core::Vec2 cp = position + lookDir * d;
            if (world.getIntersectionAt(cp.x, cp.y)) { clearAhead = std::min(clearAhead, d); break; }
        }
    }
    if (clearAhead < maneuverLen) return false;

    // Voie d'en face libre sur toute la longueur de la manoeuvre.
    if (!isOncomingLaneFree(agents, maneuverLen)) return false;

    // Creneau de rabattement libre devant le leader (sinon on reste derriere).
    if (!isReturnSlotFree(agents, leader)) return false;

    // Dice-roll personnalite : meme si tout est OK, certaines persos hesitent.
    if (!rng_.bernoulli(personality_.overtakeWillingness)) return false;

    overtakeState   = OvertakeState::OVERTAKING;
    overtakeTarget  = -tileSize;   // bascule vers la voie d'en face (gauche du heading)
    overtakeLeader  = lastPerception.directObstacleAgent;
    overtakeElapsed = 0.f;
    return true;
}

void Vehicle::updateOvertakeDecision(
    const std::vector<std::unique_ptr<IAgent>>& agents,
    const World& world)
{
    if (overtakeState == OvertakeState::NONE) return;

    // Abort si oncoming devient dangereux.
    if (overtakeState == OvertakeState::OVERTAKING &&
        !isOncomingLaneFree(agents, currentSpeed * 2.5f)) {
        overtakeState  = OvertakeState::RETURNING;
        overtakeTarget = 0.f;
    }

    // Abort si on rentre dans une intersection.
    if (world.getIntersectionAt(position.x, position.y)) {
        overtakeState  = OvertakeState::RETURNING;
        overtakeTarget = 0.f;
    }

    // Si on a depasse le leader, on se rabat.
    if (overtakeState == OvertakeState::OVERTAKING && overtakeLeader) {
        const core::Vec2 leadDiff = overtakeLeader->getPosition() - position;
        const float headRad = currentAngle * core::math::DEG2RAD;
        const core::Vec2 fwd{ std::cos(headRad), std::sin(headRad) };
        const float forwardDist = leadDiff.x * fwd.x + leadDiff.y * fwd.y;
        if (forwardDist < -bodySize.x * 1.8f) {
            overtakeState  = OvertakeState::RETURNING;
            overtakeTarget = 0.f;
        }
    }
}

void Vehicle::updateOvertakeMotion(float dt) {
    const float prevLateral = lateralOffset;

    if (overtakeState == OvertakeState::NONE) {
        // Lateral toujours nul quand pas en manoeuvre.
        lateralOffset = 0.f;
        lateralVel_   = (dt > 0.f) ? (lateralOffset - prevLateral) / dt : 0.f;
        return;
    }
    overtakeElapsed += dt;

    // Abort si trop long.
    if (overtakeElapsed > 12.f) {
        overtakeState   = OvertakeState::RETURNING;
        overtakeTarget  = 0.f;
        overtakeLeader  = nullptr;
    }

    // Lissage : ~40 px/s lateral.
    const float maxRate = 40.f * dt;
    const float delta   = overtakeTarget - lateralOffset;
    if (std::abs(delta) <= maxRate) {
        lateralOffset = overtakeTarget;
    } else {
        lateralOffset += (delta > 0.f ? maxRate : -maxRate);
    }

    if (overtakeState == OvertakeState::RETURNING && std::abs(lateralOffset) < 1.f) {
        lateralOffset  = 0.f;
        overtakeState  = OvertakeState::NONE;
        overtakeLeader = nullptr;
        overtakeElapsed = 0.f;
    }

    // Vitesse laterale courante -> yaw visuel naturel dans integrate().
    lateralVel_ = (dt > 0.f) ? (lateralOffset - prevLateral) / dt : 0.f;
}
