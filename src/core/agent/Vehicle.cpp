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

    // --- Instrumentation diagnostic : remise a neuf chaque pas ---
    dbgLeaderSrc_ = 0; dbgLeaderVin_ = -1; dbgLeaderRelHeadingDeg_ = 999.f;
    dbgLeaderGap_ = -1.f; dbgLeaderSpeed_ = 0.f; dbgOnInter_ = false;
    const IAgent* dbgLeaderPtr = nullptr;   // leader-vehicule final (frame courante)
    int           dbgSrc       = 0;         // 1 perception, 2 filet

    // Dilemme cannotStop : je m'engage parce que je ne peux plus m'arreter
    // avant la ligne. Pour minimiser le DEPASSEMENT (sinon l'IDM, sans leader,
    // ACCELERE vers v0 alors que je rentre), on freine quand meme au max sur ce
    // pas. Une fois dans la boite (interOn), l'override est leve -> je degage.
    bool dilemmaBrakeOverride = false;

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

    lastPerception = Perception::scan(position, currentAngle, this, agents, world,
                                      visionParams, currentLane.get(), s);

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
            dbgLeaderPtr    = lastPerception.directObstacleAgent;  // diag
            dbgSrc          = 1;
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
    bool leaderFromBox   = false;   // anti-gridlock : sortie de carrefour occupee
    bool stopForceHalt   = false;   // arret FERME a la ligne d'un STOP

    const Intersection* interOn = world.getIntersectionAt(position.x, position.y);
    dbgOnInter_ = (interOn != nullptr);   // diag
    if (interOn) {
        isCommittedToPass       = true;
        committedIntersectionId = interOn->getId();
        wasOnCommittedInter     = true;          // on est PHYSIQUEMENT dessus
        keepClearWaited_        = 0.f;            // franchi -> timer keep-clear neuf
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

        if (interAhead) {   // re-evalue a CHAQUE pas (aucun "commit" collant)
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

            const bool isStopAhead = (interAhead->getType() == RegulationType::STOP);

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
                    stopHeldTime     = 0.f;
                    stopReleased     = false;
                    stopEverYielded_ = false;
                }

                // Axe MAJEUR (route principale, pas de panneau STOP) : la policy
                // renvoie {canEnter=true, shouldStop=false, stopLineGap=0}. Le
                // halt protocol ne doit PAS s'enclencher : stopLineGap=0 == "a la
                // ligne" -> sans ce garde, stopForceHalt freinait brutalement
                // l'agent prioritaire avant son passage. On ne marque l'arret QUE
                // si la policy a deja exige shouldStop sur cette approche.
                if (decision.shouldStop) stopEverYielded_ = true;

                if (!stopEverYielded_) {
                    // Jamais cede ici -> je suis sur l'axe majeur, passage libre.
                    stopReleased = true;
                } else if (!stopReleased) {
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

            // --- ENGAGEMENT : modele UNIFIE, re-evalue a CHAQUE pas -------------
            // Plus aucun "commit collant" : c'est lui qui laissait les suiveurs
            // passer EN FILE (sans verifier leur propre priorite), ignorait le
            // passage au ROUGE et figeait le vehicule "engage". Regle unique :
            //   j'entre SSI j'ai le droit de passage (canPass) ET je pourrai
            //   DEGAGER la sortie (keep clear). Sinon je cede a la ligne.
            // Exception DILEMME : si je ne peux plus m'arreter avant la ligne meme
            // en freinage d'urgence, je passe (m'arreter me figerait EN PLEIN
            // carrefour). Jamais pour un STOP non libere (je dois marquer l'arret).
            const RegulationType dtype = interAhead->getType();

            const float v            = std::max(0.f, currentSpeed);
            const float bEmergency   = idm.params().bComf * 2.f;
            const float brakingDistE = (v * v) / (2.f * std::max(1.f, bEmergency));
            const float gapToLine    = std::max(0.f, distToInter - ctx.self.length / 2.f);
            const bool  cannotStop   = brakingDistE > gapToLine;

            // Keep clear : la BOITE + la sortie sur MA trajectoire sont-elles
            // degageables (aucun vehicule a l'ARRET dedans / juste apres) ? Sinon je
            // n'entre pas -> anti-gridlock + empeche un suiveur de se figer au milieu.
            // Regulations a ARRET (feux/STOP/priorite) : on surveille la BOITE +
            // la sortie. Les schemas qui s'entrelacent gerent leur propre debit.
            // ROND-POINT : meme garde anti-gridlock, mais la fenetre surveillee est
            // la SEULE route de SORTIE (apres l'anneau), PAS l'arc lui-meme -- sinon
            // on refuserait d'entrer pour tout vehicule circulant normalement sur
            // l'anneau (= famine). Le vrai risque rond-point = sortie bouchee ->
            // j'entre, je tourne, et je bloque l'anneau faute de pouvoir sortir.
            // Borne dans le temps -> jamais de famine (passe le delai, on cede au
            // bris de cycle VIN du filet).
            float sEnter = -1.f;
            bool  exitBlocked = false;
            {
                const bool keepClearApplies =
                    dtype == RegulationType::TRAFFIC_LIGHT  ||
                    dtype == RegulationType::STOP           ||
                    dtype == RegulationType::FIXED_PRIORITY ||
                    dtype == RegulationType::PRIORITY_RIGHT ||
                    dtype == RegulationType::YIELD          ||
                    dtype == RegulationType::ROUNDABOUT     ||
                    dtype == RegulationType::P2P            ||
                    dtype == RegulationType::AIM            ||
                    dtype == RegulationType::VIRTUAL_PLATOON||
                    dtype == RegulationType::ORCA;
                if (keepClearApplies) {
                    const float laneLen = currentLane->getLength();
                    float sExit = -1.f;
                    // Rond-point : l'arc traverse de longues portions de lane DANS
                    // l'anneau ; il faut aller plus loin pour trouver la sortie.
                    const float scanReach =
                        (dtype == RegulationType::ROUNDABOUT) ? 400.f : 240.f;
                    for (float ds = 0.f; ds < scanReach; ds += tileSize * 0.25f) {
                        const float sp = std::min(s + ds, laneLen);
                        const core::Vec2 lp = currentLane->getPositionAt(sp);
                        const bool onI = (world.getIntersectionAt(lp.x, lp.y) != nullptr);
                        if (sEnter < 0.f && onI) sEnter = sp;
                        else if (sEnter >= 0.f && !onI) { sExit = sp; break; }
                        if (sp >= laneLen) break;
                    }
                    if (sEnter >= 0.f && sExit > sEnter) {
                        // FULL-BODY FIT : interdire l'engagement si ma carrosserie
                        // ne tient pas en entier dans la boite + sortie. need
                        // couvre longueur + s0 + petite marge. Si sExit + need
                        // depasse la fin de la voie (cas degenere : route trop
                        // courte apres la boite), on attend -- mieux vaut
                        // patienter que se figer mi-engage.
                        const float need = getLength() + idm.params().s0 + 4.f;
                        if (sExit + need > laneLen) { exitBlocked = true; }
                        // Borne basse de la fenetre surveillee. Carrefour : depuis
                        // l'ENTREE (on veut la boite ELLE-MEME degagee). Rond-point :
                        // depuis la SORTIE seulement (on ignore l'arc circulant).
                        const bool  isRoundabout = (dtype == RegulationType::ROUNDABOUT);
                        const float blockFrom = isRoundabout ? sExit : sEnter;
                        // Prefiltre de proximite. Carrefour compact : ancre sur
                        // l'agent, rayon 250 px (comportement historique inchange).
                        // Rond-point : la sortie est de l'autre cote de l'anneau
                        // (>250 px de moi) -> ancre sur la FENETRE surveillee, rayon =
                        // longueur de fenetre + marge (ecart lateral + carrosserie du
                        // bloqueur), sinon un filtre centre sur moi raterait le bloqueur.
                        const core::Vec2 winAnchor = isRoundabout
                            ? currentLane->getPositionAt(std::min(blockFrom, laneLen))
                            : position;
                        const float winReach = isRoundabout
                            ? (sExit + need - blockFrom) + 120.f
                            : 250.f;
                        bool blocked = false;
                        for (const auto& other : agents) {
                            if (!other || other.get() == this) continue;
                            // Bouchon = vehicule lent OU en train de s'arreter. Le
                            // terme predictif (vitesse projetee sous l'horizon de
                            // reaction) rattrape le vehicule encore >35 px/s mais qui
                            // FREINE vers l'arret DANS la boite : sans lui, il etait
                            // juge "fluide" -> j'entrais -> il s'arretait -> je me
                            // figeais au milieu.
                            constexpr float kReactHorizon = 0.6f;   // s
                            const float predSpeed =
                                other->getSpeed() + other->getCurrentAccel() * kReactHorizon;
                            if (other->getSpeed() > 35.f && predSpeed > 35.f) continue;
                            const core::Vec2 od = other->getPosition() - winAnchor;
                            if (od.x * od.x + od.y * od.y > winReach * winReach) continue;
                            const LaneProjection pr =
                                currentLane->project(other->getPosition(), blockFrom, sExit + need);
                            if (!pr.valid) continue;
                            // Couloir HEADING-AWARE pour la portion DANS la boite, mais
                            // SEULEMENT si l'autre est PHYSIQUEMENT dans cette boite.
                            // Un perpendiculaire arrete a SA ligne (hors boite, a 50 px
                            // de ma voie) ne doit PAS declencher le couloir elargi sinon
                            // les prioritaires en ligne droite (STOP majeur, feu vert
                            // tout-droit...) se figent en DEGAGE alors que la boite est
                            // libre. Un PERPENDICULAIRE DANS la boite s'etend de sa
                            // LONGUEUR lateralement chez moi -> couloir elargi pour le
                            // capter. APRES la boite ou hors boite : couloir etroit
                            // strict (22 px), ne filtre que la file de MEME sens.
                            const bool inBoxPortion = (pr.s <= sExit);
                            const bool otherInBox = inBoxPortion &&
                                (world.getIntersectionAt(other->getPosition().x,
                                                          other->getPosition().y) == interAhead);
                            float laterMax;
                            if (otherInBox) {
                                const float dHabs = std::abs(core::math::wrapDeg180(
                                    other->getHeading() - currentAngle));
                                const float thRad = dHabs * core::math::DEG2RAD;
                                const float oLatExtent =
                                    (other->getLength() * 0.5f) * std::abs(std::sin(thRad)) +
                                    (other->getBodySize().y * 0.5f) * std::abs(std::cos(thRad));
                                laterMax = visionParams.laneCorridorHalf + oLatExtent + 8.f;
                            } else {
                                laterMax = visionParams.laneCorridorHalf;
                            }
                            if (std::abs(pr.lateral) > laterMax) continue;
                            if (pr.s >= blockFrom && pr.s <= sExit + need) { blocked = true; break; }
                        }
                        constexpr float kFixedDt = 1.f / 60.f;
                        // KEEP-CLEAR ABSOLU : boite/sortie non degageable -> je
                        // n'entre PAS, sans limite de temps. (L'ancien override
                        // temporise FORCAIT l'entree apres un delai -> block-the-box ->
                        // GRIDLOCK PERMANENT.) Invariant : aucun vehicule ne s'IMMOBILISE
                        // dans la boite -> la boite reste franchissable -> le reseau se
                        // draine des qu'une sortie se libere. Les entrees simultanees
                        // conflictuelles sont tranchees en AMONT par la policy (droit de
                        // passage) et, a la boite, par le filet VIN -- JAMAIS en forcant
                        // l'entree dans une sortie pleine. Une vraie sur-saturation
                        // (toutes les sorties d'un cycle pleines, sans drainage) reste un
                        // probleme de DEBIT, pas de logique : insoluble localement, mais
                        // ici au moins RECUPERABLE (les boites restent degagees).
                        if (blocked) { exitBlocked = true; keepClearWaited_ += kFixedDt; }
                        else         { keepClearWaited_ = 0.f; }
                    } else {
                        keepClearWaited_ = 0.f;
                    }
                }
            }

            // --- Tiebreak NARROW pour entrees simultanees CROISEES (anti two-turner
            //     deadlock). Si deux trajectoires de virage se croisent dans la boite
            //     et que les deux entrent au meme pas -> wedge mutuel. On serialise :
            //     je cede a un perpendiculaire de plus PETIT VIN EN MOUVEMENT vers la
            //     boite, TRES proche du bord. Conditions strictes pour eviter le
            //     sur-yield aux files queue-ees a gauche/droite :
            //       - speed >= 12 px/s  : exclut les arretes (files, lignes de stop)
            //       - distOC <= 1.2 tile : juste au bord, pas en milieu d'approche
            //       - heading vers le centre boite (dot > 0.3)
            //       - cap perpendiculaire au mien (45 < dH < 135)
            //     Auto-correcteur : si le contender petit-VIN s'arrete (cale), il
            //     cesse d'etre un contender mouvant -> je ne lui cede plus.
            bool yieldToCrossingMover = false;
            {
                const int        myVin  = getVehicleId();
                const core::Vec2 interC = interAhead->getWorldCenter(tileSize);
                for (const auto& other : agents) {
                    if (!other || other.get() == this) continue;
                    const int ov = other->getVehicleId();
                    if (ov < 0 || ov >= myVin)    continue;
                    if (other->getSpeed() < 12.f) continue;
                    const float dH = std::abs(core::math::wrapDeg180(
                        other->getHeading() - currentAngle));
                    if (dH < 45.f || dH > 135.f) continue;
                    const core::Vec2 op = other->getPosition();
                    const core::Vec2 dC = interC - op;
                    const float distOC = dC.length();
                    if (distOC > tileSize * 1.2f) continue;
                    const float oRad = other->getHeading() * core::math::DEG2RAD;
                    const core::Vec2 oDir{ std::cos(oRad), std::sin(oRad) };
                    if (distOC > 1.f &&
                        (oDir.x * dC.x + oDir.y * dC.y) / distOC < 0.3f) continue;
                    yieldToCrossingMover = true;
                    break;
                }
            }

            const bool mayEnter = canPass && !exitBlocked && !yieldToCrossingMover;
            const bool proceed  = mayEnter || (cannotStop && !isStopAhead);

            if (proceed) {
                // On passe. Aucun leader virtuel d'intersection : une fois
                // PHYSIQUEMENT sur l'aire (interOn), le degagement est garanti.
                // EXCEPTION : si proceed n'est du QU'AU dilemme cannotStop (la
                // policy disait shouldStop mais je ne peux plus m'arreter), on
                // force un freinage d'urgence sur ce pas. Ralentir au max avant
                // d'entrer minimise le depassement de ligne. Sans cet override,
                // l'IDM (sans leader) accelerait vers v0 -> overshoot maximal.
                if (!mayEnter && cannotStop && decision.shouldStop) {
                    dilemmaBrakeOverride = true;
                }
            } else if (stopForceHalt) {
                // Arret ferme PILE a la ligne d'un STOP (gap 0, point FIXE).
                leader            = core::behavior::LeaderInfo{};
                leader.present    = true;
                leader.gap        = 0.f;
                leader.speed      = 0.f;
                leader.stopTarget = true;
                leaderIsVehicle   = false;
                leaderFromStop    = true;
            } else if (decision.shouldStop) {
                // Je cede : leader virtuel FIXE a la ligne d'arret.
                core::behavior::LeaderInfo virtualLeader;
                virtualLeader.present    = true;
                virtualLeader.gap        = std::max(decision.stopLineGap, 0.f);
                virtualLeader.speed      = 0.f;
                virtualLeader.stopTarget = true;
                if (!leader.present || virtualLeader.gap < leader.gap) {
                    leader          = virtualLeader;
                    leaderIsVehicle = false;
                    if      (dtype == RegulationType::TRAFFIC_LIGHT) leaderFromRed   = true;
                    else if (dtype == RegulationType::STOP)          leaderFromStop  = true;
                    else if (dtype == RegulationType::P2P)           leaderFromP2P   = true;
                    else                                             leaderFromYield = true;
                }
            } else if (exitBlocked) {
                // Droit de passage MAIS sortie pleine : j'attends a la ligne
                // d'ENTREE plutot que de bloquer la boite (et les flux croises).
                core::behavior::LeaderInfo box;
                box.present    = true;
                box.gap        = (sEnter >= 0.f)
                                 ? std::max((sEnter - s) - getLength() / 2.f, 0.f) : 0.f;
                box.speed      = 0.f;
                box.stopTarget = true;
                if (!leader.present || box.gap < leader.gap) {
                    leader          = box;
                    leaderIsVehicle = false;
                    leaderFromBox   = true;
                }
            } else if (yieldToCrossingMover) {
                // Tiebreak : un perpendiculaire au plus PETIT VIN va entrer en meme
                // temps que moi -> je cede a la ligne (serialise les deux turners).
                core::behavior::LeaderInfo cx;
                cx.present    = true;
                cx.gap        = std::max(distToInter - getLength() / 2.f - 4.f, 0.f);
                cx.speed      = 0.f;
                cx.stopTarget = true;
                if (!leader.present || cx.gap < leader.gap) {
                    leader          = cx;
                    leaderIsVehicle = false;
                    leaderFromYield = true;   // diag : cede (tiebreak VIN)
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

            // (keep clear : calcule plus haut -> integre directement a la decision
            //  d'engagement via 'exitBlocked', donc plus de bloc separe ici.)
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
            const float headingDiff =
                std::abs(core::math::wrapDeg180(other->getHeading() - currentAngle));
            // Empreinte de l'autre dans MON repere = projection de sa boite ORIENTEE
            // (longueur L, largeur W, cap relatif theta) sur mes axes :
            //   lateral  = (L/2)|sin t| + (W/2)|cos t|
            //   longi    = (L/2)|cos t| + (W/2)|sin t|
            // -> meme sens OU sens inverse (parallele, t≈0 ou π) : empreinte
            //    laterale = LARGEUR (un camion long sur la voie d'a cote ne barre
            //    PAS la mienne) ; CROISE (t≈π/2) : empreinte laterale = LONGUEUR
            //    (le corps du camion barre vraiment ma voie). Corrige le cas ou un
            //    camion (long de 80 px) bloquait les voitures d'une AUTRE voie.
            const float thRad      = headingDiff * core::math::DEG2RAD;
            const float oc         = std::abs(std::cos(thRad));
            const float os         = std::abs(std::sin(thRad));
            const float otherHalfL = other->getLength() / 2.f;       // = bodySize.x/2
            const float otherHalfW = other->getBodySize().y / 2.f;
            const float otherLatExtent = otherHalfL * os + otherHalfW * oc;
            const float otherLonExtent = otherHalfL * oc + otherHalfW * os;
            if (lateral > myHalfWidth + otherLatExtent + 3.f) continue;

            const float bumper = forwardDist - myHalfLen - otherLonExtent;

            // Composante de vitesse de l'autre le long de MON cap (croise -> ~0).
            const float oRad      = other->getHeading() * core::math::DEG2RAD;
            const core::Vec2 oFwd{ std::cos(oRad), std::sin(oRad) };
            const float oFwdSpeed = (oFwd.x * fwd.x + oFwd.y * fwd.y) * other->getSpeed();
            const float closing   = std::max(currentSpeed - oFwdSpeed, 0.f);

            // Marge proportionnelle a la vitesse de rapprochement (plus de
            // distance de reaction quand ca ferme vite), bornee petit.
            const float margin = 6.f + 0.25f * closing;
            if (bumper >= margin) continue;                         // pas imminent

            // --- EXCLUSIVITE EN BOITE (commit-to-clear strict) ---------------
            // Regle : une fois PHYSIQUEMENT engage sur l'aire d'intersection
            // (dbgOnInter_), je ne dois freiner QUE pour un leader same-direction
            // sur MA TRAJECTOIRE (file de sortie qui ralentit). Tout corps croise
            // / oppose / en virage DANS la boite est ignore -- la priorite a ete
            // jouee a l'entree (canEnter), une fois dedans on degage coute que
            // coute. Sinon : je freine pour un perpendiculaire qui passe ->
            // wedge mutuel insoluble.
            // GARDE imminent-crash : bumper tres petit (kHardMin) -> on freine
            // quand meme (collision absolue prime sur fluidite).
            constexpr float kBoxHardMin = 4.f;
            if (dbgOnInter_ && bumper > kBoxHardMin && headingDiff > 45.f) {
                continue;                                            // cross/oppose en boite -> ignore
            }

            // --- FAUX LEADER cross-lane (anti-blocage en/apres intersection) ---
            // Le check d'empreinte ci-dessus capture un perpendiculaire QUI VIT
            // dans la boite ou la borde -- meme si son corps n'est PAS sur ma
            // trajectoire reelle. Resultat : "je freine pour un vehicule de la
            // voie d'a cote" -> je fige -> les autres bloquent. Garde supplementaire :
            // hors imminent-crash (kHardMin), exiger que l'autre se projette sur
            // MA LIGNE (lane) dans un couloir resserre. Un perpendiculaire qui
            // traverse projette s>>s avec |lateral| grand (sa position est en
            // dehors du tracé curviligne) -> rejete. Un vrai leader same-direction
            // a |lateral| petit -> garde.
            // EXCEPTION : imminent-crash (bumper tres petit) -> on ne filtre pas,
            // securite collision absolue.
            constexpr float kCrossLaneHardMin = 8.f;
            if (currentLane && bumper > kCrossLaneHardMin) {
                const float reach = std::max(forwardDist + 30.f, 60.f);
                const LaneProjection pr =
                    currentLane->project(other->getPosition(), s, s + reach);
                if (!pr.valid) continue;
                if (pr.s <= s) continue;                            // pied derriere -> pas un leader
                // Corridor : largeur de ma voie + demi-empreinte LATERALE de l'autre
                // dans MON repere (tient compte de son cap relatif). Tres etroit pour
                // les meme-sens / oppose (parallele), large pour un perpendiculaire
                // REELLEMENT en travers de ma voie. Pour un perpendiculaire sur une
                // VOIE adjacente (pas sur ma trajectoire), pr.lateral est l'ecart
                // CARTESIEN au tracé -- grand -> rejete.
                const float corridorHalf =
                    visionParams.laneCorridorHalf + otherLatExtent * 0.5f + 4.f;
                if (std::abs(pr.lateral) > corridorHalf) continue;  // pas sur ma voie
            }

            // --- Anti-DEADLOCK -----------------------------------------------
            // Trafic de MEME sens (suivi de file) -> je freine TOUJOURS : une
            // chaine de suiveurs ne se bloque jamais (sa tete est libre, seul le
            // suiveur arriere freine, pas reciproque).
            // CONFLIT croise/oppose (cap different) -> ordre TOTAL par VIN : je ne
            // cede qu'a un VIN plus PETIT (prioritaire). Dans n'importe quel CYCLE
            // de conflits (A cede a B, B a C, C a A : gel rotatif dans le
            // carrefour), le plus petit VIN ne cede a personne -> il avance et
            // brise le blocage. Resout les cycles a N vehicules, pas seulement les
            // paires (c'etait la limite de la version precedente).
            // En DEPASSEMENT, un vehicule quasi-oppose (>135°) est un FRONTAL sur
            // la voie d'en face : l'ordre par VIN ne s'y applique pas (un frontal
            // ne se "negocie" pas). Je freine TOUJOURS -> jamais de collision tete-
            // a-queue pendant un double rate (criticite de securite MOBIL b_safe).
            const bool overtakeHeadOn =
                (overtakeState != OvertakeState::NONE) && (headingDiff > 135.f);
            // CHEVAUCHEMENT IMMINENT (carrosseries qui se touchent presque) : on
            // freine TOUJOURS, meme prioritaire. La priorite n'autorise PAS a
            // traverser le corps d'un autre -> c'est ce qui faisait "passer" les
            // voitures par-dessus les camions (longs) aux intersections. L'ordre
            // par VIN ne sert qu'a departager TANT QU'IL RESTE une marge.
            constexpr float kHardMin = 5.f;
            // Bris d'egalite par VIN pour le conflit CROISE (ordre TOTAL -> casse
            // tout cycle a N vehicules : le plus petit VIN ne cede jamais, brise le
            // gel rotatif). S'applique aussi a un corps a l'arret : c'est CE bypass
            // qui permet au plus petit VIN d'AVANCER (jusqu'a kHardMin) pour degager
            // un standoff dans la boite. Sans lui, deux corps perpendiculaires
            // figes restent figes pour toujours. Le garde 'bumper>kHardMin' borne
            // l'avance : a chevauchement imminent on freine TOUJOURS (la priorite
            // n'autorise pas a traverser un corps).
            if (headingDiff >= 45.f && !overtakeHeadOn && bumper > kHardMin) {
                const int myV = getVehicleId();
                const int oV  = other->getVehicleId();
                const bool otherHasPriority = (oV >= 0) && (myV < 0 || oV < myV);
                if (!otherHasPriority) continue;             // priorite + marge -> je passe
            }

            // Leader d'urgence : on retient le plus contraignant (plus petit gap).
            if (!leader.present || bumper < leader.gap) {
                leader            = core::behavior::LeaderInfo{};
                leader.present    = true;
                leader.gap        = std::max(bumper, 0.f);
                leader.speed      = std::max(oFwdSpeed, 0.f);
                leader.stopTarget = (oFwdSpeed < 5.f);   // obstacle ~fixe -> arret net
                leaderIsVehicle   = true;
                dbgLeaderPtr      = other.get();         // diag
                dbgSrc            = 2;
            }
        }
    }

    // --- Loi de poursuite longitudinale : CACC (peloton connecte) ou IDM ------
    if (personality_.cooperative && leaderIsVehicle &&
        lastPerception.hasDirectObstacle && lastPerception.directObstacleAgent) {
        // PELOTON COOPERATIF (CACC) : distance CONSTANTE + feed-forward de
        // l'acceleration du predecesseur (communiquee). Consequences voulues :
        //   * demarrage en UNISSON (le suiveur recoit aLead, pas besoin d'attendre
        //     que le gap s'ouvre -> aucun delai de reaction perceptible) ;
        //   * intervalle court CONSTANT (gapDesired fixe), qui NE s'ouvre PAS
        //     quand la vitesse augmente -- impossible avec l'IDM seul ou le gain
        //     d'equilibre vaut s0 + v*T.
        const float vLead      = lastPerception.directObstacleSpeed;
        const float aLead      = lastPerception.directObstacleAgent->getCurrentAccel();
        const float gap        = lastPerception.directObstacleDistance;
        const float gapDesired = idm.params().s0 + 12.f;        // px : distance fixe courte
        constexpr float kp = 0.6f, kv = 1.2f;
        float aCacc = aLead + kv * (vLead - currentSpeed) + kp * (gap - gapDesired);
        // Ne jamais depasser l'allure libre (respect de v0) ; borner aux capacites.
        const float aFree = idm.computeAcceleration(currentSpeed, v0,
                                                    core::behavior::LeaderInfo{});
        aCacc = std::min(aCacc, aFree);
        pendingAccel = std::clamp(aCacc, -idm.params().bComf * 2.f, idm.params().aMax);
    } else {
        // --- IDM ---
        pendingAccel = idm.computeAcceleration(currentSpeed, v0, leader);
    }

    // STOP : marquage a la ligne. Frein FERME mais LISSE. Le leader virtuel
    // colle (gap 0) ferait tendre l'IDM vers une deceleration quasi infinie qui
    // annule la vitesse en UNE frame (arret en a-coup). On impose a la place une
    // deceleration bornee (~0.75x le confort) : l'arret se fait en ~0.5 s,
    // franc mais souple. L'anti-fluage (kCreepSpeed) absorbe le dernier px/s.
    if (stopForceHalt && !stopReleased) {
        pendingAccel = -idm.params().bComf * 0.75f;
    }

    // Dilemme cannotStop : freinage d'urgence simultane a l'engagement. Reduit
    // drastiquement le depassement de ligne par rapport au "proceed sans frein"
    // (qui laissait l'IDM accelerer vers v0). Une fois interOn (boite), cette
    // branche ne fire plus -> je degage normalement (pas de blocage en boite).
    if (dilemmaBrakeOverride) {
        pendingAccel = std::min(pendingAccel, -idm.params().bComf * 2.f);
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
    } else if (leaderFromBox) {
        currentBlockReason = BlockReason::KEEP_CLEAR;
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

    // --- Deadlock auto-recovery (AGRESSIVE) ---------------------------------
    // Aucun blocage mutuel ne doit durer plus de quelques secondes. Deux modes :
    //
    //   MODE A -- ma voie est PHYSIQUEMENT vide devant : creep immediat.
    //             (typique : cycle rotatif non resolu, faux leader cross-lane
    //              residuel, attente fantome.)
    //
    //   MODE B -- un corps BLOQUE ma voie ET ce corps est lui-meme A L'ARRET
    //             (mutual wedge dans/autour la boite : croises figes l'un
    //             contre l'autre, opposes nez-a-nez, deux turners qui se
    //             coupent). VIN-arbitrage : si j'ai le plus PETIT VIN du
    //             cluster fige, je force le passage MEME si cela signifie
    //             traverser brievement la boite de l'autre. Le plus GROS VIN
    //             attend -- des que je passe, il se libere naturellement.
    //             Sans cela, le wedge geometrique est indeblocable (le bumper
    //             tend vers 0 -> brake reciproque permanent).
    //
    // Attentes LEGITIMES (feu rouge, STOP, peloton, depass, panne) : exclues
    // -- un agent qui attend correctement un signal ne doit pas etre force.
    {
        constexpr float kFixedDt = 1.f / 60.f;
        // Liste DRASTIQUEMENT reduite : seules les vraies attentes immuables
        // (panne, but, no_path) et le rouge restent. Les ex-"legitimes"
        // (STOP, PLATOONING, OVERTAKING) etaient des trous : un blocage
        // pathologique deguise en l'un d'eux ne se debloquait jamais.
        const bool legitWait =
            currentBlockReason == BlockReason::INTERSECTION_RED  ||
            currentBlockReason == BlockReason::BREAKDOWN         ||
            currentBlockReason == BlockReason::AT_GOAL           ||
            currentBlockReason == BlockReason::NO_PATH;
        // Condition d'accumulation TRES LARGE : tout proche de l'arret compte,
        // peu importe le pendingAccel. (L'ancien `pendingAccel<1` ratait les
        // frames ou le force-advance precedent avait mis pendingAccel haut puis
        // ou l'IDM ramenait juste apres en negatif violent.)
        if (!legitWait && currentSpeed < 8.f) {
            deadlockStuckTime_ += kFixedDt;
        } else if (currentSpeed > 15.f) {
            deadlockStuckTime_ = 0.f;
        }

        // Grace agressive. En boite : 0.2 s (paralyser les flux croises est
        // INACCEPTABLE). Hors boite : 0.7 s (assez court pour ne pas faire
        // attendre l'utilisateur, assez long pour qu'une file IDM normale
        // n'oscille pas).
        const float kDeadlockGrace = dbgOnInter_ ? 0.2f : 0.7f;
        if (deadlockStuckTime_ > kDeadlockGrace && currentLane) {
            // Inspection : qu'est-ce qui me barre VRAIMENT la voie ?
            // CRITERE STRICT : SAME-DIRECTION uniquement. Un perpendiculaire /
            // oppose dans la boite traverse PHYSIQUEMENT mon tracé curvilineaire
            // (lane projection lateral ~0 a l'intersection -- piege !) mais
            // n'est PAS un blocker de file. Filtre par cap : headingDiff < 45°.
            constexpr float kClearScan = 70.f;
            bool        laneAheadClear = true;
            const IAgent* blocker     = nullptr;
            for (const auto& other : agents) {
                if (!other || other.get() == this) continue;
                const core::Vec2 dp = other->getPosition() - position;
                if (dp.x * dp.x + dp.y * dp.y > 110.f * 110.f) continue;
                // FILTRE CAP : ne considerer comme blocker que les SAME-DIRECTION.
                // Sinon, un croise/oppose dans la boite (mon lane passe DESSUS)
                // serait pris pour un suiveur de file et empecherait MODE A.
                const float hDiff = std::abs(core::math::wrapDeg180(
                    other->getHeading() - currentAngle));
                if (hDiff > 45.f) continue;
                const LaneProjection pr =
                    currentLane->project(other->getPosition(), s, s + kClearScan);
                if (!pr.valid) continue;
                if (pr.s <= s) continue;
                if (std::abs(pr.lateral) >
                    visionParams.laneCorridorHalf + 6.f) continue;
                laneAheadClear = false;
                if (!blocker ||
                    (other->getPosition() - position).length() <
                    (blocker->getPosition() - position).length()) {
                    blocker = other.get();
                }
            }

            bool forceAdvance = false;
            // EN BOITE : declenchement INCONDITIONNEL apres grace. Une fois
            // dans la boite, peu importe qui me barre la voie, je DOIS sortir
            // (sinon je paralyse toutes les approches croisees). Aucune
            // arbitrage VIN : tout le monde en boite force a la fois -> le
            // wedge se desserre.
            if (dbgOnInter_) {
                forceAdvance = true;
            } else if (laneAheadClear) {
                forceAdvance = true;                                  // MODE A
            } else if (blocker && blocker->getSpeed() < 8.f) {
                // MODE B : wedge mutuel. VIN-arbitrage sur tout le cluster
                // d'agents stationnaires autour de nous : suis-je le plus
                // petit VIN ? Sinon -> j'attends, le petit VIN forcera.
                const int myVin = getVehicleId();
                bool isSmallestStuckVin = true;
                for (const auto& other : agents) {
                    if (!other || other.get() == this) continue;
                    if (other->getSpeed() > 8.f) continue;            // bouge -> hors cluster
                    const core::Vec2 dp = other->getPosition() - position;
                    if (dp.x * dp.x + dp.y * dp.y > 95.f * 95.f) continue;
                    const int oVin = other->getVehicleId();
                    if (oVin >= 0 && (myVin < 0 || oVin < myVin)) {
                        isSmallestStuckVin = false;
                        break;
                    }
                }
                if (isSmallestStuckVin) forceAdvance = true;
            }

            if (forceAdvance) {
                // Override absolu : on neutralise tout frein (safety net, IDM,
                // stopForceHalt residuel) et on impose un creep franc. Bornes
                // basses (40 px/s) suffisent a degager le wedge en ~0.5 s sans
                // creer de pic de vitesse dangereux.
                pendingAccel        = std::max(idm.params().aMax * 0.8f,
                                               pendingAccel);
                pendingDesiredSpeed = std::max(pendingDesiredSpeed, 40.f);
                currentBlockReason  = BlockReason::NONE;
                // Hysteresis : decrement plutot que reset complet, pour
                // redeclencher rapidement si le wedge persiste sur le pas suivant.
                deadlockStuckTime_  = std::max(0.f, deadlockStuckTime_ - 0.6f);
            }
        }
    }

    // --- Finalisation instrumentation diagnostic ---
    if (dbgLeaderPtr) {
        dbgLeaderSrc_           = dbgSrc;                  // 1 perception / 2 filet
        dbgLeaderVin_           = dbgLeaderPtr->getVehicleId();
        dbgLeaderRelHeadingDeg_ =
            core::math::wrapDeg180(dbgLeaderPtr->getHeading() - currentAngle);
    } else if (leaderFromRed || leaderFromStop || leaderFromYield ||
               leaderFromP2P  || leaderFromPlatoon || leaderFromBox) {
        dbgLeaderSrc_ = 3;                                 // leader virtuel de policy
    }
    if (leader.present) { dbgLeaderGap_ = leader.gap; dbgLeaderSpeed_ = leader.speed; }
}

// -----------------------------------------------------------------
// PHASE 2 : integrate (Euler, dt fixe)
// -----------------------------------------------------------------

void Vehicle::integrate(float dt) {
    // Publie la commande d'acceleration de CE pas pour le feed-forward CACC des
    // suiveurs (lue par eux au pas SUIVANT, en phase 1 -> jamais de course).
    lastAccel_ = pendingAccel;

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
    // EXCEPTION : on est PHYSIQUEMENT sur l'aire d'une intersection (dbgOnInter_).
    // Coller currentSpeed a 0 dans la boite cree un wedge persistant : le
    // vehicule freine pour un faux leader (cross-lane), retombe a 0, et bloque
    // les flux croises. On laisse l'IDM se debrouiller -> degagement naturel.
    constexpr float kCreepSpeed = 14.f;   // px/s
    constexpr float kCreepAccel = 2.f;    // px/s^2
    if (!dbgOnInter_ &&
        currentSpeed < kCreepSpeed && pendingAccel < kCreepAccel) {
        currentSpeed = 0.f;
    }

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

Vehicle::DecisionDiagnostic Vehicle::getDecisionDiagnostic() const {
    DecisionDiagnostic d;
    d.vin                 = vehicleId_;
    d.x                   = position.x;
    d.y                   = position.y;
    d.headingDeg          = currentAngle;
    d.speed               = currentSpeed;
    d.blockReason         = static_cast<int>(currentBlockReason);
    d.leaderSource        = dbgLeaderSrc_;
    d.leaderVin           = dbgLeaderVin_;
    d.leaderRelHeadingDeg = dbgLeaderRelHeadingDeg_;
    d.leaderGap           = dbgLeaderGap_;
    d.leaderSpeed         = dbgLeaderSpeed_;
    d.onIntersection      = dbgOnInter_;
    return d;
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
    stopEverYielded_     = false;
    keepClearWaited_     = 0.f;
    deadlockStuckTime_   = 0.f;

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
    // |delta| faible -> tout droit ; sinon le SIGNE du delta donne le sens.
    if (!currentLane || hasFinishedPath) return core::agent::TurnIntent::UNKNOWN;
    const float here  = currentLane->getHeadingAt(s);
    const float ahead = currentLane->getHeadingAt(
        std::min(s + tileSize * 3.f, currentLane->getLength()));
    const float delta = core::math::wrapDeg180(ahead - here);   // signe = sens
    if (std::abs(delta) < 30.f) return core::agent::TurnIntent::STRAIGHT;
    // Convention ecran : x+ = EST, y+ = SUD, cap croissant = horaire (0 E, 90 S,
    // 180 O, 270 N). Un cap qui DECROIT (delta<0 : E->N, S->E, O->S, N->O) tourne
    // vers la GAUCHE du conducteur ; qui croit -> DROITE.
    return (delta < 0.f) ? core::agent::TurnIntent::LEFT
                         : core::agent::TurnIntent::RIGHT;
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

        // Critere de securite MOBIL adapte au depassement bidirectionnel.
        // Le "nouveau suiveur" pertinent est le vehicule en sens INVERSE : la
        // manoeuvre n'est sure que s'il n'aurait PAS a freiner plus fort que
        // b_safe pour eviter le frontal, ET qu'il reste une marge temporelle.
        const float closingSpeed = std::max(currentSpeed + other->getSpeed(), 30.f);
        const float ttc = forwardDist / closingSpeed;
        if (ttc < 4.0f) return false;                       // marge temporelle mini
        // Deceleration que l'oncoming devrait subir pour s'arreter avant l'impact.
        const float reqDecel = (closingSpeed * closingSpeed) / (2.f * std::max(forwardDist, 1.f));
        const float bSafe    = idm.params().bComf * 2.0f;   // freinage d'urgence tolere
        if (reqDecel > bSafe) return false;
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

    // SECURITE (use-after-free) : le vehicule double (overtakeLeader) a pu etre
    // DETRUIT entre deux pas (il est arrive a destination -> efface du vecteur).
    // overtakeLeader est un pointeur NU conserve d'un pas a l'autre : on verifie
    // donc qu'il pointe toujours sur un agent VIVANT avant tout dereferencement
    // (cf. crash de la scene XXXXL apres ~10 s, quand les premiers arrivent).
    if (overtakeLeader) {
        bool stillAlive = false;
        for (const auto& a : agents)
            if (a.get() == overtakeLeader) { stillAlive = true; break; }
        if (!stillAlive) {
            // Le vehicule double a disparu -> on annule la reference pendante et
            // on se rabat proprement dans la voie.
            overtakeLeader = nullptr;
            overtakeState  = OvertakeState::RETURNING;
            overtakeTarget = 0.f;
        }
    }

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
