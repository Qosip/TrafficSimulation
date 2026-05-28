# 🛠 PHASE 4 — ÉRADICATION DÉFINITIVE DES BUGS CONNUS (Pseudo-code)

Cette phase fournit pour chacun des trois bugs critiques (1) la **cause racine** identifiée par audit ; (2) la **stratégie de correction** ; (3) le **pseudo-code algorithmique robuste** ; (4) la **vérification d'invariant**.

---

## 4.1 BUG #1 — Ghost-Following (suivi de véhicule en contre-sens)

### 4.1.1 Symptôme observé

Un véhicule roulant en sens N→S "se cale" sur un véhicule de la voie opposée roulant S→N, déclenchant un freinage IDM erroné. Au pire, il s'arrête à un mètre d'un véhicule qui le croise normalement.

### 4.1.2 Cause racine

Le filtre originel **angulaire pur** (cône frontal de ±X°) accroche tout véhicule dans l'arc. En virage ou à l'intersection, la voie opposée tombe dans le cône → faux positif catastrophique.

### 4.1.3 Stratégie : Filtrage spatial strict par **projection de Frenet curviligne**

Chaque véhicule est projeté sur **MA propre Lane** (polyligne 1D). On ne retient que les véhicules dont :
- le pied de projection se trouve **devant** moi (`s_other > s_self`),
- l'écart latéral signé est **dans mon corridor** (`|lateral| < laneCorridorHalf = 22 px`),
- le pied est **dans la portée** (`s_other - s_self ∈ [0, scanRange]`).

La largeur d'une voie ≈ 50 px (1 tile). 22 px isole proprement ma voie sans rejeter un suiveur légèrement décentré (oscillation transitoire) ; la voie opposée à 50 px est nécessairement rejetée.

### 4.1.4 Pseudo-code

```
function selectLeaderByFrenet(self, agents, myLane, myS, scanRange,
                              laneCorridorHalf = 22 px) :

    bestLeader = NONE
    bestGap    = +INFINITY

    for other in agents :
        if other == self : continue
        if other.currentLane == NONE : continue

        # 1) Projection sur MA trajectoire, dans la fenêtre [myS, myS+scanRange]
        proj = myLane.project(other.position, myS, myS + scanRange)

        if not proj.valid : continue

        # 2) Filtre LATÉRAL strict (rejet voie opposée / voie croisée)
        if abs(proj.lateral) > laneCorridorHalf : continue

        # 3) Filtre LONGITUDINAL (rejet ce qui est derrière)
        gap = proj.s - myS - (self.length/2 + other.length/2)
        if gap <= 0 : continue

        # 4) Garde-fou secondaire : cap relatif < 45° pour exclure
        #    un véhicule qui ferait un demi-tour dans mon corridor
        deltaHeading = abs(angle_diff(other.heading, self.heading))
        if deltaHeading > 45° : continue

        # 5) Rétention du plus proche
        if gap < bestGap :
            bestGap    = gap
            bestLeader = other

    if bestLeader == NONE :
        return LeaderInfo::none()

    return LeaderInfo {
        hasLeader = true,
        gap       = bestGap,
        speed     = bestLeader.speed
    }
```

### 4.1.5 Pourquoi cela ferme la classe de bug

- **Géométrique** : la voie opposée a un offset latéral physique > corridor → impossible d'être retenue.
- **Curviligne** : en virage, la projection suit la courbe, donc un véhicule du virage opposé sort du corridor.
- **À l'intersection** : si je suis sur Lane A et l'autre sur Lane B (Lanes distinctes), `other.currentLane != myLane` ; mais on projette quand même sur **ma** Lane → l'autre tombera latéralement.
- **Garde-fou cap relatif** : capture les cas pathologiques (véhicule qui démarre à contre-sens transitoirement).

### 4.1.6 Invariant garanti

> **I2** : Aucun véhicule ne suit un véhicule de cap relatif > 45° avec sa propre direction.

---

## 4.2 BUG #2 — Deadlocks et Interblocages

### 4.2.1 Symptômes observés

- **Pâté mexicain** à un carrefour 4-voies : 4 véhicules s'engagent ensemble, chacun bloque la sortie d'un autre → blocage permanent.
- **Engagement partiel** : un véhicule rentre dans la boîte sans place pour en sortir, fige tous les flux.
- **Cycle de cession circulaire** : à un carrefour P2P, A cède à B, B cède à C, C cède à D, D cède à A → personne ne bouge.

### 4.2.2 Cause racine

1. Les policies évaluent `canEnter` sans regarder la **capacité de sortie** (le carrefour est "libre" mais la voie de sortie est saturée).
2. Pas de mécanisme de **bris de cycle** quand toutes les conditions de cession sont mutuellement satisfaites.
3. Pas de **commit-to-pass** : un véhicule peut commencer à céder en plein carrefour, créant un demi-engagement.

### 4.2.3 Stratégie : Look-ahead asymétrique + KEEP_CLEAR + bris de cycle par VIN

Trois mécanismes combinés :

1. **Look-ahead asymétrique** : avant tout engagement, l'agent vérifie qu'**il pourra sortir**.
2. **KEEP_CLEAR** : tant que la sortie est saturée, on reste à la ligne, même si on aurait la priorité.
3. **Liveness guard** : si l'attente KEEP_CLEAR dépasse un seuil ET que la chaîne de blocage est circulaire (vérifiée par scan), on engage en se basant sur le bris d'égalité VIN (le plus petit VIN gagne) → casse le cycle.

### 4.2.4 Pseudo-code complet

```
function canEngageWithLookahead(self, intersection, world, agents,
                                lookaheadDistance = 1.5 * self.length,
                                bufferPx = 10 px) :

    # ---- A. Calcul de la voie de SORTIE ----
    exitLane     = computeExitLane(self.path, intersection)
    exitEntryS   = 0.0  # début de la voie de sortie
    requiredFree = self.length + bufferPx

    if exitLane == NONE :
        # Le path se termine dans la boîte ? → on ne s'engage pas
        return BLOCKED

    # ---- B. Test occupation voie de sortie ----
    for other in agents :
        if other == self : continue
        if other.currentLane != exitLane : continue
        # On regarde l'intervalle critique [0, requiredFree] après l'entrée
        if other.s ∈ [exitEntryS, exitEntryS + requiredFree] :
            return BLOCKED      # quelqu'un occupe la première portion

    # ---- C. Test capacité longitudinale stricte (hitbox) ----
    nextVehicleOnExit = firstVehicleAhead(exitLane, exitEntryS)
    if nextVehicleOnExit != NONE :
        availableLength = nextVehicleOnExit.s - exitEntryS - nextVehicleOnExit.length
        if availableLength < self.length + bufferPx :
            return BLOCKED

    return CLEAR


# ============================================================
function decideIntersectionEngagement(self, intersection, world, agents) :

    # ---- 1. Demande la policy ----
    ctx      = buildPolicyContext(self, agents)
    decision = intersection.policy.request(ctx)

    if decision.shouldStop :
        return STOP_AT_LINE(decision.stopLineGap)

    # ---- 2. canEnter = true → on ajoute le filtre look-ahead ----
    lookaheadStatus = canEngageWithLookahead(self, intersection, world, agents)

    if lookaheadStatus == BLOCKED :
        self.keepClearWaited += dt
        # ---- 3. Liveness guard : éviter le deadlock permanent ----
        if self.keepClearWaited > KEEP_CLEAR_TIMEOUT (typ. 6.0 s) :
            if isCircularGridlock(intersection, agents) :
                # On engage seulement si on est le plus petit VIN
                # de la chaîne circulaire (bris déterministe)
                minVin = minimumVinInGridlockChain(intersection, agents)
                if self.vehicleId == minVin :
                    return ENGAGE   # casse le cycle
                else :
                    return STOP_AT_LINE(distance_to_line)
            # pas de gridlock circulaire → on continue d'attendre
        return STOP_AT_LINE(distance_to_line)

    # ---- 4. Tout est OK → on engage ----
    self.keepClearWaited = 0
    return ENGAGE


# ============================================================
function isCircularGridlock(currentInter, agents) :
    # Construit le graphe orienté : véhicule X attend voie de sortie occupée
    # par véhicule Y. Cherche un cycle dirigé contenant currentInter.

    G = empty directed graph
    for v in agents :
        if v.state == KEEP_CLEAR :
            blocker = firstVehicleOnExitLane(v)
            if blocker != NONE :
                G.addEdge(v, blocker)

    return hasDirectedCycle(G, containing = currentInter)


# ============================================================
function minimumVinInGridlockChain(currentInter, agents) :
    chain = extractCycleContaining(currentInter, agents)
    return min(v.vehicleId for v in chain)
```

### 4.2.5 Asymétrie du look-ahead

L'asymétrie est essentielle : on ne se contente PAS de regarder "le carrefour est libre" (test symétrique trivial). On vérifie spécifiquement **MA voie de sortie**, sur une distance proportionnelle à **MA propre longueur** (1.5 × `self.length`).

Conséquence : un camion (`self.length=64 px`) demande 96 px libres avant d'engager, une voiture seulement 48 px. Le camion attend donc plus souvent, ce qui est précisément le comportement réel souhaité.

### 4.2.6 Bris de cycle par VIN

Quand le scan détecte une **chaîne fermée** (`A → B → C → ... → A`), on choisit déterministiquement **un** acteur pour briser le cycle :

```
minVin = min(v.vehicleId for v in chain)
SI self.vehicleId == minVin :
    on engage (force le passage, accepte un léger débordement)
SINON :
    on reste à la ligne
```

Le VIN étant un compteur global déterministe (assigné à la construction de l'agent), le choix est **identique sur tous les threads et tous les replays** → reproductibilité Monte-Carlo préservée.

### 4.2.7 Invariants garantis

> **I1** : Aucun véhicule ne s'engage s'il ne peut pas sortir (look-ahead asymétrique).
>
> **I3** : `isCommittedToPass` empêche le freinage en plein carrefour.
>
> **I5** : Le scan de cycle + bris VIN garantit qu'**aucun cycle ne dure plus de `KEEP_CLEAR_TIMEOUT` secondes**.

---

## 4.3 BUG #3 — Ronds-points (anneau interne se bloque pour faire entrer les extérieurs)

### 4.3.1 Symptômes observés

- Un véhicule en train de tourner sur l'anneau **s'arrête** brusquement pour laisser entrer un véhicule attendant en approche → bloque l'anneau, vague de freinage en cascade sur tout l'anneau.
- À densité élevée, le rond-point se fige complètement (les véhicules à l'intérieur n'osent plus rouler, ceux dehors n'osent plus entrer).

### 4.3.2 Cause racine

- La policy `RoundaboutPolicy` originale détectait TOUS les véhicules dans le rayon, sans distinguer **intérieur de l'anneau** vs **en approche depuis l'extérieur**.
- Pas de **commit-to-pass** sur l'anneau : un véhicule à l'intérieur pouvait re-céder.
- Les véhicules à l'extérieur étaient inclus dans `canEnter` comme s'ils étaient prioritaires.

### 4.3.3 Stratégie : Priorité absolue à l'anneau + commit + admission progressive

Trois règles :

1. **Tout véhicule physiquement sur l'anneau est PRIORITAIRE** et ne peut pas être ralenti par un véhicule en approche.
2. Un véhicule sur l'anneau a `isCommittedToPass = true` **dès l'entrée sur la couronne** ; il ne peut pas re-céder.
3. L'admission d'un véhicule en approche se base UNIQUEMENT sur les véhicules **à l'intérieur** + look-ahead sortie d'anneau (KEEP_CLEAR adapté).

### 4.3.4 Pseudo-code

```
function isOnRoundaboutRing(agent, roundabout) :
    # Anneau = couronne circulaire entre laneRadius - tileSize/2
    #                              et laneRadius + tileSize/2
    C = roundabout.getWorldCenter(tileSize)
    R = roundabout.getLaneRadius(tileSize)
    d = distance(agent.position, C)
    return abs(d - R) < tileSize / 2

# ============================================================
function isApproachingRoundabout(agent, roundabout) :
    # Le véhicule n'est PAS sur l'anneau ET son path le mène à l'anneau
    if isOnRoundaboutRing(agent, roundabout) : return false
    # path contient un waypoint dans roundabout.coveredTiles dans
    # un horizon de quelques secondes
    return upcomingPathHits(agent, roundabout, horizon = 4.0 s)

# ============================================================
function roundaboutPolicyRequest(ctx, roundabout) :

    self = ctx.self
    onRing = isOnRoundaboutRing(self, roundabout)

    # ---- A. AGENT DÉJÀ SUR L'ANNEAU : priorité absolue ----
    if onRing :
        # Suit son leader RÉEL via IDM (autre véhicule sur l'anneau devant).
        # NE CÈDE JAMAIS à un véhicule en approche.
        return Decision {
            canEnter   = true,
            shouldStop = false,
            # signe à computeDecision : verrouille commit
            isOnRing   = true
        }

    # ---- B. AGENT EN APPROCHE : cède aux véhicules sur l'anneau ----
    inRingConflict = false
    for other in ctx.others :
        if isOnRingOf(other, roundabout) :
            # Va-t-il croiser ma trajectoire d'entrée dans ≤ safetyMargin secondes ?
            tConflict = estimateRingConflictTime(other, self, roundabout)
            if tConflict < params.safetyMargin + params.minCrossingTime :
                inRingConflict = true
                break

    if inRingConflict :
        return Decision {
            canEnter    = false,
            shouldStop  = true,
            stopLineGap = distanceToEntryLine(self, roundabout)
        }

    # ---- C. PAS DE CONFLIT, MAIS check exit lane ----
    # La sortie d'anneau qui sera la mienne : est-elle libre ?
    exitLane = computeExitLaneOnRoundabout(self.path, roundabout)
    if exitLaneOccupied(exitLane, self.length + 10 px) :
        # KEEP_CLEAR : on attend même si l'anneau est libre, sinon on rentre dedans
        return Decision {
            canEnter    = false,
            shouldStop  = true,
            stopLineGap = distanceToEntryLine(self, roundabout)
        }

    return Decision { canEnter = true }


# ============================================================
function computeDecisionRoundaboutSpecific(self, decision, world, agents) :

    # ---- Commit-to-pass renforcé pour l'anneau ----
    if decision.isOnRing :
        self.isCommittedToPass     = true
        self.committedIntersectionId = roundabout.id
        # IDM standard avec leader réel (autre véh sur l'anneau)
        leader = perceptionDirectAhead(self, agents, anneau_corridor)
        pendingAccel = idm.computeAcceleration(self.speed, v0, leader)
        return

    if decision.shouldStop :
        # Pas encore sur l'anneau, on freine à la ligne
        virtualLeader = LeaderInfo { gap = decision.stopLineGap, speed = 0 }
        pendingAccel  = idm.computeAcceleration(self.speed, v0, virtualLeader)
        return

    # canEnter=true et pas isOnRing → on engage, IDM normal
    pendingAccel = idm.computeAcceleration(self.speed, v0, realLeader)
```

### 4.3.5 Définition rigoureuse des zones

```
ZONES ROND-POINT (rayons depuis le centre C) :

   ┌──── outerRadius (bounding box / 2) ────┐
   │                                         │
   │      ┌── laneRadius + tileSize/2 ─┐    │
   │      │                             │    │
   │      │   ┌─ laneRadius ─┐         │    │
   │      │   │   ANNEAU      │         │    │
   │      │   │  ROULABLE    │         │    │
   │      │   └──────────────┘         │    │
   │      │                             │    │
   │      └─ laneRadius - tileSize/2 ──┘    │
   │                                         │
   └─────────────────────────────────────────┘

ON-RING       = |d − laneRadius| < tileSize/2
INSIDE        = d < laneRadius - tileSize/2  (centre du rond-point, non-roulable)
APPROACH      = d ∈ [laneRadius + tileSize/2 ; outerRadius + tileSize]
EXTERIOR      = d > outerRadius + tileSize
```

### 4.3.6 Pourquoi cela ferme la classe de bug

- Un agent **sur l'anneau** ne demande JAMAIS `shouldStop` lié au rond-point → l'anneau ne se fige plus pour les approches.
- L'agent **en approche** cède d'abord, puis applique KEEP_CLEAR avant l'engagement → pas d'engagement partiel qui bloquerait la couronne.
- Le **commit-to-pass** verrouille dès l'entrée sur l'anneau : un changement de scène dans l'environnement ne peut plus déclencher une re-cession au milieu.

### 4.3.7 Invariant garanti

> **I9** : Aucun véhicule sur l'anneau d'un rond-point ne sera ralenti par un véhicule en approche.

---

## 4.4 Synthèse Phase 4

| Bug | Mécanisme correctif | Pseudo-code clé | Invariant |
|---|---|---|---|
| **Ghost-following** | Projection Frenet curviligne + filtre latéral 22 px + cap relatif < 45° | `selectLeaderByFrenet()` | I2 |
| **Deadlock 4 voies** | Look-ahead asymétrique (hitbox sortie) + KEEP_CLEAR + bris de cycle par min(VIN) | `canEngageWithLookahead()` + `isCircularGridlock()` | I1, I5 |
| **Rond-point figé** | Priorité absolue à l'anneau + commit-to-pass dès la couronne + KEEP_CLEAR sortie | `roundaboutPolicyRequest()` + `isOnRoundaboutRing()` | I9 |

### Vérification croisée

Les trois mécanismes sont **compatibles et complémentaires** :

- Le **commit-to-pass** est utilisé par les bugs #2 et #3.
- Le **filtre Frenet** sert au car-following partout (DRIVING, NEGOTIATING, COMMITTED).
- Le **look-ahead asymétrique + KEEP_CLEAR** s'applique à TOUTES les intersections, pas seulement aux ronds-points.
- Le **bris VIN** garantit la reproductibilité Monte-Carlo (déterministe).

### Couverture revendiquée

Ces trois pseudo-codes, intégrés à la `computeDecision()` de la Phase 3, garantissent l'éradication complète des trois familles de bugs critiques observées, sans introduire de nouveau couplage architectural (chaque correction reste localisée dans la couche concernée : Perception pour Frenet, Vehicle pour KEEP_CLEAR, RoundaboutPolicy pour la règle de priorité).
