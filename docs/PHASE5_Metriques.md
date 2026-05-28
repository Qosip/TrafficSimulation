# 📊 PHASE 5 — MÉTRIQUES ET OBSERVABILITÉ

Cette phase liste **toutes** les métriques pertinentes pour prouver la fluidité de la simulation, précise leur **lieu d'instrumentation** dans le code, et indique le **mode d'agrégation** (instantané, glissant, accumulé).

---

## 5.1 Tableau récapitulatif des métriques

| # | Métrique | Unité | Type | Lieu de capture | Lieu de calcul |
|---|---|---|---|---|---|
| 1 | **Throughput (Débit)** | véh/min | Glissant 60 s | Détection AT_GOAL | `MetricsCollector::sample()` |
| 2 | **Mean Delay (Temps perdu)** | s | Cumulé moyenné | Comparaison trajet réel vs flux libre | `MetricsCollector::sample()` (par-agent) puis agrégation `aggregate()` |
| 3 | **Mean Speed** | px/s | Instantané, moyenne sur agents actifs | Boucle `agents` dans `sample()` | `sample()` |
| 4 | **TTC min (Time-To-Collision)** | s | Instantané, min sur paires | `computeTtc()` paire à paire | `computeTtc()` |
| 5 | **TTC violations** | nb cumulé | Compteur | `computeTtc()` quand `ttc < kCriticalTTC=1.5 s` | `sample()` |
| 6 | **Jerk accumulé** | px/s³ | ∫\|da/dt\| dt | Phase 2 (`integrate`) → différence d'accélération | `sample()` (par-agent) |
| 7 | **Mean Jerk completed** | px/s³ moyenné | Moyenne sur arrivés | Lors de la transition AT_GOAL | `aggregate()` |
| 8 | **Total stops** | nb cumulé | Hystérésis | Bascule speed < ε pour la première fois | `sample()` |
| 9 | **Active vehicles** | nb | Instantané | Compte agents non-AT_GOAL | `sample()` |
| 10 | **Completed vehicles** | nb | Cumulé | Compteur AT_GOAL | `sample()` |
| 11 | **Block reason distribution** | % par catégorie | Histogramme | Lecture `agent.getBlockReason()` | À ajouter dans `sample()` |
| 12 | **Intersection occupancy** | % temps occupé | Glissant | Test `position in coveredTiles` | À ajouter, par carrefour |
| 13 | **Queue length par approche** | nb véh | Instantané | Scan véhicules arrêtés sur approche | À ajouter, par carrefour |
| 14 | **Mean waiting time per intersection** | s | Cumulé moyenné | Δ temps entre arrivée et engagement | À ajouter, par carrefour |
| 15 | **Headway distribution** | s | Histogramme | Δ temps entre 2 passages successifs | À ajouter, par approche |
| 16 | **Fuel/CO₂ proxy** (Vincent) | proxy (∫v² dt) | Cumulé par-agent | Phase 2 `integrate` | À ajouter |
| 17 | **Throughput per strategy** | véh/min | Comparatif | Banc Monte-Carlo | `ExperimentRunner::run()` → `ResultRow` |
| 18 | **Deadlock incidents** | nb | Compteur | Détection `keepClearWaited_ > seuil` | À ajouter dans `Vehicle::computeDecision` |

---

## 5.2 Détail métrique par métrique

### #1 — Throughput (Débit)

```
Définition : nombre de véhicules arrivés à destination par minute (fenêtre glissante 60 s).

Capture (dans MetricsCollector::sample) :
    pour chaque agent :
        si agent.blockReason == AT_GOAL ET !agentsRec[vin].completed :
            agentsRec[vin].completed = true
            completionTimes_.push_back(simTime_)
            agg_.completedVehicles += 1

Calcul (dans aggregate) :
    purger completionTimes_ des entrées < simTime_ - 60 s
    agg_.throughputPerMin = completionTimes_.size()
```

Lieu code : `src/core/metrics/MetricsCollector.cpp`, méthode `sample()` + `aggregate()`.

### #2 — Mean Delay

```
Définition : temps réel - temps en flux libre (proxy = distance / maxSpeed observé).

Capture (par agent, dans sample) :
    rec.travelTime += dt
    rec.distance   += agent.speed * dt
    rec.maxSpeed   = max(rec.maxSpeed, agent.speed)

À l'arrivée (AT_GOAL) :
    freeFlowTime = rec.distance / rec.maxSpeed
    delay        = rec.travelTime - freeFlowTime
    sumDelay_   += delay
    countDelay_ += 1

Calcul agrégé :
    agg_.meanDelaySec = sumDelay_ / countDelay_
```

### #3 — Mean Speed (vitesse moyenne du réseau)

```
Capture (sample) :
    sumSpeed = 0; nActive = 0
    pour chaque agent non-AT_GOAL :
        sumSpeed += agent.speed
        nActive  += 1
    agg_.meanSpeed = sumSpeed / nActive
```

### #4 — TTC min (Time-To-Collision)

```
Capture (computeTtc, appelée depuis sample) :
    minTtc = +∞
    pour chaque paire (i,j) avec i < j :
        # On ne considère la TTC que si les véhicules se rapprochent
        rel_pos = j.position - i.position
        rel_vel = i.velocity - j.velocity   # vitesse de fermeture
        closingSpeed = dot(rel_vel, normalize(rel_pos))
        if closingSpeed <= 0 : continue
        distance = |rel_pos| - (i.length/2 + j.length/2)
        if distance <= 0 : minTtc = 0; continue
        ttc = distance / closingSpeed
        if ttc < minTtc : minTtc = ttc

    agg_.minTTC = minTtc

    Complexité : O(n²). Acceptable pour n ≤ 500.
    Optim future : grid spatial pour O(n·k).
```

Lieu code : `MetricsCollector::computeTtc()`.

### #5 — TTC violations

```
Capture (sample) :
    pour chaque paire à TTC < kCriticalTTC (1.5 s) :
        agg_.ttcViolations += 1
    # Pas de déduplication par paire : violations continues comptent chaque tick
    # → métrique de "temps passé dans une situation dangereuse"
```

### #6 / #7 — Jerk (confort)

```
Capture (par agent, sample, après integrate) :
    currentAccel = agent.getCurrentAccel()
    jerkInstant  = abs(currentAccel - rec.prevAccel) / dt
    rec.accumJerk += jerkInstant * dt
    rec.prevAccel = currentAccel

À l'arrivée :
    sumJerk_     += rec.accumJerk
    nCompletedJ_ += 1

Agrégation :
    agg_.meanJerkCompleted = sumJerk_ / nCompletedJ_
```

### #8 — Total stops

```
Capture (sample, par agent, hystérésis) :
    if agent.speed < stopEpsilon (typ. 2 px/s) ET !rec.stopped :
        rec.stopped = true
        agg_.totalStops += 1
    else if agent.speed > releaseEpsilon (typ. 8 px/s) ET rec.stopped :
        rec.stopped = false

L'hystérésis évite de compter une oscillation au point d'arrêt.
```

### #11 — Block reason distribution

À ajouter pour le banc d'essai :

```
Champs supplémentaires AggregateMetrics :
    int blockCounts[NUM_BLOCK_REASONS] = {0}

Capture (sample) :
    pour chaque agent :
        agg_.blockCounts[ (int)agent.blockReason ] += 1

Permet : pie chart "où passent leur temps les véhicules ?"
         (10% libre / 30% car-follow / 15% yield / 5% breakdown / ...).
```

### #12 — Intersection occupancy

À ajouter au niveau de chaque intersection :

```
Champs supplémentaires Intersection :
    float occupiedTimeAccum = 0.f
    float totalTimeAccum    = 0.f

Capture (intersection.update(dt), depuis World::updateIntersections) :
    pour chaque agent :
        if agent.position ∈ this.coveredTiles :
            occupiedTimeAccum += dt
            break  # un seul comptage par tick (binaire)
    totalTimeAccum += dt

Métrique exposée :
    occupancyRatio = occupiedTimeAccum / totalTimeAccum  ∈ [0, 1]
```

### #13 — Queue length par approche

À ajouter :

```
pour chaque approach in intersection.approaches :
    queueLen = 0
    pour chaque agent dont currentLane se dirige vers cette approach :
        if agent.distanceToIntersectionLine < 200 px ET agent.speed < 5 px/s :
            queueLen += 1
    approach.queueLength = queueLen
```

Métrique très utile pour diagnostiquer si une stratégie de régulation favorise un axe.

### #14 — Mean waiting time per intersection

À ajouter :

```
Vehicle nouveau champ :
    float waitedAtCurrentIntersection_ = 0.f
    int   currentWaitingIntId_         = -1

Capture (dans Vehicle::integrate) :
    if currentBlockReason ∈ {INTERSECTION_*, KEEP_CLEAR, NEGOTIATING} :
        if currentWaitingIntId_ != currentInter.id :
            currentWaitingIntId_         = currentInter.id
            waitedAtCurrentIntersection_ = 0
        waitedAtCurrentIntersection_ += dt
    else if isCommittedToPass :
        # On vient d'engager : le wait est consommé
        Intersection &it = world.intersectionById(currentWaitingIntId_)
        it.recordWait(waitedAtCurrentIntersection_)   # accumule pour la moyenne
        currentWaitingIntId_ = -1
        waitedAtCurrentIntersection_ = 0
```

### #15 — Headway distribution

```
Par approche, mémoriser le simTime du dernier véhicule à avoir franchi la ligne d'entrée :
    approach.lastCrossSimTime
À chaque franchissement nouveau :
    headway = simTime - lastCrossSimTime
    approach.headwayHistogram.add(headway)
    lastCrossSimTime = simTime
```

### #16 — Fuel/CO₂ proxy

Proxy énergétique simple : `∫ v² dt` (énergie cinétique consommée par freinage + accélération).

```
Capture (Vehicle::integrate, ou MetricsCollector::sample) :
    rec.fuelProxy += agent.speed * agent.speed * dt
```

À l'arrivée, exposer `meanFuelProxyPerKm = fuelProxy / distance`.

### #17 — Throughput per strategy (banc d'essai)

```
ExperimentRunner::run() boucle sur :
    for strategy in cfg.strategies :
        for density in [0.1, 0.2, ..., 0.8] (veh/s) :
            for run in [1..cfg.runsPerPoint] :
                world, agents = buildIsolatedIntersection(strategy)
                metrics       = MetricsCollector()
                simulate(world, agents, metrics, cfg.durationSec)
                results.push_back(ResultRow{
                    strategy, density,
                    throughputPerMin = metrics.aggregate().throughputPerMin,
                    meanDelaySec     = metrics.aggregate().meanDelaySec,
                    minTTC           = metrics.aggregate().minTTC
                })
    return results
```

Lieu : `src/sim/ExperimentRunner.cpp`.

### #18 — Deadlock incidents

À ajouter :

```
AggregateMetrics nouveau champ :
    int deadlockIncidents = 0

Capture (Vehicle::computeDecision, quand le bris VIN s'active) :
    if forcedBreakOfGridlockCycle :
        metrics.deadlockIncidents += 1   # via callback ou évènement
```

Permet de quantifier la qualité d'une stratégie : un système robuste a peu de déclenchements de bris VIN.

---

## 5.3 Localisation dans le code (résumé)

```
src/
├── core/
│   └── metrics/
│       ├── MetricsCollector.hpp     ← interface publique + AggregateMetrics
│       └── MetricsCollector.cpp     ← sample(), computeTtc(), aggregate(), exportCsv/Json
│
├── core/intersection/
│   └── Intersection.cpp             ← (à étendre) occupiedTimeAccum, queueLength
│
├── core/agent/
│   └── Vehicle.cpp                  ← (à étendre) waitedAtCurrentIntersection_,
│                                       deadlock incident callback
│
├── sim/
│   └── ExperimentRunner.cpp         ← lance balayage densité × stratégie,
│                                       agrège dans ResultRow
│
└── main.cpp                         ← Dashboard ImGui :
                                       - panneau "Métriques (recherche)" 
                                       - plot series + export CSV/JSON
                                       - bouton "diagnostic blocages"
```

### Points d'instrumentation existants vs à ajouter

| Métrique | État | Lieu existant |
|---|---|---|
| Throughput | ✅ | `MetricsCollector::sample` + `aggregate` |
| Mean delay | ✅ | `MetricsCollector::sample` |
| Mean speed | ✅ | `MetricsCollector::sample` |
| TTC min/violations | ✅ | `MetricsCollector::computeTtc` |
| Jerk | ✅ | `MetricsCollector::sample` (via `getCurrentAccel`) |
| Total stops | ✅ | `MetricsCollector::sample` (hystérésis) |
| Block reason distribution | ⬜ À ajouter | `MetricsCollector::sample` |
| Intersection occupancy | ⬜ À ajouter | `Intersection::update` |
| Queue length | ⬜ À ajouter | `Intersection::update` |
| Mean waiting time | ⬜ À ajouter | `Vehicle::integrate` + `Intersection::recordWait` |
| Headway distribution | ⬜ À ajouter | `Approach::onCross` |
| Fuel proxy | ⬜ À ajouter | `MetricsCollector::sample` |
| Throughput per strategy | ✅ | `ExperimentRunner::run` → `ResultRow` |
| Deadlock incidents | ⬜ À ajouter | callback `Vehicle::computeDecision` → `MetricsCollector` |

---

## 5.4 Modes d'agrégation et fréquences

| Métrique | Capture | Agrégation | Période |
|---|---|---|---|
| Throughput | À chaque AT_GOAL | Fenêtre glissante 60 s | Re-calculé à chaque `sample()` |
| Mean delay | À chaque AT_GOAL | Moyenne incrémentale | Stable une fois agent arrivé |
| Mean speed | Chaque tick (60 Hz) | Moyenne instantanée | Re-calculée chaque pas |
| TTC min | Chaque tick | Minimum sur paires | Re-calculé chaque pas |
| TTC violations | Chaque tick | Compteur cumulatif | Croît monotone |
| Jerk | Chaque tick | Intégrale ∫\|da/dt\| | Croît monotone |
| Total stops | Chaque tick (hystérésis) | Compteur | Croît monotone |
| Series UI (graphes) | ~2 Hz | Buffer 256 points | `pushHist()` dans `sample()` |

### Snapshot rate (séries temporelles UI)

```
const float SAMPLE_PERIOD = 0.5f;  // 2 Hz, suffisant pour ImGui PlotLines
sampleAccum_ += dt;
if (sampleAccum_ >= SAMPLE_PERIOD) :
    sampleAccum_ -= SAMPLE_PERIOD
    pushHist(histThroughput_, agg_.throughputPerMin)
    pushHist(histDelay_,      agg_.meanDelaySec)
    pushHist(histSpeed_,      agg_.meanSpeed)
    pushHist(histMinTTC_,     agg_.minTTC)
```

---

## 5.5 Exports : formats et structure

### CSV (résumé + séries)

```csv
# summary
key,value
simTime,124.5
activeVehicles,42
completedVehicles,180
throughputPerMin,87.0
meanDelaySec,4.30
meanSpeed,118.4
minTTC,2.1
ttcViolations,3
totalStops,52
meanJerkCompleted,12.5

# series (period=0.5s)
t,throughput,delay,speed,minTTC
0.0,0,0,0,999
0.5,0,0,80,999
1.0,2,0.5,90,5.2
...
```

### JSON

```json
{
  "summary": {
    "simTime": 124.5,
    "activeVehicles": 42,
    "completedVehicles": 180,
    "throughputPerMin": 87.0,
    "meanDelaySec": 4.30,
    "meanSpeed": 118.4,
    "minTTC": 2.1,
    "ttcViolations": 3,
    "totalStops": 52,
    "meanJerkCompleted": 12.5
  },
  "series": {
    "period": 0.5,
    "throughput": [0, 0, 2, 4, ...],
    "delay":      [0, 0, 0.5, 1.0, ...],
    "speed":      [0, 80, 90, 95, ...],
    "minTTC":     [999, 999, 5.2, 4.8, ...]
  }
}
```

### Diagnostic blocages (export ad-hoc UI)

Format CSV par-véhicule à l'instant T (mettre en PAUSE avant) :

```csv
simTime,124.5
vin,x,y,heading,speed,block,leaderSrc,leaderVin,leaderRelHead,leaderClass,leaderGap,leaderSpeed,onInter
0,123.5,456.7,90.0,8.2,KEEP_CLEAR,perception,7,5.2,SAME,42.1,3.5,1
1,123.5,408.7,90.0,0.0,LEADER_VEHICLE,perception,0,0.0,SAME,8.4,0.0,0
...
```

Permet d'isoler les agents bloqués et la **chaîne de cause** (qui bloque qui).

---

## 5.6 KPI synthétiques pour les présentations

Pour comparer 2 stratégies sur 1 carrefour, 4 chiffres suffisent :

| KPI | Bon si | Mauvais si |
|---|---|---|
| Throughput per minute | élevé (>60) | bas (<30) |
| Mean delay (s) | bas (<3) | élevé (>10) |
| Min TTC (s) | élevé (>2) | bas (<1.5) |
| Stops per completed vehicle | bas (<0.5) | élevé (>2) |

Calculé via : `stopsPerVehicle = totalStops / completedVehicles`.

Pour les vidéos : afficher en overlay temps réel le **bandeau métriques** présent dans le dashboard ImGui (`Métriques (recherche)`), avec barre colorée pour TTC (rouge si < 1.5 s).

---

## 5.7 Synthèse Phase 5

- **6 métriques** déjà instrumentées dans `MetricsCollector` (throughput, delay, speed, TTC, jerk, stops).
- **6 métriques additionnelles** à ajouter pour couvrir l'observabilité par-intersection (occupancy, queue, waiting, headway, block-reason, deadlocks).
- **2 modes d'agrégation** : par-agent (rec dans `unordered_map<int, PerAgent>`) et global (`AggregateMetrics`).
- **3 formats d'export** : CSV (résumé+séries), JSON (idem), diagnostic ad-hoc par-véhicule.
- **Coût** : `MetricsCollector::sample()` est O(n) pour les métriques de base, `computeTtc` est O(n²) ; négligeable pour n ≤ 500 agents.
