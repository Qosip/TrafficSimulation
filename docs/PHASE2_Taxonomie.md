# 🚦 PHASE 2 — TAXONOMIE DES INTERSECTIONS ET FICHES DE COMPORTEMENTS

Cette phase fournit deux livrables : (1) le catalogue complet des **types d'intersections** gérés par le simulateur ; (2) une **fiche par comportement de conduite** au format script (Nom / Déclencheur / Actions modifiées) destinée à servir de **storyboard** pour les démonstrations vidéo.

---

## 2.1 Taxonomie des intersections

Toutes les politiques sont des implémentations de `IIntersectionPolicy` et peuvent être attachées à n'importe quelle géométrie de carrefour (sauf le rond-point, dont la géométrie est circulaire et non convertible).

### Tableau récapitulatif

| # | Type (`RegulationType`) | Famille | Idée centrale | Référence |
|---|---|---|---|---|
| 1 | `PRIORITY_RIGHT` | Classique | Le véhicule venant de la droite a la priorité ; gap-acceptance | Code de la route FR |
| 2 | `STOP` | Classique | Arrêt complet ≥ 0.8 s puis gap-acceptance 360° | Code de la route FR |
| 3 | `YIELD` | Classique | Cédez le passage : pas d'arrêt obligatoire, gap-acceptance directionnelle | Code de la route FR |
| 4 | `TRAFFIC_LIGHT` | Signalisée | Cycle vert / orange / rouge ; gestion dilemme zone orange ; tourne-à-gauche non protégé | — |
| 5 | `ROUNDABOUT` | Géométrique | Anneau circulaire ; priorité totale aux véhicules à l'intérieur (règle FR 1984) | Code de la route FR |
| 6 | `FIXED_PRIORITY` | Recherche SMA | Axe principal jamais arrêté ; axe secondaire cède | Architecture rapport |
| 7 | `P2P` | Recherche SMA | Négociation pair-à-pair décentralisée, hiérarchie de dominance | VanMiddlesworth et al. AAMAS 2008 |
| 8 | `AIM` | Recherche SMA | Réservation centralisée FCFS de créneaux spatio-temporels | Dresner & Stone 2008 |
| 9 | `VIRTUAL_PLATOON` | Recherche SMA | Insertion "fermeture éclair" : meneur virtuel mobile projeté | Medina et al. |
| 10 | `ORCA` | Recherche SMA | Évitement réciproque continu, mode espace ouvert | van den Berg et al. 2011 |

### Fiches détaillées

---

#### 🟦 #1 — `PRIORITY_RIGHT` (Priorité à droite)

- **Géométrie** : carrefour 2×2 tuiles, 4 approches cardinales.
- **Algorithme** : Gap-Acceptance directionnelle.
- **Décision** : l'agent scrute un disque de `scanRadius=200 px` autour du centre ; il refuse de passer si un véhicule conflictuel (venant de sa droite) arrive dans une fenêtre temporelle < `safetyMargin + crossingDistance/v`.
- **État interne** : aucun (policy stateless).
- **Sortie typique** : `{ shouldStop=true, stopLineGap≈distance jusqu'à la ligne }`.

---

#### 🟥 #2 — `STOP` (Panneau STOP, 2 voies)

- **Géométrie** : 2×2 tuiles, **axe principal** déclaré (E-O par défaut). L'axe principal NE S'ARRÊTE JAMAIS, seule la branche secondaire porte le STOP.
- **Algorithme** : 
  1. Arrêt complet exigé (`stopHeldTime ≥ 0.8 s`, `currentSpeed < haltSpeedEpsilon=5 px/s`).
  2. Une fois libéré (`stopReleased=true`), gap-acceptance 360°.
- **État interne** : `stopHeldTime`, `stopReleased` vivent dans le `Vehicle` (par-agent, par-intersection).
- **Sortie** : `shouldStop=true` jusqu'à libération.

---

#### 🟨 #3 — `YIELD` (Cédez le passage)

- **Variante de PRIORITY_RIGHT** sans obligation d'arrêt.
- L'agent peut rouler lentement tant que le créneau est jugé suffisant.
- Gap-acceptance avec `safetyMargin` plus faible que STOP.

---

#### 🟢🟠🔴 #4 — `TRAFFIC_LIGHT` (Feux tricolores)

- **Géométrie** : 2×2 tuiles, phases gérées par l'`Intersection` elle-même (`lightTimer`, `currentPhase`).
- **Phases** :
  - `GREEN` (5 s) → `canEnter=true`.
  - `ORANGE` (1.5 s) → résout le **dilemme zone d'orange** : si la distance de freinage confortable (`v²/(2·bComf)`) > distance restante → on **passe** (sinon freinage d'urgence dangereux) ; sinon on s'arrête.
  - `RED` → arrêt sur la ligne.
- **Tourne-à-gauche non protégé** : les deux feux opposés partagent le même vert. Un véhicule tournant à gauche doit **céder à l'oncoming** tout droit s'il arrive dans `leftYieldTime=2.5 s`.
- **Sortie** : `{shouldStop, stopLineGap}` ou `{canEnter, virtual leader sur oncoming}`.

---

#### 🔵 #5 — `ROUNDABOUT` (Rond-point)

- **Géométrie** : carré PAIR de côté ∈ {2, 4, 6, 8} tuiles, anneau circulaire central. Branches dynamiques recalculées via `World::refreshRoundaboutApproaches()`.
- **Algorithme** : priorité TOTALE aux véhicules **déjà à l'intérieur** du disque (`insideDetectRadius=70 px`). L'entrant cède via gap-acceptance.
- **Trajectoire** : reconstruction par **arcs tangentiels** dans `Vehicle::rebuildLaneFromPath()` (raccord par Bézier quadratique entrée/sortie) pour éviter les virages 90° qui provoquaient des "vagues".
- **Sens** : trigonométrique français (angle décroissant en atan2).
- **Sortie typique** : `{ shouldStop=true }` si un véhicule occupe l'anneau, sinon `{ canEnter=true }`.

---

#### 🔷 #6 — `FIXED_PRIORITY` (Priorité fixe, axe principal)

- **Idée** : analogue à `STOP` mais sans obligation d'arrêt sur l'axe secondaire (cède simplement).
- **Stratégie de référence** dans le rapport SMA pour comparer au P2P décentralisé.
- **Algorithme** : `isStopMajorAxisHorizontal()` lit l'axe principal. Vehicules majeurs : `canEnter=true` (pas de scan). Véhicules mineurs : gap-acceptance sur les véhicules majeurs.

---

#### 🟢🔵 #7 — `P2P` (VANET pair-à-pair décentralisé)

- **Référence** : VanMiddlesworth, Dresner & Stone, *"Replacing the Stop Sign"*, AAMAS 2008.
- **Aucune autorité centrale.** Chaque agent évalue localement la même hiérarchie de dominance.
- **Machine à états (dérivée de la distance)** :
  - `LURKING` (loin, > `claimDistance=160 px`) : observe seulement.
  - `CLAIMING` (proche) : revendique et arbitre via la hiérarchie.
  - `TRAVERSAL` (engagé) : géré par le commit-to-pass du Vehicle.
- **Hiérarchie de dominance (le Claim dominant gagne ; je cède si dominé)** :
  1. Si les deux roulent → plus petit `estimated_exit_time` gagne.
  2. Si les deux sont arrêtés → véhicule à DROITE gagne.
  3. À égalité → trajectoire rectiligne > virage.
  4. Bris d'égalité ultime → plus petit VIN gagne.
  - Cas mixte arrêté/roulant : l'arrêté cède au roulant.
- **Sortie** : `{shouldStop=true}` si dominé, sinon `{canEnter=true}`.

---

#### 🟪 #8 — `AIM` (Autonomous Intersection Management)

- **Référence** : Dresner & Stone 2008.
- **Centralisé par carrefour** : un "Intersection Manager" tient la table `reservations_[VIN] = {tEnterAbs, tExitAbs, horizontal}`.
- **Algorithme FCFS** :
  1. Agent calcule sa fenêtre `[tEnter, tExit]` selon vitesse + distance.
  2. Manager teste chevauchement avec réservations existantes sur trajectoires **perpendiculaires**.
  3. Si libre → reserve, `canEnter=true`. Sinon → ralentir, re-soumettre.
- **Granularité** : 1 boîte / carrefour. Trajectoires parallèles peuvent coexister.
- **Mutex** : `Intersection::reqMutex_` protège la table en multi-threading.
- **Limitation déterminisme** : l'ordre des requêtes dépend du scheduling thread → AIM n'est PAS bit-déterministe en parallèle (Monte-Carlo tourne en séquentiel).

---

#### 🟦 #9 — `VIRTUAL_PLATOON` (Peloton virtuel)

- **Référence** : Medina et al., insertion "zipping".
- **Idée** : pas d'arrêt. Projection 1D : le véhicule croisé qui arriverait juste avant moi devient mon **meneur virtuel mobile**. J'adopte sa vitesse à une distance projetée → fluide.
- **Sortie spécifique** : `{ canEnter=true, followVirtualLeader=true, virtualLeaderGap, virtualLeaderSpeed }`.
- **Sécurité** : si le meneur projeté ralentit/s'arrête, l'IDM me ralentit automatiquement.

---

#### 🟢 #10 — `ORCA` (Évitement réciproque continu)

- **Référence** : van den Berg et al. 2011, adapté au mouvement 1D le long d'une Lane curviligne.
- **Aucune ligne directrice stricte.** Tout est modulation continue.
- **Algorithme** :
  - Raisonnement en temps d'arrivée `t = distance / vitesse`.
  - Réciprocité : sur 2 trajectoires en conflit, celui qui arrive **plus tard** cède (cède souple = `followVirtualLeader`). À égalité, le plus grand VIN cède.
  - **Cède SOUPLE** par défaut (entrelacement zip) ; devient FERME (`shouldStop=true`) si conflit imminent (`hardStopArrival=0.6 s`) ou autre quasi à l'arrêt en travers.
- **Stateless** → thread-safe par construction.

---

## 2.2 Fiches détaillées des comportements de conduite

Format script vidéo : **Nom / Déclencheur exact (Trigger) / Actions modifiées (cap, vitesse, état)**.

> Tous les comportements coexistent dans la même `computeDecision()` du `Vehicle` et se résolvent par fusion IDM "plus contraignant l'emporte". Les couleurs proposées peuvent servir à un overlay vidéo.

---

### 🎬 Fiche #1 — **CRUISING** (Roule libre)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::NONE` |
| **Couleur overlay** | ⚪ Blanc / transparent |
| **Déclencheur** | Aucun leader détecté ET aucune intersection à freiner ET pas en virage serré ET pas en panne |
| **Action — Cap** | Tangent à `Lane::getHeadingAt(s)` |
| **Action — Vitesse désirée v₀** | `min(maxSpeed, road_limit, cornering_factor)` |
| **Action — Accélération** | IDM avec `leader.hasLeader=false` → `a = aMax · (1 − (v/v₀)^4)` |
| **Sortie** | Approche progressive de v₀ |
| **Script vidéo** | *"Sur autoroute libre, le véhicule accélère progressivement vers la vitesse limite, sans à-coup, en suivant fidèlement les courbes de la voie."* |

---

### 🎬 Fiche #2 — **CAR-FOLLOWING** (Suivi de leader réel)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::LEADER_VEHICLE` |
| **Couleur overlay** | 🟠 Orange |
| **Déclencheur** | `PerceptionResult::hasDirectObstacle == true` ; un véhicule projeté sur MA Lane à `directObstacleDistance` raisonnable |
| **Filtrage** | Projection de Frenet sur `currentLane` (gap mesuré LE LONG du tracé, |lateral| < `laneCorridorHalf=22 px`) |
| **Action — Cap** | Inchangé (continue sur Lane) |
| **Action — Vitesse** | IDM : `s* = s₀ + max(0, vT + v·Δv/(2√(a·b)))`, `a = aMax·[1 − (v/v₀)^4 − (s*/gap)²]` |
| **Sortie** | Gap dynamique stable ≈ `vT + s₀` |
| **Script vidéo** | *"Le véhicule détecte le leader sur sa voie via projection curviligne (insensible au contre-sens). Il maintient un gap proportionnel à sa vitesse — la signature même du modèle IDM."* |

---

### 🎬 Fiche #3 — **CORNERING** (Anticipation virage)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::CORNERING` |
| **Couleur overlay** | 🟡 Jaune |
| **Déclencheur** | Courbure locale `|dθ/ds|` au-dessus d'un seuil dans `[s, s+lookahead]` |
| **Action — Cap** | Inchangé (suit la Lane lissée par Bézier quadratique) |
| **Action — Vitesse désirée v₀** | Plafonnée par `v_max_curvature = √(a_lat_max / κ)` |
| **Sortie** | Ralentissement avant le virage, ré-accélération après |
| **Script vidéo** | *"Avant chaque virage, le véhicule réduit sa vitesse désirée en fonction de la courbure — un comportement naturel hérité de la cinématique latérale maximale tolérée."* |

---

### 🎬 Fiche #4 — **YIELDING — Gap Acceptance** (Cède au flux conflictuel)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INTERSECTION_YIELD` |
| **Couleur overlay** | 🟣 Magenta |
| **Déclencheur** | Approche d'une intersection ; `policy_->request(ctx)` retourne `{ shouldStop=true }` ; fenêtre de conflit jugée insuffisante |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Leader virtuel inséré à `stopLineGap` devant moi, `speed=0` → IDM freine vers la ligne d'arrêt |
| **Sortie** | Décélération douce + arrêt précis sur la ligne |
| **Script vidéo** | *"À l'approche d'un carrefour à priorité à droite, le véhicule projette un véhicule fantôme à zéro km/h sur la ligne d'arrêt. L'IDM produit alors un freinage progressif identique à celui face à un vrai obstacle."* |

---

### 🎬 Fiche #5 — **STOP-HOLD** (Arrêt complet obligatoire)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INTERSECTION_STOP` |
| **Couleur overlay** | 🔴 Rouge clignotant |
| **Déclencheur** | Intersection `STOP` ET (`stopHeldTime < 0.8 s` OU `currentSpeed > 5 px/s`) |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Leader virtuel à `stopLineGap=0` → arrêt strict |
| **État** | `stopHeldTime` accumule, libère `stopReleased=true` une fois 0.8 s à l'arrêt |
| **Sortie** | Arrêt complet ≥ 0.8 s puis transition vers `YIELDING` |
| **Script vidéo** | *"Au panneau STOP, le véhicule s'immobilise complètement pendant au moins huit dixièmes de seconde — comportement réglementaire — avant d'évaluer la gap-acceptance comme aux intersections cédez-le-passage."* |

---

### 🎬 Fiche #6 — **TRAFFIC-LIGHT** (Feu tricolore + dilemme orange)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INTERSECTION_RED` |
| **Couleur overlay** | 🚦 Rouge ou Orange selon la phase |
| **Déclencheur** | `Intersection::getLightState(approach)` ∈ {RED, ORANGE} |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | RED → leader virtuel à la ligne. ORANGE → on calcule `d_freinage = v²/(2·bComf)`. Si `d_freinage > distance restante` → on PASSE (sinon freinage d'urgence dangereux). |
| **Tourne-à-gauche** | Si vert ET intention `LEFT` → cède à l'oncoming `STRAIGHT` arrivant sous `2.5 s` |
| **Script vidéo** | *"Le dilemme de la zone d'orange est résolu cinématiquement : si la distance restante est inférieure à la distance de freinage confortable, le véhicule franchit prudemment plutôt que d'imposer un freinage brutal."* |

---

### 🎬 Fiche #7 — **ROUNDABOUT-INSIDE-PRIORITY** (Cède aux véhicules sur l'anneau)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INTERSECTION_YIELD` (variante ronde) |
| **Couleur overlay** | 🔵 Bleu pulsant |
| **Déclencheur** | Approche d'un `ROUNDABOUT` ; au moins un véhicule dans le disque `insideDetectRadius=70 px` |
| **Action — Cap** | Inchangé (suit l'arc tangentiel calculé) |
| **Action — Vitesse** | Leader virtuel à `stopLineGap` jusqu'à la ligne d'entrée |
| **Engagement** | Dès dans l'anneau → `isCommittedToPass=true`, ne re-cède plus |
| **Script vidéo** | *"À l'approche du rond-point, le véhicule s'arrête à la ligne tant qu'un autre tourne sur l'anneau. Une fois engagé sur la couronne, il devient prioritaire et ne re-cède plus."* |

---

### 🎬 Fiche #8 — **LANE-CHANGING / OVERTAKING** (Dépassement avec offset latéral)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::OVERTAKING` |
| **Couleur overlay** | 🟢 Vert vif |
| **Déclencheur** | Leader trop lent (`leader.speed < α·v₀`) ET voie opposée libre (`isOncomingLaneFree(requiredClearAhead)`) ET créneau de rabattement libre (`isReturnSlotFree`) ET personnalité agressive |
| **États** | `NONE` → `OVERTAKING` (lateralOffset cible = + largeur voie) → `RETURNING` → `NONE` |
| **Action — Cap** | Cap dérivé de la vélocité latérale (`yaw visuel naturel`) |
| **Action — Vitesse** | Pleine vitesse v₀ pendant `OVERTAKING` ; IDM normal après rabattement |
| **Sécurité** | Anti-éternel : timer `overtakeElapsed` borne la manœuvre |
| **Script vidéo** | *"Lorsque le leader est trop lent et que la voie opposée est libre sur une distance suffisante, le véhicule glisse latéralement, dépasse à pleine vitesse, puis revient s'insérer dans le créneau libre devant l'ex-leader."* |

---

### 🎬 Fiche #9 — **P2P-NEGOTIATING** (Négociation pair-à-pair)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::NEGOTIATING` |
| **Couleur overlay** | 🟢🔵 Cyan |
| **Déclencheur** | Intersection `P2P` ; agent en zone CLAIMING ; au moins un Claim conflictuel dominant selon la hiérarchie |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Leader virtuel à la ligne (cède) |
| **Hiérarchie évaluée** | (1) exitTime, (2) à droite, (3) rectiligne > virage, (4) min VIN |
| **Sortie** | Si Claim dominant → cède ; sinon `canEnter=true` |
| **Script vidéo** | *"Sans aucun panneau ni feu, chaque véhicule arbitre localement : la trajectoire la plus rapide à dégager gagne ; à égalité, celle à droite ; puis la rectiligne ; puis le plus petit identifiant. L'ordre émerge sans contrôleur central."* |

---

### 🎬 Fiche #10 — **PLATOONING / VIRTUAL ZIPPING** (Insertion en fermeture éclair)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::PLATOONING` |
| **Couleur overlay** | 🟦 Bleu acier |
| **Déclencheur** | Intersection `VIRTUAL_PLATOON` ; un véhicule croisé projeté à arriver juste avant moi |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | IDM suit un **leader virtuel mobile** `{ gap=virtualLeaderGap, speed=virtualLeaderSpeed }` |
| **Sortie** | Vitesse quasi constante ; insertion fluide alternée |
| **Script vidéo** | *"Aucun véhicule ne s'arrête. Chacun projette ses voisins croisés sur sa propre trajectoire et se cale derrière le plus immédiat — exactement comme deux files convergeant en fermeture éclair."* |

---

### 🎬 Fiche #11 — **AIM-RESERVATION** (Allocation centralisée FCFS)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INTERSECTION_YIELD` (variante AIM) |
| **Couleur overlay** | 🟪 Violet |
| **Déclencheur** | Intersection `AIM` ; demande de réservation refusée (chevauchement avec slot perpendiculaire) |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Ralentit pour repousser sa fenêtre `[tEnter, tExit]` et re-soumettre |
| **État** | `reservations_[VIN]` cache la dernière demande accordée |
| **Script vidéo** | *"Le carrefour agit comme une tour de contrôle : chaque véhicule réserve une fenêtre temporelle. En cas de chevauchement avec une réservation perpendiculaire, il ralentit jusqu'à obtenir un créneau libre."* |

---

### 🎬 Fiche #12 — **ORCA-SOFT-YIELD** (Cède souple continu)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::PLATOONING` (catégorie cède souple) |
| **Couleur overlay** | 🟢 Vert pâle |
| **Déclencheur** | Intersection `ORCA` ; conflit non imminent ; mon temps d'arrivée > celui de l'autre + tolérance |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Cible = `softSlowFactor · v₀` (≈ 55 %) via leader virtuel mobile |
| **Bascule en arrêt FERME** | Si l'autre arrive sous `hardStopArrival=0.6 s` |
| **Script vidéo** | *"Plutôt qu'un arrêt franc, le véhicule effleure sa vitesse — il se glisse dans l'interstice naturel laissé par le voisin. Le moindre risque imminent suffit à passer en arrêt ferme."* |

---

### 🎬 Fiche #13 — **KEEP-CLEAR** (Anti-gridlock sortie occupée)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::KEEP_CLEAR` |
| **Couleur overlay** | 🟥 Rouge avec bordure jaune |
| **Déclencheur** | Voie de sortie de l'intersection physiquement occupée ; `canEnter=true` selon la policy MAIS engager bloquerait |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Leader virtuel à la ligne d'arrêt → refus d'engager |
| **Garde liveness** | `keepClearWaited_` accumule ; au-delà d'un seuil → on engage quand même (bris de cycle par VIN) |
| **Script vidéo** | *"Même priorité accordée, le véhicule refuse de s'engager si l'exutoire est saturé. Cette règle 'keep clear' suffit à éviter les pâtés mexicains classiques où chacun avance d'un mètre dans la boîte."* |

---

### 🎬 Fiche #14 — **BREAKDOWN** (Panne)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::BREAKDOWN` |
| **Couleur overlay** | ⚫ Noir clignotant |
| **Déclencheur** | Tirage RNG (`forceBreakdown(seconds)`) ; `breakdownTimer > 0` |
| **Action — Cap** | Inchangé |
| **Action — Vitesse** | Freinage maximal → arrêt total. Les véhicules derrière le traitent comme un leader immobile (IDM les ralentit naturellement). |
| **Script vidéo** | *"En cas de panne, le véhicule devient un obstacle statique. Les véhicules suivants le détectent comme un leader à vitesse nulle et s'arrêtent à distance de sécurité — la résilience est intrinsèque au modèle de suivi."* |

---

### 🎬 Fiche #15 — **INITIALIZING** (Démarrage à froid)

| Champ | Contenu |
|---|---|
| **Nom interne** | `BlockReason::INITIALIZING` |
| **Couleur overlay** | 🟦 Bleu clair |
| **Déclencheur** | `currentSpeed ≈ 0` ET aucun leader devant ET voie libre derrière (`isClearBehindForRestart`) |
| **Action — Vitesse** | IDM démarre depuis 0 vers v₀, accélération maximale lissée par `delta=4` |
| **Script vidéo** | *"Au démarrage, le véhicule accélère selon la cinématique du modèle IDM — accélération maximale tant que la vitesse reste bien sous v₀, puis décroissance lisse jusqu'au régime stable."* |

---

## 2.3 Diagramme de relation comportement ↔ source

```mermaid
flowchart TB
    subgraph "Sources de contrainte (chacune produit un LeaderInfo)"
        A1[Perception réelle<br/>directObstacle]
        A2[Intersection Policy<br/>stop-line virtuel]
        A3[Intersection Policy<br/>virtual leader mobile<br/>PLATOON / ORCA]
        A4[Cornering<br/>cap v₀ par courbure]
        A5[Personality / Breakdown<br/>brake override]
    end

    A1 & A2 & A3 --> F[Fusion IDM<br/>le plus contraignant<br/>l'emporte]
    A4 -. clip v₀ .-> F
    A5 -. override .-> F

    F --> P[pendingAccel<br/>+ pendingDesiredSpeed]
    P --> I[integrate(dt) :<br/>Euler symplectique]
    I --> S[Nouvelle position<br/>+ vitesse + cap]
```

---

## 2.4 Synthèse Phase 2

- **10 types d'intersections** couvrent l'éventail classique (5) + l'état de l'art SMA (5).
- **15 fiches de comportements** documentent toutes les transitions visibles en présentation vidéo.
- Chaque fiche se lit comme **un script** : nom, déclencheur, conséquences observables.
- La signature commune : **toutes** les contraintes se transforment en `LeaderInfo` que l'IDM fusionne uniformément — c'est ce qui rend le système modulaire et explicable.
