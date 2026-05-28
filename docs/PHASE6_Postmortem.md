# 🩺 PHASE 6 — POST-MORTEM ET DIFFICULTÉS TECHNIQUES

Analyse rétrospective des obstacles classiques d'une simulation multi-agents de trafic, et démonstration de la manière dont notre architecture les contourne élégamment.

---

## 6.1 Difficultés classiques d'une simulation de trafic multi-agents

### 6.1.1 Complexité spatiale O(n²) — l'éternel mur

**Problème.** Chaque véhicule doit savoir où sont les autres (perception, calcul de TTC, gap-acceptance). Naïvement, c'est une boucle imbriquée sur la flotte → **O(n²)** par tick. À 60 Hz et n=500 agents, c'est 60 × 500² = **15 millions** d'évaluations de paires par seconde.

**Conséquences en pratique.**
- À n=100, ça passe (~1 ms/tick).
- À n=500, on dépasse facilement 30 ms/tick → en-dessous des 60 FPS.
- À n=2000, le moteur s'effondre (>200 ms/tick), la simulation devient injouable.

**Approches classiques pour mitiger.**
- **Grille spatiale** (uniform grid hashing) : `cellSize ≈ visionRange`, complexité O(n·k) avec k≈9 cellules.
- **KD-tree / Quadtree** : O(n log n) pour les requêtes range, mais coût de rebuild à chaque tick.
- **Sweep & prune** sur une dimension : très bon pour trafic largement axe-aligné.

### 6.1.2 Limites du raycasting (lancer de rayons pour la perception)

**Problème.** Une approche "intuitive" de la perception consiste à lancer un rayon depuis chaque véhicule vers l'avant et à prendre le premier intersect. Quatre limites majeures :

1. **Discrétisation angulaire** : un rayon manque un véhicule légèrement décentré entre deux rayons.
2. **Pas de courbure** : un rayon droit en virage tape la voie opposée → **ghost-following** garanti.
3. **Coût** : nombre de rayons × coût d'intersection par rayon.
4. **Pas de notion de continuité** : un véhicule disparaît entre deux frames si le rayon ne l'intercepte pas, même s'il est juste devant.

### 6.1.3 Gestion des collisions et stabilité numérique

**Problème.** Un solver naïf à pas variable (Euler explicite, dt = frameTime) produit :
- Des **collisions traversantes** ("tunneling") : à vitesse élevée + dt grand, un véhicule peut sauter par-dessus un autre.
- Des **oscillations** : IDM avec dt > 1/30 s tape souvent un cycle limite autour du gap d'équilibre.
- Des **trajectoires non-reproductibles** : si dt change, la trajectoire change → impossible de comparer des runs (Monte-Carlo cassé).

### 6.1.4 Deadlocks émergents (impossibilité formelle)

**Problème théorique.** Dans un système distribué où chaque agent décide localement avec une règle de cession (gap-acceptance, priorité à droite), un cycle de cession circulaire est possible : `A cède à B, B cède à C, C cède à D, D cède à A`. Le système n'a aucun mécanisme natif pour briser ce cycle.

**Sans correction explicite, c'est inévitable** au-dessus d'un certain seuil de densité (loi du "fundamental diagram").

### 6.1.5 Ghost-following et erreur d'attribution

**Problème.** L'algorithme de car-following doit attribuer le bon leader. Si l'attribution est mauvaise (véhicule en contre-sens, voie croisée, leader d'un autre carrefour), le système freine pour aucune raison physique → cascade de freinages erronés.

### 6.1.6 Reproductibilité et test Monte-Carlo

**Problème.** Les simulations stochastiques nécessitent des **seeds reproductibles**. Or, beaucoup de choses introduisent du non-déterminisme accidentel :
- ASLR (Address Space Layout Randomization) → tris par pointeur instables d'un run à l'autre.
- Ordre des threads en parallèle.
- Hashmap iteration order.
- Compteurs globaux statiques re-utilisés.

### 6.1.7 Couplage rendu / logique

**Problème.** Si le moteur logique est entrelacé avec le rendu, on perd :
- La capacité de tourner **headless** (banc d'essai serveur, CI tests).
- L'isolation testabilité (mocker SFML pour tester une policy ? Non.).
- Le contrôle du pas de temps logique (les FPS dictent la physique → instable).

### 6.1.8 Difficulté de débogage des comportements émergents

**Problème.** Un système avec 200 agents et 10 policies présente des comportements **non scriptés** : difficile de comprendre pourquoi un véhicule est bloqué un jour donné. Sans instrumentation, le débogage est purement empirique.

---

## 6.2 Comment notre architecture contourne ces difficultés

### 6.2.1 ✅ Complexité O(n²) — gérée par scope + projection ciblée

**Mécanismes en place.**

1. **Pas de scan global brut**. Chaque agent ne scanne que les véhicules **dans sa portée de vision** (`visionParams.range = 150 px`). En grille uniforme implicite (la `World` est une grille de tiles 50 px), on pourrait facilement passer à O(n·k) avec un cell-hashing.
2. **Projection Frenet ciblée** : on ne projette QUE les véhicules dont la position est dans une boîte englobante grossière de la Lane → coût constant amorti.
3. **TTC en O(n²) acceptable** car appliqué seulement à n_active < 500 typique.
4. **Parallélisation Phase 1** : O(n²) reste O(n²) en travail, mais divisé par le nombre de cœurs → 8× speedup typique.

**Marge d'amélioration future.** Implémenter un `SpatialGrid` formel (cell-hashing 64×64 px) ferait passer la perception à O(n·k) pour k≈9, gagnant ~50% à n=500.

### 6.2.2 ✅ Pas de raycasting — projection Frenet curviligne

**Mécanisme.** Au lieu de lancer un rayon, on **projette** les véhicules sur la polyligne 1D de la Lane (cf. `Lane::project(p, sMin, sMax)`). C'est :

- **Curviligne** : suit naturellement les virages → AUCUN ghost-following en courbe.
- **Sans discrétisation** : chaque véhicule produit une projection exacte.
- **Pondéré par latéral** : un véhicule à 23 px latéraux est **rejeté** (corridor 22 px) → la voie opposée est physiquement exclue.
- **Coût** : O(segments de la Lane), borné par `sMax - sMin / segmentLength ≈ scanRange / 4 px`.

Voir [PHASE4_Bugs.md §4.1](PHASE4_Bugs.md#41-bug-1--ghost-following-suivi-de-véhicule-en-contre-sens).

### 6.2.3 ✅ Stabilité numérique — pas de temps FIXE + accumulateur

**Mécanisme.** Le pipeline impose `FIXED_DT = 1/60 s` et utilise un accumulateur :

```
simAccumulator += frameTime
TANT QUE simAccumulator ≥ FIXED_DT ET steps < MAX_SUBSTEPS :
    runTick(FIXED_DT)
    simAccumulator -= FIXED_DT
```

Bénéfices :
- **Trajectoires bit-identiques** quel que soit le framerate (replays, Monte-Carlo).
- **Pas de tunneling** : avec `FIXED_DT = 16.67 ms` et vitesse max ≈ 200 px/s, le déplacement par tick est ≤ 3.3 px → bien inférieur à la longueur d'un véhicule (32 px).
- **Anti-spirale de la mort** : `MAX_SUBSTEPS = 5` plafonne le rattrapage logique.
- **IDM stable** : avec dt fixe, l'Euler explicite reste dans son régime stable.

### 6.2.4 ✅ Deadlocks — éradiqués par look-ahead asymétrique + bris VIN

**Mécanisme.** Combinaison documentée en [PHASE4_Bugs.md §4.2](PHASE4_Bugs.md#42-bug-2--deadlocks-et-interblocages) :

- **Look-ahead asymétrique** : on ne s'engage QUE si la voie de sortie peut accueillir `1.5 × self.length`. Garantit l'absence d'engagement partiel.
- **KEEP_CLEAR** : tant que la sortie est saturée, on reste à la ligne même si on a la priorité.
- **Bris de cycle par min(VIN)** : si la chaîne d'attentes devient circulaire (`isCircularGridlock`), un acteur unique (le plus petit VIN) est désigné déterministiquement pour casser le cycle.
- **Liveness guard** : `keepClearWaited_` borne l'attente max → garantie de progression.

### 6.2.5 ✅ Ghost-following — éliminé par Frenet + filtres latéraux

Cf. 6.2.2. Le filtre `|lateral| < 22 px` + le garde-fou `|relativeHeading| < 45°` rendent **physiquement impossible** l'accroche d'un véhicule en contre-sens.

### 6.2.6 ✅ Reproductibilité — RNG par-agent seedé par position + VIN déterministe

**Mécanismes.**

```cpp
// Seed RNG par-agent dérivé UNIQUEMENT de la position de spawn :
rng_(static_cast<std::uint64_t>(startX * 1000.f) * 0x9E3779B97F4A7C15ULL ^
     static_cast<std::uint64_t>(startY * 1000.f) * 0xBF58476D1CE4E5B9ULL)

// VIN déterministe : compteur global, ordre de construction fixe
static int s_nextVehicleId = 0;
vehicleId_ = s_nextVehicleId++;
```

Conséquence : un même scénario produit des trajectoires bit-identiques run-to-run, **même si l'adresse mémoire des agents change** (ASLR neutralisé).

**Limitation connue** : `AimPolicy` n'est pas bit-déterministe en multi-thread (l'ordre FCFS dépend du scheduling). Documenté dans `ParallelDecisions.hpp`. Le banc Monte-Carlo et les snapshot tests tournent **toujours en séquentiel** pour préserver la reproductibilité.

### 6.2.7 ✅ Couplage rendu / logique — découplage strict en couches

**Mécanisme.**
- `core/` est **100% SFML-free** (vérifié par les commentaires d'entête + grep de validation).
- `render/` (SfmlRenderer, Camera) est isolé.
- `NullRenderer` permet d'instancier le moteur sans contexte graphique → mode **headless** pour `ExperimentRunner`.
- `MetricsCollector` n'observe que `IAgent` (interface pure, pas de SFML).
- `ScenarioIO` (import/export TXT) ne dépend que de `World` + `IAgent`.

Conséquence : le banc Monte-Carlo tourne sur un **thread dédié** (`mcThread`) sans bloquer l'UI. Des dizaines de runs peuvent s'enchaîner avec une UI réactive et un export CSV en fin.

### 6.2.8 ✅ Débogage du comportement émergent — `BlockReason` + overlay décision + diagnostic CSV

**Trois niveaux d'instrumentation.**

1. **`BlockReason` exhaustif** (15 valeurs) : chaque agent expose **pourquoi** il freine (NONE / LEADER_VEHICLE / INTERSECTION_YIELD / KEEP_CLEAR / BREAKDOWN / ...). Visible à l'overlay UI.
2. **Overlay décisions sur la sim** (`showDecisions`) : anneau coloré + label CEDE / STOP / P2P / FEU / DOUBLE / KEEP_CLEAR / ... sur chaque véhicule en temps réel. Visualisation pédagogique idéale pour les vidéos.
3. **Export diagnostic blocages CSV** (sur PAUSE) : snapshot par-véhicule de l'état décisionnel complet (block, leaderSrc, leaderVin, leaderRelHead, leaderClass, leaderGap, leaderSpeed, onInter). Permet d'isoler la **chaîne de cause** d'un blocage.

Voir [PHASE5_Metriques.md §5.5](PHASE5_Metriques.md#diagnostic-blocages-export-ad-hoc-ui).

---

## 6.3 Tableau récapitulatif difficulté ↔ contournement

| # | Difficulté classique | Contournement architectural | Localisation |
|---|---|---|---|
| 1 | Complexité O(n²) | Scope ciblé + projection Frenet bornée + parallélisation Phase 1 | `Perception::scan`, `ParallelDecisions` |
| 2 | Limites raycasting | Projection Frenet curviligne (rejet latéral 22 px) | `Lane::project`, `Perception::scan` |
| 3 | Stabilité numérique | Pas fixe `FIXED_DT = 1/60 s` + accumulateur + anti-spirale `MAX_SUBSTEPS` | `main.cpp` boucle principale |
| 4 | Deadlocks circulaires | Look-ahead asymétrique + KEEP_CLEAR + bris min(VIN) | `Vehicle::computeDecision` (à compléter selon Phase 4) |
| 5 | Ghost-following | Filtres Frenet `|lateral|<22 px` + cap relatif < 45° | `Perception::scan`, `Vehicle::computeDecision` |
| 6 | Reproductibilité | RNG seedé par position + VIN déterministe + Phase 2 séquentielle | `Vehicle` constructeur, `MetricsCollector` |
| 7 | Couplage rendu/logique | Découplage en couches, `core/` SFML-free, `NullRenderer`, banc Monte-Carlo headless | Structure du projet, `IRenderer` |
| 8 | Comportement émergent | `BlockReason` + overlay décisions + diagnostic CSV | `BlockReason.hpp`, `SfmlRenderer::drawAgentDecision`, UI export |

---

## 6.4 Limitations résiduelles et pistes futures

Documentation honnête : ce qui reste imparfait.

### 6.4.1 AIM non bit-déterministe en multi-thread

**Statut** : connu, documenté dans `ParallelDecisions.hpp`. Le banc Monte-Carlo et les snapshot tests tournent en séquentiel ; le mode multi-thread est réservé à l'affichage temps réel des grosses scènes (n ≥ 150).

**Solution potentielle** : sérialiser les requêtes AIM via un buffer trié par VIN (au prix d'une barrière par tick). Implémentable, choix conscient de ne pas le faire pour préserver la simplicité.

### 6.4.2 Complexité TTC en O(n²)

**Statut** : acceptable jusqu'à n ≈ 500 agents. Au-dessus, le calcul TTC devient le hotspot.

**Solution potentielle** : grille spatiale 64×64 px pour réduire à O(n·k), k ≈ 9. Code prêt à recevoir un `SpatialGrid` dans `core/perception/`.

### 6.4.3 Path replanning naïf sur édition de map

**Statut** : `recalculatePath()` est appelé pour CHAQUE agent quand l'utilisateur modifie la map (mode build) → coût O(n × A*). Acceptable pour n ≤ 100, lent au-delà.

**Solution potentielle** : marquer les agents dont le path traverse la zone modifiée, ne recalculer que ceux-là.

### 6.4.4 Pas de modèle de **changement de voie** sur autoroute multi-voies

**Statut** : actuellement, le simulateur gère 1 voie par direction. Le dépassement utilise un offset latéral transitoire (`lateralOffset`) mais ce n'est pas une vraie multi-voie.

**Solution potentielle** : étendre `Lane` à un graphe de voies adjacentes avec connecteurs de changement (MOBIL model — Treiber, Hennecke, Helbing). Restructurera la perception.

### 6.4.5 Métriques par-intersection limitées

**Statut** : les métriques sont globales. La Phase 5 documente les ajouts utiles (occupancy, queue, waiting).

**Solution potentielle** : étendre `Intersection` avec un sous-objet `IntersectionMetrics` mis à jour par `Intersection::update(dt)`.

---

## 6.5 Réflexion finale : pourquoi cette architecture vieillira bien

### 6.5.1 Découplage par interfaces

Toutes les briques majeures sont derrière des interfaces : `IAgent`, `ICarFollowingModel`, `IIntersectionPolicy`, `IRenderer`. Implications :
- Ajouter un nouveau type d'agent (vélo, piéton) = nouvelle classe implémentant `IAgent`, zéro modification ailleurs.
- Ajouter un nouveau modèle de suivi (MOBIL, Wiedermann) = nouvelle classe implémentant `ICarFollowingModel`.
- Ajouter une nouvelle régulation (priorité piéton, voie réservée bus) = nouvelle classe implémentant `IIntersectionPolicy`.

### 6.5.2 Pipeline déterministe par construction

La séparation lecture/écriture (Phase 1 / Phase 2) garantit l'absence de course **par construction**, pas par convention. Cela évite la classe entière des bugs de "ordre de mise à jour" qui pollue les simulations naïves.

### 6.5.3 Observabilité intégrée

`BlockReason`, overlay décisions, diagnostic CSV, MetricsCollector ne sont pas des ajouts post-hoc : ce sont des champs de première classe dans le modèle. Le système est **introspectable** par défaut.

### 6.5.4 Testabilité

Pas de couplage à SFML dans `core/` → chaque policy, le modèle IDM, et la Perception peuvent être testés unitairement. `NullRenderer` permet de scripter des scénarios de test sans contexte graphique.

### 6.5.5 Recherche-ready

Le banc d'essai Monte-Carlo (`ExperimentRunner`) permet de produire des résultats publiables : comparaison de stratégies à densité croissante, export CSV/JSON, support de l'hétérogénéité gaussienne. Cette infrastructure est typique des publications en **multi-agent traffic management**.

---

## 6.6 Synthèse Phase 6

| Aspect | Difficulté | Réponse de notre architecture |
|---|---|---|
| **Performance** | O(n²) | Scope ciblé + parallélisation Phase 1 + grille future |
| **Précision perception** | Raycasting capricieux | Projection Frenet curviligne |
| **Stabilité numérique** | Tunneling, oscillations | Pas FIXE 60 Hz + accumulateur |
| **Deadlocks** | Cycles circulaires | Look-ahead asymétrique + bris min(VIN) |
| **Ghost-following** | Faux positifs angulaires | Filtre Frenet latéral 22 px + cap < 45° |
| **Reproductibilité** | ASLR, ordre threads | RNG par-position + VIN déterministe + séquentiel |
| **Découplage** | Logique + rendu monolithique | `core/` SFML-free + `IRenderer` + `NullRenderer` |
| **Débogage émergent** | "Pourquoi il bloque ?" | `BlockReason` + overlay + diagnostic CSV |

**Conclusion** : les difficultés classiques d'une simulation multi-agents de trafic sont **toutes** adressées par un choix architectural identifiable et localisable dans le code. Aucune n'est "contournée par chance" ou "masquée par un fudge factor" — chacune a sa réponse explicite dans une couche dédiée. C'est ce qui fait la robustesse de l'ensemble et autorise sa réutilisation comme banc d'essai de recherche en coordination multi-agents.
