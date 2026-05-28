# 📚 Documentation Architecturale — Traffic Simulation MAS

Refonte logique complète + documentation d'illustration pour présentations vidéo.

## Sommaire

| # | Document | Objet |
|---|---|---|
| 1 | [PHASE1_Architecture.md](PHASE1_Architecture.md) | Rétro-ingénierie, patron architectural, flux de données, diagramme de classes |
| 2 | [PHASE2_Taxonomie.md](PHASE2_Taxonomie.md) | Types d'intersections + fiches de comportements (scripts vidéo) |
| 3 | [PHASE3_Audit.md](PHASE3_Audit.md) | Machine à états décisionnelle, hitbox, cinématique, contexte carrefour |
| 4 | [PHASE4_Bugs.md](PHASE4_Bugs.md) | Pseudo-code Ghost-following, Deadlocks (look-ahead asymétrique), Ronds-points |
| 5 | [PHASE5_Metriques.md](PHASE5_Metriques.md) | Métriques d'observabilité + lieux d'instrumentation |
| 6 | [PHASE6_Postmortem.md](PHASE6_Postmortem.md) | Difficultés classiques (O(n²), raycasting, collisions) + contournements |

## Légende rapide

- **Pipeline 2-phases** : `computeDecision()` (lecture pure, parallélisable) puis `integrate(dt)` (écriture privée, séquentielle).
- **Oracle passif** : l'`Intersection` répond à `request()` mais ne commande jamais l'agent.
- **Strategy** : `IIntersectionPolicy` injectée → 9 régulations swappables à chaud sur la même géométrie.
- **Frenet projection** : filtrage same-lane par projection curviligne (élimine le ghost-following).
- **Commit-to-pass** : un véhicule engagé ne re-décide plus de freiner en plein carrefour.

## Glossaire

| Acronyme | Signification |
|---|---|
| IDM | Intelligent Driver Model (Treiber et al., 2000) |
| FCFS | First-Come First-Served |
| VIN | Vehicle Identification Number — bris d'égalité ultime |
| TTC | Time To Collision (métrique de sécurité) |
| AIM | Autonomous Intersection Management (Dresner & Stone) |
| ORCA | Optimal Reciprocal Collision Avoidance (van den Berg) |
| P2P | Pair-à-Pair (VanMiddlesworth, Dresner & Stone, AAMAS 2008) |
| FSM | Finite State Machine |
