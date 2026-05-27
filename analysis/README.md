# analysis/ — rapport de recherche reproductible

Dossier reserve a la couche d'**analyse scientifique** du simulateur. Il est
volontairement decouple du moteur C++ : le moteur **produit des donnees**
(CSV/JSON via `MetricsCollector` et `ExperimentRunner`/`simbench`), cette couche
les **interprete et trace**.

## Pipeline cible (CI/CD)

Le job `report` de `.github/workflows/main.yml` (aujourd'hui un squelette
commente) sera active une fois des notebooks presents ici. Il :

1. exécute les notebooks (`jupyter nbconvert --execute`) — qui lisent les CSV de
   métriques et génèrent les figures (débit/délai/TTC vs densité, comparaison des
   stratégies de coordination : Priorité fixe / P2P / AIM / ORCA / Peloton…) ;
2. assemble un rapport HTML stylisé ;
3. le publie sur **GitHub Pages** via `actions/deploy-pages`.

## Sources de données

| Source | Production | Format |
|--------|-----------|--------|
| Banc d'essai Monte-Carlo | `ExperimentRunner::exportCsv` / écran Monte-Carlo de l'app | CSV (`strategy,density,throughput,delay,…`) |
| Métriques live d'une simulation | `MetricsCollector::exportCsv` / `exportJson` | CSV / JSON |
| Benchmark de performance | `simbench` (lignes `BENCH key=value`) | texte |

## Pour démarrer (local)

```bash
pip install -r analysis/requirements.txt
# Générer un jeu de données headless puis le tracer dans un notebook :
cmake -S . -B build -DTRAFFIC_BUILD_APP=OFF && cmake --build build -t simbench
./build/simbench 30 3 > analysis/data/bench.txt
jupyter lab
```

> Les notebooks et leurs sorties de données vont dans `analysis/` ; `analysis/data/`
> est ignoré par git (artefacts régénérables).
