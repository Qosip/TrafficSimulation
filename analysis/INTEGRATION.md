# À intégrer vous-même — checklist du rapport

Le notebook `analysis/report.ipynb` est **complet et exécutable dès maintenant**
(il affiche des données *synthétiques étiquetées* tant que vos vrais CSV/images
ne sont pas là). Pour le finaliser avec vos propres données :

## 1. Installer la couche d'analyse

```bash
pip install -r analysis/requirements.txt
jupyter lab        # ou : jupyter nbconvert --to html --execute analysis/report.ipynb
```

## 2. Produire les CSV (déposez-les dans `analysis/data/`)

`analysis/data/` est *git-ignored* (artefacts régénérables). Le notebook
détecte les fichiers par motif de nom ; respectez les noms ci-dessous.

| Fichier attendu | Comment le produire | Colonnes |
|-----------------|---------------------|----------|
| `montecarlo.csv` | App → panneau **« Monte-Carlo headless »** : cochez les stratégies à comparer (P2P, Priorité fixe, AIM…), lancez, puis **Exporter CSV**. | `strategy,density_veh_s,throughput_per_min,mean_delay_s,mean_speed_px_s,min_ttc_s,ttc_violations,total_stops,completed` |
| `live_xxxxl.csv` | Chargez **Ville XXXXL**, laissez tourner, panneau **« Métriques »** → **Exporter CSV**. | résumé `metric,value` + section `sample,throughput_per_min,mean_delay_s,mean_speed,min_ttc` |
| `perf.csv` *(optionnel)* | Mesurez le coût/tick à 100/300/500/800/1000 agents, avec et sans threading. | `agents,ms_per_tick_single,ms_per_tick_threaded` |

`simbench` (headless) donne en plus les lignes `BENCH … speedup_x_realtime=…` :
```bash
cmake -S . -B build -DTRAFFIC_BUILD_APP=OFF && cmake --build build -t simbench
./build/simbench 30 3
```

## 3. Capturer les visuels (déposez-les dans `analysis/figures/`)

La cellule *Galerie* affiche automatiquement ces fichiers s'ils existent :

| Nom de fichier | Contenu à capturer |
|----------------|--------------------|
| `xxxxl_fluide.png` | Vue **dézoomée** de la Ville XXXXL, ~1000 agents fluides (molette pour dézoomer loin). |
| `suit_avant.png` / `suit_apres.png` | Faux *SUIT* sur la voie opposée (avant) vs corrigé (après). |
| `gridlock_avant.png` / `gridlock_apres.png` | Carrefour bloqué (avant) vs fluidifié par « keep clear » (après). |
| `feu_rouge.png` | Un véhicule qui **freine net** à un feu rouge (label *FEU*). |

GIF/vidéo : ajoutez-les en Markdown (`![](figures/xxx.gif)`) ou via une cellule
HTML `<video>`. Idées : carrefour P2P qui se dégrade quand on monte la densité ;
dépassement réussi vs avorté ; peloton virtuel entrelacé.

## 4. Régénérer le notebook après édition de texte

Le contenu rédigé vit dans `analysis/_build_report.py`. Éditez-y le texte puis :
```bash
python analysis/_build_report.py     # réécrit report.ipynb
```

## 5. Note — test de non-régression

Les corrections de comportement (leader Frenet, commit feu rouge, keep-clear)
**changent volontairement** la trajectoire de référence. Si un `tests/baseline.csv`
existe déjà localement, supprimez-le et relancez `simtests` pour le régénérer :
```bash
rm tests/baseline.csv        # PowerShell : Remove-Item tests/baseline.csv
```
