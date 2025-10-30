## Veille Reachy Mini — Installation et automatisation

### Dépendances
```bash
python3 -m venv .venv-veille
. .venv-veille/bin/activate
pip install -r requirements/requirements-veille.txt
```

Variables utiles:
- `GH_TOKEN`: jeton GitHub (fortement recommandé)
- `VEILLE_MIN_SCORE`: filtrer par score minimal (ex: `2`)
- `VEILLE_EXCLUDE_OFFICIAL`: `true` pour exclure Pollen (focaliser communauté)
- `VEILLE_SLEEP_BETWEEN`: pause entre sources (ex: `0.5`)
- `VEILLE_STOP_ON_RATE_LIMIT`: `true` = stop sur 1er rate-limit GitHub
- `VEILLE_ONLY_CHANGES`: `true` = sortie console seulement si changements
- `VEILLE_NOTIFY_MACOS`: `true` = notification macOS si changements
- `VEILLE_WATCH_OFFICIAL`: `true` = suivre repo officiel
- `OFFICIAL_NEWS_FILE`: chemin vers un email/texte à parser (shipping.*)

### Exécution manuelle
```bash
export GH_TOKEN=...  # recommandé
export VEILLE_MIN_SCORE=2
export VEILLE_EXCLUDE_OFFICIAL=true
export VEILLE_SLEEP_BETWEEN=0.5
export VEILLE_STOP_ON_RATE_LIMIT=true
export VEILLE_ONLY_CHANGES=true
export VEILLE_NOTIFY_MACOS=true
export VEILLE_WATCH_OFFICIAL=true
export OFFICIAL_NEWS_FILE=/absolute/path/to/reachy_mini_news.txt
python scripts/veille_reachy_mini.py
```

Sorties:
- CSV: `log/veille_reachy_mini.csv`
- JSON: `log/veille_reachy_mini.json`
- DIFF: `log/veille_diff.json` (new/removed/score changes)
- OFFICIEL: `log/reachy_official_status.json` + `log/reachy_official_diff.json`

### Automatisation macOS (launchd)
Créer `~/Library/LaunchAgents/com.reachymini.veille.plist`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>com.reachymini.veille</string>
  <key>ProgramArguments</key>
  <array>
    <string>/usr/bin/env</string>
    <string>bash</string>
    <string>-lc</string>
    <string>cd /Volumes/T7/bbia-reachy-sim && . .venv-veille/bin/activate && GH_TOKEN="$GH_TOKEN" VEILLE_MIN_SCORE=2 VEILLE_EXCLUDE_OFFICIAL=true VEILLE_ONLY_CHANGES=true VEILLE_NOTIFY_MACOS=true python scripts/veille_reachy_mini.py</string>
  </array>
  <key>StartInterval</key><integer>86400</integer>
  <key>StandardOutPath</key><string>/Volumes/T7/bbia-reachy-sim/log/veille_launchd.out</string>
  <key>StandardErrorPath</key><string>/Volumes/T7/bbia-reachy-sim/log/veille_launchd.err</string>
  <key>EnvironmentVariables</key>
  <dict>
    <key>GH_TOKEN</key><string>REPLACE_WITH_TOKEN_IF_USED</string>
    <key>OFFICIAL_NEWS_FILE</key><string>/absolute/path/to/reachy_mini_news.txt</string>
  </dict>
</dict>
  </plist>
```

Charger:
```bash
launchctl load ~/Library/LaunchAgents/com.reachymini.veille.plist
launchctl start com.reachymini.veille
```

### Automatisation Linux (cron)
```cron
15 2 * * * cd /Volumes/T7/bbia-reachy-sim && . .venv-veille/bin/activate && GH_TOKEN="REPLACE" VEILLE_MIN_SCORE=2 VEILLE_EXCLUDE_OFFICIAL=true VEILLE_ONLY_CHANGES=true python scripts/veille_reachy_mini.py >> log/veille_cron.out 2>> log/veille_cron.err
```
## Mise en place de la veille Reachy Mini (automatisation)

Ce guide permet d'exécuter régulièrement la veille sans garder Cursor ouvert.

### 1) Installation des dépendances

Option dédiée (recommandé, environnement existant non perturbé):

```bash
python3 -m venv .venv-veille
. .venv-veille/bin/activate
pip install -r requirements/requirements-veille.txt
```

Variables optionnelles:
- `GH_TOKEN`: jeton GitHub pour élargir les quotas API (Settings → Developer settings → tokens)
- `VEILLE_SLEEP_BETWEEN`: pause (secondes) entre sources pour limiter les rate-limits (ex: `0.5`)
- `VEILLE_STOP_ON_RATE_LIMIT`: si `true`, arrêter la collecte GitHub à la première alerte
- `VEILLE_WATCH_OFFICIAL`: active le suivi officiel Reachy Mini (par défaut `true`)
- `OFFICIAL_NEWS_FILE`: chemin vers un fichier texte/email à parser (expéditions, jalons)

### 2) Exécution manuelle

```bash
# (optionnel) export GH_TOKEN=ton_token_github
# (optionnel) surveiller officiel / ingestion email
export VEILLE_WATCH_OFFICIAL=true
export OFFICIAL_NEWS_FILE=/absolute/path/to/reachy_mini_news.txt
# (optionnel) anti rate-limit
export VEILLE_SLEEP_BETWEEN=0.5
export VEILLE_STOP_ON_RATE_LIMIT=false
python scripts/veille_reachy_mini.py
```

Sortie:
- Résultats ajoutés dans `log/veille_reachy_mini.csv` (horodatés)
- Export détaillé JSON: `log/veille_reachy_mini.json`
- Diff JSON (nouveaux/retirés/scores modifiés): `log/veille_diff.json`
- Suivi officiel: `log/reachy_official_status.json`
- Diff officiel: `log/reachy_official_diff.json`
- Résumé dans la console

### 3) Automatisation macOS (launchd)

Créer un LaunchAgent ex: `~/Library/LaunchAgents/com.reachymini.veille.plist`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>com.reachymini.veille</string>
  <key>ProgramArguments</key>
  <array>
    <string>/usr/bin/env</string>
    <string>bash</string>
    <string>-lc</string>
    <string>cd /Volumes/T7/bbia-reachy-sim && . .venv-veille/bin/activate && GH_TOKEN="$GH_TOKEN" python scripts/veille_reachy_mini.py</string>
  </array>
  <key>StartInterval</key><integer>86400</integer>
  <key>StandardOutPath</key><string>/Volumes/T7/bbia-reachy-sim/log/veille_launchd.out</string>
  <key>StandardErrorPath</key><string>/Volumes/T7/bbia-reachy-sim/log/veille_launchd.err</string>
  <key>EnvironmentVariables</key>
  <dict>
    <key>GH_TOKEN</key><string>REPLACE_WITH_TOKEN_IF_USED</string>
    <key>VEILLE_MIN_SCORE</key><string>2</string>
    <key>VEILLE_EXCLUDE_OFFICIAL</key><string>true</string>
    <key>VEILLE_WATCH_OFFICIAL</key><string>true</string>
    <key>OFFICIAL_NEWS_FILE</key><string>/absolute/path/to/reachy_mini_news.txt</string>
  </dict>
</dict>
  </plist>
```

Charger et démarrer:

```bash
launchctl load ~/Library/LaunchAgents/com.reachymini.veille.plist
launchctl start com.reachymini.veille
```

### 4) Automatisation Linux (cron)

```bash
crontab -e
```

Entrée exemple (tous les jours à 02:15):

```cron
15 2 * * * cd /Volumes/T7/bbia-reachy-sim && . .venv-veille/bin/activate && GH_TOKEN="REPLACE_IF_USED" VEILLE_MIN_SCORE=2 VEILLE_EXCLUDE_OFFICIAL=true python scripts/veille_reachy_mini.py >> log/veille_cron.out 2>> log/veille_cron.err
```

### 5) Exploitation des résultats
- CSV: `log/veille_reachy_mini.csv`
- Document: `docs/veille_reachy_mini.md` (requêtes, scoring, tableau de suivi)

### Notes
- Respect du dossier `log/` pour éviter d’encombrer la racine du projet.
- Le script peut tourner sans `GH_TOKEN` mais avec limites API.
- Vous pouvez adapter la fréquence (jour/heure) selon vos besoins.


