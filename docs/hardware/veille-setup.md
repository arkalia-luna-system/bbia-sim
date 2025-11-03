## Mise en place de la veille Reachy Mini (automatisation)

> Référence état global
>
> Voir `docs/reference/project-status.md` → "État par axe" (versions Python, CI, packaging) pour l’état actuel et les axes futurs.

Ce guide permet d'exécuter régulièrement la veille sans garder Cursor ouvert.

### 1) Installation des dépendances

Option recommandée: utiliser ton venv principal

```bash
. venv/bin/activate
pip install -r requirements/requirements-veille.txt
```

Variables optionnelles:
- `GH_TOKEN`: jeton GitHub pour élargir les quotas API (Settings → Developer settings → tokens)
- `VEILLE_SLEEP_BETWEEN`: pause (secondes) entre sources pour limiter les rate-limits (ex: `0.5`)
- `VEILLE_STOP_ON_RATE_LIMIT`: si `true`, arrêter la collecte GitHub à la première alerte (par défaut `true`)
- `VEILLE_WATCH_OFFICIAL`: active le suivi officiel Reachy Mini (par défaut `true`)
- `OFFICIAL_NEWS_FILE`: chemin vers un fichier texte/email à parser (expéditions, jalons)
- `VEILLE_ONLY_CHANGES`: si `true`, sortie console uniquement quand changements détectés
- `VEILLE_NOTIFY_MACOS`: si `true`, notification macOS quand changements détectés
- `VEILLE_MAX_ENTRIES`: nombre max de lignes conservées dans `log/veille_reachy_mini.csv` (défaut `5`)

### 2) Exécution manuelle

```bash
export GH_TOKEN=ton_token_github
export VEILLE_WATCH_OFFICIAL=true
export OFFICIAL_NEWS_FILE=/absolute/path/to/reachy_mini_news.txt
export VEILLE_SLEEP_BETWEEN=0.5
export VEILLE_STOP_ON_RATE_LIMIT=true
export VEILLE_ONLY_CHANGES=true
export VEILLE_NOTIFY_MACOS=true
export VEILLE_NOTIFY_THRESHOLD=2
export VEILLE_MAX_ENTRIES=5
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
    <string>cd /Volumes/T7/bbia-reachy-sim && . venv/bin/activate && GH_TOKEN="$GH_TOKEN" VEILLE_ONLY_CHANGES=true VEILLE_NOTIFY_MACOS=true VEILLE_NOTIFY_THRESHOLD=2 VEILLE_MAX_ENTRIES=5 python scripts/veille_reachy_mini.py</string>
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
15 2 * * * cd /Volumes/T7/bbia-reachy-sim && . venv/bin/activate && GH_TOKEN="REPLACE_IF_USED" VEILLE_MIN_SCORE=2 VEILLE_EXCLUDE_OFFICIAL=true VEILLE_ONLY_CHANGES=true VEILLE_NOTIFY_MACOS=true VEILLE_NOTIFY_THRESHOLD=2 VEILLE_MAX_ENTRIES=5 python scripts/veille_reachy_mini.py >> log/veille_cron.out 2>> log/veille_cron.err
```

### 5) Exploitation des résultats
- CSV: `log/veille_reachy_mini.csv`
- Document: `docs/veille_reachy_mini.md` (requêtes, scoring, tableau de suivi)

### Notes
- Respect du dossier `log/` pour éviter d’encombrer la racine du projet.
- Le script peut tourner sans `GH_TOKEN` mais avec limites API.
- Vous pouvez adapter la fréquence (jour/heure) selon vos besoins.

## Onboarding rapide Reachy‑mini Wireless (Jour J)

Scripts prêts à l’emploi (voir `scripts/onboarding/`):

```bash
# Préparer l’environnement
bash scripts/onboarding/setup_env.sh
source venv/bin/activate

# Renseigner puis exporter l’environnement BBIA
open -a TextEdit scripts/onboarding/env_bbia_example.txt
export $(tr -d '\r' < scripts/onboarding/env_bbia_example.txt | xargs)

# Vérifier le réseau (mDNS/IP)
bash scripts/onboarding/check_network.sh "$BBIA_REACHY_HOST"

# Lancer une démo "safe" et journaliser dans log/onboarding_demo.log
bash scripts/onboarding/run_demo_safe.sh
```

Bonnes pratiques réseau: même SSID/VLAN, éviter réseau invité/isolation client. En cas d’instabilité Wi‑Fi, envisager CPL (Powerline). Les journaux sont centralisés dans `log/`.

