# 🎨 Dashboard Officiel-Like - Documentation

**Date** : 26 Novembre 2025  
**Version** : 1.4.0  
**Conformité** : Identique au dashboard officiel `pollen-robotics/reachy_mini`

---

## 📋 Vue d'ensemble

Le dashboard BBIA-SIM a été créé pour être **identique** au dashboard officiel Reachy Mini, avec la même structure, le même design et les mêmes fonctionnalités.

---

## 🏗️ Structure

```text
src/bbia_sim/daemon/app/dashboard/
├── templates/
│   ├── base.html              # Template de base (Tailwind CSS)
│   ├── index.html             # Page principale
│   └── sections/
│       ├── daemon.html        # Contrôle daemon (ON/OFF)
│       ├── apps.html          # Applications installées
│       ├── appstore.html      # Hugging Face App Store
│       └── move_player.html   # Lecteur mouvements
├── static/
│   ├── style.css              # Styles (Archivo/Asap fonts)
│   ├── js/
│   │   ├── daemon.js          # Gestion daemon
│   │   ├── apps.js            # Gestion apps
│   │   ├── appstore.js        # App Store HF
│   │   └── move_player.js    # Lecteur mouvements
│   └── assets/
│       └── README.md          # Documentation assets SVG

```

---

## 🎨 Design

### Identique à l'officiel :

- ✅ **Tailwind CSS** : Framework CSS utilisé
- ✅ **Polices** : Archivo (titre) + Asap (texte)
- ✅ **Layout** : Container centré, responsive
- ✅ **Couleurs** : Fond gris clair (#f9fafb), blanc pour sections
- ✅ **Toggle switch** : Vert/rouge pour ON/OFF daemon

---

## 🚀 Fonctionnalités

### 1. **Section Daemon**

- ✅ Toggle ON/OFF pour démarrer/arrêter le daemon
- ✅ Animation SVG selon l'état (réveil, éveillé, endormi, erreur)
- ✅ Statut backend affiché (Up and ready, Waking up, etc.)
- ✅ Lecteur de mouvements enregistrés intégré

### 2. **Section Mode Démo** (Nouveau - 24 Nov 2025)

- ✅ Toggle pour activer/désactiver le mode read-only
- ✅ Désactivation automatique de tous les contrôles en mode démo
- ✅ Persistance dans localStorage
- ✅ Message d'information quand mode démo actif

### 3. **Section Télémétrie Temps Réel** (Nouveau - 24 Nov 2025)

- ✅ Graphiques Chart.js pour latence, FPS, CPU, RAM
- ✅ Métriques affichées en temps réel
- ✅ Connexion WebSocket `/ws/telemetry` automatique
- ✅ Mise à jour fluide avec limite de 30 points de données

### 4. **Section Apps**

- ✅ Liste des applications installées
- ✅ Toggle pour démarrer/arrêter chaque app
- ✅ Bouton suppression d'app

### 5. **Section App Store**

- ✅ Liste des apps disponibles sur Hugging Face
- ✅ Bouton "Install" pour chaque app
- ✅ Modal d'installation avec logs
- ✅ WebSocket pour suivi installation en temps réel

### 6. **Section Émotions avec Intensité** (Nouveau - 24 Nov 2025)

- ✅ Sliders pour 6 émotions (happy, sad, excited, angry, surprised, neutral)
- ✅ Intensité ajustable 0-100%
- ✅ Mise à jour en temps réel avec debounce
- ✅ API endpoint `/api/motion/emotion`

### 7. **Section Presets** (Nouveau - 24 Nov 2025)

- ✅ Export/Import de presets d'émotions en JSON
- ✅ API complète `/api/presets` (GET, POST, DELETE, apply)
- ✅ Stockage dans `~/.bbia_sim/presets/`

### 8. **Lecteur de Mouvements**

- ✅ Sélection dataset (Dances/Emotions)
- ✅ Liste des mouvements disponibles
- ✅ Boutons Play/Stop
- ✅ WebSocket pour statut en temps réel

### 9. **PWA Support** (Nouveau - 24 Nov 2025)

- ✅ Manifest.json pour installation PWA
- ✅ Service Worker avec cache offline
- ✅ Icônes 192x192 et 512x512
- ✅ Installation native sur mobile/desktop

---

## 🔧 Utilisation

### Démarrage du dashboard

```bash
# Via module Python
python -m bbia_sim.daemon.app.main

# Ou via script
python scripts/start_public_api.py

```

### Accès

- **Dashboard** : http://localhost:8000/
- **API Docs** : http://localhost:8000/docs
- **ReDoc** : http://localhost:8000/redoc

---

## 📦 Assets SVG

Les animations SVG pour le daemon sont optionnelles. Si absentes, le dashboard fonctionne mais sans animations visuelles.

### Fichiers SVG attendus (optionnels) :

- `awake-cartoon.svg` - Animation réveil
- `awake-cartoon-static.svg` - Robot éveillé (statique)
- `go-to-sleep-cartoon.svg` - Animation endormissement
- `reachy-mini-sleeping-static.svg` - Robot endormi
- `no-wifi-cartoon.svg` - Erreur/connexion

**Source** : Ces fichiers peuvent être copiés depuis le repo officiel `pollen-robotics/reachy_mini` si disponibles dans `src/reachy_mini/daemon/app/dashboard/static/assets/`.

---

## ✅ Conformité avec SDK Officiel

### Structure identique :

- ✅ Templates Jinja2 modulaires
- ✅ Sections séparées (daemon, apps, appstore)
- ✅ Static files montés sur `/static`
- ✅ Route `GET /` qui rend le dashboard

### API conforme :

- ✅ Endpoints `/development/api/daemon/*` pour contrôle daemon
- ✅ Endpoints `/development/api/apps/*` pour gestion apps
- ✅ Endpoints `/development/api/move/*` pour mouvements
- ✅ WebSocket pour temps réel

### JavaScript identique :

- ✅ Même logique de gestion daemon
- ✅ Même structure apps/appstore
- ✅ Même lecteur de mouvements

---

## 🎯 Différences avec Dashboard BBIA Avancé

Ce dashboard **officiel-like** est différent du dashboard avancé BBIA (`dashboard_advanced.py`) :

| Aspect | Dashboard Officiel-Like | Dashboard Avancé BBIA |
|--------|------------------------|----------------------|
| **Design** | Minimaliste, épuré | Plus riche, métriques |
| **Focus** | Reachy Mini uniquement | BBIA + Reachy Mini |
| **Fonctionnalités** | Daemon, Apps, Mouvements | + Émotions, Chat, Vision |
| **Template** | Jinja2 (modulaire) | HTML inline |
| **Usage** | Principal (route `/`) | Secondaire (optionnel) |

**Recommandation** : Le dashboard officiel-like est le dashboard **principal** accessible sur `/`, conforme au SDK officiel.

---

## 🔍 Vérification

### Checklist de vérification :

```bash
# 1. Vérifier structure
ls -la src/bbia_sim/daemon/app/dashboard/

# 2. Vérifier templates
ls src/bbia_sim/daemon/app/dashboard/templates/
ls src/bbia_sim/daemon/app/dashboard/templates/sections/

# 3. Vérifier static files
ls src/bbia_sim/daemon/app/dashboard/static/
ls src/bbia_sim/daemon/app/dashboard/static/js/

# 4. Vérifier intégration
python -c "from bbia_sim.daemon.app.main import app; import logging; logging.info('✅ OK')"

```

---

## 📚 Références

- **SDK Officiel** : <https://github.com/pollen-robotics/reachy_mini>
- **Documentation BBIA** : `docs/quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md`
- **Comparaison** : `docs/quality/audits/COMPARAISON_DOCUMENTATION_OFFICIELLE.md`
- **Dashboard avancé** : [`docs/development/dashboard-advanced.md`](../development/dashboard-advanced.md) - Dashboard BBIA avec métriques temps réel, chat et vision
- **Captures d'écran** : 4 captures du dashboard avancé disponibles dans `assets/images/` (Nov 2025) - Voir [`assets/MEDIAS_INVENTAIRE.md`](../../assets/MEDIAS_INVENTAIRE.md)

---

## 🎉 Statut

✅ **Dashboard créé et fonctionnel**
✅ **Structure identique à l'officiel**
✅ **Design conforme**
✅ **Intégration complète dans main.py**
✅ **Extensions ajoutées (24 Nov 2025)** :
- Graphiques temps réel (Chart.js)
- Mode démo read-only
- Sliders émotions avec intensité
- Presets exportables
- PWA support complet
⚠️ **Assets SVG optionnels (peuvent être ajoutés plus tard)**

## 📁 Fichiers Créés/Modifiés

### Templates Sections
- ✅ `sections/daemon.html` - Contrôle daemon
- ✅ `sections/apps.html` - Applications installées
- ✅ `sections/appstore.html` - Hugging Face App Store
- ✅ `sections/move_player.html` - Lecteur mouvements
- ✅ `sections/media.html` - Contrôles media
- ✅ `sections/quick_actions.html` - Actions rapides
- ✅ `sections/installation_wizard.html` - Assistant installation
- ✅ `sections/telemetry_charts.html` - Graphiques temps réel (24 Nov 2025)
- ✅ `sections/demo_mode.html` - Mode démo read-only (24 Nov 2025)
- ✅ `sections/emotions.html` - Sliders émotions (24 Nov 2025)

### Static Files
- ✅ `static/manifest.json` - Manifest PWA (24 Nov 2025)
- ✅ `static/sw.js` - Service Worker (24 Nov 2025)
- ✅ `static/images/icon-192.png` - Icône PWA 192x192 (24 Nov 2025)
- ✅ `static/images/icon-512.png` - Icône PWA 512x512 (24 Nov 2025)
- ✅ `static/style.css` - Styles Tailwind
- ✅ `static/js/*.js` - Scripts JavaScript
- ✅ `static/logos/` - Logos BBIA professionnels (30 logos + 4 assets identity) (7 Déc 2025)

### API Routers
- ✅ `routers/presets.py` - API presets (24 Nov 2025)
