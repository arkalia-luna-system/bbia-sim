# 🎨 Dashboard Officiel-Like - Documentation

**Date** : Oct / Nov. 2025  
**Version** : 1.0  
**Conformité** : Identique au dashboard officiel `pollen-robotics/reachy_mini`

---

---

## 📋 Vue d'ensemble

Le dashboard BBIA-SIM a été créé pour être **identique** au dashboard officiel Reachy Mini, avec la même structure, le même design et les mêmes fonctionnalités.

---

## 🏗️ Structure

```
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

### 2. **Section Apps**
- ✅ Liste des applications installées
- ✅ Toggle pour démarrer/arrêter chaque app
- ✅ Bouton suppression d'app

### 3. **Section App Store**
- ✅ Liste des apps disponibles sur Hugging Face
- ✅ Bouton "Install" pour chaque app
- ✅ Modal d'installation avec logs
- ✅ WebSocket pour suivi installation en temps réel

### 4. **Lecteur de Mouvements**
- ✅ Sélection dataset (Dances/Emotions)
- ✅ Liste des mouvements disponibles
- ✅ Boutons Play/Stop
- ✅ WebSocket pour statut en temps réel

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
python -c "from bbia_sim.daemon.app.main import app; print('✅ OK')"
```

---

## 📚 Références

- **SDK Officiel** : https://github.com/pollen-robotics/reachy_mini
- **Documentation BBIA** : `docs/quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md`
- **Comparaison** : `docs/quality/audits/COMPARAISON_DOCUMENTATION_OFFICIELLE.md`

---

## 🎉 Statut

✅ **Dashboard créé et fonctionnel**
✅ **Structure identique à l'officiel**
✅ **Design conforme**
✅ **Intégration complète dans main.py**
⚠️ **Assets SVG optionnels (peuvent être ajoutés plus tard)**

