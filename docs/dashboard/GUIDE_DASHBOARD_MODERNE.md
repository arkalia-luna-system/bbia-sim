# 🎨 Guide Dashboard Moderne BBIA

**Dernière mise à jour : 15 Décembre 2025
**Version BBIA** : 1.3.2
**Objectif** : Guide complet pour utiliser le dashboard moderne BBIA

---

## 📋 Vue d'ensemble

Le dashboard moderne BBIA offre une interface épurée et moderne pour contrôler votre robot Reachy Mini, similaire au dashboard des testeurs bêta.

### Fonctionnalités principales

- ✅ **Contrôles Media Visuels** : Sliders volume + waveforms audio
- ✅ **Vue 3D Robot** : Visualisation 3D avec Three.js
- ✅ **Quick Actions** : 15 boutons emoji pour actions rapides
- ✅ **FPS Display** : Indicateur de performance en temps réel
- ✅ **Design Épuré** : Fond blanc avec image floutée
- ✅ **Graphiques Temps Réel** : Chart.js pour latence, FPS, CPU, RAM (24 Nov 2025)
- ✅ **Sliders Émotions** : Contrôle intensité émotions 0-100% (24 Nov 2025)
- ✅ **Mode Démo Read-only** : Toggle pour désactiver tous les contrôles (24 Nov 2025)
- ✅ **Presets Exportables** : Export/Import JSON des configurations émotions (24 Nov 2025)
- ✅ **PWA Support** : Installation native avec manifest + service worker (24 Nov 2025)

---

## 🎵 Contrôles Media

### Section Speaker (Haut-parleur)

**Fonctionnalités :**
- Slider volume (0-100%)
- Waveform audio en temps réel
- Statut actif/inactif

**Utilisation :**
1. Ouvrir le dashboard : `http://localhost:8000/`
2. Section "Media Controls" → "Built-in Speaker"
3. Ajuster le slider volume
4. Observer la waveform en temps réel

**API Endpoint :**
```bash
POST /development/api/media/speaker/volume
Content-Type: application/json

{
  "volume": 0.75  # 0.0 à 1.0
}
```

### Section Microphone

**Fonctionnalités :**
- Slider volume (0-100%)
- Waveform audio en temps réel
- Statut actif/inactif

**API Endpoint :**
```bash
POST /development/api/media/microphone/volume
Content-Type: application/json

{
  "volume": 0.5  # 0.0 à 1.0
}
```

### Section Camera

**Fonctionnalités :**
- Toggle ON/OFF
- Statut activé/désactivé
- Placeholder pour flux vidéo (à venir)

**API Endpoint :**
```bash
POST /development/api/media/camera/toggle
Content-Type: application/json

{
  "enabled": true  # true ou false
}
```

### Récupérer Statut Media

```bash
GET /development/api/media/status
```

**Réponse :**
```json
{
  "speaker_volume": 0.5,
  "microphone_volume": 0.5,
  "camera_enabled": true,
  "speaker_active": true,
  "microphone_active": true
}
```

---

## 🤖 Vue 3D Robot

### Description

Visualisation 3D du robot Reachy Mini avec Three.js, animée selon l'état du daemon.

**États possibles :**
- 🟢 **running** : Vert - Robot actif
- 🟡 **starting** : Jaune - Démarrage
- 🟠 **stopping** : Orange - Arrêt
- ⚪ **stopped** : Gris - Arrêté
- 🔴 **error** : Rouge - Erreur

### Utilisation

La vue 3D s'affiche automatiquement dans la section "Daemon Status" du dashboard.

**Fonctionnalités :**
- Rotation automatique selon état
- Changement de couleur selon état
- Synchronisation avec statut daemon (polling 1s)

**Améliorations futures :**
- Chargement modèle STL réel (actuellement placeholder géométrie basique)
- Synchronisation WebSocket (actuellement polling)

---

## ⚡ Quick Actions

### Description

Grille de 15 boutons emoji pour actions rapides (émotions et actions).

**Boutons disponibles :**
- 😊 **Heureux** - Émotion happy
- 😢 **Triste** - Émotion sad
- 😕 **Confus** - Émotion confused
- 😮 **Surpris** - Émotion surprised
- 😠 **En colère** - Émotion angry
- 🕶️ **Cool** - Émotion cool
- 🤔 **Curieux** - Émotion curious
- 👋 **Saluer** - Action wave
- 🙏 **Prier** - Action pray
- 😴 **Dormir** - Action sleep
- 🎉 **Excité** - Émotion excited
- 🎭 **Danser** - Action dance
- 🎨 **Art** - Action art
- 🎪 **Fête** - Action party
- 🎬 **Film** - Action movie

### Utilisation

1. Cliquer sur un bouton emoji
2. L'action/émotion est déclenchée (structure créée, intégration WebSocket à compléter)

---

## 📊 FPS Display

### Description

Indicateur de performance en temps réel affiché en haut à droite du dashboard.

**Fonctionnalités :**
- Affichage FPS (Frames Per Second)
- Couleur dynamique :
  - 🟢 **Vert** : ≥30 FPS (performance bonne)
  - 🟠 **Orange** : <30 FPS (performance faible)
- Mise à jour temps réel (requestAnimationFrame)

### Utilisation

Le FPS display s'affiche automatiquement. Aucune action requise.

---

## 🎨 Design Épuré

### Caractéristiques

- **Fond blanc** : `bg-white` avec image SVG floutée en arrière-plan
- **Sections arrondies** : `rounded-lg` avec ombres légères
- **Espacement cohérent** : `gap-4` entre sections
- **Polices** : Archivo (titres) + Asap (texte)

### Personnalisation

Pour modifier le design, éditer :
- `src/bbia_sim/daemon/app/dashboard/templates/base.html` - Fond et styles globaux
- `src/bbia_sim/daemon/app/dashboard/static/style.css` - Styles personnalisés

---

## 🔧 Architecture Technique

### Fichiers JavaScript

- `media.js` : Gestion contrôles media (sliders, toggles, API calls)
- `waveform.js` : Visualisation waveforms audio (Web Audio API + Canvas)
- `robot_3d.js` : Visualisation 3D robot (Three.js)
- `fps_display.js` : Affichage FPS temps réel

### Fichiers Templates

- `base.html` : Template de base (Three.js CDN, fond blanc)
- `index.html` : Page principale (sections + scripts)
- `sections/media.html` : Section contrôles media
- `sections/quick_actions.html` : Section Quick Actions
- `sections/daemon.html` : Section daemon (avec canvas 3D)

### API Backend

- `src/bbia_sim/daemon/app/routers/media.py` : Endpoints API media
  - `POST /development/api/media/speaker/volume`
  - `POST /development/api/media/microphone/volume`
  - `POST /development/api/media/camera/toggle`
  - `GET /development/api/media/status`

---

## 🚀 Démarrage Rapide

### 1. Démarrer le daemon

```bash
# Activer venv
source venv/bin/activate

# Démarrer daemon
python -m bbia_sim.daemon.app.main
```

### 2. Ouvrir le dashboard

Ouvrir dans le navigateur : `http://localhost:8000/`

### 3. Utiliser les contrôles

- Ajuster volumes speaker/microphone
- Activer/désactiver caméra
- Observer vue 3D robot
- Utiliser Quick Actions

---

## 📚 Références

- **Guide Comportements** : `docs/guides/GUIDE_COMPORTEMENTS.md`
- **Guide LLM** : `docs/ai/llm.md`
- **Guide Chat** : `docs/guides/GUIDE_CHAT_BBIA.md`
- **Comparaison Dashboard** : `docs/dashboard/COMPARAISON_DASHBOARD_TESTEURS.md`

---

**Document créé le :** 8 Décembre 2025
**Version BBIA :** 1.3.2
**Auteur :** Arkalia Luna System

