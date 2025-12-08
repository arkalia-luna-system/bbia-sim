# 📊 Guide Dashboard Advanced - BBIA-SIM

**Date** : 8 Décembre 2025  
**Version** : 1.1  
**Compatibilité Python** : 3.11+

> **Dashboard avancé** avec monitoring temps réel, WebSocket, métriques performance et contrôle robot

---

## 🎯 Vue d'ensemble

Le `dashboard_advanced.py` est un dashboard amélioré qui offre :

- ✅ **Monitoring temps réel** via WebSocket
- ✅ **Métriques de performance** (CPU, RAM, latence)
- ✅ **Métriques vision** (FPS, détections, objets)
- ✅ **Métriques audio** (latence, buffer, underruns)
- ✅ **Contrôle robot** (emotions, mouvements)
- ✅ **Chat interactif** avec IA
- ✅ **Panneau troubleshooting interactif** (détection automatique + tests)
- ✅ **Interface Web** moderne avec FastAPI
- ✅ **Logos BBIA intégrés** : Favicon et logo dans le header (7 Déc 2025)

**Note** : Le dashboard principal (`src/bbia_sim/daemon/app/dashboard/`) inclut maintenant aussi :
- ✅ **Graphiques temps réel** avec Chart.js (`sections/telemetry_charts.html`) (24 Nov 2025)
- ✅ **Sliders émotions avec intensité** (`sections/emotions.html`) (24 Nov 2025)
- ✅ **Mode démo read-only** (`sections/demo_mode.html`) (24 Nov 2025)
- ✅ **Presets exportables** (API `/api/presets` via `routers/presets.py`) (24 Nov 2025)
- ✅ **PWA support** avec manifest + service worker (`static/manifest.json`, `static/sw.js`, icônes) (24 Nov 2025)

### Fichiers Créés Dashboard Principal (24 Nov 2025)
- ✅ `templates/sections/telemetry_charts.html` - Graphiques Chart.js
- ✅ `templates/sections/demo_mode.html` - Mode démo read-only
- ✅ `templates/sections/emotions.html` - Sliders émotions
- ✅ `static/manifest.json` - Manifest PWA
- ✅ `static/sw.js` - Service Worker
- ✅ `static/images/icon-192.png` - Icône PWA 192x192
- ✅ `static/images/icon-512.png` - Icône PWA 512x512
- ✅ `routers/presets.py` - API presets

### 📸 Captures d'écran

Des captures d'écran du dashboard sont disponibles dans `assets/images/` :

- `Capture d'écran 2025-11-13 à 14.14.20.png` - Vue initiale avec sliders joints
- `Capture d'écran 2025-11-13 à 14.14.27.png` - Vue complète avec indicateurs de statut
- `Capture d'écran 2025-11-13 à 14.14.51.png` - Métriques détaillées et graphiques
- `Capture d'écran 2025-11-13 à 14.14.59.png` - Vue complète avec tous les panneaux

**Voir** : [`assets/MEDIAS_INVENTAIRE.md`](../../assets/MEDIAS_INVENTAIRE.md) pour l'inventaire complet.

---

## 🚀 Installation et Démarrage

### Prérequis

```bash
# Activer l'environnement virtuel
source venv/bin/activate

# Vérifier dépendances
pip install fastapi uvicorn websockets

```

### Lancement

```bash
# Depuis le répertoire racine
python -m bbia_sim.dashboard_advanced

# Ou directement
python src/bbia_sim/dashboard_advanced.py

```

Le dashboard sera accessible sur : <http://localhost:8080>

---

## 📊 Fonctionnalités Principales

### 1. WebSocket Manager

**Gestionnaire de connexions WebSocket avec métriques** :

```python
from bbia_sim.dashboard_advanced import WebSocketManager

manager = WebSocketManager()
await manager.connect(websocket)
await manager.broadcast({"type": "status", "data": {...}})

```

**Métriques disponibles** :

- Nombre de connexions actives
- Historique des métriques (limite configurable)
- Performance (latence, CPU, RAM)
- Vision (FPS, détections, objets)
- Audio (latence, buffer, underruns)

### 2. Métriques Performance

**Métriques système** :

- CPU usage (%)
- RAM usage (MB)
- Latence réseau (ms)
- FPS vision
- Buffer audio

**Accès** :

```python
metrics = manager.get_performance_metrics()
# Retourne: {"cpu": 15.2, "ram": 245.5, "latency": 12.3, ...}

```

### 3. Contrôle Robot

**Commandes disponibles** :

- **Emotions** : `happy`, `sad`, `angry`, `surprised`, `neutral`
- **Mouvements** : Head pose, antennas, body yaw
- **Actions** : Wake up, sleep, stop

**Exemple** :

```python
# Via WebSocket
await websocket.send_json({
    "type": "robot_command",
    "command": "emotion",
    "emotion": "happy",
    "intensity": 0.8
})

```

### 4. Chat Interactif

**Chat avec IA pour contrôle robot** :

```python
# Envoyer message
await websocket.send_json({
    "type": "chat",
    "message": "Souriez et tournez la tête à droite"
})

# Réponse IA
{
    "type": "chat_response",
    "response": "Je vais sourire et tourner la tête à droite.",
    "actions": ["emotion:happy", "head_pose:right"]
}

```

---

## 🧪 Tests

**Coverage** : **76.71%** ✅ (47 tests, 1156 lignes)

**Lancer les tests** :

```bash
# Tests dashboard_advanced
pytest tests/test_dashboard_advanced.py -v

# Avec coverage
pytest tests/test_dashboard_advanced.py --cov=src/bbia_sim/dashboard_advanced --cov-report=term-missing

```

**Tests principaux** :

- ✅ Initialisation WebSocketManager
- ✅ Connexion/déconnexion WebSocket
- ✅ Métriques performance
- ✅ Métriques vision/audio
- ✅ Contrôle robot (emotions, mouvements)
- ✅ Chat interactif
- ✅ Gestion erreurs

---

## 📡 API WebSocket

### Endpoints WebSocket

**`/ws/dashboard`** : Connexion principale dashboard

**Messages entrants** :

```json
{
    "type": "robot_command",
    "command": "emotion",
    "emotion": "happy",
    "intensity": 0.8
}

```

**Messages sortants** :

```json
{
    "type": "metrics_update",
    "timestamp": "2025-12-20T22:00:00Z",
    "performance": {"cpu": 15.2, "ram": 245.5},
    "vision": {"fps": 30.0, "detections": 5},
    "audio": {"latency": 12.3, "buffer": 0.95}
}

```

---

## 🔧 Configuration

### Variables d'environnement

```bash
# Port du dashboard (défaut: 8080)
BBIA_DASHBOARD_PORT=8080

# Host (défaut: 0.0.0.0)
BBIA_DASHBOARD_HOST=0.0.0.0

# Limite historique métriques (défaut: 100)
BBIA_METRICS_HISTORY_LIMIT=100

# Log level (défaut: INFO)
BBIA_LOG_LEVEL=INFO

```

---

## 🎨 Interface Web

**Interface moderne** accessible sur <http://localhost:8080> :

- **Dashboard principal** : Vue d'ensemble métriques
- **Contrôle robot** : Panel commandes (emotions, mouvements)
- **Chat IA** : Interface conversation
- **Métriques temps réel** : Graphiques performance

---

## 📊 Métriques Détaillées

### Performance

- **CPU** : Usage CPU en % (moyenne sur 1s)
- **RAM** : Mémoire utilisée en MB
- **Latence** : Latence réseau en ms
- **FPS** : Images par seconde vision

### Vision

- **FPS** : Fréquence traitement images
- **Détections** : Nombre objets détectés
- **Objets** : Liste objets détectés (YOLO)
- **Latence pipeline** : Temps traitement image

### Audio

- **Latence** : Latence audio E2E en ms
- **Buffer** : Niveau buffer (0.0-1.0)
- **Underruns** : Nombre underruns
- **Overruns** : Nombre overruns

---

## 🐛 Dépannage

### Panneau Troubleshooting Interactif

**Nouveau** : Le dashboard inclut maintenant un panneau troubleshooting interactif !

**Utilisation** :

1. Ouvrir le dashboard : <http://localhost:8080>
2. Descendre jusqu'au panneau "🔧 Troubleshooting"
3. Cliquer sur "🔍 Vérifier Tout" pour un diagnostic complet
4. Utiliser les boutons "Test" pour vérifier individuellement :
   - 📷 Test Caméra
   - 🔊 Test Audio
   - 🌐 Test Réseau

**Fonctionnalités** :

- ✅ Détection automatique de problèmes (Python, dépendances, caméra, audio, réseau, MuJoCo, ports, permissions)
- ✅ Score global de santé système
- ✅ Solutions suggérées pour chaque problème
- ✅ Liens automatiques vers documentation
- ✅ Tests interactifs par composant

**API Endpoints** :

```bash
# Vérification complète
curl <http://localhost:8080/api/troubleshooting/check>

# Tests individuels
curl -X POST <http://localhost:8080/api/troubleshooting/test/camera>
curl -X POST <http://localhost:8080/api/troubleshooting/test/audio>
curl -X POST <http://localhost:8080/api/troubleshooting/test/network>

# Documentation
curl <http://localhost:8080/api/troubleshooting/docs>
```

### WebSocket ne se connecte pas

**Vérifier** :

1. Utiliser le panneau troubleshooting (voir ci-dessus)
2. Port 8080 disponible
3. Firewall ne bloque pas
4. Logs serveur pour erreurs

```bash
# Vérifier port
lsof -i :8080

# Relancer dashboard
python -m bbia_sim.dashboard_advanced

```

### Métriques ne s'affichent pas

**Vérifier** :

1. Utiliser le panneau troubleshooting pour diagnostic
2. Robot connecté (`backend.connect()`)
3. WebSocket connecté
4. Métriques activées dans code

### Chat IA ne répond pas

**Vérifier** :

1. Module `bbia_huggingface` disponible
2. Modèle LLM chargé
3. Logs pour erreurs

---

## 📚 Références

- **Code source** : `src/bbia_sim/dashboard_advanced.py`
- **Tests** : `tests/test_dashboard_advanced.py`
- **Dashboard standard** : `src/bbia_sim/dashboard.py`
- **API WebSocket** : `docs/development/api/CONTRATS_REST_WS.md`

---

**BBIA-SIM** - Dashboard Advanced 📊✨

**Version** : 1.0  
**Date** : 8 Décembre 2025  
**Coverage** : **76.71%** ✅ (47 tests)

---

**Dernière mise à jour** : 8 Décembre 2025

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Guide de Démarrage](../guides/GUIDE_DEMARRAGE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)
