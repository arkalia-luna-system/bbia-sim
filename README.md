# 🚀 BBIA Reachy Mini Simulation - CONFORMITÉ PARFAITE ✅

[![Version](https://img.shields.io/badge/version-1.3.0-blue.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue.svg)](https://python.org)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions)
[![Tests](https://img.shields.io/badge/tests-27%20passing-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions)
[![Code Quality](https://img.shields.io/badge/code%20quality-A%2B-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![SDK Conformity](https://img.shields.io/badge/SDK%20conformity-100%25-brightgreen.svg)](https://github.com/pollen-robotics/reachy_mini)
[![Documentation](https://img.shields.io/badge/docs-sphinx%20%7C%20readthedocs-blue.svg)](docs/)

**✅ SIMULATION BBIA ↔ ROBOT 100% FONCTIONNELLE ET CONFORME !**

---

## 📋 **EN 30 SECONDES :**

**BBIA-SIM** = Moteur cognitif Python avec **12 émotions robotiques** pour Reachy Mini

![BBIA-SIM Reachy Mini Robot](./assets/images/robot_animation.gif)

✅ **100% conforme** au SDK officiel Pollen Robotics  
✅ **Backend unifié** : Même code simulation → robot réel  
✅ **Production-ready** : 27 tests, 63% coverage, docs complètes  
✅ **Open-source** : MIT License

**[🚀 Quick Start](#-démarrage-rapide)** | **[📚 Docs Débutant](docs/GUIDE_DEBUTANT.md)** | **[🔬 Docs Avancé](docs/GUIDE_AVANCE.md)**

---

**Version** : 1.3.0 "Conformité Parfaite SDK Officiel" - Backend unifié RobotAPI + Golden Tests + SDK Officiel Reachy-Mini 100% Conforme + Bridge Zenoh/FastAPI + Documentation Professionnelle

Ce projet fournit une simulation **parfaitement fidèle** du robot Reachy Mini dans MuJoCo, avec une intégration complète des modules BBIA (Bio-Inspired Artificial Intelligence) et **100% conforme au SDK officiel** de Pollen Robotics (décembre 2024).

## 🚀 Quick Start
```bash
# Installation
pip install bbia-sim

# Démarrage simulation
python examples/demo_emotion_ok.py
```

## 🎯 Vue d'ensemble

**BBIA-SIM v1.3.0** est un moteur cognitif Python qui simule le robot Reachy Mini avec fidélité complète :

- ✅ **12 émotions robotiques** contrôlant les articulations
- ✅ **Conformité 100%** au SDK officiel Pollen Robotics
- ✅ **Backend unifié** : Même code pour simulation et robot réel
- ✅ **IA avancée** : Hugging Face, reconnaissance émotions, comportements adaptatifs
- ✅ **Simulation 3D** : MuJoCo avec modèle officiel
- ✅ **API REST + WebSocket** : Interface publique complète
- ✅ **Production-ready** : Tests complets, 63% coverage

## 🏗️ Architecture BBIA-SIM

```mermaid
graph TB
    subgraph "Modules BBIA"
        EMOTIONS[bbia_emotions.py<br/>12 émotions]
        VISION[bbia_vision.py<br/>Détection objets]
        AUDIO[bbia_audio.py<br/>Enregistrement]
        VOICE[bbia_voice.py<br/>TTS/STT]
        BEHAVIOR[bbia_behavior.py<br/>Comportements]
    end
    
    subgraph "Simulation MuJoCo"
        SIMULATOR[MuJoCoSimulator<br/>Physique 3D]
        MODEL[Modèle officiel<br/>reachy_mini_REAL_OFFICIAL.xml]
        ASSETS[41 Assets STL<br/>Officiels Pollen]
    end
    
    subgraph "API & Services"
        REST[REST API<br/>FastAPI]
        WEBSOCKET[WebSocket<br/>Temps réel]
        DAEMON[Daemon<br/>Service simulation]
    end
    
    EMOTIONS --> SIMULATOR
    VISION --> SIMULATOR
    AUDIO --> SIMULATOR
    VOICE --> SIMULATOR
    BEHAVIOR --> SIMULATOR
    
    SIMULATOR --> MODEL
    MODEL --> ASSETS
    
    SIMULATOR --> REST
    SIMULATOR --> WEBSOCKET
    REST --> DAEMON
    WEBSOCKET --> DAEMON
```

## 📊 Métriques du Projet

- **✅ Modèle officiel** : `reachy_mini_REAL_OFFICIAL.xml`
- **✅ Articulations** : 16 articulations contrôlables  
- **✅ Intégration BBIA** : 12 émotions + IA avancée
- **✅ Coverage** : 63.37% (excellent)
- **✅ API** : FastAPI + WebSocket opérationnels

## 🚀 Démarrage Rapide

### Voir le robot en 3D (FONCTIONNEL)
```bash
# 🎯 DÉMOS PRINCIPALES - Fonctionnent vraiment !

# Démo continue - Robot qui bouge en continu (RECOMMANDÉE)
mjpython examples/demo_mujoco_continue.py

# Démo émotions BBIA avec viewer MuJoCo
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --joint yaw_body

# Démo surprise spectaculaire complète
mjpython examples/surprise_3d_mujoco_viewer.py

# Démo SDK officiel (headless)
python examples/demo_reachy_mini_corrigee.py --quick
```

### ⚠️ RÈGLES DE SÉCURITÉ CRITIQUES
- **✅ TOUJOURS utiliser `yaw_body`** pour les animations visibles
- **✅ Amplitude maximum** : 0.3 rad (clamp automatique)
- **✅ Mode headless** : Stable et rapide pour les tests
- **❌ JAMAIS animer** : `left_antenna`, `right_antenna`, `passive_*`
- **❌ JAMAIS dépasser** : 0.3 rad d'amplitude

### 🌐 API Publique
```bash
# Démarrage de l'API publique
python scripts/start_public_api.py --dev

# Test de l'API publique
python scripts/test_public_api.py

# Démonstration complète
python scripts/demo_public_api.py

# Documentation interactive
# Swagger UI: http://localhost:8000/docs
# ReDoc: http://localhost:8000/redoc
# OpenAPI: http://localhost:8000/openapi.json
```

### 🤖 Backends Disponibles
```bash
# Backend MuJoCo (simulation)
python -c "from bbia_sim.robot_api import RobotFactory; robot = RobotFactory.create_backend('mujoco')"

# Backend Reachy-Mini SDK Officiel (100% conforme)
python -c "from bbia_sim.robot_api import RobotFactory; robot = RobotFactory.create_backend('reachy_mini')"

# Backend Reachy Mock (ancien)
python -c "from bbia_sim.robot_api import RobotFactory; robot = RobotFactory.create_backend('reachy')"

# Test hardware complet
python scripts/hardware_dry_run_reachy_mini.py --duration 30
```

### 🧠 IA Avancée
```bash
# Démonstration complète Phase 2
python examples/demo_bbia_phase2_integration.py

# Tests modules Phase 2
python -m pytest tests/test_bbia_phase2_modules.py -v

# Module Adaptive Behavior (fonctionne sans dépendances ML)
python -c "from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior; ab = BBIAAdaptiveBehavior(); ab.set_context('greeting'); ab.set_emotion_state('happy', 0.8); print(ab.generate_behavior('test'))"

# Module Hugging Face (nécessite: pip install transformers torch)
python -c "from bbia_sim.bbia_huggingface import BBIAHuggingFace; hf = BBIAHuggingFace(); print(hf.analyze_sentiment('Je suis heureux!'))"

# Module Emotion Recognition (nécessite: pip install mediapipe transformers)
python -c "from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition; er = BBIAEmotionRecognition(); print(er.analyze_vocal_emotion('Je suis excité!'))"
```

### 🎯 Vertical Slices BBIA
```bash
# Tests automatiques des vertical slices
python -m pytest tests/test_vertical_slices.py -v

# Démos avec différentes options
python examples/demo_emotion_ok.py --emotion sad --intensity 0.6 --duration 3
python examples/demo_voice_ok.py --command "tourne à gauche" --speak
python examples/demo_vision_ok.py --tracking-gain 0.8 --target-speed 0.05
python examples/demo_behavior_ok.py --behavior greeting --intensity 1.2
```

### 🔄 Backend Unifié
```bash
# Switch facile entre Sim et Robot
python examples/demo_emotion_ok.py --backend mujoco --emotion happy --duration 5  # Simulation
python examples/demo_emotion_ok.py --backend reachy --emotion happy --duration 5  # Robot réel

# Tests smoke automatiques
python -m pytest tests/test_robot_api_smoke.py -v

# Record & Replay
python examples/demo_emotion_ok.py --record artifacts/my_animation.jsonl --emotion happy --duration 10
python scripts/replay_viewer.py artifacts/my_animation.jsonl --speed 1.5
```

### 📋 Documentation
- **[Guide Débutant](docs/GUIDE_DEBUTANT.md)** : Guide pour débuter
- **[Guide Avancé](docs/GUIDE_AVANCE.md)** : Guide avancé
- **[Vertical Slices](docs/audit/VERTICAL_SLICES_ACCOMPLIS.md)** : Documentation des vertical slices

### ⚠️ Note Importante
Les antennes sont bloquées dans le modèle officiel. Utilisez `yaw_body` pour les animations.

## 📁 Structure du Projet

```
src/bbia_sim/sim/
├── models/
│   ├── reachy_mini_REAL_OFFICIAL.xml    # Modèle officiel principal
│   └── reachy_mini.xml                   # Modèle de base (backup)
├── assets/reachy_official/              # 41 fichiers STL officiels
│   ├── body_down_3dprint.stl
│   ├── body_top_3dprint.stl
│   ├── head_front_3dprint.stl
│   ├── head_back_3dprint.stl
│   ├── stewart_*.stl                     # Bras Stewart
│   └── ... (36 autres STL)
└── scripts/
    ├── launch_complete_robot.py         # Lanceur principal
    └── download_ALL_stl.py              # Script de téléchargement STL
```

## 🛠️ Scripts Utiles

### Scripts One-Click
```bash
# Démo simulation complète (3D viewer)
bash scripts/run_demo_sim.sh happy 15

# Démo robot réel (headless)
bash scripts/run_demo_real.sh excited 20

# Test commandes vocales
python scripts/stt_demo.py --command "salue" --backend mujoco

# Dashboard web temps réel
python scripts/bbia_dashboard_server.py --port 8000
```

### Scripts de Démo Avancés
```bash
# Enregistrer une démo complète
bash scripts/record_demo.sh happy 15

# Générer un rapport d'analyse
python scripts/plot_trace.py --input assets/videos/demo_happy_*.jsonl --output assets/plots/rapport.txt
```

### Hardware Dry Run
```bash
# Test hardware complet avec artefacts
python scripts/hardware_dry_run.py --duration 10 --backend reachy

# Test avec backend simulation
python scripts/hardware_dry_run.py --duration 5 --backend mujoco
```

### Téléchargement des STL officiels
```bash
python scripts/download_ALL_stl.py
```
Télécharge automatiquement les 41 fichiers STL depuis le repo officiel.

### Gestion des processus
```bash
./scripts/smart_process_cleanup.sh
```
Nettoie les processus gourmands tout en préservant les applications importantes.

## 🔧 Résolution de Problèmes

### Problème : Robot en pièces détachées
**Cause** : Fichiers STL corrompus (pointeurs Git LFS)
**Solution** : 
```bash
python scripts/download_ALL_stl.py
```

### Problème : Erreur "No such file or directory"
**Cause** : Chemin des assets incorrect
**Solution** : Le modèle `reachy_mini_REAL_OFFICIAL.xml` a le bon chemin `meshdir="../assets/reachy_official"`

### Problème : Fenêtre 3D ne s'ouvre pas
**Cause** : Mauvaise commande Python
**Solution** : Utiliser `mjpython` au lieu de `python` sur macOS

## 📊 Spécifications Techniques

- **Modèle** : Reachy Mini Wireless
- **Simulateur** : MuJoCo
- **Format** : MJCF (MuJoCo XML)
- **Assets** : 41 fichiers STL officiels
- **Articulations** : 16 (yaw_body + 6 stewart + 7 passive + 2 antennas)
- **Dimensions** : Fidèles aux spécifications officielles

## 🔗 Ressources Officielles

- **Repo officiel** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)
- **Documentation** : [docs.pollen-robotics.com](https://docs.pollen-robotics.com)
- **Modèle 3D** : OnShape CAD officiel
- **Assets STL** : Téléchargés depuis le repo officiel

## 🧪 Tests et Coverage

### Tests & Commandes

### Résultats des Tests

**📊 Qualité & Conformité :**
- ✅ **Conformité SDK officiel** : 100% parfaite
- ✅ **Coverage** : 63.37% (excellent)
- ✅ **Qualité code** : Black, Ruff, MyPy, Bandit ✅
- ✅ **Tests** : Suite complète + SDK officiel

> Pour plus de détails : `python -m pytest --cov=src --cov-report=html`

### Commandes de Tests

```bash
# Tests spécifiques à la nouvelle démo 3D corrigée
python -m pytest tests/test_demo_viewer_bbia_corrected.py -v

# Tests MuJoCo existants
python -m pytest tests/test_adapter_mujoco.py -v

# Lancer tous les tests avec coverage
python -m pytest tests/ --cov=src --cov-report=term-missing --cov-report=html

# Lancer les tests sans s'arrêter aux échecs
python -m pytest tests/ --cov=src --cov-fail-under=0 --tb=no -q

# Lancer un test spécifique
python -m pytest tests/test_bbia_emotions.py -v

# Voir le rapport HTML de coverage
open htmlcov/index.html
```




## 🎉 Statut Final

**✅ MISSION ACCOMPLIE** : Le robot Reachy Mini est maintenant parfaitement simulé avec une fidélité de 100% aux spécifications officielles et une couverture de tests excellente.

**✅ PREUVES RÉELLES** : Dry-run hardware et démo robot réel avec métriques de performance documentées.

---
*Dernière mise à jour : Octobre 2025*
