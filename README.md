# <img src="assets/logos/bbia/identity/bbia-speaking-512.svg" alt="BBIA" width="96" height="96" style="vertical-align: middle; margin-bottom: 0.2em;"> BBIA Reachy Mini Simulation

<div align="center">

[![Version](https://img.shields.io/badge/version-1.4.0-blue.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![Python](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://python.org)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![CI Status](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/badge/tests-1362-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions)
[![Code Quality](https://img.shields.io/badge/code%20quality-A%2B-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![SDK Conformity](https://img.shields.io/badge/SDK%20conformity-100%25-brightgreen.svg)](https://github.com/pollen-robotics/reachy_mini)
[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue.svg)](docs/)
[![Coverage](https://img.shields.io/badge/coverage-68.86%25-brightgreen)](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim)
[![Typing: mypy](https://img.shields.io/badge/typing-mypy-4B8BBE)](https://github.com/arkalia-luna-system/bbia-sim)
[![Lint: ruff](https://img.shields.io/badge/lint-ruff-46A2F1)](https://github.com/arkalia-luna-system/bbia-sim)
[![Security: bandit](https://img.shields.io/badge/security-bandit-2E5E82)](https://github.com/arkalia-luna-system/bbia-sim)
[![Release](https://img.shields.io/github/v/release/arkalia-luna-system/bbia-sim)](https://github.com/arkalia-luna-system/bbia-sim/releases)

[![Stars](https://img.shields.io/github/stars/arkalia-luna-system/bbia-sim?style=social)](https://github.com/arkalia-luna-system/bbia-sim)
[![Forks](https://img.shields.io/github/forks/arkalia-luna-system/bbia-sim?style=social)](https://github.com/arkalia-luna-system/bbia-sim)
[![Issues](https://img.shields.io/github/issues/arkalia-luna-system/bbia-sim)](https://github.com/arkalia-luna-system/bbia-sim/issues)
[![PRs](https://img.shields.io/github/issues-pr/arkalia-luna-system/bbia-sim)](https://github.com/arkalia-luna-system/bbia-sim/pulls)

</div>

<div align="center">

## 🧠 Moteur Cognitif Python pour Robot Reachy Mini

### 🌟 Simulation Fidèle • IA Avancée • SDK 100% Conforme

[![🚀 Quick Start](https://img.shields.io/badge/🚀-Quick%20Start-green)](#-démarrage-rapide)
[![📚 Documentation](https://img.shields.io/badge/📚-Documentation-blue)](docs/)
[![🤖 Try Demo](https://img.shields.io/badge/🤖-Try%20Demo-orange)](#-voir-le-robot-en-3d)
[![🔧 Installation](https://img.shields.io/badge/🔧-Installation-purple)](#-démarrage-rapide)

</div>

---

<div align="center">

<img src="assets/logos/bbia/logos/bbia-horizontal-clean-serenity-512.svg" alt="BBIA" width="400" height="auto">

</div>

---

## 📋 **EN 30 SECONDES**

<div align="center">

### 🎯 BBIA-SIM : Moteur Cognitif Python pour Reachy Mini

**12 émotions robotiques** (6 SDK officiel + 6 étendues) • **Simulation 3D fidèle** • **IA avancée**

**🎨 [Voir tous les logos BBIA →](http://localhost:8000/test-logos)** *(Lancez le dashboard d'abord)*

</div>

</div>

### ✨ Points Clés

<div align="center">

| 🎯 **Caractéristique** | 📊 **Détails** |
|:---:|:---|
| ✅ **SDK Officiel** | 100% conforme Pollen Robotics |
| 🔄 **Backend Unifié** | Simulation ↔ Robot réel |
| 🧪 **Tests Automatisés** | 1362 tests (unitaires + E2E) |
| 📚 **Documentation** | 128 fichiers Markdown |
| ⚡ **Performance** | Caches IA optimisés |
| 🔒 **CI/CD Pro** | Black, Ruff, MyPy, Bandit |
| 🌟 **Open Source** | License MIT |

</div>

## 🧠 Stack IA : Pourquoi ces Dépendances ?

BBIA utilise une stack IA lourde pour permettre des interactions naturelles et une perception avancée. Voici pourquoi chaque dépendance majeure est nécessaire :

| Dépendance | Rôle | Poids approximatif | Justification |
|------------|------|-------------------|---------------|
| **PyTorch** (`torch>=2.0.0`) | Backend pour modèles deep learning (transformers, YOLO) | ~2 GB | Framework de base pour tous les modèles IA (LLM, vision, NLP) |
| **transformers** (`>=4.30.0`) | Modèles HuggingFace (sentiment, LLM conversationnel, reconnaissance émotions) | ~500 MB | Bibliothèque standard pour modèles pré-entraînés (Phi-2, Mistral, RoBERTa) |
| **MediaPipe** (`>=0.10.0`) | Détection visages, pose humaine, landmarks en temps réel | ~50 MB | Détection pose 33 points, visages, gestes (bras levés, debout/assis) |
| **ultralytics (YOLO)** (`>=8.0.0`) | Détection objets temps réel (personnes, objets, mains) | ~100 MB | Détection objets COCO dataset, tracking temps réel |
| **openai-whisper** (`>=20231117`) | Reconnaissance vocale (Speech-to-Text) multilingue | ~3 GB | Transcription audio → texte, support multilingue |

**Note importante** : Les dépendances IA sont **facultatives** si tu veux juste tester la simulation sans reconnaissance. Voir [`docs/development/setup/environments.md`](docs/development/setup/environments.md) pour des setups allégés.

**Total approximatif** : ~6 GB (principalement PyTorch + Whisper). C'est normal pour un projet IA robotique avec vision, audio et LLM.

## 📊 Statistiques

- **Fichiers Python source**: 92 fichiers (35,988 lignes)
- **Fichiers de tests**: 166 fichiers (39,200+ lignes) — [✅ Mis à jour 22 Nov. 2025](docs/quality/RESUME_AUDIT_26NOV2025.md)
- **Fonctions de test**: 1,804 tests identifiés
- **Tests collectés**: 1,743 tests (1,805 total, 62 deselected) — [✅ Vérifié 26 Novembre 2025](docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_26NOV2025.md)
- **Coverage global**: 68.86% ([Codecov](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim))
- **Coverage modules core**: ~50% (mesure pertinente)
- **Modules BBIA**: 15+ modules spécialisés
- **Comportements**: 21 comportements intelligents (7 de base + 14 avancés) — [✅ Vérifié 26 Novembre 2025](docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_26NOV2025.md)
- **Exemples**: **44 exemples** (39 existants + 5 nouveaux créés 22 Nov. 2025) — [✅ 100% exploitation capacités](docs/quality/audits/AUDIT_EXPLOITATION_100_PERCENT_2626NOV2025.md)
- **Documentation**: 219 fichiers Markdown (dans `docs/`) — [✅ Vérifié 26 Novembre 2025](docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_26NOV2025.md)
- **Commits Git**: 423 commits
- **Identité visuelle**: 10 variantes émotionnelles, 3 formats de logos, interface live interactive — [✅ Complète 7 Décembre 2025](assets/logos/bbia/README.md)

## 🎨 Identité Visuelle BBIA

**BBIA dispose d'une identité visuelle complète** adaptée à ses différentes émotions et contextes d'utilisation :

- 🎭 **10 variantes émotionnelles** : Chaque émotion de BBIA a son propre style visuel (Serenity, Power, Mystery, Awakening, Creative, Rainy, Stormy, Explosive, Sunny, Snowy)
- 📐 **3 formats de logos** : Horizontal, vertical et symbole seul pour tous les usages
- 🎨 **2 styles visuels** : Clean (apaisant) et Wireframe (énergique) selon le contexte
- 🎯 **Assets d'interface** : HUD Cyber-HUD, icônes, bannières pour une expérience cohérente

**Note** : Les logos sont un **gadget visuel** pour l'identité de BBIA. La **simulation principale** utilise MuJoCo (voir section "Voir le robot en 3D").  
**🎨 [Voir tous les logos →](http://localhost:8000/test-logos)** *(Lancez le dashboard d'abord)*  
**Documentation** : [`assets/logos/bbia/README.md`](assets/logos/bbia/README.md)

## 🏆 Qualité Code

BBIA-SIM maintient des standards stricts de qualité:

- **Tests de couverture**: 80% minimum
- **Linting**: black, ruff, mypy, bandit
- **Python**: 3.11+ (3.12 testé dans CI) ✅
- **CI/CD**: GitHub Actions avec vérifications automatiques (Python 3.11 + 3.12, gitleaks) ✅
- **Sécurité**: Analyse statique avec Bandit + scan secrets (gitleaks) ✅
- **Performance**: Optimisations continues et monitoring (métriques Prometheus complétées) ✅

**[🚀 Quick Start](#-démarrage-rapide)** | **[📚 Guide de Démarrage](docs/guides/GUIDE_DEMARRAGE.md)** | **[🔬 Docs Avancé](docs/guides/GUIDE_AVANCE.md)** | **[🌐 English README](README_EN.md)** | **[🎬 Vidéos Démo](docs/guides/GUIDE_VIDEOS_DEMONSTRATION.md)** | **[🌍 Guide Communauté](docs/community/GUIDE_COMMUNAUTE.md)** | **[🤝 Contribuer](docs/community/GUIDE_CONTRIBUTEURS_COMPLET.md)** | **[🚀 Innovations](docs/community/INNOVATIONS_BBIA.md)**

---

Version 1.4.0 – 100% d'exploitation des capacités, 67 exemples fonctionnels, tests améliorés, documentation enrichie.

**🎉 Mise à jour 26 Novembre 2025** : 
- ✅ **100% d'exploitation des capacités**
- ✅ **19 issues Reachy officiel traitées** (95% des applicables)
- ✅ **65 appels logging optimisés** (performance +10-20%)
- ✅ **3 nouveaux exemples** (sleeping_pose, collision, registry)
- ✅ **Code quality excellent** (black, ruff, mypy, bandit OK)
- ✅ 44 exemples totaux (39 existants + 5 nouveaux créés pour atteindre 100%)
- ✅ Tous les comportements avancés ont maintenant des exemples dédiés (15/15)
- ✅ Tous les endpoints API ont maintenant des exemples dédiés (11/11)
- ✅ Tous les modules BBIA ont maintenant des exemples dédiés (16/16)
- ✅ Qualité code vérifiée (Black, Ruff, MyPy, Bandit) ✅
- ✅ Tests créés pour tous les nouveaux exemples ✅

Ce projet fournit une simulation fidèle du robot Reachy Mini dans MuJoCo, avec une intégration des modules BBIA (Bio-Inspired Artificial Intelligence) et une conformité validée avec le SDK officiel de Pollen Robotics (Oct / Nov. 2025).

## 🚀 Quick Start

### Option 1 : Script All-in-One (Recommandé) ⚡

```bash
# 🚀 Installation automatique complète + dashboard
./scripts/reachy-mini-sim-starter.sh

# 🤖 Le script fait tout automatiquement :
# ✅ Vérifie prérequis (Python, pip, mjpython)
# ✅ Crée/active l'environnement virtuel
# ✅ Installe BBIA-SIM + dépendances
# ✅ Vérifie l'installation (bbia_doctor)
# ✅ Lance le dashboard sur http://localhost:8000
```

**Options disponibles** :

- `--skip-install` : Saute l'installation (vérification uniquement)
- `--skip-dashboard` : Ne lance pas le dashboard
- `--help` : Affiche l'aide complète

### Option 2 : Docker Compose (Recommandé pour environnement isolé) 🐳

```bash
# 🐳 Démarrage avec Docker Compose (environnement pré-configuré)
docker-compose up -d

# ✅ Vérifier que l'API est accessible
curl http://localhost:8000/api/health

# 📊 Accéder au dashboard
open http://localhost:8000

# 🛑 Arrêter
docker-compose down
```

**Avantages Docker** :
- ✅ Environnement isolé et reproductible
- ✅ Pas besoin d'installer dépendances système
- ✅ Configuration optimale pré-configurée
- ✅ Mode développement avec hot-reload disponible (`docker-compose -f docker-compose.yml -f docker-compose.dev.yml up`)

### Option 3 : Installation Manuelle

```bash
# 🔧 Installation (mode dev recommandé)
pip install -e .[dev]

# 🎮 Démarrage simulation (macOS: utiliser mjpython)
mjpython examples/demo_emotion_ok.py
# ou sur Linux/Windows:
python examples/demo_emotion_ok.py
```

## 🤖 Première utilisation – Reachy Mini (robot réel)

- Prérequis:
  - SDK Reachy Mini installé côté PC
  - Robot allumé, alimenté, sur le même réseau Wi‑Fi que le PC
  - Ports réseau nécessaires ouverts

### ✅ Check‑list

- Alimentation/LED OK
- Wi‑Fi opérationnel (SSID/mot de passe)
- Version SDK compatible (voir docs/quality/compliance)
- Dry‑run: connexion → wake_up → look_at_world → emergency_stop

### 📦 Dry‑run rapide

```bash
pip install -e .[dev,test]
python scripts/hardware_dry_run_reachy_mini.py --output-dir artifacts
```

### 🧩 Installation du SDK officiel + Daemon Reachy Mini

> Le paquet `reachy-mini` n’est pas embarqué dans les dépendances de ce projet. Installez-le séparément (PyPI) pour contrôler le robot réel ou lancer la simulation officielle.

```bash
# 📦 Installer le SDK officiel (PC local)
pip install -U reachy-mini

# (Option simulation MuJoCo officielle)
pip install -U "reachy-mini[mujoco]"
```

### 🟣 Lancer le daemon Reachy Mini (officiel)

```bash
# 🚀 Démarrage standard (daemon officiel)
reachy-mini-daemon

# 🎯 Mode simulation MuJoCo (daemon officiel)
reachy-mini-daemon --sim

# 🎬 Scènes disponibles (simulation): minimal (table+objets)
reachy-mini-daemon --sim --scene minimal
```

#### Note macOS (MuJoCo)

Sur macOS, l’exécution MuJoCo peut nécessiter `mjpython` :

```bash
mjpython -m reachy_mini.daemon.app.main --sim --scene minimal
```

### 🔍 Vérifier l'API du daemon officiel

```bash
curl http://localhost:8000/api/state/full
# ou ouvrir la doc OpenAPI quand le daemon tourne :
# http://localhost:8000/docs
```

### 🏥 Health Checks et Métriques

```bash
# ❤️ Health check (liveness)
curl http://localhost:8000/metrics/healthz

# Readiness check
curl http://localhost:8000/metrics/readyz

# Health détaillé
curl http://localhost:8000/metrics/health

# Métriques Prometheus ✅ **Complétées** (24 Nov. 2025)

**Nouvelles métriques disponibles** :
- ✅ `bbia_watchdog_heartbeat_age_seconds` - âge heartbeat watchdog
- ✅ `bbia_robot_connected` - statut connexion robot (1/0)
- ✅ `bbia_latency_p50_ms`, `bbia_latency_p95_ms`, `bbia_latency_p99_ms` - latence percentiles

Voir `docs/observabilite/OBSERVABILITE.md` pour plus de détails.

# Métriques Prometheus
curl http://localhost:8000/metrics/prometheus
```

### 🔧 Diagnostic Environnement

```bash
# Diagnostic complet de l'environnement (inclut Zenoh, daemon, WiFi)
python -m bbia_sim --doctor
# ou
python scripts/bbia_doctor.py
```

**Vérifie :**
- ✅ Dépendances (numpy, opencv, transformers, zenoh, reachy_mini, etc.)
- ✅ Modèles IA disponibles
- ✅ Variables d'environnement
- ✅ Configuration projet
- ✅ **Zenoh** (installation + session locale)
- ✅ **Daemon** `reachy-mini-daemon`
- ✅ **Réseau** (IP locale + ports 8000, 7447)

### 🔗 Intégration côté BBIA‑SIM

- BBIA‑SIM détecte et utilise le backend `reachy_mini` si le SDK est installé et le daemon accessible.
- Pour la simulation interne BBIA‑SIM (sans daemon officiel), continuez à utiliser les démos `examples/*.py` et le backend `mujoco`.

### 🔗 Guide complet Reachy Mini

- `docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md`

## 🏆 Badges Qualité & CI/CD

<div align="center">

[![CI Status](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml)
[![Coverage](https://img.shields.io/badge/coverage-68.86%25-brightgreen)](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim)
[![Typing: mypy](https://img.shields.io/badge/typing-mypy-4B8BBE)](https://github.com/arkalia-luna-system/bbia-sim)
[![Lint: ruff](https://img.shields.io/badge/lint-ruff-46A2F1)](https://github.com/arkalia-luna-system/bbia-sim)
[![Security: bandit](https://img.shields.io/badge/security-bandit-2E5E82)](https://github.com/arkalia-luna-system/bbia-sim)
[![Release](https://img.shields.io/github/v/release/arkalia-luna-system/bbia-sim)](https://github.com/arkalia-luna-system/bbia-sim/releases)

</div>

## 🎯 Vue d'ensemble

BBIA-SIM v1.4.0 simule le robot Reachy Mini avec un haut niveau de fidélité :

- 12 émotions robotiques BBIA (6 émotions SDK officiel + 6 émotions étendues)
- Conformité au SDK officiel Pollen Robotics
- Backend unifié : même code pour simulation et robot réel
- IA optionnelle : Hugging Face, reconnaissance d’émotions, comportements
- Simulation 3D : MuJoCo avec modèle officiel
- API REST + WebSocket
- Tests automatisés avec suivi de la couverture en CI

## 🏗️ Architecture BBIA-SIM

### Vue d'Ensemble Architecture Complète

```mermaid
graph TB
    subgraph "Interface Utilisateur"
        WEB[Dashboard Web<br/>FastAPI + WebSocket]
        CLI[CLI Python<br/>Scripts]
        API[REST API<br/>OpenAPI/Swagger]
    end
    
    subgraph "Modules BBIA Cognitive"
        EMOTIONS[bbia_emotions.py<br/>12 émotions robotiques]
        VISION[bbia_vision.py<br/>YOLOv8n + MediaPipe]
        AUDIO[bbia_audio.py<br/>Enregistrement/Audio]
        VOICE[bbia_voice.py<br/>Whisper STT + pyttsx3 TTS]
        BEHAVIOR[bbia_behavior.py<br/>Comportements intelligents]
        ADAPTIVE[bbia_adaptive_behavior.py<br/>Comportements adaptatifs]
        EMOTION_REC[bbia_emotion_recognition.py<br/>Reconnaissance émotions]
        HF[bbia_huggingface.py<br/>LLM + NLP + Tools]
        TOOLS[bbia_tools.py<br/>Outils LLM robot]
        MEMORY[bbia_memory.py<br/>Mémoire contextuelle]
    end
    
    subgraph "Couche Abstraction"
        ROBOTAPI[RobotAPI Unifié<br/>Interface Abstraite]
        FACTORY[RobotFactory<br/>Backend Factory]
    end
    
    subgraph "Backends Robot"
        MUJOCO[MuJoCo Simulator<br/>Physique 3D]
        REACHY[Reachy Mini SDK<br/>Robot Physique]
    end
    
    subgraph "Modèles & Assets"
        MODEL[Modèle officiel<br/>reachy_mini_REAL_OFFICIAL.xml]
        ASSETS[41 Assets STL<br/>Officiels Pollen]
    end
    
    WEB --> BEHAVIOR
    CLI --> ROBOTAPI
    API --> ROBOTAPI
    
    EMOTIONS --> ROBOTAPI
    VISION --> ROBOTAPI
    AUDIO --> ROBOTAPI
    VOICE --> ROBOTAPI
    BEHAVIOR --> HF
    BEHAVIOR --> ADAPTIVE
    BEHAVIOR --> MEMORY
    BEHAVIOR --> EMOTION_REC
    HF --> TOOLS
    TOOLS --> ROBOTAPI
    HF --> ROBOTAPI
    
    ROBOTAPI --> FACTORY
    FACTORY --> MUJOCO
    FACTORY --> REACHY
    
    MUJOCO --> MODEL
    MODEL --> ASSETS
    
    style WEB fill:#90EE90
    style ROBOTAPI fill:#FFD700
    style HF fill:#87CEEB
    style MUJOCO fill:#FFB6C1
    style REACHY fill:#FFB6C1
```

### Flux de Traitement Conversationnel

```mermaid
sequenceDiagram
    participant User as Utilisateur
    participant Web as Dashboard/API
    participant Behavior as BBIABehavior
    participant HF as BBIAHuggingFace
    participant NLP as NLP Detection
    participant Tools as BBIATools
    participant Robot as RobotAPI
    participant Backend as MuJoCo/Reachy
    
    User->>Web: Message vocal/texte
    Web->>Behavior: execute(message)
    Behavior->>HF: chat(message, enable_tools=True)
    
    HF->>NLP: Détection outil (sentence-transformers)
    NLP-->>HF: Outil détecté + confiance
    
    alt Outil détecté
        HF->>Tools: execute_tool(name, params)
        Tools->>Robot: Action robot
        Robot->>Backend: Commande mouvement
        Backend-->>Robot: ✅ Mouvement exécuté
    else Conversation normale
        HF->>HF: Génération réponse LLM
        HF-->>Behavior: Réponse textuelle
    end
    
    Behavior->>Web: Réponse + Action
    Web-->>User: Feedback visuel/audio
```

## 📊 Métriques du projet

<div align="center">

| Métrique | Valeur | Statut |
|:--------:|:------:|:-----:|
| **Modèle officiel** | `reachy_mini_REAL_OFFICIAL.xml` | ✅ |
| **Articulations** | 16 total (9 contrôlables) | ✅ |
| **Intégration BBIA** | 12 émotions + IA avancée | ✅ |
| **Coverage modules core** | ~50% | ✅ [📊 Rapport](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim) |
| **API** | FastAPI + WebSocket | ✅ |

</div>

> Suivi global (État par axe)
>
> Consultez `docs/reference/project-status.md` (section "État par axe") pour l'état actuel et les axes d'amélioration vérifiés: Observabilité, Performance, Sécurité, CI/CD, Packaging, API/SDK, Robot, Dashboard/UX, Vision/Audio/IA, Docs, Qualité, Communauté.

## 🚀 Démarrage Rapide

### Voir le robot en 3D (Simulation MuJoCo)

**BBIA-SIM simule le robot Reachy Mini avec une fidélité élevée** dans MuJoCo. Voici comment démarrer :

```bash
# 🎯 Démos principales

# Démo continue - Robot qui bouge en continu (RECOMMANDÉE)
mjpython examples/demo_mujoco_continue.py

# Démo émotions BBIA avec viewer MuJoCo
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --joint yaw_body

# Démo SDK officiel (RECOMMANDÉ)
python examples/reachy_mini/minimal_demo.py
# ⚠️ Note: surprise_3d_mujoco_viewer.py et demo_reachy_mini_corrigee.py sont obsolètes
```

### 🎮 Interface Live Interactive (Bonus)

**Note** : Cette interface est un **gadget visuel** pour explorer l'identité de BBIA. La **vraie simulation** se fait avec MuJoCo (voir ci-dessus).

> 🚀 **Accès direct** : [`http://localhost:8000/bbia-interface`](http://localhost:8000/bbia-interface)  
> 💡 *Lancez d'abord* : `python -m uvicorn bbia_sim.daemon.app.main:app --host 127.0.0.1 --port 8000`

**Fonctionnalités** :
- 🎭 **6 modes d'émotion** : Neutre, Joyeux, Curieux, Écoute, Veille, Erreur
- ✨ **Animations fluides** : Respiration automatique, clignement des yeux
- 🎨 **HUD Cyber-HUD** : Interface graphique rotative style futuriste
- 🎮 **Contrôle interactif** : Changez l'émotion de BBIA en temps réel
- 🗣️ **Interface vocale** : BBIA communique avec vous

<div align="center">

<img src="assets/logos/bbia/identity/bbia-speaking-512.svg" alt="BBIA Speaking" width="200" height="auto">

</div>

### ⚠️ Règles de sécurité

- **✅ TOUJOURS utiliser `yaw_body`** pour les animations visibles
- **✅ Amplitude maximum** : 0.3 rad (clamp automatique)
- **✅ Mode headless** : Stable et rapide pour les tests
- **✅ Antennes animables** : `left_antenna`, `right_antenna` avec limites de sécurité (-0.3 à 0.3 rad)
- **⚠️ Joints Stewart** : `stewart_1` à `stewart_6` ne peuvent PAS être contrôlés individuellement (utiliser `goto_target()` ou `look_at_world()` avec cinématique inverse)
- **❌ JAMAIS animer** : `passive_*` (joints passifs bloqués)
- **❌ JAMAIS dépasser** : 0.3 rad d'amplitude pour antennes

### 🌐 API Publique

```bash
# Démarrage de l'API publique
python deploy/public_api.py --dev

# Test de l'API publique
python deploy/public_api.py --check

# Démonstration complète
python scripts/demo_public_api.py

# Documentation interactive
# Swagger UI: http://localhost:8000/docs
# ReDoc: http://localhost:8000/redoc
# OpenAPI: http://localhost:8000/openapi.json
```

### 📡 Endpoints REST Conformes SDK Officiel

> **Note** : La conformité des endpoints REST est validée à 100% pour les méthodes SDK officielles. Les endpoints BBIA étendus sont en plus.

- **`/api/move/*`** - Mouvements conformes SDK (goto, running, stop, play, set_target, ws/updates)
- **`/api/motors/*`** - Contrôle moteurs (status, set_mode)
- **`/api/daemon/*`** - Contrôle daemon (start, stop, restart, status)
- **`/api/motion/*`** - Mouvements BBIA (goto_pose, wake_up, goto_sleep, joints, etc.)
- **`/api/state/*`** - État robot (full avec 11 params, present_head_pose avec use_pose_matrix, ws/full)
- **`/api/kinematics/*`** - Cinématique (info, urdf, stl)
- **`/api/apps/*`** - Gestion applications HuggingFace (install, start, stop, status)

### 🤖 Backends Disponibles

```bash
# Backend MuJoCo (simulation)
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('mujoco')"

# Backend Reachy-Mini SDK Officiel (100% conforme)
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('reachy_mini')"

# Backend Reachy Mock (ancien)
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('reachy')"

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
python -c "from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior; import logging; ab = BBIAAdaptiveBehavior(); ab.set_context('greeting'); ab.set_emotion_state('happy', 0.8); logging.info(ab.generate_behavior('test'))"

# Module Hugging Face (nécessite: pip install transformers torch)
python -c "from bbia_sim.bbia_huggingface import BBIAHuggingFace; import logging; hf = BBIAHuggingFace(); logging.info(hf.analyze_sentiment('Je suis heureux!'))"

# Chat intelligent BBIA
python -c "from bbia_sim.bbia_huggingface import BBIAHuggingFace; import logging; hf = BBIAHuggingFace(); logging.info(hf.chat('Bonjour'))"

# Module Emotion Recognition (nécessite: pip install mediapipe transformers)
python -c "from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition; import logging; er = BBIAEmotionRecognition(); logging.info(er.analyze_vocal_emotion('Je suis excité!'))"
```

### ⚙️ Variables d’environnement utiles

```bash
# Exécutions headless/CI (désactive audio matériel)
export BBIA_DISABLE_AUDIO=1

# Sélecteurs IA (optionnels, imports paresseux)
export BBIA_TTS_BACKEND=pyttsx3      # ou kitten/kokoro/neutts si installés
export BBIA_STT_BACKEND=whisper      # ou parakeet
export BBIA_LLM_BACKEND=llama.cpp    # ou qwen
```

### 🎯 Vertical Slices BBIA

```bash
# Exemple de test e2e (motion roundtrip)
python -m pytest tests/e2e/test_motion_roundtrip.py -v

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

### 📹 Vidéos de Démonstration

**Nouvelles vidéos disponibles** (Oct / Nov. 2025) : 3 captures vidéo de la simulation MuJoCo

- `assets/videos/Enregistrement de l'écran Oct / Nov. 2025 à 17.00.52.mov` (2.7 MB)
- `assets/videos/Enregistrement de l'écran Oct / Nov. 2025 à 17.01.27.mov` (3.4 MB)
- `assets/videos/Enregistrement de l'écran Oct / Nov. 2025 à 17.01.56.mov` (3.1 MB)

**Source** : Toutes ces vidéos ont été enregistrées depuis `examples/demo_mujoco_continue.py` qui montre le robot en mouvement continu (tête + corps).

**Captures d'écran dashboard** (Nov 2025) : 4 captures du dashboard BBIA Advanced Dashboard disponibles dans `assets/images/` montrant l'interface complète avec métriques temps réel, contrôles joints, vision et chat.

**Inventaire complet** : Voir [`assets/MEDIAS_INVENTAIRE.md`](assets/MEDIAS_INVENTAIRE.md)
**Roadmap vidéos** : Voir [`docs/guides/DEMO_VIDEO_ROADMAP.md`](docs/guides/DEMO_VIDEO_ROADMAP.md)

### 📋 Documentation

**Documentation interactive HTML** (recommandé)

```bash
# Générer la documentation HTML complète avec navigation
scripts/docs/build_docs_html.sh
# Ouvrir artifacts/docs_html/index.html dans votre navigateur
```

La documentation HTML offre :

- Navigation latérale avec table des matières
- Rendu Mermaid automatique des diagrammes
- Thème sombre adapté au développement
- Liens ancres pour navigation rapide

**Guides principaux :**

- **[Guide de Démarrage](docs/guides/GUIDE_DEMARRAGE.md)** : Guide pour démarrer
- **[Guide Avancé](docs/guides/GUIDE_AVANCE.md)** : Guide avancé
- **[Vertical Slices](docs/quality/audits/VERTICAL_SLICES_ACCOMPLIS.md)** : Documentation des vertical slices
- **[Index thématique (par profils)](docs/INDEX_FINAL.md)**
- **[FAQ Troubleshooting](docs/development/troubleshooting.md)**
- **[Observabilité (logs/metrics/santé)](docs/observabilite/OBSERVABILITE.md)**
- **[Pipeline CI/CD](docs/deployment/PIPELINE_CI.md)**
- **[Contrats REST/WS](docs/development/api/CONTRATS_REST_WS.md)**
- **[Sécurité robot DO/DON'T](docs/hardware/SECURITE_ROBOT.md)**
- **[Datasets & Golden Images](docs/ai/datasets.md)**
- **[Roadmap Dashboard/UX](docs/dashboard/ROADMAP_DASHBOARD.md)**
- **[Guide Contribution](docs/community/CONTRIBUTION_GUIDE.md)**

### Note importante sur les joints

- **Antennes** : Animables avec limites de sécurité (-0.3 à 0.3 rad). Utilisez `yaw_body` pour animations principales, antennes pour expressivité fine.
- **Joints Stewart** : Les 6 joints de la plateforme Stewart (`stewart_1` à `stewart_6`) ne peuvent PAS être contrôlés individuellement car ils utilisent la cinématique inverse (IK). Utilisez `goto_target(head=pose)` ou `look_at_world(x, y, z)` pour contrôler la tête.
- **Joints passifs** : Les 7 joints passifs (`passive_1` à `passive_7`) sont bloqués et ne peuvent pas être animés.

## 📁 Structure du Projet

```text
src/bbia_sim/
├── sim/
│   ├── models/
│   │   ├── reachy_mini_REAL_OFFICIAL.xml    # Modèle officiel principal
│   │   └── reachy_mini.xml                   # Modèle de base (backup)
│   └── assets/reachy_official/              # 41 fichiers STL officiels
│       ├── body_down_3dprint.stl
│       ├── body_top_3dprint.stl
│       ├── head_front_3dprint.stl
│       ├── head_back_3dprint.stl
│       ├── stewart_*.stl                     # Bras Stewart
│       └── ... (36 autres STL)
└── scripts/
    ├── launch_complete_robot.py         # Lanceur principal (dans scripts/)
    └── download_ALL_stl.py              # Script de téléchargement STL (dans scripts/)
```

### 🚀 Guide rapide – Commandes essentielles

#### 1) Activer les bons environnements

- venv principal (simulation/hardware, dashboard, chat):

```bash
source venv/bin/activate
```

- venv vision (MediaPipe/YOLO/OpenCV):

```bash
source venv-vision-py310/bin/activate
```

#### 2) Démo 3D immédiate (macOS)

```bash
# venv principal
source venv/bin/activate
./examples/LANCE_DEMO_3D.sh
```

#### 3) Dashboard + chat en temps réel

```bash
# venv principal
source venv/bin/activate
python src/bbia_sim/dashboard_advanced.py --port 8000
# Ouvrir http://127.0.0.1:8000 dans le navigateur
```

#### 4) Activer l’intelligence (LLM) une fois (internet requis)

```bash
# venv principal
source venv/bin/activate
python - <<'PY'
from bbia_sim.bbia_huggingface import BBIAHuggingFace
bbia = BBIAHuggingFace()
ok = bbia.enable_llm_chat()
logging.info('LLM activé' if ok else 'LLM non chargé (fallback réponses enrichies)')
PY
```

#### 5) Test vision rapide (MediaPipe/OpenCV)

```bash
# venv vision
source venv-vision-py310/bin/activate
python - <<'PY'
import mediapipe, cv2
logging.info('VISION OK')
PY
```

#### 5bis) Test webcam USB (Logitech MX Brio)

```bash
# venv vision
source venv-vision-py310/bin/activate

# Test simple (preview)
export BBIA_CAMERA_INDEX=0  # défaut: 0
python scripts/test_webcam_simple.py
# Appuie sur 'q' pour quitter, 's' pour sauvegarder

# Test vision complète (YOLO + MediaPipe)
python scripts/test_vision_webcam.py
# Détection objets et visages en temps réel
```

**Note** : Au premier lancement, macOS demande l'autorisation caméra. Autoriser dans Réglages Système > Confidentialité > Caméra.

#### 5ter) DeepFace - Reconnaissance visage personnalisée (optionnel)

```bash
# venv vision
source venv-vision-py310/bin/activate

# Installer DeepFace (une fois)
pip install -r requirements/requirements-deepface.txt

# Enregistrer une personne (famille, ami)
python scripts/test_deepface.py --register photo_alice.jpg --name Alice

# Reconnaître une personne dans une image
python scripts/test_deepface.py --recognize frame.jpg

# Détecter émotion
python scripts/test_deepface.py --emotion photo.jpg
```

**Note** : DeepFace permet à BBIA de reconnaître des personnes spécifiques et leurs émotions. Compatible SDK Reachy Mini ✅

#### 5quater) MediaPipe Pose - Détection postures/gestes (optionnel)

```bash
# venv vision (MediaPipe déjà installé)
source venv-vision-py310/bin/activate

# Test avec webcam (temps réel)
python scripts/test_pose_detection.py --webcam

# Test avec image
python scripts/test_pose_detection.py --image photo.jpg
```

**Note** : MediaPipe Pose détecte la posture complète (33 points clés) et les gestes (bras levés, debout/assis). Déjà installé via MediaPipe ✅

#### 6) Télémétrie WebSocket (5 messages)

```bash
# venv principal
source venv/bin/activate
python examples/subscribe_telemetry.py --count 5
```

#### 7) Démo émotions (simulation)

```bash
# venv principal pour logique + viewer (macOS: mjpython conseillé)
source venv/bin/activate
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --backend mujoco
```

Pour les détails et recommandations matériel (webcam USB UVC) et les profils d'environnements, voir `docs/development/ENV_PROFILS.md`.

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
python src/bbia_sim/dashboard_advanced.py --port 8000
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
# Test hardware complet avec artefacts (Reachy Mini)
python scripts/hardware_dry_run_reachy_mini.py --duration 10 --output-dir artifacts

# Test avec backend simulation (ancien script)
python scripts/hardware_dry_run.py --duration 5 --backend mujoco
```

### Téléchargement des STL officiels

```bash
python scripts/download_ALL_stl.py
```

Télécharge automatiquement les 41 fichiers STL depuis le repo officiel.

### Gestion des processus

```bash
python scripts/process_manager.py --cleanup
```

Nettoie les processus gourmands tout en préservant les applications importantes.

## 🔧 Résolution de problèmes

### Problème : Robot en pièces détachées

**Cause** : Fichiers STL corrompus (pointeurs Git LFS)
**Solution** :

```bash
python scripts/download_ALL_stl.py
```

### Problème : Erreur "No such file or directory"

**Cause** : Chemin des assets incorrect
**Solution** : Le modèle `reachy_mini_REAL_OFFICIAL.xml` a le bon chemin `meshdir="../assets/reachy_official"`

### Problème : la fenêtre 3D ne s'ouvre pas

**Cause** : Mauvaise commande Python
**Solution** : Utiliser `mjpython` au lieu de `python` sur macOS

## 📊 Spécifications Techniques

- **Modèle** : Reachy Mini Wireless
- **Simulateur** : MuJoCo
- **Format** : MJCF (MuJoCo XML)
- **Assets** : 41 fichiers STL officiels
- **Articulations** : 16 au total (1 yaw_body + 6 stewart + 2 antennes + 7 passifs)
- **Dimensions** : Fidèles aux spécifications officielles

## 🔗 Ressources Officielles

- **Repo officiel** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)
- **Documentation** : [docs.pollen-robotics.com](https://docs.pollen-robotics.com)
- **Modèle 3D** : OnShape CAD officiel
- **Assets STL** : Téléchargés depuis le repo officiel

## 🧪 Tests et Coverage

### 📊 **Couverture de Code - Clarification Importante**

**Il existe DEUX mesures de coverage selon le périmètre analysé :**

#### 1. **Coverage Modules Core** (~50%) ✅ **MESURE PERTINENTE**

```bash
# Mesure sur src/bbia_sim/* (modules métier testés)
python -m pytest --cov=src/bbia_sim --cov-report=term
```

- **Périmètre** : Modules core (`robot_api`, `bbia_*`, `backends/`)
- **Résultat** : **~50% coverage** sur modules métier
- **Statut** : ✅ **Niveau correct** pour projet open source
- **C'est cette mesure qui compte** pour la qualité du code métier

#### 2. **Coverage Global** (~6-7%) ⚠️ **MESURE CONTEXTUELLE**

```bash
# Si on mesure TOUT le projet (exemples, scripts, etc.)
# Résultat : ~6-7% car beaucoup de fichiers ne sont pas testés unitairement
```

- **Périmètre** : Tout `src/` + `examples/` + `scripts/` (fichiers de démo)
- **Résultat** : **~6-7% coverage global**
- **Statut** : ⚠️ **Normal et attendu** (exemples/scripts ne sont pas testés unitairement)

**📝 Note** : La mesure globale inclut des fichiers qui ne sont **pas censés** être testés unitairement (exemples, scripts de démo, etc.). C'est pourquoi le coverage global est faible, mais c'est **normal**.

**🎯 Recommandation** : Utiliser **coverage modules core** comme mesure principale de qualité.

---

### Tests & Commandes

Recommandation (locale, mesure complète non bloquante) :

```bash
python -m pytest \
  --cov=src/bbia_sim \
  --cov-report=term-missing \
  --cov-report=xml:coverage.xml \
  --cov-report=html:htmlcov \
  --cov-fail-under=0
```

CI (seuil progressif initial 30%) :

```bash
pytest -q --disable-warnings \
  --cov=src/bbia_sim \
  --cov-report=xml:coverage.xml \
  --cov-fail-under=30
```

### Résultats des Tests

**📊 Qualité & Conformité :**

- ✅ **Conformité SDK officiel** : 100% validée (47 tests de conformité complète) — [📊 Vérification 26 Novembre 2025](docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_26NOV2025.md)
- ✅ **Coverage global** : **68.86%** (excellent) — [📊 Rapport Codecov](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim) | [📁 Rapport HTML local](htmlcov/index.html)
- ✅ **Coverage modules core** : **~50%** (mesure pertinente) — [📊 Rapport Codecov](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim) | [📁 Rapport HTML local](htmlcov/index.html)
- ✅ **Tests totaux** : **1,743 tests collectés** (1,805 total, 62 deselected) — [📊 Détails CI](https://github.com/arkalia-luna-system/bbia-sim/actions) | [✅ Vérification 26 Novembre 2025](docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_26NOV2025.md)
- ✅ **Fichiers Python** : **68 modules** dans `src/bbia_sim/` — [📁 Source](src/bbia_sim)
- ✅ **Qualité code** : Black, Ruff, MyPy, Bandit ✅
- ✅ **Tests** : Suite complète + SDK officiel

> **Pour plus de détails** :
> - 📊 [Rapport Codecov en ligne](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim)
> - 📁 Rapport HTML local : `python -m pytest --cov=src/bbia_sim --cov-report=html && open htmlcov/index.html`

### Commandes de Tests

```bash
# Tests avec visibilité maximale (identifie les tests lents)
./scripts/test_verbose.sh

# Tests MuJoCo existants
python -m pytest tests/test_simulation_integration.py -v

# Lancer tous les tests avec coverage (chemin projet standardisé)
python -m pytest tests/ --cov=src/bbia_sim --cov-report=term-missing --cov-report=html

# Lancer les tests sans s'arrêter aux échecs
python -m pytest tests/ --cov=src/bbia_sim --cov-fail-under=0 --tb=no -q

# Lancer un test spécifique
python -m pytest tests/test_bbia_emotions.py -v

# Voir le rapport HTML de coverage
open htmlcov/index.html
```

## Statut du projet

Le robot Reachy Mini est simulé avec une fidélité élevée aux spécifications officielles. La suite de tests fournit une couverture suivie et des mesures de performance documentées (dry-run matériel et démonstrations sur robot réel).

---

## 📊 Projets & Portfolio

---

<div align="center">

<img src="assets/logos/bbia/identity/bbia-speaking-512.svg" alt="BBIA Speaking" width="300" height="auto">

</div>

---

*Dernière mise à jour : 7 Décembre 2025*
