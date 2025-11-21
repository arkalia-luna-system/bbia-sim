# 🚀 Guide Avancé - BBIA-SIM

<div align="center">

**Date** : 21 novembre 2025
[![Version](https://img.shields.io/badge/version-1.3.2-blue.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![Niveau](https://img.shields.io/badge/niveau-Avancé-orange.svg)](docs/guides/)
[![Temps](https://img.shields.io/badge/temps-15%20min-purple.svg)](#architecture-détaillée)

</div>

> **Liens utiles** : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md) · [`docs/reference/project-status.md`](../reference/project-status.md)

> **Compatibilité Python et CI**
>
> - **Python** : 3.11+
> - **CI** : `.github/workflows/ci.yml`
> - **Setup rapide** :
> ```bash
> pyenv install 3.11.9 && pyenv local 3.11.9
> python -m pip install --upgrade pip
> pip install -e .
> ```

## 🎯 Pour Développeurs Avancés

<div align="center">

**Version :** 1.3.2  
**Public cible :** développeurs expérimentés souhaitant exploiter les fonctionnalités de BBIA

[![🔧 Backend](https://img.shields.io/badge/🔧-Backend%20Unifié-blue)](#backend-unifié)
[![🧠 Modules](https://img.shields.io/badge/🧠-Modules%20BBIA-green)](#modules-bbia-avancés)
[![🚀 API](https://img.shields.io/badge/🚀-API%20Avancée-purple)](#api-avancée)
[![🧪 Tests](https://img.shields.io/badge/🧪-Tests%20Avancés-orange)](#tests-avancés)

</div>

---

## 📋 Table des Matières

- [Guide avancé - BBIA-SIM](#guide-avancé---bbia-sim)
  - [Pour développeurs avancés](#pour-développeurs-avancés)
  - [📋 Table des Matières](#-table-des-matières)
  - [Architecture détaillée](#architecture-détaillée)
    - [Backend unifié](#backend-unifié)
      - [Flux Backend Unifié](#flux-backend-unifié)
    - [Modules BBIA avancés](#modules-bbia-avancés)
      - [Architecture Modules BBIA](#architecture-modules-bbia)
      - [1. Module Émotions](#1-module-émotions)
      - [2. Module Comportements](#2-module-comportements)
      - [3. Module Vision](#3-module-vision)
    - [Tests avancés](#tests-avancés)
    - [Optimisations de performance](#optimisations-de-performance)
  - [Migration simulation → robot réel](#migration-simulation--robot-réel)
  - [API avancée](#api-avancée)
    - [Conformité SDK officiel](#conformité-sdk-officiel)
  - [🎯 Navigation](#-navigation)

---

## 🏗️ Architecture Détaillée

### 🔧 Backend Unifié

<div align="center">

### 🌟 Développez une fois, déployez partout !

[![🔄 Unifié](https://img.shields.io/badge/🔄-Simulation%20↔%20Robot-blue)](#flux-backend-unifié)
[![⚡ Performance](https://img.shields.io/badge/⚡-Performance%20Optimisée-green)](#optimisations-de-performance)

</div>

Le backend unifié permet de développer et tester en simulation, puis de basculer vers le robot physique avec le même code.

```python
# Backend unifié : Simulation ↔ Robot réel
from bbia_sim.robot_factory import RobotFactory

# Simulation (développement et tests)
robot_sim = RobotFactory.create_backend('mujoco')
robot_sim.connect()

# Robot réel (production)
robot_real = RobotFactory.create_backend('reachy_mini')
robot_real.connect()

# Même code fonctionne sur les deux
robot_sim.set_emotion('happy', 0.8)
robot_real.set_emotion('happy', 0.8)

# Basculer facilement
backend = 'mujoco' if os.getenv('DEV') else 'reachy_mini'
robot = RobotFactory.create_backend(backend)

```

#### Flux Backend Unifié

```mermaid
flowchart TB
    CODE[Code Unifié<br/>RobotAPI] --> FACTORY[RobotFactory<br/>Factory Pattern]
    
    FACTORY --> CHECK{Environnement?}
    
    CHECK -->|DEV/TEST| MUJOCO[Backend MuJoCo<br/>Simulation 3D]
    CHECK -->|PROD| REACHY[Backend Reachy Mini<br/>Robot Physique]
    
    MUJOCO --> MODEL[Modèle XML<br/>Physique Réaliste]
    REACHY --> SDK[SDK Officiel<br/>Pollen Robotics]
    
    MODEL --> ACTIONS[Actions Robot<br/>Même Interface]
    SDK --> ACTIONS
    
    ACTIONS --> EMOTIONS[12 Émotions<br/>Contrôlables]
    ACTIONS --> MOVEMENT[Mouvements<br/>Danses/Animations]
    
    style CODE fill:#90EE90
    style FACTORY fill:#FFD700
    style ACTIONS fill:#87CEEB

```

**Avantages :**

- Développement sans matériel
- Tests automatisés
- Debugging facilité
- Migration transparente vers robot réel

### 🧠 Modules BBIA Avancés

#### 🏛️ Architecture Modules BBIA

<div align="center">

### 🤖 12 modules intelligents pour robot avancé

[![🧠 IA](https://img.shields.io/badge/🧠-Intelligence%20Artificielle-purple)](#1-module-émotions)
[![👁️ Vision](https://img.shields.io/badge/👁️-Vision%20Avancée-blue)](#3-module-vision)
[![🎤 Audio](https://img.shields.io/badge/🎤-Audio%20Intelligent-green)](#2-module-comportements)

</div>

```mermaid
graph LR
    subgraph "Modules BBIA"
        EMOTIONS[BBIAEmotions<br/>12 émotions]
        VISION[BBIAVision<br/>YOLO + MediaPipe + SmolVLM2]
        VOICE[BBIAVoice<br/>Whisper + TTS]
        BEHAVIOR[BBIABehavior<br/>Comportements]
        HF[BBIAHuggingFace<br/>LLM + NLP]
        MEMORY[BBIAMemory<br/>Contexte]
        TOOLS[BBIATools<br/>8 outils LLM]
    end
    
    subgraph "Intégration"
        ROBOT[RobotAPI<br/>Unifié]
    end
    
    EMOTIONS --> ROBOT
    VISION --> ROBOT
    VOICE --> ROBOT
    BEHAVIOR --> HF
    BEHAVIOR --> MEMORY
    HF --> TOOLS
    TOOLS --> ROBOT
    
    style HF fill:#90EE90
    style TOOLS fill:#FFD700
    style ROBOT fill:#87CEEB

```

#### 1. Module Émotions

```python
from bbia_sim.bbia_emotions import BBIAEmotions

emotions = BBIAEmotions()

# Toutes les émotions disponibles
emotions.set_emotion('excited', intensity=0.9)
emotions.set_emotion('curious', intensity=0.6)
emotions.set_emotion('calm', intensity=0.4)

```

**Flux Émotions** :

```mermaid
flowchart LR
    INPUT[Input Utilisateur] --> SENTIMENT[Analyse Sentiment<br/>RoBERTa]
    SENTIMENT --> EMOTION[Émotion Associée<br/>12 disponibles]
    EMOTION --> ROBOT[RobotAPI<br/>Contrôle Articulations]
    ROBOT --> DISPLAY[Affichage Robot]
    
    style EMOTION fill:#90EE90
    style ROBOT fill:#87CEEB

```

#### 2. Module Comportements

```python
from bbia_sim.bbia_behavior import BBIABehaviorManager

manager = BBIABehaviorManager()

# Créer comportement personnalisé
class MyBehavior(BBIABehavior):
    def execute(self, context):
        # Votre logique ici
        pass

manager.register_behavior(MyBehavior())

```

#### 3. Module Vision

```python
from bbia_sim.bbia_vision import BBIAVision

vision = BBIAVision()
objects = vision.scan_environment()
faces = vision.detect_faces()

```

### Tests avancés

```bash
# Lancer tous les tests
pytest tests/ -v --tb=short

# Tests avec coverage
pytest tests/ --cov=src --cov-report=html

# Voir rapport coverage
open htmlcov/index.html

```

### Optimisations de performance

```python
# Mode headless (rapide)
MUJOCO_GL=disable python your_script.py

# Activer cache simulation
bbia_sim.sim.enable_caching()

```

---

## 🔄 Migration Simulation → Robot Réel

<div align="center">

### 🚀 Passage transparent du développement à la production

[![📖 Guide](https://img.shields.io/badge/📖-Guide%20Complet-blue)](../development/migration.md)
[![✅ Validé](https://img.shields.io/badge/✅-100%25%20Conforme-green)](../quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md)

</div>

Voir : [MIGRATION_GUIDE.md](../development/migration.md)

---

## 🚀 API Avancée

### ✅ Conformité SDK Officiel

<div align="center">

### 🎯 100% compatible avec le SDK Pollen Robotics

[![🔧 SDK](https://img.shields.io/badge/🔧-SDK%20Officiel-blue)](../quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md)
[![📊 Méthodes](https://img.shields.io/badge/📊-Toutes%20Méthodes-green)](#-toutes-les-méthodes-sdk-officiel-disponibles)

</div>

```python
# Toutes les méthodes SDK officiel disponibles
robot.get_current_head_pose()
robot.get_current_joint_positions()
robot.look_at_world(x, y, z)
robot.goto_target(head=pose)
robot.start_recording()
robot.stop_recording()

```

Voir : [Conformité SDK Complète](../quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md)

---

**Pour plus d'infos :** Toute la documentation dans `docs/` 📚

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Guide Débutant](GUIDE_DEMARRAGE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)
