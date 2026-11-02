# Index — Documentation BBIA-SIM

> **Documentation du moteur cognitif / simulation BBIA-SIM**  
> *Organisée par usage • Navigation intuitive*

**Cadre** : **sim-first** — sans `BBIA_ALLOW_HARDWARE=1`, pas de matériel. SDK Pollen **1.13** (alignement app-layer) · hardware **≤7** · pas de claim « robot 100 % validé ». Voir [`reference/project-status.md`](reference/project-status.md).

---

## Démarrage Rapide

> **Nouveau sur le projet ?** Commencez ici (simulation).

| Action | Commande | Temps | Lien |
|--------|----------|-------|------|
| **Installation** | `pip install -e .[dev]` | ~2 min | [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md) |
| **Dashboard** | `python src/bbia_sim/dashboard_advanced.py` | Instant | [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md) |
| **Première sim** | Voir guide | 5 min | [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md) ⭐ |
| **Robot réel** | Opt-in + checklist | — | [Premier jour robot](hardware/PREMIER_JOUR_ROBOT.md) |

**Version** : v1.4.0 • **Dernière mise à jour** : 2 Novembre 2026

> **💡 Départ** : [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md)

---

## 🎯 Navigation par Profil

### 🟢 Nouveau Utilisateur

1. [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md) — Premiers pas en 5 minutes
2. [Installation Audio](installation/AUDIO_SETUP.md) — Configuration audio
3. [Guide Reachy Mini](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md) — Utiliser le robot

### 💻 Développeur

1. [Guide Technique](guides/GUIDE_AVANCE.md) — Fonctionnalités techniques
2. [Architecture Vue d'Ensemble](development/architecture/ARCHITECTURE_OVERVIEW.md) — Structure du projet
3. [Guide d'Intégration](development/integration.md) — Intégrer BBIA-SIM
4. [Guide de Test](development/testing.md) — Tests et qualité

### 🤖 Utilisateur Robotique

1. [Guide Reachy Mini](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md) — Guide
2. [Mouvements Reachy Mini](hardware/MOUVEMENTS_REACHY_MINI.md) — Contrôle des mouvements
3. [Sécurité Robot](hardware/SECURITE_ROBOT.md) — DO/DON'T
4. [Simulation MuJoCo](simulations/MUJOCO_SIMULATION_GUIDE.md) — Simulation 3D

### ✅ QA / Tests

1. [Guide de Test](development/testing.md) — Guide
2. [Index Audits](quality/INDEX_AUDITS.md) — Actifs + archives historiques
3. [État projet](reference/project-status.md) — Tableau de bord tip

### 📊 Chef de Projet

1. [Tableau de Bord Complet](reference/project-status.md) — État par axe (Fiabilité, Performance, etc.)
2. [Release Notes](reference/RELEASE_NOTES.md) — Notes de version
3. [Historique](reference/PROJECT_HISTORY.md) — Historique complet
4. [Métriques](reference/METRICS.md) — Métriques du projet

---

## 📖 Documentation par Catégorie

### 🚀 Démarrage & Guides

- [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md) — Installation et premiers pas
- [Guide Technique](guides/GUIDE_AVANCE.md) — Fonctionnalités techniques
- [Guide Chat BBIA](guides/GUIDE_CHAT_BBIA.md) — Système de chat
- [Guide Reachy Mini Wireless](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md) — Guide
- [FAQ Troubleshooting](development/troubleshooting.md) — Dépannage

### 🏗️ Architecture & Design

- [Architecture Vue d'Ensemble](development/architecture/ARCHITECTURE_OVERVIEW.md) — Vue générale
- [Architecture Détaillée](development/architecture/ARCHITECTURE_DETAILED.md) — Détails techniques
- [Guide Architecture](development/architecture/ARCHITECTURE.md) — Référence principale
- [Index Thématique](reference/INDEX_THEMATIQUE.md) — Navigation par profils

### ✅ Qualité & Conformité

- [Conformité Reachy-Mini](quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md) — Conformité SDK
- [Index Audits](quality/INDEX_AUDITS.md) — Actifs + [`_archived/quality/`](_archived/quality/)
- [Pipeline CI/CD](deployment/PIPELINE_CI.md) — Intégration continue

### 🧠 Intelligence & IA

- [Modules IA](ai/modules.md) — Audit modules pré-entraînés
- [Datasets & Golden Images](ai/datasets.md) — Datasets et traces
- [Intelligence LLM](ai/llm.md) — Intelligence conversationnelle
- [Analyse Voix](ai/voice.md) — Analyse voix et intelligence

### 🔧 Développement

- [Guide d'Intégration](development/integration.md) — Intégrer BBIA-SIM
- [Switch Sim → Robot](development/switch-sim-robot.md) — Basculer entre simulation et robot réel
- [Guide de Test](development/testing.md) — Tests et qualité
- [Setup Guides](development/setup/) — Configuration (environments, deepface, webcam, etc.)
- [API REST/WS](development/api/CONTRATS_REST_WS.md) — Contrats API

### 🤖 Hardware & Robotique

- [Mouvements Reachy Mini](hardware/MOUVEMENTS_REACHY_MINI.md) — Contrôle des mouvements
- [Sécurité Robot](hardware/SECURITE_ROBOT.md) — DO/DON'T

### ⚡ Performance

- [Watchdog](quality/performance/WATCHDOG_IMPLEMENTATION.md) — Implémentation watchdog
- [Optimisations (archivé déc. 2025)](_archived/quality/performance/OPTIMISATIONS_PERFORMANCE_DEC2025.md)

### 🎯 Simulation

- [Simulation BBIA](simulations/SIMULATION_BBIA_COMPLETE.md) — Simulation
- [Guide MuJoCo](simulations/MUJOCO_SIMULATION_GUIDE.md) — Simulation 3D
- [Observabilité](observabilite/OBSERVABILITE.md) — Logs, metrics, santé

### 📊 Qualité & Audits

- [Index Audits](quality/INDEX_AUDITS.md) — Audits et conformité
- [Guide Dashboard](dashboard/README.md) — Dashboard

### 📋 Références

- [Statut du Projet](reference/project-status.md) — État opérationnel et tableau de bord
- [Release Notes](reference/RELEASE_NOTES.md) — Notes de version
- [Historique](reference/PROJECT_HISTORY.md) — Historique

### 🎮 Unity & Jeux

- [Guide Unity BBIA](unity/UNITY_BBIA_GUIDE.md) — Intégration Unity
- [Dépannage Unity](unity/UNITY_TROUBLESHOOTING.md) — Dépannage Unity

---

## 🆘 Besoin d'Aide ?

1. **Nouveau sur le projet ?** → [Guide de Démarrage](guides/GUIDE_DEMARRAGE.md)
2. **Installer le projet ?** → [Installation Audio](installation/AUDIO_SETUP.md)
3. **Utiliser le robot ?** → [Guide Reachy Mini](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md)
4. **Développer ?** → [Guide Technique](guides/GUIDE_AVANCE.md)
5. **Tester ?** → [Guide de Test](development/testing.md)

---

## Points Clés (Fonctionnel Actuel)

- **Mode défaut** : MuJoCo / simulation — Reachy Mini matériel = `BBIA_ALLOW_HARDWARE=1`
- **Behaviors** : `wake_up`, `goto_sleep`, `nod` (sim + dry-run ; e2e hardware non revendiqué CI)
- **Sécurité** : watchdog + `emergency_stop` (app-layer)
- **Audio** : STT/TTS (SDK-first + fallback ; optionnel)
- **Vision** : YOLO/MediaPipe (optionnel)
- **Télémétrie** : SDK-first batterie/IMU avec fallback simulation (non bloquant)
- **Scores** : app-layer logiciels documentés · hardware Pollen **≤7**

### Flags Utiles

```bash
# Simulation/headless (désactive audio matériel)
export BBIA_DISABLE_AUDIO=1

# Robot physique (opt-in explicite)
export BBIA_ALLOW_HARDWARE=1

# Télémétrie SDK-first (avec robot, non bloquant si absent)
export BBIA_TELEMETRY_SDK=true
export BBIA_TELEMETRY_TIMEOUT=1.0
```

---

**Retour** : [README](README.md) • [Index Thématique](reference/INDEX_THEMATIQUE.md)

**Dernière mise à jour** : 2 Novembre 2026
