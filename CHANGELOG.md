# 📋 CHANGELOG

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.1] - Octobre 2025

### 🔧 Corrigé

- **Formatage Code** : Correction de tous les espaces dans lignes vides et espaces en fin de ligne (ruff)
- **Typage MyPy** : Correction de 6 erreurs de typage dans `reachy_mini_backend.py`
- **Tests TypeError** : Correction des erreurs "NoneType object is not callable" dans les tests
- **Imports** : Suppression des imports inutilisés détectés par ruff
- **SDK Compatibility** : Gestion correcte du mode simulation sans SDK reachy_mini

### 🛠️ Amélioré

- **Type Safety** : Ajout d'annotations de type explicites pour conformité mypy
- **Error Handling** : Amélioration de la gestion d'erreurs avec SDK non disponible
- **Return Types** : Correction des types de retour pour éviter les erreurs Any

### 📊 Qualité

- **Ruff** : All checks passed ✅
- **MyPy** : Success, no issues found ✅  
- **Bandit** : 0 security issues ✅
- **Black** : All files formatted ✅
- **Tests** : 38 passed, 2 skipped ✅

## [1.2.0] - Octobre 2025

### 🚀 Ajouté
- **IA Légère Activée** : Intégration Whisper STT + YOLOv8n + MediaPipe Face Detection (OFF par défaut, activation via flags)
- **Dashboard Web Minimal** : Interface FastAPI + WebSocket pour contrôle temps réel
- **Scripts One-Click** : `run_demo_sim.sh` et `run_demo_real.sh` pour démos simplifiées
- **Tests de Sécurité** : Nouveau module `test_safety_parameters.py` avec 6 tests
- **Artefacts Automatiques** : Génération CSV/log/JSON pour CI et diagnostics
- **Golden Tests** : Système de validation non-régression avec traces de référence

### 🔧 Modifié
- **Vitesses Sûres** : Fréquence par défaut 0.1 Hz (au lieu de 0.5 Hz) pour sécurité robot
- **Amplitudes Sûres** : Amplitude par défaut 0.2 rad (au lieu de 0.3 rad) pour protection
- **Mapping Centralisé** : `mapping_reachy.py` comme source de vérité unique
- **CI/CD Renforcée** : Upload automatique d'artefacts, déterminisme SEED=42
- **Documentation Synchronisée** : README, ROADMAP, PORTFOLIO mis à jour avec chiffres réels

### 🛡️ Sécurité
- **Clamp Automatique** : Limite globale 0.3 rad respectée dans tous les scripts
- **Joints Interdits** : Validation centralisée des articulations autorisées
- **Tests Hardware** : `hardware_dry_run.py` avec validation latence <40ms
- **Paramètres Sûrs** : 6 scripts corrigés avec paramètres sécurisés par défaut

### 📊 Métriques
- **Tests** : 581 collectés, 453 passent (78% de réussite)
- **Coverage** : 63.37% de couverture de code
- **IA Modules** : 18 tests passent, 2 skippés (MediaPipe matplotlib)
- **Latence** : Moyenne 0.02ms, max 4.77ms (cible <40ms ✅)

### 🔄 Changements Techniques
- **RobotAPI Unifiée** : Interface abstraite simulation ↔ robot réel
- **Modules IA Optionnels** : Activation via flags `--enable-yolo`, `--enable-face`
- **Artefacts CI** : Upload automatique JSONL/CSV en cas d'échec
- **Déterminisme** : SEED fixé pour reproductibilité des tests

---

## [1.1.1] - Octobre 2025

### 🔧 Modifié
- **RobotAPI** : Interface unifiée pour simulation et robot réel
- **Golden Tests** : Système de validation non-régression
- **CI/CD** : Pipeline GitHub Actions avec artefacts

### 🛡️ Sécurité
- **Limites Centralisées** : Clamp automatique à 0.3 rad
- **Joints Interdits** : Validation des articulations autorisées

---

## [1.1.0] - Octobre 2025

### 🚀 Ajouté
- **Simulation Complète** : Robot Reachy Mini parfaitement assemblé
- **Modules BBIA** : Émotions, vision, voix, comportements intégrés
- **API REST** : FastAPI avec endpoints complets
- **WebSocket** : Communication temps réel

### 🔧 Modifié
- **Architecture** : Refactoring complet pour modularité
- **Tests** : Couverture étendue avec tests d'intégration

---

## [1.0.0] - Octobre 2025

### 🚀 Première Release
- **Simulation MuJoCo** : Robot Reachy Mini de base
- **Modules BBIA** : Émotions, vision, voix, comportements
- **API Basique** : Endpoints REST fondamentaux
- **Tests Unitaires** : Couverture de base
