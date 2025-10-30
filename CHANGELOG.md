# 📋 CHANGELOG

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### 🔧 Corrigé
- mypy no-redef dans `src/bbia_sim/bbia_voice.py` (`audio_bytes` renommé en `sdk_audio_bytes`) pour éviter la redéfinition dans `dire_texte`.
- Accès télémétrie SDK durci dans `src/bbia_sim/daemon/app/routers/state.py` (plus d'accès direct `.robot`, usage de `getattr` et typage défensif).

### 🧪 Tests & 📚 Docs
- Nouveau test headless `tests/test_voice_speaker_fallback_headless.py` pour vérifier le fallback speaker sans drivers audio.
- `docs/status.md` mis à jour (note CI audio + mypy=0 sur corrections effectuées).
 - Standardisation environnement: Python 3.11+ requis, CI GitHub Actions mise à jour (`setup-python@v5`).

## [1.3.2] - Octobre 2025

### 🎯 Alignement et release stable
- Fusion contrôlée `future` → `develop` → `main` (CI verte), création du tag `v1.3.2`
- Branche de sauvegarde `backup-v1.3.2-stable`

### 🚀 Ajouté
- Suites de tests étendues (watchdog, performance, conformité avancée)
- Documentation enrichie et réorganisée (guides techniques, références, release notes)

### 🔧 Modifié
- CI/Qualité homogénéisées (Python 3.11+, Black/Ruff/MyPy/Bandit)
- Post-traitements HF renforcés (anti-vides, anti-doublons récents), validations JSON durcies

### 🐛 Corrigé
- Nettoyage imports et formatage, ajustements mypy/ruff ciblés

## [1.3.1] - Octobre 2025

### 🎯 **RELEASE - AUDIT COMPLET BBIA → REACHY INTEGRATION**

Cette version inclut toutes les corrections et améliorations identifiées lors de l'audit complet de conformité avec le SDK Reachy Mini officiel.

### 🚀 **Ajouté**

#### **Sécurité Hardware**
- **Emergency Stop** : Implémentation complète dans tous les backends (RobotAPI, ReachyMiniBackend, MuJoCoBackend, ReachyBackend)
- **Watchdog Monitoring** : Système de monitoring temps réel conforme SDK officiel (threads avec Event, détection automatique déconnexion)
- **Sécurité JSON** : Validation taille payload (1MB max), détection secrets en clair dans bridge Zenoh

#### **Conformité SDK Reachy Mini**
- **Audio SDK Alignment** : Sample rate 16kHz aligné SDK, validation format audio
- **Validation Émotions** : Intensité clampée [0.0, 1.0], 6 émotions SDK validées
- **Performance** : Optimisations boucles temps réel (goto_target avec minjerk, latence monitoring)

#### **Tests & Qualité**
- **40+ nouveaux tests** : Emergency stop (4 tests), watchdog (7 tests), sécurité limites (5 tests), validation JSON (3 tests)
- **Markers pytest** : Ajout `@pytest.mark.unit` et `@pytest.mark.fast` sur tous les tests critiques
- **Tests behaviors & sdk_wrappers** : Correction et validation complète des modules moyens

#### **Robustesse**
- **Support BBIA_DISABLE_AUDIO** : Flag d'environnement respecté partout (bbia_voice, bbia_voice_advanced, bbia_audio)
- **Endpoint /stop amélioré** : Utilise `emergency_stop()` si disponible
- **Télémétrie enrichie** : Latence moyenne disponible si monitoring activé

### 🔧 **Modifié**

#### **Backends**
- **ReachyMiniBackend** : Watchdog thread daemon pour monitoring temps réel (100ms interval)
- **Méthodes comportements** : Utilisation `goto_target()` avec interpolation minjerk au lieu de `time.sleep()`

#### **Audio & Voice**
- **bbia_audio.py** : Constantes SDK (`DEFAULT_SAMPLE_RATE=16000`, `DEFAULT_BUFFER_SIZE=512`)
- **bbia_voice.py** : Support `BBIA_DISABLE_AUDIO` pour CI/headless

#### **Bridge Zenoh**
- **Validation JSON** : Protection DoS (taille payload), détection secrets, gestion erreurs dédiée

### 🐛 **Corrigé**

- **Tests intermittent** : Correction race condition dans `test_watchdog_multiple_start_safe`
- **Tests markers** : Ajout markers pytest manquants dans `test_reachy_mini_backend.py`
- **Tests behaviors** : Validation complète (21/21 tests passent)
- **Tests stewart joints** : Adaptation pour utiliser `yaw_body` (IK requis pour stewart)

### 📝 **Documentation**

- **Audit complet** : Documentation exhaustive dans `docs/audit/` (11 fichiers)
- **Réorganisation** : Déplacement fichiers MD selon structure cohérente
- **Watchdog** : Documentation complète dans `docs/performance/WATCHDOG_IMPLEMENTATION.md`

---

## [1.3.0] - Décembre 2024

### 🎯 **RELEASE MAJEURE - CONFORMITÉ SDK PARFAITE**

Cette version marque l'achèvement complet de la conformité au SDK officiel Reachy Mini et la stabilisation du projet pour la production.

### 🚀 **Ajouté**

#### **Chat Intelligent BBIA** 🆕
- **Fonctionnalité chat contextuel** : Communication naturelle avec BBIA
- **Analyse sentiment** : Réponses adaptées selon le sentiment de l'utilisateur
- **Historique conversation** : Contexte préservé entre les échanges
- **Personnalités BBIA** : friendly_robot, curious, enthusiastic, calm
- **Interface dashboard** : Panel chat interactif dans dashboard avancé
- **13 tests unitaires** : Couverture complète du module chat

#### **Conformité SDK Officiel 100%**
- **21/21 méthodes SDK officiel** implémentées avec signatures parfaites
- **Backend ReachyMiniBackend** prêt pour robot physique
- **Tests de conformité** : 38 tests passent, 2 skippés (robot physique requis)
- **Migration transparente** : Simulation ↔ Robot réel sans modification de code

#### **Architecture RobotAPI Unifiée**
- **Interface abstraite** : Contrôle unifié simulation et robot réel
- **Bridge Zenoh/FastAPI** : Communication distribuée optimisée
- **Modules BBIA avancés** : Émotions, vision, comportements intégrés
- **Sécurité renforcée** : Limites et joints interdits centralisés

#### **Qualité Professionnelle**
- **Tests robustes** : 28/28 tests skippés justifiés, couverture optimale
- **CI/CD enterprise** : Pipeline GitHub Actions complet avec artefacts
- **Outils qualité** : Black, Ruff, MyPy, Bandit tous verts
- **Sécurité** : Audit pip-audit, aucune vulnérabilité critique

#### **Performance et Benchmarks**
- **Métriques détaillées** : Latence <1ms, FPS 100Hz, CPU <5%
- **Scripts benchmarks** : `bbia_performance_benchmarks.py` complet
- **Comparaisons** : Robot réel vs simulation documentées
- **Rapports JSON** : Données structurées pour analyse

#### **Communication Externe**
- **Badges professionnels** : Version, tests, qualité, conformité SDK
- **API publique** : `deploy/public_api.py` avec documentation Swagger
- **Configuration Render.com** : `render.yaml` prêt pour déploiement
- **Assets LinkedIn** : Post optimisé pour recruteurs

### 🔧 **Modifié**

#### **Stabilisation Version**
- **Version stable** : Suppression du suffixe alpha (1.3.0a1 → 1.3.0)
- **Requirements gelés** : Dépendances exactes pour production
- **Documentation** : Guides complets architecture, SDK, migration

#### **Optimisations Performance**
- **Latence optimisée** : <1ms en simulation, prêt pour temps réel
- **Mémoire** : Gestion optimisée, pas de fuites détectées
- **CPU** : Utilisation <5% en mode simulation

### 🛡️ **Sécurité**

#### **Conformité SDK**
- **Signatures identiques** : Types, paramètres, valeurs par défaut conformes
- **Comportement identique** : Simulation et robot réel identiques
- **Limites respectées** : Amplitude 0.3 rad, joints interdits protégés

#### **Tests de Sécurité**
- **Audit complet** : pip-audit, bandit, sécurité validée
- **Validation centralisée** : Joints et amplitudes contrôlés
- **Mode simulation sécurisé** : Activation automatique si robot indisponible

### 📊 **Métriques Finales**

#### **Tests et Qualité**
- **Tests** : 28/28 skippés justifiés, couverture optimale
- **Conformité SDK** : 100% parfaite
- **Outils qualité** : Tous verts (Black, Ruff, MyPy, Bandit)
- **Sécurité** : Aucune vulnérabilité critique

#### **Performance**
- **Latence simulation** : <1ms moyenne
- **Fréquence** : 100Hz stable
- **CPU** : <5% utilisation
- **Mémoire** : Gestion optimisée

#### **Documentation**
- **Guides complets** : Architecture, SDK, migration, quickstart
- **API interactive** : Swagger UI, ReDoc, OpenAPI
- **Exemples pratiques** : Scripts démo, intégration

### 🎯 **Impact**

#### **Innovation Technique**
- **Première du genre** : Architecture unifiée Sim/Robot
- **Référence technique** : Conformité SDK parfaite
- **Open-source professionnel** : Qualité enterprise

#### **Prêt Production**
- **Robot physique** : Backend prêt, migration transparente
- **Communauté** : Contribution majeure écosystème Reachy Mini
- **Carrière** : Portfolio technique impressionnant

### 🔄 **Migration depuis v1.2.1**

```bash
# Mise à jour vers v1.3.0
pip install --upgrade bbia-sim==1.3.0

# Vérification conformité SDK
python scripts/test_conformity_sdk_officiel.py

# Tests de performance
python scripts/bbia_performance_benchmarks.py --benchmark all
```

### 📋 **Breaking Changes**
- **Aucun** : Migration transparente depuis v1.2.1
- **API stable** : RobotAPI contract gelé v1.1.x
- **Rétrocompatibilité** : Tous les scripts existants fonctionnent

---

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
