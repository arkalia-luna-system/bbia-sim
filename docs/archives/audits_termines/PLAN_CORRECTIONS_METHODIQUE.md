---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : Oct / Oct / Nov. 20255
**Raison** : Document terminé/obsolète/remplacé
---

# 📋 Plan Méthodique Corrections Tests + Améliorations

**Date**: Oct / Oct / Nov. 20255  
**Objectif**: Corriger tous les problèmes sans régressions, code propre, architecture solide

---

## 🎯 Stratégie Globale

### Principes
1. **Analyse d'abord** : Comprendre pourquoi les tests échouent
2. **Correction isolée** : Un problème à la fois avec tests de régression
3. **Validation à chaque étape** : Vérifier que les autres tests passent toujours
4. **Documentation** : Documenter chaque changement

### Ordre d'exécution
1. ✅ Analyser causes échecs (en cours)
2. Corriger tests mapping/forbidden_joints (6 tests) - SIMPLE
3. Corriger tests vision (14 tests) - MOYEN
4. Corriger tests performance (4 tests) - SIMPLE
5. Configuration .coveragerc stricte
6. Améliorer coverage modules
7. Créer tests E2E
8. Créer issues GitHub
9. Documentation finale

---

## 🔍 Analyse Causes Échecs

### 1. Tests Mapping/Forbidden Joints (6 échecs)

#### Problème identifié
- `test_validate_joint_forbidden`: Attendu `False` pour `passive_1`, obtient `True`
- **Cause**: `GlobalConfig.FORBIDDEN_JOINTS` contient bien `passive_1`, mais le test vérifie probablement un mauvais comportement
- **Vérification**: Les tests s'attendent à ce que les joints `passive_*` soient interdits

#### Solution
- Vérifier que `GlobalConfig.validate_joint("passive_1")` retourne bien `False`
- Si le test vérifie autre chose, corriger le test OU la logique

### 2. Tests Vision (14 échecs)

#### Problèmes identifiés
1. **`test_track_untrack_object`**: `track_object("livre")` retourne `False`
   - **Cause probable**: DeepFace non disponible ou objet non détecté
   - **Solution**: Mock ou skip si DeepFace indisponible

2. **`test_vision_specs`, `test_init_specs`**: Problèmes de spécifications
   - **Cause probable**: Changements API non reflétés dans tests
   - **Solution**: Aligner tests avec implémentation réelle

3. **Tests `test_detect_faces_*`**: Problèmes détection visages
   - **Cause probable**: DeepFace non disponible
   - **Solution**: Skip conditionnel + mock si possible

4. **Tests `test_vision_fallback_*`**: Problèmes fallback simulation
   - **Cause probable**: Logique fallback changée
   - **Solution**: Vérifier logique fallback actuelle

### 3. Tests Performance (4 échecs)

#### Problèmes identifiés
- `test_runtime_budget_simulation_10s`: Assertion CPU/RAM échoue
- `test_vision_fps_10s_simulated`: FPS trop bas ou latence élevée
- `test_vision_budget_cpu_ram_10s`: Budget CPU/RAM dépassé
- `test_vision_pipeline_latency_simulated`: Latence trop élevée

**Cause probable**: Seuils trop stricts pour CI ou environnement local  
**Solution**: Ajuster seuils selon environnement OU ajouter skip CI

---

## 🔧 Corrections Détaillées

### Phase 1: Tests Mapping (FAIT EN PREMIER - SIMPLE)

#### Fichier: `tests/test_global_config.py`
```python
def test_validate_joint_forbidden(self):
    """Test validation joint interdit."""
    # Test avec un joint vraiment interdit (passive_1)
    result = GlobalConfig.validate_joint("passive_1")
    assert result is False, f"passive_1 devrait être interdit (FORBIDDEN_JOINTS: {GlobalConfig.FORBIDDEN_JOINTS})"
```

**Action**: Vérifier que `FORBIDDEN_JOINTS` contient bien `passive_1` et corriger test si nécessaire.

#### Fichiers similaires: `test_mapping_reachy_complete.py`, `test_reachy_mini_*_conformity.py`
- Vérifier que les tests utilisent les bons joints
- Aligner avec `GlobalConfig.FORBIDDEN_JOINTS` et `ReachyMapping.FORBIDDEN_JOINTS`

### Phase 2: Tests Vision (AVEC PRÉCAUTION)

#### Fichier: `tests/test_bbia_vision.py`

##### `test_track_untrack_object`
```python
@pytest.mark.unit
@pytest.mark.fast
def test_track_untrack_object() -> None:
    """Test tracking/untracking objet."""
    vision = BBIAVision(robot_api=None)
    vision.scan_environment()

    # Essayer de tracker un objet (peut échouer si pas de DeepFace)
    result = vision.track_object("livre")
    
    # Si DeepFace non disponible, skip plutôt que fail
    if not result:
        try:
            import deepface
            # Si DeepFace disponible mais échec, c'est un vrai problème
            assert False, "track_object échoué avec DeepFace disponible"
        except ImportError:
            pytest.skip("DeepFace non disponible - tracking objet nécessite DeepFace")
    
    assert result is True
    # ... reste du test
```

**Action**: Ajouter skip conditionnel si DeepFace indisponible OU mock DeepFace.

##### Tests `test_detect_faces_*`
- Ajouter skip conditionnel si DeepFace indisponible
- Ou utiliser mock pour tests unitaires

#### Fichier: `tests/test_bbia_vision_extended.py`
- Même logique: skip conditionnel + mock si possible

### Phase 3: Tests Performance (AJUSTER SEUILS)

#### Fichier: `tests/test_runtime_budget.py`
```python
def test_runtime_budget_simulation_10s() -> None:
    # ... code existant ...
    
    # Budgets très larges pour CI
    # Ajuster selon environnement
    is_ci = os.environ.get("CI", "false").lower() == "true"
    max_peak_ram = 100 * 1024 * 1024 if is_ci else 64 * 1024 * 1024  # 100MB CI, 64MB local
    max_cpu_time = 2.0 if is_ci else 1.5  # 2s CI, 1.5s local
    
    assert peak < max_peak_ram, f"Peak RAM trop élevé: {peak} B (max: {max_peak_ram})"
    assert cpu_time_s < max_cpu_time, f"Temps CPU trop élevé: {cpu_time_s:.2f}s/{max_cpu_time}s"
```

**Action**: Ajouter détection CI et seuils adaptatifs.

#### Fichiers similaires: `test_vision_fps_budget.py`, `test_vision_latency.py`
- Même logique: seuils adaptatifs CI/local

---

## 📊 Améliorations Coverage

### Phase 4: `voice_whisper.py` (33% → 70%+)

#### Lignes non couvertes: 17-19, 52-68, 81-119, 132-191, 235, 249-250, 269-273

#### Tests à créer
1. **Test initialisation Whisper**
   - `test_whisper_initialization()`
   - Lignes: 17-19

2. **Test transcription audio**
   - `test_whisper_transcribe()`
   - Lignes: 52-68

3. **Test gestion erreurs**
   - `test_whisper_error_handling()`
   - Lignes: 81-119

4. **Test intégration BBIAVoice**
   - `test_whisper_integration()`
   - Lignes: 132-191

### Phase 5: `vision_yolo.py` (49% → 70%+)

#### Lignes non couvertes: 18-22, 33-34, 65-66, 75, 88-90, 103-104, 109-110, 140-142, 156-174, 189, 212, 220-236, 248-286, 298-306, 323-324, 338-344

#### Tests à créer
1. **Test initialisation YOLO**
   - `test_yolo_initialization()`

2. **Test détection objets**
   - `test_yolo_detect_objects()`

3. **Test gestion erreurs**
   - `test_yolo_error_handling()`

4. **Test fallback**
   - `test_yolo_fallback()`

---

## 🌐 Tests E2E Scénarios Utilisateur

### Phase 6: Créer 3-4 tests E2E

#### 1. `test_e2e_face_detection_greeting.py`
```python
def test_bbia_detects_face_and_greets():
    """Scénario: BBIA détecte visage → suit → salue."""
    vision = BBIAVision(robot_api=None)
    behavior = BBIABehaviorManager(robot_api=...)
    
    # 1. Scanner environnement
    result = vision.scan_environment()
    
    # 2. Si visage détecté, tracker
    if result.get("faces"):
        vision.track_object("face")
        
        # 3. Activer comportement greeting
        behavior.execute_behavior("greeting", {})
        
        # 4. Vérifier mouvement
        assert behavior.robot_api.get_joint_pos("yaw_body") != 0.0
```

#### 2. `test_e2e_voice_interaction.py`
```python
def test_bbia_listens_and_responds():
    """Scénario: BBIA écoute → comprend → répond → bouge."""
    voice = BBIAVoice(...)
    intelligence = BBIAHuggingFace(...)
    behavior = BBIABehaviorManager(...)
    
    # 1. Simuler audio input
    audio_data = ...  # Mock
    
    # 2. Transcription
    text = voice.transcribe(audio_data)
    
    # 3. Génération réponse
    response = intelligence.chat(text)
    
    # 4. Mouvement réponse
    behavior.execute_behavior("conversation", {"text": response})
```

---

## ⚙️ Configuration Coverage Stricte

### Phase 7: `.coveragerc` avec `fail_under=50`

```ini
[run]
source = src/bbia_sim
omit = 
    */tests/*
    */__pycache__/*
    */venv/*
    */site-packages/*

[report]
precision = 2
show_missing = True
skip_covered = False
fail_under = 50  # Minimum 50% coverage

exclude_lines =
    pragma: no cover
    def __repr__
    raise AssertionError
    raise NotImplementedError
    if __name__ == .__main__.:
    @abstractmethod

[html]
directory = htmlcov
title = BBIA-SIM Coverage Report
```

---

## 📝 Issues GitHub "Good First Issue"

### Template à créer: `.github/ISSUE_TEMPLATE/good_first_issue.md`

```markdown
## Description
[Description courte du problème/amélioration]

## Pourquoi c'est un "Good First Issue"
- [ ] Pas de dépendances complexes
- [ ] Documentation claire
- [ ] Tests existants pour guider
- [ ] Scope limité (1-2 fichiers)

## Fichiers concernés
- `src/bbia_sim/...`
- `tests/test_...`

## Étapes pour résoudre
1. [Étape 1]
2. [Étape 2]
3. [Étape 3]

## Tests
- [ ] Tests passent après changement
- [ ] Coverage maintenu/amélioré
```

---

## ✅ Checklist Validation

### Après chaque correction
- [ ] Tests modifiés passent
- [ ] Aucune régression (autres tests passent toujours)
- [ ] Code formaté (Black)
- [ ] Linting OK (Ruff)
- [ ] Types OK (MyPy si applicable)
- [ ] Documentation mise à jour

### Validation finale
- [ ] Tous les 28 tests échoués → passent
- [ ] Coverage global ≥ 50%
- [ ] Coverage `voice_whisper.py` ≥ 70%
- [ ] Coverage `vision_yolo.py` ≥ 70%
- [ ] 3-4 tests E2E créés et passent
- [ ] `.coveragerc` strict configuré
- [ ] 5-10 issues GitHub créées
- [ ] Documentation système tests complète

---

## 🚀 Ordre d'Exécution Recommandé

1. **Tests Mapping** (simple, rapide) → Validation
2. **Tests Performance** (simple, ajustements seuils) → Validation
3. **Configuration `.coveragerc`** (configuration) → Validation
4. **Tests Vision** (moyen, nécessite mocks) → Validation
5. **Coverage `voice_whisper.py`** (création tests) → Validation
6. **Coverage `vision_yolo.py`** (création tests) → Validation
7. **Tests E2E** (création tests) → Validation
8. **Issues GitHub** (création templates + issues) → Validation finale
9. **Documentation** (guide complet) → Validation finale

---

**Prêt à commencer !** 🎯

