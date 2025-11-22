# 🎯 Good First Issues - Suggestions pour Nouveaux Contributeurs

**Date**: 21 novembre 2025  
**Pour nouveaux contributeurs** : Idéal pour débuter avec le projet

---

## 📋 Liste des Issues Suggérées

### 1. ⚠️ Améliorer Coverage `bbia_audio.py` - **À VÉRIFIER**

**Priorité** : ⚠️ À clarifier  
**Difficulté** : -  
**Temps estimé** : -

**Description** :

- ⚠️ **Issue GitHub #4** : La fonction `_capture_audio_chunk()` mentionnée dans l'issue **n'existe pas** dans le code actuel
- ✅ Coverage actuel : **87.76%** (excellent ✅)
- Tests existants : `test_bbia_audio_coverage_high.py`, `test_bbia_audio_improved.py`

**Statut** : ⚠️ **À VÉRIFIER** - Fonction mentionnée absente, issue potentiellement obsolète

**Action recommandée** :
- Vérifier si la fonction existe ailleurs ou a été renommée
- Mettre à jour l'issue GitHub pour refléter l'état actuel du code

**Ressources** :

- 📊 **Analyse détaillée** : Voir `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`
- Voir `tests/test_voice_whisper_comprehensive.py` comme exemple
- Documentation : `docs/development/testing.md`

---

### 2. 📝 Ajouter Tests pour `bbia_memory.py`

**Priorité** : Moyenne  
**Difficulté** : Facile  
**Temps estimé** : 2-3 heures

**Description** :

- Module mémoire persistante manque tests
- Tester :
  - Sauvegarde/conservation conversations
  - Chargement mémoire
  - Gestion fichiers JSON/YAML

**Fichiers concernés** :

- `src/bbia_sim/bbia_memory.py`
- `tests/test_bbia_memory.py` (créer)

**Étapes** :

1. Lire `bbia_memory.py` pour comprendre fonctionnalités
2. Créer tests avec fichiers temporaires (`tempfile`)
3. Tester sauvegarde/chargement
4. Tester gestion erreurs (fichier corrompu, permissions)

---

### 3. 🎨 Améliorer Tests `bbia_emotions.py` - **Issue #6**

**Priorité** : Moyenne  
**Difficulté** : Facile-Moyenne  
**Temps estimé** : 3-4 heures

**Description** :

- ✅ Coverage actuel : **81.71%** (déjà excellent ✅)
- **Issue GitHub #6** : Tests de transitions complexes manquants :
  - ❌ Séquences rapides (happy → sad → excited en < 1 seconde)
  - ❌ Transitions avec durées différentes
  - ❌ Tests de stress (10+ transitions successives)
  - ❌ Transitions avec intensités extrêmes (0.0 → 1.0 → 0.0)

**Fichiers concernés** :

- `src/bbia_sim/bbia_emotions.py`
- `tests/test_bbia_emotions.py` (améliorer)

**Étapes** :

1. Analyser coverage actuel
2. Identifier branches non testées
3. Créer tests transition émotions complexes
4. Tester cas limites (intensité 0, 1, négative, >1)

**Ressources** :

- 📊 **Analyse détaillée** : Voir `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`

---

### 4. 🔍 Ajouter Tests Vision Structure Bbox - **Issue #7**

**Priorité** : Moyenne  
**Difficulté** : Très Facile  
**Temps estimé** : 1-2 heures

**Description** :

- **Issue GitHub #7** : Vérifier structure bbox retournées par vision
- ✅ Le code crée bien des bbox avec les 6 champs requis
- ❌ Test spécifique manquant pour valider la structure complète

**Fichiers concernés** :

- `tests/test_bbia_vision_extended.py`

**Étapes** :

1. Ajouter test `test_bbox_structure_valid()` dans `test_bbia_vision_extended.py`
2. Vérifier champs requis : `x`, `y`, `width`, `height`, `center_x`, `center_y`
3. Vérifier types corrects (int) pour tous les champs bbox
4. Tester valeurs limites (bbox hors image, coordonnées négatives)

**Ressources** :

- 📊 **Analyse détaillée** : Voir `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`

---

### 5. 🗣️ Tests Mapping Commandes Vocales Avancés - **Issue #8**

**Priorité** : 🔴 **Haute**  
**Difficulté** : Facile  
**Temps estimé** : 2 heures

**Description** :

- **Issue GitHub #8** : Étendre tests `VoiceCommandMapper`
- ✅ Tests basiques existent
- ❌ Tests avancés manquants :
  - Commandes avec ponctuation (`"salue!"`, `"regarde."`, `"arrête?"`)
  - Commandes multi-mots avec apostrophes (`"regarde moi s'il te plaît"`)
  - Variations linguistiques (`"slt"` → `"greet"`, abréviations)
  - Commandes partielles dans phrases longues

**Fichiers concernés** :

- `tests/test_voice_whisper_comprehensive.py`

**Étapes** :

1. Ajouter tests dans `TestVoiceCommandMapper`
2. Tester commandes : `"salue!"`, `"regarde moi s'il te plaît"`
3. Tester commandes partielles complexes
4. Documenter commandes supportées

**Ressources** :

- 📊 **Analyse détaillée** : Voir `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`

---

### 6. 📊 Benchmark Latence Vision Pipeline

**Priorité** : Faible  
**Difficulté** : Moyenne  
**Temps estimé** : 3-4 heures

**Description** :

- Créer benchmark détaillé pipeline vision
- Mesurer latence chaque étape :
  - Capture image
  - Détection YOLO
  - Détection MediaPipe
  - Post-processing

**Fichiers concernés** :

- `tests/test_vision_pipeline_benchmark.py` (créer)

**Étapes** :

1. Analyser `bbia_vision.py` pipeline
2. Instrumenter avec timestamps
3. Mesurer latence chaque étape
4. Générer rapport statistiques (p50, p95, p99)

**Ressources** :

- Voir `tests/test_vision_latency.py` comme référence

---

### 7. 🔒 Tests Sécurité Input Validation

**Priorité** : Haute  
**Difficulté** : Moyenne  
**Temps estimé** : 4-5 heures

**Description** :

- Tester validation inputs utilisateur
- Protection injection prompts
- Validation joints, émotions, intensités

**Fichiers concernés** :

- `tests/test_input_validation_advanced.py` (améliorer)

**Étapes** :

1. Identifier tous points d'entrée utilisateur
2. Créer tests injection prompts
3. Tester validation limites (joints, émotions)
4. Tester caractères spéciaux, unicode

---

### 8. 🎭 Mock Amélioré Robot API

**Priorité** : Moyenne  
**Difficulté** : Facile-Moyenne  
**Temps estimé** : 2-3 heures

**Description** :

- Enrichir `ReachyMiniMock` avec comportements réalistes
- Simuler latence joints
- Simuler erreurs (joint bloqué, timeout)

**Fichiers concernés** :

- `tests/reachy_mini_mock.py`

**Étapes** :

1. Analyser `ReachyMiniBackend` comportements réels
2. Ajouter latence simulée (`time.sleep` proportionnel)
3. Simuler erreurs conditionnelles
4. Tester mock avec tests existants

---

### 9. 📖 Documentation Tests Spécifiques

**Priorité** : Faible  
**Difficulté** : Très Facile  
**Temps estimé** : 1-2 heures par module

**Description** :

- Documenter architecture tests par module
- Exemples utilisation mocks
- Cas d'usage typiques

**Fichiers concernés** :

- `docs/tests/VISION_TESTS.md` (créer)
- `docs/tests/VOICE_TESTS.md` (créer)
- `docs/tests/EMOTIONS_TESTS.md` (créer)

**Étapes** :

1. Choisir module (vision, voice, emotions)
2. Documenter tests existants
3. Ajouter exemples code
4. Expliquer stratégie mocks

---

### 10. 🧹 Nettoyage Tests Redondants

**Priorité** : Faible  
**Difficulté** : Facile  
**Temps estimé** : 2-3 heures

**Description** :

- Identifier tests redondants/dupliqués
- Consolider tests similaires
- Supprimer tests obsolètes

**Étapes** :

1. Analyser tous fichiers tests
2. Identifier duplications
3. Consolider en tests paramétrés (`@pytest.mark.parametrize`)
4. Supprimer tests obsolètes

---

## 🎓 Guide pour Contributeurs

### Comment Commencer

1. **Fork** le repository
2. **Cloner** votre fork localement
3. **Créer branche** : `git checkout -b feature/issue-X`
4. **Lire** : `docs/development/testing.md`
5. **Créer** tests/fonctionnalités
6. **Vérifier** : `pytest`, `black`, `ruff`
7. **Push** et créer Pull Request

### Standards de Code

- **Formatage** : Black (auto-format)
- **Linting** : Ruff
- **Type hints** : MyPy
- **Tests** : pytest avec docstrings

### Tests Requis

- ✅ Tous tests passent
- ✅ Coverage maintenu/amélioré
- ✅ Pas de régressions
- ✅ Documentation mise à jour

---

## 📝 Template Issue GitHub

Pour créer une issue, copier ce template :

```markdown
---
name: Good First Issue
about: [Titre de l'issue]
title: "[Good First Issue] [Titre]"
labels: ["good first issue", "help wanted"]
assignees: []
---

## 📋 Description

[Description de l'issue]

## 🎯 Pourquoi c'est un "Good First Issue"

- [ ] Pas de dépendances complexes
- [ ] Documentation claire
- [ ] Tests existants pour guider
- [ ] Scope limité (1-2 fichiers maximum)

## 📁 Fichiers concernés

- `src/bbia_sim/...`
- `tests/test_...`

## ✅ Étapes pour résoudre

1. [Étape 1]
2. [Étape 2]
3. [Étape 3]

## 🧪 Tests

- [ ] Les tests existants passent après changement
- [ ] Coverage maintenu/amélioré
- [ ] Aucune régression introduite

## 📚 Ressources utiles

- [Documentation du projet](../README.md)
- [Guide de tests](../development/testing.md)

```

---

## 🤝 Contribution

**Questions ?** Ouvrir une discussion ou issue GitHub.

**Besoin d'aide ?** Mentionner `@maintainers` dans commentaire issue.

**Prêt à contribuer ?** Choisir une issue et commencer ! 🚀

---

## 🔗 Guide Complet de Contribution

> **💡 Pour le guide complet de contribution**  
> Consultez le [Guide de Contribution Complet](../community/CONTRIBUTION_GUIDE.md) qui inclut :
> - Templates GitHub (bug report, feature request)
> - Standards de code détaillés
> - Processus de Pull Request
> - Guidelines de commit

---

**Dernière mise à jour** : 21 novembre 2025

---

## 📊 Analyse Technique Détaillée

Pour une analyse technique complète de l'état actuel des issues GitHub (vérification code, tests existants, recommandations détaillées), consultez :

**→ [Analyse Complète Issues GitHub](../../quality/audits/ANALYSE_ISSUES_GITHUB.md)**

Cette analyse inclut :
- ✅ Vérification de l'existence des fonctions mentionnées dans les issues
- ✅ État actuel des tests existants
- ✅ Détails techniques sur ce qui manque
- ✅ Recommandations prioritaires

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Guide Contribution Complet](../community/CONTRIBUTION_GUIDE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md) • [Analyse Issues GitHub](../../quality/audits/ANALYSE_ISSUES_GITHUB.md)
