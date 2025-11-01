# 📋 Issues GitHub à Créer - Prêt à Copier/Coller

**Date** : 2025-01-31  
**Source** : `docs/GOOD_FIRST_ISSUES.md`

---

## Issue 1: Améliorer Coverage `bbia_audio.py`

```markdown
---
name: Good First Issue
about: Améliorer Coverage bbia_audio.py
title: "[Good First Issue] Améliorer Coverage bbia_audio.py"
labels: ["good first issue", "help wanted", "testing"]
assignees: []
---

## 📋 Description

Actuellement coverage `bbia_audio.py` est faible (~30-40%). 
Créer tests pour fonctions principales :
- `detecter_son()`
- `_capture_audio_chunk()`
- Gestion erreurs audio

## 🎯 Pourquoi c'est un "Good First Issue"

- ✅ Pas de dépendances complexes
- ✅ Documentation claire
- ✅ Tests existants pour guider
- ✅ Scope limité (1-2 fichiers maximum)

## 📁 Fichiers concernés

- `src/bbia_sim/bbia_audio.py`
- `tests/test_bbia_audio.py` (créer/améliorer)

## ✅ Étapes pour résoudre

1. Analyser `bbia_audio.py` et identifier fonctions non testées
2. Créer tests avec mocks `sounddevice`
3. Tester cas limites (audio désactivé, erreurs)
4. Vérifier coverage atteint 70%+

## 🧪 Tests

- [ ] Les tests existants passent après changement
- [ ] Coverage maintenu/amélioré (70%+)
- [ ] Aucune régression introduite

## 📚 Ressources utiles

- [Documentation du projet](README.md)
- [Guide de tests](docs/GUIDE_SYSTEME_TESTS.md)
- Voir `tests/test_voice_whisper_comprehensive.py` comme exemple
```

---

## Issue 2: Ajouter Tests pour `bbia_memory.py`

```markdown
---
name: Good First Issue
about: Ajouter Tests pour bbia_memory.py
title: "[Good First Issue] Ajouter Tests pour bbia_memory.py"
labels: ["good first issue", "help wanted", "testing"]
assignees: []
---

## 📋 Description

Module mémoire persistante manque tests (actuellement 0% coverage).
Tester :
- Sauvegarde/conservation conversations
- Chargement mémoire
- Gestion fichiers JSON/YAML
- Préférences utilisateur
- Apprentissages

## 🎯 Pourquoi c'est un "Good First Issue"

- ✅ Pas de dépendances complexes
- ✅ Tests avec fichiers temporaires (`tempfile`)
- ✅ Scope limité
- ✅ Fonctions claires à tester

## 📁 Fichiers concernés

- `src/bbia_sim/bbia_memory.py`
- `tests/test_bbia_memory.py` (créer)

## ✅ Étapes pour résoudre

1. Lire `bbia_memory.py` pour comprendre fonctionnalités
2. Créer tests avec fichiers temporaires (`tempfile`)
3. Tester sauvegarde/chargement
4. Tester gestion erreurs (fichier corrompu, permissions)

## 🧪 Tests

- [ ] Tests créés pour toutes méthodes principales
- [ ] Coverage atteint 70%+
- [ ] Aucune régression introduite

## 📚 Ressources utiles

- [Guide de tests](docs/GUIDE_SYSTEME_TESTS.md)
- Exemple : `tests/test_bbia_memory.py` (base créée)
```

---

## Issue 3: Améliorer Tests `bbia_emotions.py`

```markdown
---
name: Good First Issue
about: Améliorer Tests bbia_emotions.py
title: "[Good First Issue] Améliorer Tests bbia_emotions.py"
labels: ["good first issue", "help wanted", "testing"]
assignees: []
---

## 📋 Description

Coverage `bbia_emotions.py` peut être amélioré.
Ajouter tests :
- Transitions émotions complexes
- Historique émotions
- Validation intensités limites

## 🎯 Pourquoi c'est un "Good First Issue"

- ✅ Module bien documenté
- ✅ Tests existants pour guider
- ✅ Scope limité

## 📁 Fichiers concernés

- `src/bbia_sim/bbia_emotions.py`
- `tests/test_bbia_emotions.py` (améliorer)

## ✅ Étapes pour résoudre

1. Analyser coverage actuel
2. Identifier branches non testées
3. Créer tests transition émotions
4. Tester cas limites (intensité 0, 1, négative, >1)

## 🧪 Tests

- [ ] Coverage amélioré à 70%+
- [ ] Tests cas limites ajoutés
- [ ] Aucune régression introduite
```

---

## Issue 4: Tests Vision Structure Bbox

```markdown
---
name: Good First Issue
about: Tests Vision Structure Bbox
title: "[Good First Issue] Tests Vision Structure Bbox"
labels: ["good first issue", "help wanted", "testing"]
assignees: []
---

## 📋 Description

Vérifier structure bbox retournées par vision.
Tests validation format données.

## 🎯 Pourquoi c'est un "Good First Issue"

- ✅ Très facile
- ✅ Scope très limité
- ✅ Test simple à écrire

## 📁 Fichiers concernés

- `tests/test_bbia_vision_extended.py`

## ✅ Étapes pour résoudre

1. Ajouter test `test_bbox_structure_valid()` dans `test_bbia_vision_extended.py`
2. Vérifier champs requis : `x`, `y`, `width`, `height`, `center_x`, `center_y`
3. Vérifier types corrects (int)
4. Tester valeurs limites (bbox hors image)

## 🧪 Tests

- [ ] Test structure bbox créé
- [ ] Test valeurs limites créé
- [ ] Aucune régression introduite
```

---

## Issue 5: Tests Mapping Commandes Vocales Avancés

```markdown
---
name: Good First Issue
about: Tests Mapping Commandes Vocales Avancés
title: "[Good First Issue] Tests Mapping Commandes Vocales Avancés"
labels: ["good first issue", "help wanted", "testing"]
assignees: []
---

## 📋 Description

Étendre tests `VoiceCommandMapper`.
Tester :
- Commandes avec ponctuation
- Commandes multi-mots
- Variations linguistiques

## 📁 Fichiers concernés

- `tests/test_voice_whisper_comprehensive.py`

## ✅ Étapes pour résoudre

1. Ajouter tests dans `TestVoiceCommandMapper`
2. Tester commandes : "salue!", "regarde moi s'il te plaît"
3. Tester commandes partielles complexes
4. Documenter commandes supportées
```

---

## Note

Ces issues sont prêtes à être copiées/collées dans GitHub Issues.
Créer une issue par section ci-dessus.

