# 📝 ACTIONS À FAIRE SUR GITHUB - Issues

**Date** : Décembre 2025  
**Repository** : arkalia-luna-system/bbia-sim

---

## ✅ Issues à FERMER (#6, #7, #8)

### Issue #8 - Tests Mapping Commandes Vocales Avancés

**Action** : Ajouter commentaire puis fermer

**Commentaire à ajouter** :
```markdown
✅ **Tests implémentés avec succès !**

**Tests ajoutés** :
- `test_map_command_with_punctuation()` - Tests avec ponctuation
- `test_map_command_multi_words_apostrophe()` - Tests multi-mots avec apostrophes
- `test_map_command_partial_in_long_sentence()` - Tests commandes dans phrases longues
- `test_map_command_variations_orthographic()` - Tests variations orthographiques

**Fichier modifié** : `tests/test_voice_whisper_comprehensive.py`

**Résultat** : ✅ Tous les tests passent

Voir `RESUME_IMPLEMENTATION_ISSUES.md` pour détails complets.
```

**Message de fermeture** :
```
✅ Tests implémentés - Issue résolue
```

---

### Issue #7 - Tests Vision Structure Bbox

**Action** : Ajouter commentaire puis fermer

**Commentaire à ajouter** :
```markdown
✅ **Tests implémentés avec succès !**

**Tests ajoutés** :
- `test_bbox_structure_valid()` - Vérifie structure complète des bbox (6 champs : x, y, width, height, center_x, center_y)
- `test_bbox_edge_cases()` - Tests valeurs limites (width/height >= 0)

**Fichier modifié** : `tests/test_bbia_vision_extended.py`

**Résultat** : ✅ Tous les tests passent
- Vérifie les 6 champs requis pour objets ET visages
- Vérifie types corrects (int)
- Gère le cas où aucun bbox n'existe (mode simulation)

Voir `RESUME_IMPLEMENTATION_ISSUES.md` pour détails complets.
```

**Message de fermeture** :
```
✅ Tests implémentés - Issue résolue
```

---

### Issue #6 - Améliorer Tests bbia_emotions.py

**Action** : Ajouter commentaire puis fermer

**Commentaire à ajouter** :
```markdown
✅ **Tests implémentés avec succès !**

**Tests ajoutés** :
- `test_emotion_rapid_sequences()` - Tests séquences rapides (happy → sad → excited en < 1 seconde)
- `test_emotion_transition_different_durations()` - Tests transitions avec durées différentes
- `test_emotion_stress_multiple_transitions()` - Tests de stress (15 transitions successives)
- `test_emotion_extreme_intensities()` - Tests intensités extrêmes (0.0 → 1.0 → 0.0)

**Fichier modifié** : `tests/test_bbia_emotions.py`

**Résultat** : ✅ Tous les tests passent

Voir `RESUME_IMPLEMENTATION_ISSUES.md` pour détails complets.
```

**Message de fermeture** :
```
✅ Tests implémentés - Issue résolue
```

---

## ⚠️ Issue à VÉRIFIER (#4)

### Issue #4 - Améliorer Coverage bbia_audio.py

**Action** : Ajouter commentaire pour clarification

**Commentaire à ajouter** :
```markdown
⚠️ **Clarification nécessaire**

Après analyse du code, la fonction `_capture_audio_chunk()` mentionnée dans l'issue **n'existe pas** dans `src/bbia_sim/bbia_audio.py`.

**Fonctions existantes** :
- ✅ `enregistrer_audio()` - Déjà bien testée
- ✅ `lire_audio()` - Déjà bien testée
- ✅ `detecter_son()` - Déjà bien testée

**Coverage actuel** : ~87.76% (excellent ✅)

**Options** :
1. Mettre à jour l'issue pour tester les fonctions existantes avec plus de cas limites
2. Fermer l'issue si elle n'est plus pertinente (coverage déjà excellent)

Quelle option préférez-vous ? 🤔
```

**Action suivante** : Attendre réponse avant de fermer ou mettre à jour

---

## 📊 RÉSUMÉ DES ACTIONS

| Issue | Action | Statut |
|-------|--------|--------|
| #8 | Ajouter commentaire + Fermer | ⏳ À faire |
| #7 | Ajouter commentaire + Fermer | ⏳ À faire |
| #6 | Ajouter commentaire + Fermer | ⏳ À faire |
| #4 | Ajouter commentaire (clarification) | ⏳ À faire |

---

## 🎯 ORDRE RECOMMANDÉ

1. **D'abord** : Ajouter les commentaires sur les 4 issues
2. **Ensuite** : Fermer les issues #6, #7, #8
3. **Enfin** : Attendre réponse pour l'issue #4

---

**Dernière mise à jour** : Décembre 2025

