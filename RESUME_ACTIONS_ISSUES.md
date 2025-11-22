# 📋 RÉSUMÉ DES ACTIONS - Issues GitHub

**Date** : Décembre 2025  
**Statut** : ✅ **3 issues sur 4 IMPLÉMENTÉES**

> ⚠️ **NOTE** : Ce fichier contient les actions initiales. Pour le statut d'implémentation, voir `RESUME_IMPLEMENTATION_ISSUES.md`

---

## 🎯 Issue #8 - Tests Mapping Commandes Vocales Avancés

**Priorité** : 🔴 **HAUTE**  
**Fichier à modifier** : `tests/test_voice_whisper_comprehensive.py`

### ✅ Ce qui existe déjà :
- Tests basiques : `test_map_command_valid()`, `test_map_command_invalid()`
- Tests correspondance exacte et partielle

### ❌ Ce qu'il faut AJOUTER :

1. **Tests avec ponctuation** :
```python
def test_map_command_with_punctuation(self):
    """Test mapping avec ponctuation."""
    mapper = VoiceCommandMapper()
    
    # Test ponctuation
    assert mapper.map_command("salue!")["action"] == "greet"
    assert mapper.map_command("regarde.")["action"] == "look_at"
    assert mapper.map_command("arrête?")["action"] == "stop"  # Si existe
```

2. **Tests multi-mots avec apostrophes** :
```python
def test_map_command_multi_words_apostrophe(self):
    """Test commandes multi-mots avec apostrophes."""
    mapper = VoiceCommandMapper()
    
    result = mapper.map_command("regarde moi s'il te plaît")
    assert result is not None
    assert result["action"] == "look_at"
    
    result = mapper.map_command("peux-tu me saluer")
    assert result is not None
    assert result["action"] == "greet"
```

3. **Tests variations linguistiques** :
```python
def test_map_command_linguistic_variations(self):
    """Test variations linguistiques (abréviations)."""
    mapper = VoiceCommandMapper()
    
    # Test abréviations (si supportées)
    # "slt" → "greet" (salut)
    # "regard" → "look_at" (regarde)
```

4. **Tests commandes partielles dans phrases longues** :
```python
def test_map_command_partial_in_long_sentence(self):
    """Test détection commande dans phrase longue."""
    mapper = VoiceCommandMapper()
    
    result = mapper.map_command("peux-tu me saluer maintenant")
    assert result is not None
    assert result["action"] == "greet"
    
    result = mapper.map_command("je veux que tu regardes par là")
    assert result is not None
    assert result["action"] == "look_at"
```

---

## 🎯 Issue #7 - Tests Vision Structure Bbox

**Priorité** : 🟡 **MOYENNE**  
**Fichier à modifier** : `tests/test_bbia_vision_extended.py`

### ✅ Ce qui existe déjà :
- Test `test_scan_environment_objects_structure()` qui vérifie `name`, `distance`, `confidence`, `position`
- Le code crée bien des bbox avec les 6 champs requis

### ❌ Ce qu'il faut AJOUTER :

**Nouveau test** : `test_bbox_structure_valid()`

```python
def test_bbox_structure_valid(self):
    """Test structure complète des bbox retournés par scan_environment()."""
    vision = BBIAVision()
    result = vision.scan_environment()
    
    # Vérifier bbox pour TOUS les objets
    for obj in result["objects"]:
        assert "bbox" in obj, "Chaque objet doit avoir un bbox"
        bbox = obj["bbox"]
        
        # Vérifier les 6 champs requis
        assert "x" in bbox, "bbox doit avoir 'x'"
        assert "y" in bbox, "bbox doit avoir 'y'"
        assert "width" in bbox, "bbox doit avoir 'width'"
        assert "height" in bbox, "bbox doit avoir 'height'"
        assert "center_x" in bbox, "bbox doit avoir 'center_x'"
        assert "center_y" in bbox, "bbox doit avoir 'center_y'"
        
        # Vérifier types corrects (int)
        assert isinstance(bbox["x"], int), "x doit être int"
        assert isinstance(bbox["y"], int), "y doit être int"
        assert isinstance(bbox["width"], int), "width doit être int"
        assert isinstance(bbox["height"], int), "height doit être int"
        assert isinstance(bbox["center_x"], int), "center_x doit être int"
        assert isinstance(bbox["center_y"], int), "center_y doit être int"
    
    # Vérifier bbox pour TOUS les visages
    for face in result["faces"]:
        assert "bbox" in face, "Chaque visage doit avoir un bbox"
        bbox = face["bbox"]
        
        # Même vérification que pour les objets
        assert all(key in bbox for key in ["x", "y", "width", "height", "center_x", "center_y"])
        assert all(isinstance(bbox[key], int) for key in ["x", "y", "width", "height", "center_x", "center_y"])

def test_bbox_edge_cases(self):
    """Test valeurs limites bbox (bbox hors image, coordonnées négatives)."""
    vision = BBIAVision()
    result = vision.scan_environment()
    
    # Vérifier que les bbox sont valides (pas de valeurs négatives incohérentes)
    for obj in result["objects"]:
        if "bbox" in obj:
            bbox = obj["bbox"]
            # Les valeurs peuvent être négatives si hors image, mais doivent être cohérentes
            assert bbox["width"] >= 0, "width ne peut pas être négatif"
            assert bbox["height"] >= 0, "height ne peut pas être négatif"
```

---

## 🎯 Issue #6 - Améliorer Tests bbia_emotions.py

**Priorité** : 🟡 **MOYENNE**  
**Fichier à modifier** : `tests/test_bbia_emotions.py`

### ✅ Ce qui existe déjà :
- Tests basiques : `test_set_emotion()`, `test_emotion_history()`, `test_set_emotion_intensity_clamping()`
- Test transition basique : `test_transition_smooth()`

### ❌ Ce qu'il faut AJOUTER :

1. **Tests séquences rapides** :
```python
def test_emotion_rapid_sequences(self):
    """Test transitions rapides (happy → sad → excited en < 1 seconde)."""
    emotions = BBIAEmotions()
    
    # Changer émotions rapidement
    emotions.set_emotion("happy", 0.8)
    emotions.set_emotion("sad", 0.6)
    emotions.set_emotion("excited", 0.9)
    
    # Vérifier que l'historique contient les 3 transitions
    history = emotions.get_emotion_history()
    assert len(history) >= 3
    assert history[-3]["emotion"] == "happy"
    assert history[-2]["emotion"] == "sad"
    assert history[-1]["emotion"] == "excited"
```

2. **Tests transitions avec durées différentes** :
```python
def test_emotion_transition_different_durations(self):
    """Test transitions avec durées différentes."""
    emotions = BBIAEmotions()
    
    # Transition rapide
    emotions.transition_duration = 0.1
    emotions.set_emotion("happy", 0.8)
    
    # Transition lente
    emotions.transition_duration = 2.0
    emotions.set_emotion("sad", 0.6)
    
    assert emotions.transition_duration == 2.0
    assert emotions.current_emotion == "sad"
```

3. **Tests de stress (10+ transitions successives)** :
```python
def test_emotion_stress_multiple_transitions(self):
    """Test stress : 10+ transitions successives rapides."""
    emotions = BBIAEmotions()
    
    # 15 transitions successives rapides
    emotion_list = ["happy", "sad", "excited", "neutral", "curious"]
    for i in range(15):
        emotion = emotion_list[i % len(emotion_list)]
        intensity = 0.5 + (i % 5) * 0.1
        emotions.set_emotion(emotion, intensity)
    
    # Vérifier que l'historique est correct
    history = emotions.get_emotion_history()
    assert len(history) >= 15
    
    # Vérifier que la dernière émotion est correcte
    assert emotions.current_emotion in emotion_list
```

4. **Tests transitions avec intensités extrêmes** :
```python
def test_emotion_extreme_intensities(self):
    """Test transitions avec intensités extrêmes (0.0 → 1.0 → 0.0)."""
    emotions = BBIAEmotions()
    
    # Intensité minimale
    emotions.set_emotion("happy", 0.0)
    assert emotions.emotion_intensity == 0.0
    
    # Intensité maximale
    emotions.set_emotion("excited", 1.0)
    assert emotions.emotion_intensity == 1.0
    
    # Retour à minimale
    emotions.set_emotion("calm", 0.0)
    assert emotions.emotion_intensity == 0.0
```

---

## 🎯 Issue #4 - Améliorer Coverage bbia_audio.py

**Priorité** : 🟢 **BASSE** (À vérifier d'abord)  
**Statut** : ⚠️ **POTENTIELLEMENT OBSOLÈTE**

### ⚠️ PROBLÈME IDENTIFIÉ :
- La fonction `_capture_audio_chunk()` mentionnée dans l'issue **N'EXISTE PAS** dans le code actuel
- Fonctions existantes : `enregistrer_audio()`, `lire_audio()`, `detecter_son()`

### ✅ Actions à faire AVANT de coder :

1. **Vérifier si la fonction existe ailleurs** :
```bash
grep -r "_capture_audio_chunk" src/
grep -r "capture.*audio.*chunk" src/
```

2. **Si la fonction n'existe pas** :
   - Option A : Mettre à jour l'issue GitHub pour tester `enregistrer_audio()` ou `detecter_son()` à la place
   - Option B : Fermer l'issue si elle n'est plus pertinente

3. **Si vous voulez quand même améliorer le coverage** :
   - Tester `enregistrer_audio()` avec différents cas limites
   - Tester `detecter_son()` avec différents seuils
   - Tester gestion erreurs audio

---

## 📊 RÉCAPITULATIF PAR PRIORITÉ

### 🔴 Priorité 1 - À faire en premier
- **Issue #8** : Ajouter tests avancés VoiceCommandMapper
  - Temps estimé : 2 heures
  - Fichier : `tests/test_voice_whisper_comprehensive.py`

### 🟡 Priorité 2 - À faire ensuite
- **Issue #7** : Ajouter test structure bbox
  - Temps estimé : 1-2 heures
  - Fichier : `tests/test_bbia_vision_extended.py`

- **Issue #6** : Ajouter tests transitions complexes émotions
  - Temps estimé : 3-4 heures
  - Fichier : `tests/test_bbia_emotions.py`

### 🟢 Priorité 3 - À vérifier d'abord
- **Issue #4** : Vérifier pertinence avant de coder
  - Temps estimé : 30 minutes (vérification)
  - Action : Chercher fonction ou mettre à jour/fermer issue

---

## ✅ CHECKLIST POUR COMMENCER

Avant de commencer à coder :

- [ ] Lire l'analyse détaillée : `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`
- [ ] Vérifier que les tests existants passent : `pytest tests/test_voice_whisper_comprehensive.py -v`
- [ ] Choisir une issue (recommandé : #8 en premier)
- [ ] Créer une branche Git : `git checkout -b feature/issue-8-tests-voice-advanced`
- [ ] Ajouter les tests
- [ ] Vérifier que les tests passent : `pytest tests/... -v`
- [ ] Vérifier coverage : `pytest --cov=src/bbia_sim/... tests/...`
- [ ] Créer Pull Request

---

## 📚 RESSOURCES UTILES

- **Analyse complète** : `docs/quality/audits/ANALYSE_ISSUES_GITHUB.md`
- **Guide contributeurs** : `docs/getting-started/contributing.md`
- **Guide tests** : `docs/development/testing.md`
- **Code source** :
  - `src/bbia_sim/voice_whisper.py` (lignes 697-756 pour VoiceCommandMapper)
  - `src/bbia_sim/bbia_vision.py` (lignes 939-946 pour bbox objets)
  - `src/bbia_sim/bbia_emotions.py` (lignes 109-131 pour set_emotion)

---

**Dernière mise à jour** : Décembre 2025

