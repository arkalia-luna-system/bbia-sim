# 🚀 AMÉLIORATIONS FUTURES - SDK REACHY MINI

> Référence état global
>
> Voir `docs/status.md` → "État par axe" pour prioriser les améliorations (API/SDK, perf, sécurité, CI/CD).

**Date :** Octobre 2025
**Référence SDK :** https://github.com/pollen-robotics/reachy_mini

---

## 🎯 **OBJECTIF**

Documenter les features avancées du SDK Reachy Mini qui sont **disponibles** dans le backend mais **non encore utilisées** dans les modules BBIA pour amélioration future.

---

## 📊 **FEATURES SDK DISPONIBLES**

### **1. Module Media SDK** ✅ **DÉJÀ INTÉGRÉ** (2025-10-30)

**Status :** ✅ **FAIT** - Intégré dans tous les modules concernés avec fallbacks gracieux

**Vérification code :**
- ✅ `bbia_vision.py` (lignes 126-137) : Utilise `robot.media.camera` si disponible
- ✅ `bbia_audio.py` (lignes 162-208) : Utilise `robot.media.microphone` et `robot.media.record_audio()` si disponible
- ✅ `bbia_voice.py` (lignes 259-342) : Utilise `robot.media.speaker` et `robot.media.play_audio()` si disponible

**Capacités :**
```python
robot.media.camera          # ✅ Utilisé dans bbia_vision.py
robot.media.microphone      # ✅ Utilisé dans bbia_audio.py (4 mics)
robot.media.speaker         # ✅ Utilisé dans bbia_voice.py (5W)
robot.media.play_audio()    # ✅ Utilisé dans bbia_voice.py et bbia_audio.py
robot.media.record_audio()  # ✅ Utilisé dans bbia_audio.py
```

**Modules améliorés :**
- ✅ `bbia_vision.py` → Utilise `robot.media.camera` avec fallback simulation
- ✅ `bbia_audio.py` → Utilise `robot.media.microphone` (4 mics) avec fallback sounddevice
- ✅ `bbia_voice.py` → Utilise `robot.media.speaker` (5W) avec fallback pyttsx3

**Avantages :**
- ✅ Qualité hardware optimale (4 microphones directionnels)
- ✅ Annulation de bruit automatique
- ✅ Caméra grand angle 1080p temps réel
- ✅ Synthèse vocale hardware optimisée

---

### **2. Module IO SDK** ⚠️ Non Utilisé

**Status :** Disponible dans `ReachyMiniBackend.io` mais NON UTILISÉ

**Capacités :**
```python
robot.io.get_camera_stream()  # Stream vidéo temps réel
robot.io.get_audio_stream()   # Stream audio temps réel
robot.io.set_leds()            # Contrôle LEDs (si disponibles)
```

**Opportunités :**
- Vision temps réel au lieu de scan périodique
- Audio streaming pour reconnaissance vocale temps réel
- Feedback visuel via LEDs

---

### **3. Techniques d'Interpolation Avancées** ✅ **DÉJÀ IMPLÉMENTÉ** (2025-10-30)

**Status :** ✅ **FAIT** - Mapping émotion → interpolation adaptative implémenté dans `bbia_integration.py`

**Vérification code :**
- ✅ `bbia_integration.py` (lignes 289-305) : `emotion_interpolation_map` avec CARTOON, EASE_IN_OUT, MIN_JERK
- ✅ `backends/reachy_mini_backend.py` (lignes 914-918) : Support toutes les techniques d'interpolation

**Disponibles et utilisées :**
```python
InterpolationTechnique.MIN_JERK       # ✅ Utilisé (neutral, curious, determined)
InterpolationTechnique.LINEAR         # ✅ Disponible (backends/reachy_mini_backend.py:914)
InterpolationTechnique.EASE_IN_OUT    # ✅ Utilisé (calm, sad, nostalgic, fearful)
InterpolationTechnique.CARTOON        # ✅ Utilisé (happy, excited, surprised, angry, proud)
```

**Implémentation actuelle :**
```python
# bbia_integration.py lignes 290-304
emotion_interpolation_map = {
    "happy": "cartoon",      # ✅ Expressif et animé
    "excited": "cartoon",    # ✅ Très expressif
    "surprised": "cartoon", # ✅ Sautillant
    "calm": "ease_in_out",   # ✅ Doux et fluide
    "sad": "ease_in_out",    # ✅ Lent et mélancolique
    "nostalgic": "ease_in_out", # ✅ Doux
    "neutral": "minjerk",   # ✅ Naturel
    "curious": "minjerk",   # ✅ Naturel
    "angry": "cartoon",     # ✅ Expressif
    "fearful": "ease_in_out", # ✅ Doux mais inquiet
    "determined": "minjerk", # ✅ Naturel mais ferme
    "proud": "cartoon",     # ✅ Expressif
}
```

---

### **4. Enregistrement/Replay Avancé** ✅ **DÉJÀ IMPLÉMENTÉ** (2025-10-30)

**Status :** ✅ **FAIT** - Implémenté dans `bbia_behavior.py` et `reachy_mini_backend.py`

**Vérification code :**
- ✅ `backends/reachy_mini_backend.py` (lignes 1065-1119) : `start_recording()`, `stop_recording()`, `play_move()`, `async_play_move()`
- ✅ `bbia_behavior.py` (lignes 1087-1166) : `BBIABehaviorManager.record_behavior()` et `play_saved_behavior()` avec support async

**Disponible et utilisé :**
```python
robot.start_recording()           # ✅ Disponible (reachy_mini_backend.py:1065)
move = robot.stop_recording()      # ✅ Disponible (reachy_mini_backend.py:1076)
robot.play_move(move)              # ✅ Disponible (reachy_mini_backend.py:1089)
robot.async_play_move(move)        # ✅ Disponible (reachy_mini_backend.py:1105)
```

**Implémentation actuelle :**
```python
# bbia_behavior.py lignes 1087-1166
class BBIABehaviorManager:
    def record_behavior(self, name: str):  # ✅ Implémenté
        robot.start_recording()
        # ... exécution comportement ...
        move = robot.stop_recording()
        self.saved_behaviors[name] = move

    def play_saved_behavior(self, name: str, use_async: bool = True):  # ✅ Implémenté
        if name in self.saved_behaviors:
            move = self.saved_behaviors[name]
            if use_async and hasattr(robot, "async_play_move"):
                robot.async_play_move(move, play_frequency=100.0)  # ✅ Non bloquant
            else:
                robot.play_move(move, play_frequency=100.0)  # ✅ Bloquant
```

---

## ✅ **STATUT D'IMPLÉMENTATION** (2025-10-30)

### **Phase 1 : Intégration Media SDK** ✅ **COMPLÉTÉE**

1. **`bbia_vision.py`** ✅ **FAIT**
   - Lignes 126-137 : Détection automatique `robot.media.camera`
   - Fallback gracieux vers simulation si SDK non disponible
   - Support OpenCV pour webcam USB (fallback supplémentaire)

2. **`bbia_audio.py`** ✅ **FAIT**
   - Lignes 162-208 : Utilise `robot.media.record_audio()` avec 4 microphones
   - Fallback gracieux vers sounddevice
   - Support flag `BBIA_DISABLE_AUDIO` pour CI/headless

3. **`bbia_voice.py`** ✅ **FAIT**
   - Lignes 259-342 : Utilise `robot.media.speaker` et `robot.media.play_audio()`
   - Priorité : `media.play_audio()` → `media.speaker.play_file()` → pyttsx3
   - Support backends TTS avancés (Kitten, Kokoro, NeuTTS)

### **Phase 2 : Interpolation Adaptative** ✅ **COMPLÉTÉE**

**Fichier :** `bbia_integration.py` (lignes 289-305)

✅ **Implémenté** : Mapping complet émotion → interpolation avec 12 émotions :
- CARTOON : happy, excited, surprised, angry, proud
- EASE_IN_OUT : calm, sad, nostalgic, fearful
- MIN_JERK : neutral, curious, determined

### **Phase 3 : Enregistrement Comportements** ✅ **COMPLÉTÉE**

**Fichier :** `bbia_behavior.py` (lignes 1087-1166)

✅ **Implémenté** :
- `BBIABehaviorManager.record_behavior()` : Enregistre comportements
- `BBIABehaviorManager.play_saved_behavior()` : Rejoue avec support async
- Bibliothèque `saved_behaviors` pour réutilisation mouvements

---

## 📈 **BÉNÉFICES ATTENDUS**

### **Performance**
- ✅ Vision temps réel (au lieu de simulation)
- ✅ Audio qualité hardware (4 mics directionnels)
- ✅ Synthèse vocale optimisée (5W hardware)

### **Expressivité**
- ✅ Mouvements plus expressifs avec `CARTOON`
- ✅ Transitions plus douces avec `EASE_IN_OUT`
- ✅ Réutilisation de mouvements complexes

### **Robustesse**
- ✅ Hardware optimisé vs logiciel générique
- ✅ Annulation de bruit automatique
- ✅ Stream temps réel pour réactivité

---

## ✅ **VALIDATION FINALE**

Toutes les améliorations sont **déjà implémentées et opérationnelles** ✅

### **État actuel :**
- ✅ Phase 1 (Media SDK) : **COMPLÉTÉE** - Tous les modules utilisent `robot.media`
- ✅ Phase 2 (Interpolation) : **COMPLÉTÉE** - Mapping émotion → technique implémenté
- ✅ Phase 3 (Record/Replay) : **COMPLÉTÉE** - `BBIABehaviorManager` avec support async

### **Validation code :**
- ✅ Backend expose `robot.media` et utilisé partout
- ✅ Fallbacks gracieux en place (simulation, sounddevice, pyttsx3)
- ✅ Architecture modulaire avec intégration complète
- ✅ Tests créés : `test_sdk_media_integration.py`, `test_emotion_interpolation_mapping()`

### **Restant à faire (optionnel) :**
- ⚠️ Module IO SDK (`robot.io.get_camera_stream()`, `robot.io.get_audio_stream()`) : Non encore utilisé
  - Opportunité : Streaming temps réel au lieu de scan périodique
  - Priorité : **Basse** (fonctionnalités actuelles suffisantes)

**Recommandation :** ✅ Toutes les améliorations prioritaires sont complétées. Le système utilise pleinement le hardware Reachy Mini avec fallbacks robustes.

