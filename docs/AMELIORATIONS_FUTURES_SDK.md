# 🚀 AMÉLIORATIONS FUTURES - SDK REACHY MINI

**Date :** Octobre 2025  
**Référence SDK :** https://github.com/pollen-robotics/reachy_mini

---

## 🎯 **OBJECTIF**

Documenter les features avancées du SDK Reachy Mini qui sont **disponibles** dans le backend mais **non encore utilisées** dans les modules BBIA pour amélioration future.

---

## 📊 **FEATURES SDK DISPONIBLES**

### **1. Module Media SDK** ⚠️ Non Utilisé

**Status :** Disponible dans `ReachyMiniBackend.media` mais NON INTÉGRÉ

**Capacités :**
```python
robot.media.camera          # Accès direct caméra grand angle
robot.media.microphone      # Accès 4 microphones avec annulation de bruit
robot.media.speaker         # Haut-parleur 5W optimisé hardware
robot.media.play_audio()    # Lecture audio optimisée
robot.media.record_audio() # Enregistrement optimisé
```

**Modules à améliorer :**
- `bbia_vision.py` → Utiliser `robot.media.camera` au lieu de simulation
- `bbia_audio.py` → Utiliser `robot.media.microphone` (4 mics) au lieu de sounddevice
- `bbia_voice.py` → Utiliser `robot.media.speaker` (5W) au lieu de pyttsx3

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

### **3. Techniques d'Interpolation Avancées** ⚠️ Sous-utilisées

**Status :** Implémentées mais LIMITÉES à `MIN_JERK`

**Disponibles :**
```python
InterpolationTechnique.MIN_JERK       # ✅ Utilisé
InterpolationTechnique.LINEAR         # ⚠️ Disponible mais non utilisé
InterpolationTechnique.EASE_IN_OUT    # ⚠️ Disponible mais non utilisé
InterpolationTechnique.CARTOON        # ⚠️ Disponible mais non utilisé (expressif!)
```

**Recommandations :**
- ✅ Utiliser `CARTOON` pour émotions expressives (happy, excited, surprised)
- ✅ Utiliser `EASE_IN_OUT` pour émotions douces (calm, sad, nostalgic)
- ✅ Utiliser `LINEAR` pour mouvements techniques précis
- ✅ Utiliser `MIN_JERK` pour émotions naturelles (neutral, curious)

**Exemple amélioré :**
```python
emotion_interpolation = {
    "happy": "CARTOON",      # Expressif et animé
    "excited": "CARTOON",    # Très expressif
    "surprised": "CARTOON", # Sautillant
    "calm": "EASE_IN_OUT",   # Doux et fluide
    "sad": "EASE_IN_OUT",    # Lent et mélancolique
    "neutral": "MIN_JERK",   # Naturel
    "curious": "MIN_JERK",   # Naturel
}
```

---

### **4. Enregistrement/Replay Avancé** ⚠️ Sous-utilisé

**Status :** Implémenté mais NON UTILISÉ dans comportements

**Disponible :**
```python
robot.start_recording()           # ✅ Disponible
move = robot.stop_recording()      # ✅ Disponible
robot.play_move(move)              # ✅ Disponible
robot.async_play_move(move)        # ✅ Disponible (performance)
```

**Opportunités :**
- ✅ Enregistrer comportements complexes pour réutilisation
- ✅ Créer bibliothèque de mouvements expressifs
- ✅ Utiliser `async_play_move()` pour performance (non bloquant)
- ✅ Valider mouvements enregistrés via tests

**Exemple d'utilisation :**
```python
# Enregistrer comportement greeting
robot.start_recording()
robot.run_behavior("greeting", duration=3.0)
greeting_move = robot.stop_recording()

# Réutiliser plus tard (performance)
robot.async_play_move(greeting_move)
```

---

## 🔧 **PLAN D'IMPLÉMENTATION**

### **Phase 1 : Intégration Media SDK (Haute Priorité)**

1. **`bbia_vision.py`**
   ```python
   def scan_environment(self, robot_api=None):
       if robot_api and robot_api.media and robot_api.media.camera:
           frame = robot_api.media.camera.get_frame()
           return self._detect_objects_real(frame)
       return self._simulate_detection()
   ```

2. **`bbia_audio.py`**
   ```python
   def enregistrer_audio(fichier, robot_api=None):
       if robot_api and robot_api.media and robot_api.media.microphone:
           audio = robot_api.media.microphone.record(...)
           return save_audio_file(fichier, audio)
       return sd.rec(...)  # Fallback
   ```

3. **`bbia_voice.py`**
   ```python
   def dire_texte(texte, robot_api=None):
       if robot_api and robot_api.media and robot_api.media.speaker:
           robot_api.media.speaker.speak(texte)
           return
       return pyttsx3.speak(...)  # Fallback
   ```

### **Phase 2 : Interpolation Adaptative (Moyenne Priorité)**

**Fichier :** `bbia_integration.py`

```python
emotion_interpolation_map = {
    "happy": "CARTOON",
    "excited": "CARTOON",
    "surprised": "CARTOON",
    "calm": "EASE_IN_OUT",
    "sad": "EASE_IN_OUT",
    "neutral": "MIN_JERK",
    "curious": "MIN_JERK",
}

method = emotion_interpolation_map.get(emotion, "MIN_JERK")
robot_api.goto_target(head=pose, body_yaw=yaw, method=method)
```

### **Phase 3 : Enregistrement Comportements (Basse Priorité)**

**Fichier :** `bbia_behavior.py`

```python
class BBIABehaviorManager:
    def __init__(self):
        self.saved_behaviors = {}  # Bibliothèque de mouvements
    
    def record_behavior(self, name: str):
        robot.start_recording()
        self.execute_behavior(name)
        self.saved_behaviors[name] = robot.stop_recording()
    
    def play_saved_behavior(self, name: str):
        if name in self.saved_behaviors:
            robot.async_play_move(self.saved_behaviors[name])
```

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

## ✅ **VALIDATION**

Ces améliorations sont **prêtes pour implémentation** car :
- ✅ Backend expose déjà `robot.media` et `robot.io`
- ✅ Fallbacks gracieux en place
- ✅ Architecture modulaire permet intégration facile

**Recommandation :** Implémenter Phase 1 (Media SDK) pour utiliser pleinement le hardware Reachy Mini.

