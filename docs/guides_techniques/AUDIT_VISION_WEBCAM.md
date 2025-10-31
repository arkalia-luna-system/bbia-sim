# Audit Vision BBIA - Support Webcam USB

**Date**: 2025-01-30  
**Objectif**: Évaluer l'état actuel du système de vision et identifier ce qui manque pour utiliser une webcam USB Logitech MX Brio.

---

## 📊 État actuel du code

### ✅ Ce qui existe déjà

1. **Module BBIAVision** (`src/bbia_sim/bbia_vision.py`)
   - ✅ Support SDK Reachy Mini via `robot.media.camera`
   - ✅ Détection objets (YOLOv8n)
   - ✅ Détection visages (MediaPipe)
   - ✅ Fallback simulation si pas de caméra
   - ✅ Validation robuste formats d'image

2. **Détecteurs**
   - ✅ YOLODetector (`src/bbia_sim/vision_yolo.py`)
   - ✅ MediaPipe Face Detection intégré
   - ✅ OpenCV disponible dans venv-vision-py310

3. **Infrastructure**
   - ✅ Venv séparé `venv-vision-py310` (évite conflits dépendances)
   - ✅ Documentation ENV_PROFILS.md (guide caméra)
   - ✅ Exemples de démo vision

---

## ❌ Ce qui manque pour webcam USB

### 1. Support OpenCV VideoCapture (critique)

**Problème actuel** :
- Le code cherche uniquement `robot.media.camera` (SDK Reachy)
- Si pas de SDK → fallback simulation (pas de vraie caméra)

**Solution nécessaire** :
- Ajouter un fallback OpenCV `cv2.VideoCapture()` dans `BBIAVision._capture_image_from_camera()`
- Permettre configuration device index via variable d'environnement

**Fichier à modifier** : `src/bbia_sim/bbia_vision.py`

### 2. Configuration device index/path

**Manque** :
- Variable d'environnement `BBIA_CAMERA_INDEX` ou `BBIA_CAMERA_DEVICE`
- Détection automatique de la webcam (liste devices disponibles)
- Support chemin de device (ex. `/dev/video0` sur macOS)

**Solution** :
- Ajouter dans `__init__()` de `BBIAVision`
- Variable d'env par défaut : `0` (première caméra USB)

### 3. Script de test webcam

**Manque** :
- Script simple pour vérifier que la webcam fonctionne
- Affichage preview en temps réel
- Test détection YOLO/MediaPipe avec vraie caméra

**Solution** :
- Créer `scripts/test_webcam.py` (simple, rapide)
- Créer `scripts/test_vision_webcam.py` (avec YOLO/MediaPipe)

### 4. Permissions macOS

**Vérification nécessaire** :
- ✅ macOS demande automatiquement permission caméra au premier `cv2.VideoCapture()`
- ⚠️ Vérifier que Terminal/Python a la permission dans Réglages Système

**Documentation** :
- Ajouter section "Permissions macOS" dans guide

---

## 🛠️ Plan d'action

### Étape 1 : Ajouter support OpenCV VideoCapture

**Fichier** : `src/bbia_sim/bbia_vision.py`

**Modifications** :
1. Ajouter attribut `_opencv_camera` dans `__init__()`
2. Initialiser `cv2.VideoCapture(device_index)` si pas de SDK camera
3. Modifier `_capture_image_from_camera()` pour utiliser OpenCV en fallback

**Variable d'environnement** :
- `BBIA_CAMERA_INDEX` : index de la caméra (défaut `0`)
- `BBIA_CAMERA_DEVICE` : chemin du device (optionnel, macOS/Linux)

### Étape 2 : Scripts de test

**Créer** :
1. `scripts/test_webcam_simple.py` : Preview simple (OpenCV)
2. `scripts/test_vision_webcam.py` : Vision complète (YOLO + MediaPipe)

### Étape 3 : Documentation

**Mettre à jour** :
- `docs/guides_techniques/ENV_PROFILS.md` : Section webcam USB
- `README.md` : Ajouter commandes test webcam

---

## 📋 Checklist finale

Avant d'utiliser la Logitech MX Brio :

- [ ] **Code** : Support OpenCV VideoCapture ajouté
- [ ] **Config** : Variable `BBIA_CAMERA_INDEX` fonctionnelle
- [ ] **Test** : Script `test_webcam_simple.py` créé et testé
- [ ] **Permissions** : macOS autorise Terminal/Python à accéder caméra
- [ ] **Venv** : `venv-vision-py310` activé avec `opencv-python` installé
- [ ] **Détection** : YOLO et MediaPipe fonctionnent avec vraie webcam

---

## 🔍 Points techniques

### Logitech MX Brio spécifique

- **Résolution max** : 4K (mais on peut réduire pour performance)
- **Format UVC** : Compatible OpenCV nativement
- **macOS** : Plug-and-play, pas besoin de drivers spéciaux
- **Device index** : Généralement `0` si seule caméra USB connectée

### Performance recommandée

- Résolution : 640x480 ou 1280x720 (pas besoin de 4K pour YOLO)
- FPS : 10-15 fps suffisent pour détection objets/visages
- Format : BGR (OpenCV standard)

---

## 📝 Notes d'implémentation

### Ordre de priorité caméra

1. `robot.media.camera` (SDK Reachy Mini) - si disponible
2. `cv2.VideoCapture(device_index)` (webcam USB) - fallback
3. Simulation - dernier recours

### Gestion erreurs

- Si `cv2.VideoCapture()` échoue → log warning, fallback simulation
- Si permission refusée → message clair pour l'utilisateur
- Si device index invalide → essayer index 0, puis 1, puis simulation

---

## ✅ Après implémentation

Une fois tout ajouté, tu pourras :

1. Brancher la Logitech MX Brio
2. Activer `venv-vision-py310`
3. Lancer `python scripts/test_webcam_simple.py`
4. Voir BBIA détecter objets/visages en temps réel ! 🎉

---

**Prochaines étapes** : Implémenter le support OpenCV VideoCapture.

