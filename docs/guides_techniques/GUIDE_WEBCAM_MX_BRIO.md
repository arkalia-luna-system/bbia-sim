# 📹 Guide Webcam Logitech MX Brio - Démarrage Rapide

**Date** : 2025-01-30  
**Webcam** : Logitech MX Brio

---

## 🚀 ÉTAPE 1 : Vérifier que la webcam est branchée

1. **Brancher** la Logitech MX Brio sur ton Mac Mini (USB-C ou USB-A)
2. **Vérifier** dans macOS :
   - Ouvrir **Réglages Système** > **Confidentialité et sécurité** > **Caméra**
   - S'assurer que Terminal (ou Python) a la permission caméra

---

## 🔧 ÉTAPE 2 : Activer le bon environnement

```bash
# Activer le venv vision (où OpenCV et MediaPipe sont installés)
source venv-vision-py310/bin/activate

# Vérifier que OpenCV est installé
python -c "import cv2; print(f'✅ OpenCV version: {cv2.__version__}')"
```

**Si erreur** :
```bash
# Installer OpenCV si manquant
pip install opencv-python
```

---

## 🎥 ÉTAPE 3 : Test simple (preview webcam)

**Test basique pour voir si la webcam fonctionne** :

```bash
# Dans venv-vision-py310
export BBIA_CAMERA_INDEX=0  # Défaut: 0 (première caméra USB)
python scripts/test_webcam_simple.py
```

**Tu devrais voir** :
- ✅ Une fenêtre avec le flux vidéo de ta webcam
- ✅ Un compteur de frames en haut
- ✅ Appuie sur `q` pour quitter
- ✅ Appuie sur `s` pour sauvegarder une capture

**Si ça ne marche pas** :
- Vérifie les permissions macOS (Réglages Système > Confidentialité > Caméra)
- Essaie un autre index : `export BBIA_CAMERA_INDEX=1` puis relance

---

## 🔍 ÉTAPE 4 : Test vision complète (détection objets + visages)

**Test avec YOLO + MediaPipe (détection en temps réel)** :

```bash
# Toujours dans venv-vision-py310
python scripts/test_vision_webcam.py
```

**Tu devrais voir** :
- ✅ Fenêtre avec flux vidéo
- ✅ **Objets détectés** : rectangles verts avec noms (person, phone, cup, etc.)
- ✅ **Visages détectés** : rectangles bleus avec confiance
- ✅ Appuie sur `q` pour quitter

**Si YOLO n'est pas installé** :
```bash
pip install ultralytics
```

**💡 Pour améliorer la détection** :
- Assure-toi d'être bien éclairé
- Réduis la distance (pas besoin d'être très loin)
- Le seuil de confiance est maintenant à 0.25 (au lieu de 0.5) pour détecter plus d'objets

---

## 👤 ÉTAPE 5 : Test DeepFace (reconnaissance visage personnalisée)

**Optionnel** : Si tu veux que BBIA reconnaisse des personnes spécifiques :

```bash
# Dans venv-vision-py310
# Installer DeepFace si pas encore fait
pip install -r requirements/requirements-deepface.txt
# Ou: pip install deepface onnxruntime

# Enregistrer une personne
python scripts/test_deepface.py --register photo.jpg --name "Alice"
# (photo.jpg = chemin vers photo de la personne)

# Tester reconnaissance depuis webcam
python scripts/test_vision_webcam.py
# (BBIA dira "Alice" si elle est détectée)
```

---

## 🧍 ÉTAPE 6 : Test détection postures/gestes (MediaPipe Pose)

**Détecter ta posture et tes gestes** :

```bash
# Dans venv-vision-py310
python scripts/test_pose_detection.py --webcam
```

**Tu devrais voir** :
- ✅ Flux vidéo avec squelette dessiné sur ton corps
- ✅ Détection gestes (main levée, debout, assis, etc.)
- ✅ Appuie sur `q` pour quitter

---

## 🎨 ÉTAPE 7 : Dashboard Gradio (interface simple)

**Interface web simple pour tester tout** :

```bash
# Dans venv-vision-py310 (ou venv principal si gradio installé)
# Installer Gradio (séparer la commande du commentaire !)
pip install gradio

# Lancer le dashboard
python scripts/dashboard_gradio.py --port 7860
```

**Ensuite** :
1. Ouvrir ton navigateur : `http://127.0.0.1:7860`
2. Onglet **📷 Vision** : Upload une image → voir détections
3. Onglet **💬 Chat** : Chat avec BBIA
4. Onglet **👤 DeepFace** : Upload photo + nom → enregistrer personne

---

## 🤖 ÉTAPE 8 : Utiliser la webcam avec BBIA complète

**Dans ton code Python** :

```python
from bbia_sim.bbia_vision import BBIAVision
import os

# Configurer index caméra (optionnel, défaut: 0)
os.environ["BBIA_CAMERA_INDEX"] = "0"

# Initialiser vision
vision = BBIAVision()

# Scanner l'environnement (utilise automatiquement la webcam)
result = vision.scan_environment()

print(f"Objets détectés: {result.get('objects')}")
print(f"Visages détectés: {result.get('faces')}")
print(f"Postures détectées: {result.get('poses')}")
```

**La webcam sera utilisée automatiquement** si :
- Pas de robot Reachy connecté
- Pas de SDK camera disponible
- OpenCV disponible dans le venv

---

## 🔧 Configuration avancée

### Améliorer la détection

**Problème** : Pas assez de détections ?

1. **Réduire le seuil de confiance** :
   ```python
   # Dans ton code, créer détecteur avec seuil plus bas
   from bbia_sim.vision_yolo import YOLODetector
   detector = YOLODetector(confidence_threshold=0.15)  # Plus sensible
   ```

2. **Meilleure éclairage** :
   - S'assurer d'être bien éclairé
   - Éviter contre-jour

3. **Distance optimale** :
   - 1-2 mètres de la caméra

### Changer l'index de la caméra

```bash
# Si tu as plusieurs caméras, essaie différents index
export BBIA_CAMERA_INDEX=0  # Première caméra (défaut)
export BBIA_CAMERA_INDEX=1  # Deuxième caméra
export BBIA_CAMERA_INDEX=2  # Troisième caméra
```

### Résolution personnalisée

Par défaut, BBIA utilise 640x480 pour performance. Pour changer (dans code) :

```python
# Dans test_webcam_simple.py ou test_vision_webcam.py
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
```

---

## ⚠️ Dépannage

### "Erreur détection YOLO: 'list' object has no attribute 'get'"

**✅ CORRIGÉ !** Cette erreur est maintenant résolue. Si tu la vois encore :
1. Relance le script après mise à jour
2. Vérifie que tu utilises la dernière version du code

### "Impossible d'ouvrir la webcam"

1. **Permissions macOS** :
   - Réglages Système > Confidentialité > Caméra
   - Autoriser Terminal ou Python

2. **Webcam branchée ?**
   ```bash
   # Lister caméras disponibles (macOS)
   system_profiler SPCameraDataType
   ```

3. **Index incorrect ?**
   - Essaie `BBIA_CAMERA_INDEX=0`, puis `1`, puis `2`

### "OpenCV non disponible"

```bash
source venv-vision-py310/bin/activate
pip install opencv-python
```

### "YOLO/MediaPipe non disponible"

```bash
source venv-vision-py310/bin/activate
pip install ultralytics mediapipe
```

### "Gradio : Invalid requirement: '#'"

**Ne pas copier-coller le commentaire !** Faire :

```bash
pip install gradio
python scripts/dashboard_gradio.py --port 7860
```

---

## ✅ Checklist rapide

- [ ] Webcam branchée et reconnue par macOS
- [ ] Permissions caméra accordées (Réglages Système)
- [ ] `venv-vision-py310` activé
- [ ] `test_webcam_simple.py` fonctionne (preview)
- [ ] `test_vision_webcam.py` fonctionne (détection) ✅ **CORRIGÉ**
- [ ] DeepFace installé (optionnel)
- [ ] Dashboard Gradio testé (optionnel)

---

## 🎯 Prochaines étapes

Une fois que la webcam fonctionne :

1. **Test avec robot BBIA** : Utiliser la webcam pour que BBIA "te voie"
2. **Enregistrer ta famille** : DeepFace pour reconnaître les personnes
3. **Tester postures** : MediaPipe Pose pour détecter gestes
4. **Dashboard interactif** : Utiliser Gradio pour interface simple

---

## 📝 Notes de performance

**Détection améliorée** :
- ✅ Seuil de confiance réduit à 0.25 (au lieu de 0.5) pour plus de détections
- ✅ Format YOLO corrigé (bbox est maintenant correctement converti)
- ✅ Détection toutes les 3 frames (au lieu de 5) pour meilleure réactivité

**Si détection toujours faible** :
- Vérifie l'éclairage
- Réduis la distance
- Augmente la confiance : `YOLODetector(confidence_threshold=0.15)`

---

**Tout est prêt ! Tu peux maintenant brancher ta webcam et la tester ! 🎉**
