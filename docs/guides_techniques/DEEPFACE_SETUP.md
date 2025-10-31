# Guide DeepFace - Reconnaissance Visage Personnalisée + Émotions

**Date** : 2025-01-30  
**Compatibilité** : ✅ SDK Reachy Mini officiel, open-source, gratuit

---

## 🎯 Qu'est-ce que DeepFace ?

**DeepFace** permet à BBIA de :
- ✅ **Reconnaître des personnes spécifiques** (famille, amis) - "Bonjour Alice !"
- ✅ **Détecter les émotions** sur visages (happy, sad, angry, etc.)
- ✅ **Compatible SDK Reachy Mini** (pas de conflit)
- ✅ **Open-source et gratuit** (100% gratuit)

---

## 📦 Installation

### Option 1 : Dans venv-vision-py310 (recommandé)

```bash
# Activer venv vision
source venv-vision-py310/bin/activate

# Installer DeepFace
pip install -r requirements/requirements-deepface.txt

# Ou directement
pip install deepface onnxruntime
```

### Option 2 : Dans venv principal (si pas de conflit)

```bash
source venv/bin/activate
pip install deepface onnxruntime
```

**Note** : `onnxruntime` est recommandé pour Raspberry Pi 5 (plus rapide que TensorFlow)

---

## 🚀 Utilisation Rapide

### 1. Enregistrer une personne (famille, ami)

```bash
# Prendre une photo claire (visage bien visible)
python scripts/test_deepface.py --register photo_alice.jpg --name Alice
```

**Résultat** : La photo est copiée dans `faces_db/Alice/`

### 2. Reconnaître une personne

```bash
# Avec webcam ou image
python scripts/test_deepface.py --recognize frame.jpg
```

**Résultat** :
```
✅ Personne reconnue:
   • Nom: Alice
   • Confiance: 87%
   • Distance: 0.132
```

### 3. Détecter l'émotion

```bash
python scripts/test_deepface.py --emotion photo.jpg
```

**Résultat** :
```
✅ Émotion détectée:
   • Émotion dominante: happy
   • Confiance: 94%

   Scores détaillés:
      • happy: 94.2%
      • neutral: 3.1%
      • sad: 1.2%
      • ...
```

---

## 🔧 Intégration dans BBIA

### Automatique avec BBIAVision

DeepFace est **automatiquement intégré** dans `BBIAVision` :

```python
from bbia_sim.bbia_vision import BBIAVision

vision = BBIAVision()

# Scan environnement (avec DeepFace automatique si disponible)
result = vision.scan_environment()

for face in result["faces"]:
    print(f"Personne: {face['name']}")  # "Alice" au lieu de "humain"
    print(f"Émotion: {face['emotion']}")  # "happy", "sad", etc.
    print(f"Confiance émotion: {face['emotion_confidence']}")
```

**Ce qui se passe** :
1. MediaPipe détecte le visage (rapide)
2. DeepFace reconnaît la personne (si enregistrée)
3. DeepFace détecte l'émotion
4. Résultat enrichi retourné

---

## ⚙️ Configuration

### Variables d'environnement

```bash
# Chemin base de données visages (défaut: faces_db)
export BBIA_FACES_DB="faces_db"

# Modèle DeepFace (défaut: VGG-Face)
# Options: VGG-Face, Facenet, OpenFace, DeepID, ArcFace
export BBIA_DEEPFACE_MODEL="VGG-Face"
```

### Modèles DeepFace

**VGG-Face** (défaut) :
- ✅ Bon équilibre vitesse/précision
- ✅ Compatible Raspberry Pi 5 (avec ONNX)

**ArcFace** (plus précis) :
- ⚠️ Plus lent, mais meilleure précision
- ⚠️ Nécessite plus de RAM

**Recommandation RPi 5** : Garder `VGG-Face` + backend `opencv` + ONNX

---

## 📁 Structure Base de Données

```
faces_db/
├── Alice/
│   ├── photo_alice.jpg
│   └── photo_alice_2.jpg
├── Maman/
│   └── photo_maman.jpg
└── Papa/
    └── photo_papa.jpg
```

**Format** : Un dossier par personne, photos à l'intérieur

---

## 🎯 Cas d'Usage

### Exemple 1 : BBIA reconnaît la famille

```python
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.bbia_voice import dire_texte

vision = BBIAVision()
result = vision.scan_environment()

for face in result["faces"]:
    if face["name"] != "humain":  # Personne reconnue
        dire_texte(f"Bonjour {face['name']} !", robot_api=None)
        
        # Adapter comportement selon émotion
        if face["emotion"] == "happy":
            dire_texte("Tu as l'air heureux aujourd'hui !", robot_api=None)
        elif face["emotion"] == "sad":
            dire_texte("Tu as l'air triste, veux-tu parler ?", robot_api=None)
```

### Exemple 2 : Enregistrer depuis webcam

```python
import cv2
from bbia_sim.face_recognition import create_face_recognition

face_rec = create_face_recognition()

# Capturer photo depuis webcam
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()

# Sauvegarder temporairement
cv2.imwrite("temp_face.jpg", frame)

# Enregistrer
face_rec.register_person("temp_face.jpg", "Alice")
```

---

## 🐛 Dépannage

### Erreur : "DeepFace non disponible"

**Solution** :
```bash
source venv-vision-py310/bin/activate
pip install deepface onnxruntime
```

### Erreur : "No face detected"

**Solutions** :
- Photo trop petite ou floue
- Visage de profil
- Éclairage insuffisant
- Utiliser `--enforce-detection=False` (par défaut)

### Performance lente sur RPi 5

**Solutions** :
- Utiliser backend `opencv` (plus rapide que `retinaface`)
- Utiliser modèle `VGG-Face` (plus léger)
- Installer `onnxruntime` (plus rapide que TensorFlow)

---

## 📚 Références

- **DeepFace GitHub** : https://github.com/serengil/deepface
- **Documentation** : https://github.com/serengil/deepface/blob/master/README.md
- **Modèles disponibles** : VGG-Face, Facenet, OpenFace, DeepID, ArcFace

---

## ✅ Checklist Installation

- [ ] DeepFace installé (`pip install deepface`)
- [ ] ONNX backend installé (`pip install onnxruntime`) - optionnel mais recommandé
- [ ] Base de données créée (`faces_db/`)
- [ ] Au moins une personne enregistrée (`--register`)
- [ ] Test reconnaissance OK (`--recognize`)
- [ ] Test émotion OK (`--emotion`)

**Une fois tout ça fait, BBIA peut reconnaître ta famille et leurs émotions !** 🎉

