# 📸 Outils de Calibration Caméra - Reachy Mini

**Dernière mise à jour** : 26 Janvier 2026  
**Compatibilité** : SDK Reachy Mini v1.2.13+ (v1.3.0 recommandée)  
**Version BBIA** : 1.4.0

## 🎯 Vue d'ensemble

Ces outils permettent de calibrer la caméra du Reachy Mini en utilisant des **Charuco boards** (ChArUco = Chessboard + ArUco markers). La calibration est nécessaire pour obtenir des mesures précises et corriger les distorsions de la caméra.

## 📋 Prérequis

```bash
# Installer les dépendances
pip install opencv-python opencv-contrib-python numpy

# Pour utiliser avec le SDK Reachy Mini
pip install reachy-mini>=1.2.13  # v1.3.0 recommandée (5 fév. 2026)
```

## 🛠️ Scripts disponibles

### 1. `acquire.py` - Acquisition d'images

Capture des images du Charuco board depuis la caméra du Reachy Mini.

```bash
python -m bbia_sim.tools.camera_calibration.acquire \
    --output ./calibration_images \
    --count 20 \
    --delay 2.0
```

**Options** :
- `--output` : Dossier de sortie pour les images (défaut: `./calibration_images`)
- `--count` : Nombre d'images à capturer (défaut: 20)
- `--delay` : Délai entre captures en secondes (défaut: 2.0)
- `--resolution` : Résolution caméra (ex: `640x480`, `1280x720`) (défaut: auto)
- `--crop` : Zone de crop (ex: `100,100,640,480`) (optionnel)

### 2. `calibrate.py` - Calibration

Calibre la caméra à partir des images capturées.

```bash
python -m bbia_sim.tools.camera_calibration.calibrate \
    --images ./calibration_images \
    --output ./camera_calibration.json
```

**Options** :
- `--images` : Dossier contenant les images (défaut: `./calibration_images`)
- `--output` : Fichier de sortie pour les paramètres de calibration (défaut: `./camera_calibration.json`)
- `--board-size` : Taille du Charuco board (ex: `7x5`) (défaut: `7x5`)
- `--square-size` : Taille d'un carré en mm (défaut: 20.0)

### 3. `scale_calibration.py` - Calibration d'échelle

Calibre l'échelle pour différentes résolutions avec crop/zoom.

```bash
python -m bbia_sim.tools.camera_calibration.scale_calibration \
    --calibration ./camera_calibration.json \
    --resolution 640x480 \
    --crop 100,100,640,480 \
    --output ./camera_calibration_scaled.json
```

### 4. `visualize_undistorted.py` - Visualisation

Visualise les images corrigées (undistorted) pour vérifier la calibration.

```bash
python -m bbia_sim.tools.camera_calibration.visualize_undistorted \
    --calibration ./camera_calibration.json \
    --image ./test_image.jpg
```

### 5. `analyze_crop.py` - Analyse de crop

Analyse les facteurs de crop pour différentes résolutions.

```bash
python -m bbia_sim.tools.camera_calibration.analyze_crop \
    --calibration ./camera_calibration.json \
    --resolutions 640x480,1280x720,1920x1080
```

## 📐 Charuco Board

Le Charuco board est une combinaison d'un échiquier et de marqueurs ArUco. Pour créer un board :

```python
import cv2
from cv2 import aruco

# Créer un Charuco board 7x5
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
board = aruco.CharucoBoard((7, 5), squareLength=20.0, markerLength=15.0, dictionary=dictionary)

# Générer l'image du board
img = board.generateImage((700, 500))
cv2.imwrite("charuco_board.png", img)
```

## 🔧 Utilisation avec Reachy Mini

### Mode SDK (robot physique)

```python
from reachy_mini import ReachyMini

with ReachyMini() as robot:
    # La caméra est accessible via robot.media.camera
    # Les outils de calibration utilisent automatiquement cette caméra
    pass
```

### Mode simulation

```python
from reachy_mini import ReachyMini

with ReachyMini(use_sim=True) as robot:
    # Utilise la caméra simulée
    pass
```

## 📊 Format de sortie

Le fichier de calibration JSON contient :

```json
{
    "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "distortion_coefficients": [k1, k2, p1, p2, k3],
    "resolution": {"width": 640, "height": 480},
    "reprojection_error": 0.15,
    "timestamp": "2026-01-26T12:00:00"
}
```

## 🔍 Support résolutions multiples

Les outils supportent différentes résolutions avec facteurs de crop/zoom :

- **640x480** : Résolution standard
- **1280x720** : HD
- **1920x1080** : Full HD

Pour chaque résolution, les intrinsics peuvent être mises à l'échelle avec `scale_intrinsics()`.

## ⚠️ Notes importantes

1. **Capture** : Déplacez le Charuco board à différents angles et distances (au moins 10-15 images)
2. **Éclairage** : Utilisez un éclairage uniforme
3. **Stabilité** : Maintenez le board stable pendant la capture
4. **Résolution** : Fermez la caméra avant de changer de résolution (nécessaire pour WebRTC)

## 📚 Références

- [OpenCV Charuco Calibration](https://docs.opencv.org/3.4/da/d13/tutorial_aruco_calibration.html)
- [SDK Reachy Mini v1.3.0](https://github.com/pollen-robotics/reachy_mini) (5 fév. 2026)
- [Documentation Hugging Face](https://huggingface.co/docs/reachy_mini/)

---

**Dernière mise à jour** : 26 Janvier 2026
