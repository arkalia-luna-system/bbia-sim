## Profils d'environnement (venv) et caméra – Guide rapide

### Objectif
- Séparer les outils pour éviter les conflits de dépendances (numpy, mediapipe, reachy, TTS avancé).
- Avoir une vision qui marche tout de suite, sans casser le reste.

---

### Profil A – venv principal (simulation/hardware)
- Emplacement: `venv`
- Usage: simulation MuJoCo, API, dashboard, intégration Reachy (hardware mock/réel), chat (HF si dispo)
- Commandes utiles:
  - Activer: `source venv/bin/activate`
  - Démarrer dashboard: `python src/bbia_sim/dashboard_advanced.py --port 8000`
  - Démo 3D (macOS): `./LANCE_DEMO_3D.sh` (utilise `mjpython`)

Notes:
- N’installer ni Coqui TTS ni mediapipe ici pour éviter conflits.

---

### Profil B – venv vision (recommandé)
- Emplacement: `venv-vision-py310`
- Usage: vision temps réel (MediaPipe, YOLO, OpenCV)
- Création (déjà fait):
  - `python -m venv venv-vision-py310`
  - `source venv-vision-py310/bin/activate`
  - `pip install numpy==1.26.4 matplotlib==3.8.4 opencv-python mediapipe ultralytics`
- Nettoyage macOS (si erreurs Matplotlib sur fichiers ._*):
  - `find venv-vision-py310/lib/python3.10/site-packages/matplotlib/mpl-data/stylelib -name '._*.mplstyle' -delete`
- Test:
  - `python -c "import mediapipe, cv2; print('VISION OK')"`

Caméra:
- Recommandée: webcam USB UVC (ex. Logitech C920) → plug-and-play, compatible OpenCV.
- iPad (Appareil photo de continuité): pratique pour apps macOS, moins fiable via OpenCV.

---

### Profil C – venv voix avancée (optionnel)
- Usage: générer des WAV jolis (pitch/émotions) avec Coqui TTS, sans toucher au venv principal
- Conseils:
  - Créer un venv séparé (ex. `venv-voice`) et y installer `TTS`/`playsound`
  - Générer des fichiers `.wav` puis les jouer via `robot.media.play_audio`
  - Évite les conflits numpy/scipy dans le venv principal

---

### Lancer les démos sans conflit
- Simulation 3D (profil A):
  - `source venv/bin/activate`
  - `./LANCE_DEMO_3D.sh` (macOS: ouvre le viewer via `mjpython`)
- Vision seule (profil B):
  - `source venv-vision-py310/bin/activate`
  - Script vision/abonnements/analyses (selon besoins)

---

### Activer l'intelligence (LLM) – optionnel
- Prérequis (profil A): `transformers`, `torch` installés (déjà présents)
- Exemple d’activation ponctuelle:
```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace
bbia = BBIAHuggingFace()
bbia.enable_llm_chat()  # Télécharge/charge le LLM (internet requis au premier run)
```

---

### FAQ
- Pourquoi séparer les venv ?
  - Certains paquets demandent des versions de numpy/scipy incompatibles entre eux (mediapipe vs reachy/others). Les séparer évite de tout casser.
- Puis-je utiliser l’iPad comme caméra ?
  - Oui pour des apps macOS (FaceTime/Zoom). Pour OpenCV/Python, l’USB UVC est plus simple.


