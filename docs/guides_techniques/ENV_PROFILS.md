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
- **Webcam USB UVC (recommandé)** : Plug-and-play, compatible OpenCV (ex. Logitech C920, Logitech MX Brio).
  - Configuration : `export BBIA_CAMERA_INDEX=0` (défaut: 0 = première caméra USB)
  - Test rapide : `python scripts/test_webcam_simple.py` (dans venv-vision-py310)
  - Test vision complète : `python scripts/test_vision_webcam.py` (YOLO + MediaPipe)
- **iPad (Appareil photo de continuité)** : Pratique pour apps macOS, moins fiable via OpenCV.
- **Permissions macOS** : Au premier lancement, macOS demande l'autorisation caméra pour Terminal/Python. Autoriser dans Réglages Système > Confidentialité > Caméra.

---

### Profil C – venv voix avancée (optionnel)
- Usage: générer des WAV jolis (pitch/émotions) avec Coqui TTS, sans toucher au venv principal
- Conseils:
  - Créer un venv séparé (ex. `venv-voice`) et y installer `TTS`/`playsound`
  - Générer des fichiers `.wav` puis les jouer via `robot.media.play_audio`
  - Évite les conflits numpy/scipy dans le venv principal

---

### Profil D – DeepFace (optionnel, dans venv-vision-py310)
- Usage: Reconnaissance visage personnalisée + détection émotions
- Installation:
  ```bash
  source venv-vision-py310/bin/activate
  pip install -r requirements/requirements-deepface.txt
  # Ou: pip install deepface onnxruntime
  ```
- Utilisation:
  - Enregistrer personnes: `python scripts/test_deepface.py --register photo.jpg --name Alice`
  - Reconnaître: `python scripts/test_deepface.py --recognize frame.jpg`
  - Détecter émotion: `python scripts/test_deepface.py --emotion photo.jpg`
- Documentation complète: `docs/guides_techniques/DEEPFACE_SETUP.md`

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

### Dépendances Optionnelles - Centralisées

**Vision et Détection :**
- **DeepFace** (reconnaissance visage + émotions) :
  - Fichier : `requirements/requirements-deepface.txt`
  - Installation : `pip install -r requirements/requirements-deepface.txt`
  - Usage : Reconnaissance visage personnalisée, détection émotions (7 émotions)
  - Profil : `venv-vision-py310` (recommandé)
  - Documentation : `docs/guides_techniques/DEEPFACE_SETUP.md`

- **MediaPipe** (détection postures/gestes) :
  - Installation : `pip install mediapipe` (déjà inclus dans venv-vision-py310)
  - Usage : Détection 33 points clés corps, gestes (bras levés, debout/assis)
  - Profil : `venv-vision-py310`
  - Scripts : `scripts/test_pose_detection.py`

- **YOLOv8n** (détection objets) :
  - Installation : `pip install ultralytics` (inclus dans `requirements.txt`)
  - Usage : Détection objets temps réel (COCO dataset)
  - Profil : `venv-vision-py310` (recommandé) ou `venv` principal
  - Intégration : Automatique via `bbia_vision.py`

**IA et LLM :**
- **Hugging Face Transformers** (LLM conversationnel) :
  - Installation : `pip install transformers torch` (déjà présents dans `requirements.txt`)
  - Modèles disponibles :
    - Mistral 7B : `~14GB RAM` (recommandé si RAM suffisante)
    - Llama 3 8B : `~16GB RAM`
    - Phi-2 : `~5GB RAM` (recommandé RPi 5)
    - TinyLlama : `~2GB RAM` (ultra-léger)
  - Activation : `python -c "from bbia_sim.bbia_huggingface import BBIAHuggingFace; hf = BBIAHuggingFace(); hf.enable_llm_chat('phi2')"`
  - Profil : `venv` principal
  - Documentation : Voir README section "IA Avancée"

- **Whisper** (reconnaissance vocale) :
  - Installation : `pip install openai-whisper` (inclus dans `requirements.txt`)
  - Usage : Transcription audio → texte (STT)
  - Profil : `venv` principal
  - Backend : Auto-détection, fallback dummy si non disponible

**TTS Avancé (optionnel) :**
- **KittenTTS / Kokoro / NeuTTS** :
  - Installation : Selon backend choisi
  - Configuration : `export BBIA_TTS_BACKEND=kitten` (ou `kokoro`, `neutts`)
  - Profil : `venv-voice` séparé (recommandé) ou `venv` principal
  - Fallback : `pyttsx3` (toujours disponible)

- **Coqui TTS** :
  - Installation : `pip install TTS` (venv séparé recommandé)
  - Usage : Génération WAV avec pitch/émotions
  - Profil : `venv-voice` séparé (évite conflits numpy)

**Dashboard :**
- **Gradio** (interface no-code) :
  - Fichier : `requirements/requirements-gradio.txt`
  - Installation : `pip install gradio`
  - Usage : `python scripts/dashboard_gradio.py --port 7860`
  - Profil : `venv` principal
  - Fonctionnalités : Upload images, chat, DeepFace registration

**Résumé Tableau :**

| Dépendance | Fichier Requirements | Profil Recommandé | Usage |
|------------|---------------------|-------------------|-------|
| DeepFace | `requirements-deepface.txt` | `venv-vision-py310` | Reconnaissance visage |
| MediaPipe | `requirements.txt` | `venv-vision-py310` | Détection postures |
| YOLOv8n | `requirements.txt` | `venv-vision-py310` | Détection objets |
| Transformers/Torch | `requirements.txt` | `venv` principal | LLM conversationnel |
| Whisper | `requirements.txt` | `venv` principal | Reconnaissance vocale |
| Gradio | `requirements-gradio.txt` | `venv` principal | Dashboard no-code |
| KittenTTS/Kokoro | Variables env | `venv-voice` ou principal | TTS avancé |

> 💡 **Note** : La plupart des dépendances principales sont déjà incluses dans `requirements.txt`. Les dépendances optionnelles sont documentées ici pour faciliter l'installation ciblée selon les besoins.

---

### FAQ
- Pourquoi séparer les venv ?
  - Certains paquets demandent des versions de numpy/scipy incompatibles entre eux (mediapipe vs reachy/others). Les séparer évite de tout casser.
- Puis-je utiliser l’iPad comme caméra ?
  - Oui pour des apps macOS (FaceTime/Zoom). Pour OpenCV/Python, l’USB UVC est plus simple.


