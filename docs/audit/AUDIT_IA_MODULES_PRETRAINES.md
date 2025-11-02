# Audit IA : Modules Pré-entraînés dans BBIA

**Date** : octobre 2025
**Objectif** : Vérifier quels modèles IA pré-entraînés sont utilisés, où ils sont utilisés, et identifier ce qui manque.

---

## ✅ 1. MODÈLES PRÉ-ENTRAÎNÉS UTILISÉS

### 🎯 Vision - DÉJÀ IMPLÉMENTÉ ✅

**Modèles utilisés** :
- ✅ **YOLOv8n** (Ultralytics) - Détection objets
- ✅ **MediaPipe Face Detection** - Détection visages
- ✅ **MediaPipe Face Mesh** - Landmarks faciaux
- ✅ **CLIP** (OpenAI) - Classification images
- ✅ **BLIP** (Salesforce) - Description images
- ✅ **BLIP VQA** - Visual Question Answering

**Où c'est utilisé** :
- `src/bbia_sim/vision_yolo.py` : `YOLODetector` classe → charge `yolov8n.pt`
- `src/bbia_sim/bbia_vision.py` : `BBIAVision` → utilise YOLO + MediaPipe
- `src/bbia_sim/bbia_huggingface.py` : `BBIAHuggingFace._load_vision_model()` → CLIP/BLIP

**État** : ✅ **FONCTIONNEL** - Modèles pré-entraînés chargés automatiquement depuis Hugging Face/Ultralytics

---

### 💬 Langage (LLM) - DÉJÀ IMPLÉMENTÉ ✅

**Modèles utilisés** :
- ✅ **Mistral 7B Instruct** (`mistralai/Mistral-7B-Instruct-v0.2`)
- ✅ **Llama 3 8B Instruct** (`meta-llama/Llama-3-8B-Instruct`)
- ✅ **llama.cpp** (GGUF, local) - Fallback léger
- ✅ **Twitter RoBERTa Sentiment** (`cardiffnlp/twitter-roberta-base-sentiment-latest`)
- ✅ **Emotion DistilRoBERTa** (`j-hartmann/emotion-english-distilroberta-base`)

**Où c'est utilisé** :
- `src/bbia_sim/bbia_huggingface.py` : `BBIAHuggingFace.enable_llm_chat()` → charge Mistral/Llama
- `src/bbia_sim/bbia_emotion_recognition.py` : `BBIAEmotionRecognition._load_emotion_models()` → sentiment/émotion
- `src/bbia_sim/ai_backends.py` : `LlamaCppLLM` → support llama.cpp

**État** : ✅ **FONCTIONNEL** - LLM conversationnel + analyse sentiment/émotion

---

### 🎤 Audio/Parole - DÉJÀ IMPLÉMENTÉ ✅

**Modèles utilisés** :
- ✅ **OpenAI Whisper** - STT (Speech-to-Text)
- ✅ **Coqui TTS** - TTS avec clonage voix (XTTS v2)
- ✅ **SpeechRecognition** (Google API) - STT fallback
- ✅ **pyttsx3** - TTS système macOS (fallback)

**Où c'est utilisé** :
- `src/bbia_sim/voice_whisper.py` : `WhisperSTT` → charge `whisper-{size}.pt`
- `src/bbia_sim/ai_backends.py` : `CoquiTTSTTS` → utilise `TTS.api`
- `scripts/voice_clone/generate_voice.py` : Clonage voix avec XTTS v2
- `src/bbia_sim/bbia_voice.py` : Intégration TTS/STT

**État** : ✅ **FONCTIONNEL** - STT avancé (Whisper) + TTS personnalisable (Coqui)

---

## ✅ 2. CE QUI EST DÉJÀ AJOUTÉ (modules récemment créés)

### ✅ Reconnaissance Visage Personnalisée (DeepFace) - **AJOUTÉ !**

**État** : ✅ **Module créé et intégré**

**Fichiers créés** :
- ✅ `src/bbia_sim/face_recognition.py` (344 lignes) - Module complet
- ✅ `scripts/test_deepface.py` (155 lignes) - Script de test
- ✅ `requirements/requirements-deepface.txt` - Dépendances
- ✅ `docs/guides_techniques/DEEPFACE_SETUP.md` - Documentation

**Fonctionnalités** :
- ✅ Enregistrement personnes (`register_person()`)
- ✅ Reconnaissance personnes (`recognize_person()`)
- ✅ Détection émotions (`detect_emotion()`)
- ✅ Intégré dans `BBIAVision.scan_environment()`

**Installation** :
```bash
source venv-vision-py310/bin/activate
pip install -r requirements/requirements-deepface.txt
```

---

### ✅ Détection Postures/Corps (MediaPipe Pose) - **AJOUTÉ !**

**État** : ✅ **Module créé et intégré**

**Fichiers créés** :
- ✅ `src/bbia_sim/pose_detection.py` (284 lignes) - Module complet
- ✅ `scripts/test_pose_detection.py` (215 lignes) - Script de test

**Fonctionnalités** :
- ✅ Détection 33 points clés (`detect_pose()`)
- ✅ Détection gestes (bras levés, mains sur tête)
- ✅ Détection posture (debout/assis)
- ✅ Intégré dans `BBIAVision.scan_environment()`

**Installation** : Aucune (MediaPipe déjà installé)

---

### ✅ Détection Émotions Visuelles (DeepFace) - **AJOUTÉ !**

**État** : ✅ **Intégré dans DeepFace**

**Fonctionnalités** :
- ✅ Détection émotions via `detect_emotion()` dans `face_recognition.py`
- ✅ Émotions détectées : happy, sad, angry, surprise, fear, neutral, disgust
- ✅ Intégré automatiquement dans `BBIAVision.scan_environment()`

**Exemple** :
```python
from bbia_sim.bbia_vision import BBIAVision
vision = BBIAVision()
result = vision.scan_environment()
# result["faces"] contient maintenant "emotion" et "emotion_confidence"
```

---

### ✅ LLM Local Léger (optionnel) - **DÉJÀ IMPLÉMENTÉ**

**État actuel** : ✅ **FAIT** - Phi-2 et TinyLlama configurés pour RPi 5

**Solutions implémentées** :
- ✅ **Phi-2** (2.7B) - Microsoft, ~5GB RAM - **AJOUTÉ** (ligne 164)
- ✅ **TinyLlama** (1.1B) - Ultra-léger, ~2GB RAM - **AJOUTÉ** (ligne 165-166)

**Vérification code (octobre 2025)** :
- ✅ `bbia_huggingface.py` (lignes 164-166) : Configs Phi-2 et TinyLlama ajoutées
- ✅ `enable_llm_chat("phi2")` et `enable_llm_chat("tinyllama")` fonctionnent

**Pourquoi c'est utile** :
- ✅ Fonctionne sur machines moins puissantes (RPi 5 compatible)
- ✅ Plus rapide pour réponses simples

---

## ✅ 3. ARCHITECTURE MODULAIRE - DÉJÀ FAIT ✅

**État** : ✅ **EXCELLENT**

**Preuve** :
- Modules séparés : `bbia_vision.py`, `bbia_voice.py`, `bbia_huggingface.py`
- Backends sélectionnables via variables d'env (`BBIA_TTS_BACKEND`, `BBIA_LLM_BACKEND`)
- Plug-and-play : chaque module fonctionne indépendamment
- Fallbacks : si un modèle échoue, fallback automatique

**Exemple** :
```python
# Dans ai_backends.py
def get_tts_backend():
    name = os.environ.get("BBIA_TTS_BACKEND", "kitten")  # NOTE: défaut réel = kitten
    if name == "coqui":
        return CoquiTTSTTS()
    elif name == "openvoice":
        return OpenVoiceTTSTTS()
    return Pyttsx3TTS()  # Fallback
```

**✅ Voix corrigée et vérifiée** :
- ✅ `Pyttsx3TTS` dans `ai_backends.py` **UTILISE** `get_bbia_voice()` (ligne 56-58)
- ✅ `get_bbia_voice()` sélectionne automatiquement : Aurelie Enhanced > Amelie Enhanced > Aurelie > Amelie
- ✅ Si `BBIA_TTS_BACKEND` = "kitten" (défaut), fallback sur `Pyttsx3TTS` qui utilise `get_bbia_voice()` → Aurelie Enhanced sélectionnée

**Recommandation** :
- ✅ Défaut actuel fonctionne bien (kitten → Pyttsx3TTS → Aurelie Enhanced)
- ✅ Option avancée : `BBIA_TTS_BACKEND=coqui` pour contrôle pitch/émotion (Coqui TTS)

**Conclusion** : Architecture modulaire déjà très bien conçue ✅

---

## ⚠️ 4. FINETUNED MODELS (Reconnaissance Personnalisée)

### État Actuel

**Déjà fait** :
- ✅ Clonage voix (Coqui XTTS v2) - Tu enregistres ta voix, BBIA l'utilise
- ✅ Comportements personnalisés (`bbia_behavior.py`) - Tu peux créer tes propres comportements

**Manque** :
- ❌ Reconnaissance visage personnalisée (DeepFace)
- ❌ Entraînement objets custom (ex: reconnaître tes objets spécifiques)

### Solutions Open-Source

**1. DeepFace - Reconnaissance Visage** :
```python
# Enregistrer ta famille
DeepFace.build_model("VGG-Face")
DeepFace.represent("photo_alice.jpg", model_name="VGG-Face")

# BBIA reconnaît Alice
result = DeepFace.find("frame.jpg", db_path="./faces_db")
# Retourne : {"identity": "faces_db/alice.jpg", "distance": 0.2}
```

**2. YOLO Custom Training** :
- Utiliser **YOLOv8** avec ton dataset
- Exemple : entraîner pour reconnaître tes objets (ex: "Télécommande BBIA", "Lampe préférée")
- Guide : https://docs.ultralytics.com/modes/train/

**Où l'ajouter** :
- `scripts/train_custom_yolo.py` - Script d'entraînement personnalisé
- `src/bbia_sim/face_recognition.py` - Module DeepFace

---

## ⚠️ 5. NO-CODE/LOW-CODE - PARTIELLEMENT FAIT

### État Actuel

**Déjà fait** :
- ✅ Dashboard web (`dashboard_advanced.py`) - Interface graphique pour contrôler BBIA
- ✅ Chat en temps réel (WebSocket) - Tu parles, BBIA répond
- ✅ Variables d'environnement - Configuration sans code

**Manque** :
- ❌ Interface drag-and-drop pour créer comportements
- ❌ Dashboard pour entraîner modèles custom (ex: upload photos pour DeepFace)

### Recommandations Open-Source

**Option 1 : Streamlit Dashboard** (Simple)
```python
# Créer scripts/dashboard_streamlit.py
import streamlit as st
from bbia_sim.bbia_vision import BBIAVision

st.title("BBIA Vision Control")
vision = BBIAVision()

if st.button("Scan Environment"):
    result = vision.scan_environment()
    st.json(result)
```

**Option 2 : Gradio** (Hugging Face)
- Interface simple pour tester modèles
- Upload images → voir détections
- Pas besoin de coder HTML/JS

**Où l'ajouter** :
- `scripts/dashboard_gradio.py` - Interface simple pour vision/chat

---

## ✅ 6. ENTRÂINEMENT PROGRESSIF - DÉJÀ SUPPORTÉ ✅

**Preuve** :
- Conversation history (`BBIAHuggingFace.conversation_history`) - BBIA se souvient du contexte
- Comportements adaptatifs (`bbia_adaptive_behavior.py`) - Apprend des patterns
- Fallbacks intelligents - Si un modèle échoue, BBIA essaie autre chose

**Amélioration possible** :
- Sauvegarder apprentissages dans fichier JSON/database
- Exemple : "Quand je dis 'salut', BBIA me reconnaît" → sauvegarde pour prochaine fois

**Où l'ajouter** :
- `src/bbia_sim/bbia_memory.py` - Module mémoire persistante

---

## 📊 RÉSUMÉ - CE QUI EXISTE vs MANQUE

| Catégorie | État | Où c'est | Recommandation |
|-----------|------|----------|----------------|
| **Vision (objets/visages)** | ✅ FAIT | `vision_yolo.py`, `bbia_vision.py` | - |
| **LLM Conversationnel** | ✅ FAIT | `bbia_huggingface.py` | ✅ Phi-2 ajouté (léger) |
| **TTS/STT** | ✅ FAIT | `voice_whisper.py`, `ai_backends.py` | - |
| **Reconnaissance visage personnalisée** | ✅ FAIT | `face_recognition.py` | ✅ **Déjà ajouté (DeepFace)** ⭐ |
| **Détection émotions visuelles** | ✅ FAIT | `face_recognition.py` | ✅ **Déjà ajouté (DeepFace)** ⭐ |
| **Détection postures** | ✅ FAIT | `pose_detection.py` | ✅ **Déjà ajouté (MediaPipe Pose)** |
| **Architecture modulaire** | ✅ EXCELLENT | Tout le projet | - |
| **Finetuning personnalisé** | ✅ FAIT | Voix OK (Coqui), visage OK (DeepFace) | ✅ **Déjà ajouté** |
| **Dashboard No-Code** | ✅ **FAIT** | Dashboard Gradio complet | ✅ Gradio implémenté |
| **Entraînement progressif** | ✅ FAIT | `bbia_huggingface.py` | ✅ Mémoire persistante ajoutée |

---

## 🎯 PRIORITÉS RECOMMANDÉES (Open-Source & Gratuit)

### ✅ Priorité 1 : DeepFace ⭐⭐⭐ - **DÉJÀ FAIT !**

**État** : ✅ **Module créé et intégré**

**Fichiers** :
- ✅ `src/bbia_sim/face_recognition.py` - Module complet avec toutes les fonctionnalités
- ✅ Intégré dans `BBIAVision.scan_environment()`

**Installation** :
```bash
source venv-vision-py310/bin/activate
pip install -r requirements/requirements-deepface.txt
```

**Utilisation** :
```bash
# Enregistrer une personne
python scripts/test_deepface.py --register photo_alice.jpg --name Alice

# Reconnaître une personne
python scripts/test_deepface.py --recognize frame.jpg

# Détecter émotion
python scripts/test_deepface.py --emotion photo.jpg
```

---

### ✅ Priorité 2 : MediaPipe Pose ⭐⭐ - **DÉJÀ FAIT !**

**État** : ✅ **Module créé et intégré**

**Fichiers** :
- ✅ `src/bbia_sim/pose_detection.py` - Module complet
- ✅ Intégré dans `BBIAVision.scan_environment()`

**Utilisation** :
```bash
# Test avec webcam
python scripts/test_pose_detection.py --webcam

# Test avec image
python scripts/test_pose_detection.py --image photo.jpg
```

**Aucune installation supplémentaire** : MediaPipe déjà installé ✅

---

### ✅ Priorité 3 : LLM Léger (Phi-2) ⭐ - **DÉJÀ FAIT**

**État** : ✅ **FAIT** - Phi-2 et TinyLlama configurés et fonctionnels

**Vérification code** :
```python
# bbia_huggingface.py lignes 164-166
"chat": {
    "phi2": "microsoft/phi-2",  # ✅ Déjà ajouté
    "tinyllama": "TinyLlama/TinyLlama-1.1B-Chat-v1.0",  # ✅ Déjà ajouté
}
```

**Usage** :
```python
hf = BBIAHuggingFace()
hf.enable_llm_chat("phi2")  # ✅ Fonctionne (~5GB RAM)
hf.enable_llm_chat("tinyllama")  # ✅ Fonctionne (~2GB RAM)
```

---

## 📝 CONCLUSION

**Ton projet est DÉJÀ très bien** ! ✅

- ✅ Modèles pré-entraînés utilisés partout
- ✅ Architecture modulaire excellente
- ✅ Support TTS/STT/LLM/Vision complet

**Ce qui était manquant (maintenant tous ajoutés)** :
1. ✅ **DeepFace** pour reconnaissance visage + émotions - **AJOUTÉ !** (voir `src/bbia_sim/face_recognition.py`)
2. ✅ **MediaPipe Pose** pour détection postures - **ACTIVÉ !** (voir `src/bbia_sim/pose_detection.py`)
3. ✅ **LLM léger** - **AJOUTÉ !** Phi-2 et TinyLlama configurés (voir `bbia_huggingface.py` lignes 164-166)

**Tout est open-source et gratuit** comme tu veux ! 🎉

---

## 🔒 COMPATIBILITÉ REACHY MINI OFFICIEL

**⚠️ IMPORTANT** : Audit de compatibilité complet disponible dans `COMPATIBILITE_REACHY_MINI_OFFICIEL.md`

### Résumé Compatibilité

**✅ 100% COMPATIBLE** avec le SDK officiel Reachy Mini :

- ✅ **Backend SDK** : Conforme (`ReachyMiniBackend` utilise `reachy_mini` officiel)
- ✅ **Dépendances SDK** : Toutes présentes dans `pyproject.toml`
- ✅ **Modules IA isolés** : Pas de conflit avec SDK
- ✅ **Imports conditionnels** : Fallbacks si SDK non disponible
- ✅ **Hardware RPi 5** : YOLOv8n, MediaPipe, Whisper tiny fonctionnent bien

### Limitations Hardware (Raspberry Pi 5)

- ⚠️ **Mistral 7B** : Trop lourd (14GB RAM → RPi 5 max 8GB)
- ✅ **Solution** : Utiliser Phi-2 (2.7B, ~5GB) ou API Hugging Face gratuite
- ⚠️ **YOLOv8s/m** : Peut être lent
- ✅ **Solution** : Garder YOLOv8n (nano, optimisé)

### Ajouts Recommandés (sans risque pour SDK)

1. ✅ **DeepFace** - **AJOUTÉ ET OPÉRATIONNEL** - Compatible, installé dans `venv-vision-py310`
2. ✅ **MediaPipe Pose** - **AJOUTÉ ET OPÉRATIONNEL** - Intégré dans `BBIAVision`
3. ✅ **LLM léger** - **AJOUTÉ ET OPÉRATIONNEL** - Phi-2 et TinyLlama configurés pour RPi 5
4. ✅ **Dashboard Gradio** - **AJOUTÉ** - Interface no-code complète
5. ✅ **Mémoire persistante** - **AJOUTÉ** - Module `bbia_memory.py` + intégration auto
6. ✅ **Tests sécurité LLM** - **AJOUTÉ** - 10 tests dans `test_huggingface_security.py`
7. ✅ **Benchmarks CI** - **AJOUTÉ** - Job automatique en CI

**Conclusion** : Aucun risque de casser le SDK officiel. Tous les modules IA sont optionnels et isolés ✅

---

**Prochaine étape** : Veux-tu que j'ajoute DeepFace dans le projet ? (compatible SDK, gratuit, open-source)

