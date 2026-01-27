# 🎤 Migration vers Coqui TTS - Guide Complet

**Dernière mise à jour** : 26 Janvier 2026  
**Version** : 1.0  
**Objectif** : Remplacer pyttsx3 par Coqui TTS pour résoudre blocages macOS

---

## 🎯 Objectifs de la Migration

✅ **Résoudre blocages macOS :**

- Pitch contrôlable (actuellement bloqué)
- Contrôle émotionnel (actuellement inexistant)
- Meilleure qualité vocale

✅ **Maintenir compatibilité :**

- Fallback automatique vers pyttsx3 si Coqui TTS non disponible
- API compatible avec code existant

---

## 📦 Installation

### Option 1 : Installation Complète (Recommandé)

```bash
# Activer venv
source venv/bin/activate

# Installer Coqui TTS + lecteur audio
pip install TTS playsound

# Vérifier installation
python -c "from TTS.api import TTS; import logging; logging.info('✅ Coqui TTS installé')"

```

### Option 2 : Installation Minimale

```bash
pip install TTS
pip install playsound

```

**Note :** Si Coqui TTS n'est pas installé, le module utilise automatiquement pyttsx3 (fallback).

---

## 🚀 Utilisation

### Utilisation Basique (Compatibilité Maximale)

```python
from bbia_sim.bbia_voice_advanced import dire_texte

# Utilisation identique à l'ancien code
dire_texte("Bonjour, je suis BBIA")

```

### Utilisation Avancée (Nouvelles Fonctionnalités)

```python
from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

# Initialiser
voice = BBIAVoiceAdvanced()

# Synthèse avec émotion BBIA
voice.say_with_emotion(
    "Je suis très content de vous voir !",
    emotion="happy",
    intensity=0.8
)

# Synthèse avec contrôle fin
voice.say(
    "Je peux maintenant contrôler le pitch sur macOS !",
    emotion="excited",
    pitch=0.3,  # ✅ Pitch contrôlable !
    speed=1.1,
    volume=0.9
)

# Définir émotion par défaut
voice.set_emotion("curious")
voice.say("Toutes mes futures phrases utiliseront cette émotion")

```

### Intégration avec Émotions Robot

```python
from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced
from bbia_sim.bbia_integration import BBIAIntegration

# Initialiser
voice = BBIAVoiceAdvanced()
bbia = BBIAIntegration()

# Synchroniser voix + mouvements robot
emotion = "happy"
bbia.apply_emotion_to_robot(emotion, intensity=0.8)
voice.say_with_emotion(
    "Je suis heureux !",
    emotion=emotion,
    intensity=0.8
)

```

---

## 🔄 Migration Progressive

Le module `bbia_voice_advanced.py` est **100% compatible** avec l'ancien code :

### Étape 1 : Installation (Sans Casser Rien)

```bash
pip install TTS playsound

```

✅ **Rien ne casse** - L'ancien code continue de fonctionner avec pyttsx3

### Étape 2 : Tester Coqui TTS (Optionnel)

```python
# Tester nouveau module
from bbia_sim.bbia_voice_advanced import dire_texte_advanced

dire_texte_advanced(
    "Test avec Coqui TTS",
    emotion="happy",
    pitch=0.2
)

```

### Étape 3 : Migration Graduelle (Quand Prêt)

Modifier progressivement les appels `dire_texte()` pour utiliser `dire_texte_advanced()` avec émotions.

---

## 🎭 Émotions Supportées

Le module mappe automatiquement les 12 émotions BBIA vers les paramètres vocaux :

| Émotion BBIA | Émotion TTS | Pitch | Description |
|--------------|-------------|-------|-------------|
| `happy` | `happy` | +0.3 | Joyeux, positif |
| `sad` | `sad` | -0.2 | Triste, mélancolique |
| `excited` | `excited` | +0.4 | Excité, énergique |
| `angry` | `angry` | +0.2 | Colère, frustration |
| `curious` | `happy` | +0.1 | Curieux, interrogatif |
| `calm` | `neutral` | -0.1 | Calme, serein |
| `neutral` | `neutral` | 0.0 | Neutre, équilibré |
| `surprised` | `happy` | +0.35 | Surpris, étonné |
| `fear` | `sad` | -0.3 | Peur, anxiété |
| `love` | `happy` | +0.25 | Amour, tendresse |

---

## ⚠️ Limitations Actuelles

1. **Modèles Coqui TTS :**
   - Certains modèles ne supportent pas le paramètre `emotion` directement
   - Le pitch est toujours contrôlable via post-processing
   - Qualité dépend du modèle choisi

2. **Performance :**
   - Génération audio : ~0.5-2 secondes (selon longueur texte)
   - Premier chargement modèle : ~2-5 secondes
   - Fichiers temporaires créés dans `/tmp` (nettoyés automatiquement)

3. **Fallback :**
   - Si Coqui TTS échoue, fallback automatique vers pyttsx3
   - Pas de contrôle pitch/émotion en mode fallback

---

## 🧪 Tests

```bash
# Tester le module
cd /Volumes/T7/bbia-reachy-sim
source venv/bin/activate
python -m bbia_sim.bbia_voice_advanced

```

---

## 📚 Modèles Coqui TTS Disponibles

### Modèles Français Recommandés

1. **`tts_models/fr/css10/vits`** ⭐ (Recommandé)
   - Qualité : Élevée
   - Taille : ~50MB
   - Support émotion : Partiel
   - Support pitch : Oui

2. **`tts_models/multilingual/multi-dataset/your_tts`**
   - Qualité : Élevée
   - Multi-langues : Oui
   - Support clonage : Oui (3 secondes audio)
   - Support émotion : Oui
   - Taille : ~1.5GB

### Liste Complète des Modèles

```python
from TTS.api import TTS

# Lister tous les modèles disponibles
logging.info(TTS.list_models())

```

---

## 🔧 Dépannage

### Problème : "TTS not available"

**Solution :**

```bash
pip install TTS playsound

```

### Problème : "playsound not working on macOS"

**Solution :**

```bash
# Alternative : utiliser pyobjc
pip install pyobjc

```

Ou modifier `bbia_voice_advanced.py` pour utiliser `afplay` (macOS natif) :

```python
import subprocess

subprocess.run(["afplay", str(audio_file)])

```

### Problème : Modèle ne se charge pas

**Solution :**

1. Vérifier connexion Internet (téléchargement modèle)
2. Vérifier espace disque (~500MB-2GB selon modèle)
3. Utiliser cache_dir personnalisé si nécessaire

---

## 📝 Exemple Complet

```python
#!/usr/bin/env python3
"""Exemple complet d'utilisation BBIA Voice Advanced"""

from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

def main():
    # Initialiser
    voice = BBIAVoiceAdvanced(
        model_name="tts_models/fr/css10/vits"
    )

    # Vérifier disponibilité
    if not voice.is_coqui_available():
        logging.warning("⚠️  Coqui TTS non disponible, utilisation fallback")
        return

    logging.info("✅ Coqui TTS disponible")

    # Test émotions
    emotions = ["happy", "sad", "excited", "calm"]
    for emotion in emotions:
        logging.info(f"\n🎭 Test émotion: {emotion}")
        voice.say_with_emotion(
            f"Je ressens l'émotion {emotion}",
            emotion=emotion,
            intensity=0.7
        )

    # Test contrôle pitch
    logging.info("\n🎵 Test contrôle pitch:")
    for pitch in [-0.3, 0.0, 0.3]:
        logging.info(f"  Pitch: {pitch}")
        voice.say(
            f"Voix avec pitch {pitch}",
            pitch=pitch
        )

    logging.info("\n✅ Tests terminés !")

if __name__ == "__main__":
    main()

```

---

## 🎯 Prochaines Étapes

1. ✅ Module créé et testé
2. ⏳ Intégration dans `bbia_behavior.py` (optionnel, fonctionne déjà)
3. ⏳ Intégration dans `bbia_integration.py` (optionnel, fonctionne déjà)
4. ⏳ Ajouter tests unitaires
5. ⏳ Documentation API complète

---

**Status** : ✅ Phase 1 complétée - Module prêt à l'utilisation !

---

**Dernière mise à jour** : 26 Janvier 2026
