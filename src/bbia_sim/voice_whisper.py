#!/usr/bin/env python3
"""
bbia_voice_whisper.py - Module Whisper STT pour BBIA
Intégration Speech-to-Text avec OpenAI Whisper (optionnel)
"""

import logging
import time
from pathlib import Path
from typing import Any, Optional

try:
    import whisper

    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    whisper = None

logger = logging.getLogger(__name__)


class WhisperSTT:
    """Module Speech-to-Text utilisant OpenAI Whisper."""

    def __init__(self, model_size: str = "tiny", language: str = "fr"):
        """
        Initialise le module Whisper STT.

        Args:
            model_size: Taille du modèle ("tiny", "base", "small", "medium", "large")
            language: Langue cible ("fr", "en", "auto")
        """
        self.model_size = model_size
        self.language = language
        self.model = None
        self.is_loaded = False

        if not WHISPER_AVAILABLE:
            logger.warning(
                "⚠️ Whisper non disponible. Fallback vers speech_recognition."
            )
            return

        logger.info(
            f"🎤 Initialisation Whisper STT (modèle: {model_size}, langue: {language})"
        )

    def load_model(self) -> bool:
        """Charge le modèle Whisper."""
        if not WHISPER_AVAILABLE:
            return False

        try:
            logger.info(f"📥 Chargement modèle Whisper {self.model_size}...")
            start_time = time.time()

            self.model = whisper.load_model(self.model_size)

            load_time = time.time() - start_time
            logger.info(f"✅ Modèle Whisper chargé en {load_time:.1f}s")
            self.is_loaded = True
            return True

        except Exception as e:
            logger.error(f"❌ Erreur chargement Whisper: {e}")
            return False

    def transcribe_audio(self, audio_path: str) -> Optional[str]:
        """
        Transcrit un fichier audio en texte.

        Args:
            audio_path: Chemin vers le fichier audio

        Returns:
            Texte transcrit ou None si erreur
        """
        if not self.is_loaded:
            if not self.load_model():
                return None

        try:
            logger.info(f"🎵 Transcription audio: {audio_path}")
            start_time = time.time()

            # Transcription avec Whisper
            result = self.model.transcribe(
                audio_path,
                language=self.language if self.language != "auto" else None,
                fp16=False,  # Éviter les problèmes de compatibilité
            )

            transcription_time = time.time() - start_time
            text = result["text"].strip()

            logger.info(
                f"✅ Transcription terminée en {transcription_time:.1f}s: '{text}'"
            )
            return text

        except Exception as e:
            logger.error(f"❌ Erreur transcription: {e}")
            return None

    def transcribe_microphone(self, duration: float = 3.0) -> Optional[str]:
        """
        Enregistre et transcrit depuis le microphone.

        Args:
            duration: Durée d'enregistrement en secondes

        Returns:
            Texte transcrit ou None si erreur
        """
        try:
            import numpy as np
            import sounddevice as sd
            import soundfile as sf

            logger.info(f"🎤 Enregistrement microphone ({duration}s)...")

            # Enregistrement audio
            sample_rate = 16000  # Whisper recommande 16kHz
            audio_data = sd.rec(
                int(duration * sample_rate),
                samplerate=sample_rate,
                channels=1,
                dtype=np.float32,
            )
            sd.wait()

            # Sauvegarde temporaire
            temp_file = Path("/tmp/bbia_whisper_temp.wav")
            sf.write(temp_file, audio_data, sample_rate)

            # Transcription
            result = self.transcribe_audio(str(temp_file))

            # Nettoyage
            temp_file.unlink(missing_ok=True)

            return result

        except ImportError:
            logger.error(
                "❌ sounddevice/soundfile requis pour l'enregistrement microphone"
            )
            return None
        except Exception as e:
            logger.error(f"❌ Erreur enregistrement microphone: {e}")
            return None


class VoiceCommandMapper:
    """Mappe les commandes vocales vers des actions RobotAPI."""

    def __init__(self):
        """Initialise le mappeur de commandes."""
        self.commands = {
            # Français
            "salue": "greet",
            "salut": "greet",
            "bonjour": "greet",
            "regarde-moi": "look_at",
            "regarde moi": "look_at",
            "sois content": "happy",
            "sois triste": "sad",
            "sois excité": "excited",
            "réveille-toi": "wake_up",
            # Anglais
            "hello": "greet",
            "hi": "greet",
            "look at me": "look_at",
            "be happy": "happy",
            "be sad": "sad",
            "be excited": "excited",
            "wake up": "wake_up",
        }

        logger.info(
            f"🗣️ Mappeur de commandes initialisé ({len(self.commands)} commandes)"
        )

    def map_command(self, text: str) -> Optional[dict[str, Any]]:
        """
        Mappe un texte vers une action RobotAPI.

        Args:
            text: Texte à mapper

        Returns:
            Dictionnaire d'action ou None si non reconnu
        """
        if not text:
            return None

        # Normalisation
        text_lower = text.lower().strip()

        # Recherche exacte
        if text_lower in self.commands:
            action = self.commands[text_lower]
            logger.info(f"🎯 Commande mappée: '{text}' → {action}")
            return {"action": action, "confidence": 1.0}

        # Recherche partielle
        for command, action in self.commands.items():
            if command in text_lower:
                logger.info(f"🎯 Commande partielle mappée: '{text}' → {action}")
                return {"action": action, "confidence": 0.8}

        logger.warning(f"❓ Commande non reconnue: '{text}'")
        return None


def create_whisper_stt(
    model_size: str = "tiny", language: str = "fr"
) -> Optional[WhisperSTT]:
    """
    Factory function pour créer une instance WhisperSTT.

    Args:
        model_size: Taille du modèle Whisper
        language: Langue cible

    Returns:
        Instance WhisperSTT ou None si non disponible
    """
    if not WHISPER_AVAILABLE:
        logger.warning("⚠️ Whisper non disponible")
        return None

    return WhisperSTT(model_size=model_size, language=language)


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("🧪 Test module Whisper STT")
    print("=" * 40)

    # Test disponibilité
    print(f"Whisper disponible: {WHISPER_AVAILABLE}")

    if WHISPER_AVAILABLE:
        # Test création
        stt = create_whisper_stt("tiny", "fr")
        if stt:
            print("✅ Module Whisper créé")

            # Test mappeur
            mapper = VoiceCommandMapper()
            test_commands = [
                "salue",
                "regarde-moi",
                "sois content",
                "commande inconnue",
            ]

            for cmd in test_commands:
                result = mapper.map_command(cmd)
                print(f"  '{cmd}' → {result}")
        else:
            print("❌ Impossible de créer le module Whisper")
    else:
        print("❌ Whisper non installé")
