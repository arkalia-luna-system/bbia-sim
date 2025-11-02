#!/usr/bin/env python3
"""
bbia_voice_whisper.py - Module Whisper STT pour BBIA
Intégration Speech-to-Text avec OpenAI Whisper (optionnel)
"""

import logging
import os
import time
from pathlib import Path
from typing import Any, cast

try:
    import whisper

    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    whisper = None

logger = logging.getLogger(__name__)


class WhisperSTT:
    """Module Speech-to-Text utilisant OpenAI Whisper."""

    def __init__(
        self, model_size: str = "tiny", language: str = "fr", enable_vad: bool = True
    ):
        """
        Initialise le module Whisper STT.

        Args:
            model_size: Taille du modèle ("tiny", "base", "small", "medium", "large")
            language: Langue cible ("fr", "en", "auto")
            enable_vad: Activer la détection d'activité vocale (VAD) pour activation auto
        """
        self.model_size = model_size
        self.language = language
        self.model = None
        self.is_loaded = False
        self.enable_vad = enable_vad
        self._vad_model: Any | None = None
        self._vad_loaded = False

        if not WHISPER_AVAILABLE:
            logger.warning(
                "⚠️ Whisper non disponible. Fallback vers speech_recognition."
            )
            return

        logger.info(
            f"🎤 Initialisation Whisper STT (modèle: {model_size}, langue: {language}, VAD: {enable_vad})"
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

    def transcribe_audio(self, audio_path: str) -> str | None:
        """
        Transcrit un fichier audio en texte.

        Args:
            audio_path: Chemin vers le fichier audio

        Returns:
            Texte transcrit ou None si erreur
        """
        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

        # Charger le modèle si nécessaire
        if not self.is_loaded:
            if not self.load_model():
                logger.error("❌ Impossible de charger le modèle Whisper")
                return None

        try:
            logger.info(f"🎵 Transcription audio: {audio_path}")
            start_time = time.time()

            # Transcription avec Whisper
            if self.model is None:
                logger.error("❌ Modèle Whisper non chargé")
                return None

            result = cast(
                dict[str, Any],
                self.model.transcribe(
                    audio_path,
                    language=self.language if self.language != "auto" else None,
                    fp16=False,  # Éviter les problèmes de compatibilité
                ),
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

    def transcribe_microphone(self, duration: float = 3.0) -> str | None:
        """
        Enregistre et transcrit depuis le microphone.

        Args:
            duration: Durée d'enregistrement en secondes

        Returns:
            Texte transcrit ou None si erreur
        """
        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.info(
                "🎤 Micro désactivé (BBIA_DISABLE_AUDIO=1) - skip enregistrement"
            )
            return None

        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

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

            # OPTIMISATION PERFORMANCE: Sauvegarde temporaire sécurisée avec cleanup garanti
            import tempfile
            import time

            # Nom unique pour éviter collisions multi-processus
            temp_file = (
                Path(tempfile.gettempdir())
                / f"bbia_whisper_{os.getpid()}_{int(time.time() * 1000)}.wav"
            )

            try:
                sf.write(temp_file, audio_data, sample_rate)

                # Transcription
                result = self.transcribe_audio(str(temp_file))
                return result
            finally:
                # OPTIMISATION: Nettoyage garanti même en cas d'erreur
                if temp_file.exists():
                    try:
                        temp_file.unlink()
                    except Exception as cleanup_error:
                        logger.debug(f"Nettoyage fichier Whisper ({cleanup_error})")

        except ImportError:
            logger.error(
                "❌ sounddevice/soundfile requis pour l'enregistrement microphone"
            )
            return None
        except Exception as e:
            logger.error(f"❌ Erreur enregistrement microphone: {e}")
            return None

    def detect_speech_activity(self, audio_chunk: Any) -> bool:
        """
        Détecte si un chunk audio contient de la parole (VAD - Voice Activity Detection).

        Args:
            audio_chunk: Chunk audio (numpy array ou fichier)

        Returns:
            True si parole détectée, False sinon
        """
        if not self.enable_vad:
            return True  # Si VAD désactivé, considérer toujours comme parole

        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            return False

        try:
            from transformers import pipeline

            # Charger modèle VAD à la demande (gratuit Hugging Face)
            if not self._vad_loaded or self._vad_model is None:
                try:
                    logger.info("📥 Chargement modèle VAD (silero/vad)...")
                    self._vad_model = pipeline(
                        "audio-classification",
                        model="silero/vad",
                    )
                    self._vad_loaded = True
                    logger.info("✅ Modèle VAD chargé")
                except Exception as e:
                    logger.warning(f"⚠️ Impossible de charger VAD, fallback activé: {e}")
                    self.enable_vad = False
                    return True  # Fallback: considérer comme parole

            # Convertir audio_chunk si nécessaire (fichier -> array)
            import numpy as np
            import soundfile as sf

            if isinstance(audio_chunk, str | Path):
                # C'est un chemin de fichier
                audio_data, sample_rate = sf.read(audio_chunk)
            elif isinstance(audio_chunk, np.ndarray):
                audio_data = audio_chunk
            else:
                logger.warning("⚠️ Format audio non supporté pour VAD")
                return True  # Fallback: considérer comme parole

            # Vérifier taille minimale (éviter erreurs)
            if len(audio_data) < 100:
                return False

            # Détection VAD
            if self._vad_model is None:
                return True  # Fallback: considérer comme parole
            result = self._vad_model(audio_data, return_timestamps=False)

            # Résultat typique: [{"label": "SPEECH", "score": 0.95}]
            if isinstance(result, list) and len(result) > 0:
                label = result[0].get("label", "")
                score = result[0].get("score", 0.0)

                # Seuil de confiance
                is_speech = bool(label == "SPEECH" and score > 0.5)
                logger.debug(f"🔍 VAD: {label} (score: {score:.2f}) → {is_speech}")

                return is_speech

            return False

        except ImportError:
            logger.warning("⚠️ transformers requis pour VAD, fallback activé")
            self.enable_vad = False
            return True  # Fallback: considérer comme parole
        except Exception as e:
            logger.debug(f"ℹ️ Erreur VAD (fallback activé): {e}")
            return True  # Fallback: considérer comme parole

    def transcribe_microphone_with_vad(
        self, duration: float = 3.0, silence_threshold: float = 0.3
    ) -> str | None:
        """
        Enregistre et transcrit depuis le microphone avec détection VAD automatique.

        Args:
            duration: Durée maximale d'enregistrement en secondes
            silence_threshold: Seuil de silence avant arrêt (secondes)

        Returns:
            Texte transcrit ou None si aucune parole détectée
        """
        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.info(
                "🎤 Micro désactivé (BBIA_DISABLE_AUDIO=1) - skip enregistrement"
            )
            return None

        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

        try:
            import tempfile

            import numpy as np
            import sounddevice as sd
            import soundfile as sf

            logger.info(f"🎤 Enregistrement microphone avec VAD ({duration}s max)...")

            # Enregistrement audio continu avec détection VAD
            sample_rate = 16000
            chunk_duration = 0.5  # Analyser par chunks de 500ms
            chunk_samples = int(chunk_duration * sample_rate)

            audio_buffer: list[np.ndarray] = []
            silence_duration = 0.0
            max_silence = silence_threshold
            total_duration = 0.0

            while total_duration < duration:
                # Enregistrer chunk
                chunk = sd.rec(
                    chunk_samples,
                    samplerate=sample_rate,
                    channels=1,
                    dtype=np.float32,
                )
                sd.wait()
                audio_buffer.append(chunk.flatten())

                # Vérifier VAD sur chunk
                if self.detect_speech_activity(chunk):
                    silence_duration = 0.0  # Reset silence
                    logger.debug("🔊 Parole détectée")
                else:
                    silence_duration += chunk_duration
                    logger.debug(f"🔇 Silence: {silence_duration:.1f}s")

                total_duration += chunk_duration

                # Arrêt si silence prolongé après au moins une détection de parole
                if silence_duration > max_silence and len(audio_buffer) > 2:
                    logger.info("🔇 Fin détectée (silence prolongé)")
                    break

            if not audio_buffer:
                logger.warning("⚠️ Aucun audio enregistré")
                return None

            # Concaténer chunks
            audio_data = np.concatenate(audio_buffer)

            # Sauvegarder temporairement pour transcription
            temp_file = (
                Path(tempfile.gettempdir())
                / f"bbia_whisper_vad_{os.getpid()}_{int(time.time() * 1000)}.wav"
            )

            try:
                sf.write(temp_file, audio_data, sample_rate)

                # Transcription
                result = self.transcribe_audio(str(temp_file))
                return result
            finally:
                # Nettoyage
                if temp_file.exists():
                    try:
                        temp_file.unlink()
                    except Exception as cleanup_error:
                        logger.debug(f"Nettoyage fichier Whisper ({cleanup_error})")

        except ImportError:
            logger.error(
                "❌ sounddevice/soundfile requis pour l'enregistrement microphone"
            )
            return None
        except Exception as e:
            logger.error(f"❌ Erreur enregistrement microphone avec VAD: {e}")
            return None

    def transcribe_streaming(
        self,
        callback: Any | None = None,
        chunk_duration: float = 0.5,
        max_duration: float = 30.0,
    ) -> str | None:
        """
        Transcription en streaming (continuelle) depuis le microphone.
        Utile pour latence réduite (500ms vs 1-2s).

        Args:
            callback: Fonction appelée à chaque chunk transcrit (optionnel)
            chunk_duration: Durée de chaque chunk en secondes (plus petit = latence plus faible)
            max_duration: Durée maximale d'enregistrement

        Returns:
            Texte final complet transcrit, ou None si erreur
        """
        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.info("🎤 Micro désactivé (BBIA_DISABLE_AUDIO=1) - skip streaming")
            return None

        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

        # Charger modèle si nécessaire
        if not self.is_loaded:
            if not self.load_model():
                logger.error("❌ Impossible de charger le modèle Whisper")
                return None

        try:
            import tempfile

            import numpy as np
            import sounddevice as sd
            import soundfile as sf

            logger.info(
                f"🎤 Transcription streaming ({chunk_duration}s chunks, max {max_duration}s)..."
            )

            sample_rate = 16000
            chunk_samples = int(chunk_duration * sample_rate)
            all_transcriptions: list[str] = []
            total_duration = 0.0

            # Buffer pour garder contexte (améliore précision)
            audio_buffer: list[np.ndarray] = []
            buffer_max_chunks = 3  # Garder 3 chunks pour contexte

            while total_duration < max_duration:
                # Enregistrer chunk
                chunk = sd.rec(
                    chunk_samples,
                    samplerate=sample_rate,
                    channels=1,
                    dtype=np.float32,
                )
                sd.wait()

                # Ajouter au buffer
                audio_buffer.append(chunk.flatten())
                if len(audio_buffer) > buffer_max_chunks:
                    audio_buffer.pop(0)

                # Transcription sur buffer complet (améliore précision)
                if len(audio_buffer) >= 2:
                    audio_segment = np.concatenate(audio_buffer)

                    # Sauvegarder temporairement
                    temp_file = (
                        Path(tempfile.gettempdir())
                        / f"bbia_whisper_stream_{os.getpid()}_{int(time.time() * 1000)}.wav"
                    )

                    try:
                        sf.write(temp_file, audio_segment, sample_rate)

                        # Transcription rapide (petit segment)
                        if self.model is None:
                            logger.error("❌ Modèle Whisper non chargé")
                            break

                        result = cast(
                            dict[str, Any],
                            self.model.transcribe(
                                str(temp_file),
                                language=(
                                    self.language if self.language != "auto" else None
                                ),
                                fp16=False,
                                # Optimisations streaming
                                initial_prompt="",  # Pas de prompt initial pour latence
                                temperature=0.0,  # Déterministe
                            ),
                        )

                        text = result["text"].strip()
                        if text and text.lower() not in ["", "you", "thank you"]:
                            all_transcriptions.append(text)
                            logger.debug(f"📝 Chunk transcrit: '{text}'")

                            # Callback si fourni
                            if callback:
                                try:
                                    callback(text, total_duration)
                                except Exception as callback_error:
                                    logger.debug(f"Erreur callback: {callback_error}")

                    finally:
                        # Nettoyage
                        if temp_file.exists():
                            try:
                                temp_file.unlink()
                            except Exception:
                                pass

                total_duration += chunk_duration

                # Arrêt si silence prolongé (VAD optionnel)
                if self.enable_vad:
                    if not self.detect_speech_activity(chunk):
                        logger.debug("🔇 Silence détecté dans streaming")
                        # Continuer quand même (détection faible)

            # Concaténer toutes les transcriptions
            final_text = " ".join(all_transcriptions).strip()
            if final_text:
                logger.info(f"✅ Streaming terminé: '{final_text}'")
                return final_text

            logger.warning("⚠️ Aucune transcription générée")
            return None

        except ImportError:
            logger.error("❌ sounddevice/soundfile requis pour streaming")
            return None
        except Exception as e:
            logger.error(f"❌ Erreur streaming: {e}")
            return None


class VoiceCommandMapper:
    """Mappe les commandes vocales vers des actions RobotAPI."""

    def __init__(self) -> None:
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

    def map_command(self, text: str) -> dict[str, Any] | None:
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
) -> WhisperSTT | None:
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
