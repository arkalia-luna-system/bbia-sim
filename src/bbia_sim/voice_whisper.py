#!/usr/bin/env python3
"""bbia_voice_whisper.py - Module Whisper STT pour BBIA
Intégration Speech-to-Text avec OpenAI Whisper (optionnel)
"""

import logging
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, cast

import numpy.typing as npt

# Déclarer whisper comme Any dès le début pour éviter conflit de types
whisper: Any

try:
    import whisper as _whisper_module

    WHISPER_AVAILABLE = True
    whisper = _whisper_module
except ImportError:
    WHISPER_AVAILABLE = False
    whisper = None

# Imports optionnels pour les patches dans les tests
try:
    from transformers import pipeline as transformers_pipeline
except ImportError:
    transformers_pipeline = None

try:
    import soundfile as sf
except ImportError:
    sf = None

try:
    import sounddevice as sd
except (ImportError, OSError):
    # OSError: PortAudio library not found (CI/headless)
    sd = None

logger = logging.getLogger(__name__)

# OPTIMISATION PERFORMANCE: Cache global pour modèle VAD (évite chargements répétés entre instances)
_vad_model_cache: Any | None = None
_vad_cache_lock = threading.Lock()

# OPTIMISATION RAM: Cache global LRU pour modèles Whisper (max 2 modèles: tiny, base)
_whisper_models_cache: dict[str, Any] = {}  # model_size -> model
_whisper_model_last_used: dict[str, float] = {}
_whisper_model_cache_lock = threading.Lock()
_MAX_WHISPER_CACHE_SIZE = (
    2  # OPTIMISATION RAM: Limiter à 2 modèles Whisper max (tiny, base)
)


class WhisperSTT:
    """Module Speech-to-Text utilisant OpenAI Whisper."""

    def __init__(
        self,
        model_size: str = "tiny",
        language: str = "fr",
        enable_vad: bool = True,
    ):
        """Initialise le module Whisper STT.

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
                "⚠️ Whisper non disponible. Fallback vers speech_recognition.",
            )
            return

        logger.info(
            f"🎤 Initialisation Whisper STT (modèle: {model_size}, langue: {language}, VAD: {enable_vad})",
        )

    def load_model(self) -> bool:
        """Charge le modèle Whisper (utilise cache global si disponible)."""
        if not WHISPER_AVAILABLE:
            return False

        # OPTIMISATION RAM: Utiliser cache global LRU pour éviter chargements répétés
        global _whisper_models_cache, _whisper_model_last_used
        with _whisper_model_cache_lock:
            if self.model_size in _whisper_models_cache:
                logger.debug(
                    f"♻️ Réutilisation modèle Whisper depuis cache ({self.model_size})",
                )
                self.model = _whisper_models_cache[self.model_size]
                # OPTIMISATION RAM: Mettre à jour timestamp usage
                _whisper_model_last_used[self.model_size] = time.time()
                self.is_loaded = True
                return True

            # OPTIMISATION RAM: Vérifier limite cache et décharger LRU si nécessaire
            if len(_whisper_models_cache) >= _MAX_WHISPER_CACHE_SIZE:
                # Trouver modèle le moins récemment utilisé
                if _whisper_model_last_used:
                    oldest_key = min(
                        _whisper_model_last_used.items(),
                        key=lambda x: x[1],
                    )[0]
                    del _whisper_models_cache[oldest_key]
                    del _whisper_model_last_used[oldest_key]
                    logger.debug(
                        f"♻️ Modèle Whisper LRU déchargé: {oldest_key} (optimisation RAM)",
                    )

        try:
            logger.info("📥 Chargement modèle Whisper %s...", self.model_size)
            start_time = time.time()

            model = whisper.load_model(self.model_size)

            load_time = time.time() - start_time
            logger.info("✅ Modèle Whisper chargé en %.1fs", load_time)

            # OPTIMISATION RAM: Mettre en cache global avec timestamp
            with _whisper_model_cache_lock:
                _whisper_models_cache[self.model_size] = model
                _whisper_model_last_used[self.model_size] = time.time()

            self.model = model
            self.is_loaded = True
            return True

        except Exception as e:
            logger.exception("❌ Erreur chargement Whisper: %s", e)
            return False

    def transcribe_audio(self, audio_path: str) -> str | None:
        """Transcrit un fichier audio en texte.

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
            logger.info("🎵 Transcription audio: %s", audio_path)
            start_time = time.time()

            # Transcription avec Whisper
            if self.model is None:
                logger.error("❌ Modèle Whisper non chargé")
                return None

            result = cast(
                "dict[str, Any]",
                self.model.transcribe(
                    audio_path,
                    language=self.language if self.language != "auto" else None,
                    fp16=False,  # Éviter les problèmes de compatibilité
                ),
            )

            transcription_time = time.time() - start_time
            text = result["text"].strip()

            logger.info(
                f"✅ Transcription terminée en {transcription_time:.1f}s: '{text}'",
            )
            return text

        except Exception as e:
            logger.exception("❌ Erreur transcription: %s", e)
            return None

    def transcribe_microphone(self, duration: float = 3.0) -> str | None:
        """Enregistre et transcrit depuis le microphone.

        Args:
            duration: Durée d'enregistrement en secondes

        Returns:
            Texte transcrit ou None si erreur

        """
        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.info(
                "🎤 Micro désactivé (BBIA_DISABLE_AUDIO=1) - skip enregistrement",
            )
            return None

        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

        try:
            import numpy as np
            import soundfile as sf

            logger.info("🎤 Enregistrement microphone (%ss)...", duration)

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
                        logger.debug("Nettoyage fichier Whisper (%s)", cleanup_error)

        except ImportError:
            logger.exception(
                "❌ sounddevice/soundfile requis pour l'enregistrement microphone",
            )
            return None
        except Exception as e:
            logger.exception("❌ Erreur enregistrement microphone: %s", e)
            return None

    def detect_speech_activity(self, audio_chunk: Any) -> bool:
        """Détecte si un chunk audio contient de la parole (VAD - Voice Activity Detection).

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
            # OPTIMISATION PERFORMANCE: Utiliser cache global pour modèle VAD
            global _vad_model_cache
            if _vad_model_cache is not None:
                logger.debug("♻️ Réutilisation modèle VAD depuis cache global")
                self._vad_model = _vad_model_cache
                self._vad_loaded = True
            elif not self._vad_loaded or self._vad_model is None:
                try:
                    logger.info("📥 Chargement modèle VAD (silero/vad)...")
                    # Utiliser l'import au niveau module si disponible, sinon import local
                    if transformers_pipeline is None:
                        from transformers import pipeline

                        vad_pipeline_func = pipeline
                    else:
                        vad_pipeline_func = transformers_pipeline

                    vad_model = vad_pipeline_func(
                        "audio-classification",
                        model="silero/vad",
                    )

                    # Mettre en cache global et local
                    with _vad_cache_lock:
                        if _vad_model_cache is None:
                            _vad_model_cache = vad_model

                    self._vad_model = vad_model
                    self._vad_loaded = True
                    logger.info("✅ Modèle VAD chargé")
                except Exception as e:
                    logger.warning("⚠️ Impossible de charger VAD, fallback activé: %s", e)
                    self.enable_vad = False
                    return True  # Fallback: considérer comme parole

            # Convertir audio_chunk si nécessaire (fichier -> array)
            import numpy as np

            # Utiliser l'import au niveau module si disponible, sinon import local
            if sf is None:
                try:
                    import soundfile as soundfile_module
                except ImportError:
                    logger.warning(
                        "⚠️ soundfile requis pour VAD fichier, fallback activé",
                    )
                    return True  # Fallback: considérer comme parole
            else:
                soundfile_module = sf

            if isinstance(audio_chunk, str | Path):
                # C'est un chemin de fichier
                audio_data, sample_rate = soundfile_module.read(audio_chunk)
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
                logger.debug("🔍 VAD: %s (score: %.2f) → %s", label, score, is_speech)

                return is_speech

            return False

        except ImportError:
            logger.warning("⚠️ transformers requis pour VAD, fallback activé")
            self.enable_vad = False
            return True  # Fallback: considérer comme parole
        except Exception as e:
            logger.debug("ℹ️ Erreur VAD (fallback activé): %s", e)
            return True  # Fallback: considérer comme parole

    def transcribe_microphone_with_vad(
        self,
        duration: float = 3.0,
        silence_threshold: float = 0.3,
    ) -> str | None:
        """Enregistre et transcrit depuis le microphone avec détection VAD automatique.

        Args:
            duration: Durée maximale d'enregistrement en secondes
            silence_threshold: Seuil de silence avant arrêt (secondes)

        Returns:
            Texte transcrit ou None si aucune parole détectée

        """
        # Désactivation explicite audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.info(
                "🎤 Micro désactivé (BBIA_DISABLE_AUDIO=1) - skip enregistrement",
            )
            return None

        # Vérification globale de disponibilité
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper non disponible")
            return None

        try:
            import tempfile

            import numpy as np
            import soundfile as sf

            logger.info("🎤 Enregistrement microphone avec VAD (%ss max)...", duration)

            # Enregistrement audio continu avec détection VAD
            sample_rate = 16000
            chunk_duration = 0.5  # Analyser par chunks de 500ms
            chunk_samples = int(chunk_duration * sample_rate)

            # OPTIMISATION RAM: Limiter taille buffer avec deque (max 10 chunks)
            audio_buffer: deque[npt.NDArray[np.float32]] = deque(maxlen=10)
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
                    logger.debug("🔇 Silence: %ss", silence_duration:.1f)

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
                        logger.debug("Nettoyage fichier Whisper (%s)", cleanup_error)

        except ImportError:
            logger.exception(
                "❌ sounddevice/soundfile requis pour l'enregistrement microphone",
            )
            return None
        except Exception as e:
            logger.exception("❌ Erreur enregistrement microphone avec VAD: %s", e)
            return None

    def transcribe_streaming(
        self,
        callback: Any | None = None,
        chunk_duration: float = 0.5,
        max_duration: float = 30.0,
        transcription_interval: float = 1.5,
    ) -> str | None:
        """Transcription en streaming (continuelle) depuis le microphone.
        Utile pour latence réduite (500ms vs 1-2s).

        Args:
            callback: Fonction appelée à chaque chunk transcrit (optionnel)
            chunk_duration: Durée de chaque chunk en secondes (plus petit = latence plus faible)
            max_duration: Durée maximale d'enregistrement
            transcription_interval: Intervalle minimum entre transcriptions (secondes).
                                   Réduire pour latence plus faible, augmenter pour économiser CPU.

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
            import soundfile as sf

            logger.info(
                f"🎤 Transcription streaming ({chunk_duration}s chunks, max {max_duration}s, intervalle {transcription_interval}s)...",
            )

            sample_rate = 16000
            chunk_samples = int(chunk_duration * sample_rate)
            all_transcriptions: list[str] = []
            total_duration = 0.0

            # OPTIMISATION RAM: Limiter taille buffer avec deque
            buffer_max_chunks = 10  # Max 10 chunks (limite sécurité)
            audio_buffer: deque[npt.NDArray[np.float32]] = deque(
                maxlen=buffer_max_chunks
            )

            # OPTIMISATION PERFORMANCE: Throttling transcription pour éviter surcharge CPU/GPU
            last_transcription_time = 0.0
            consecutive_silence_chunks = 0
            max_silence_chunks = 3  # Arrêter après 3 chunks de silence consécutifs

            while total_duration < max_duration:
                # Enregistrer chunk
                chunk = sd.rec(
                    chunk_samples,
                    samplerate=sample_rate,
                    channels=1,
                    dtype=np.float32,
                )
                sd.wait()

                # OPTIMISATION RAM: Ajouter au buffer (deque gère automatiquement maxlen)
                audio_buffer.append(chunk.flatten())

                # OPTIMISATION: Utiliser VAD pour décider si transcrire (évite traitement inutile)
                should_transcribe = True
                if self.enable_vad:
                    has_speech = self.detect_speech_activity(chunk)
                    if has_speech:
                        consecutive_silence_chunks = 0
                        logger.debug("🔊 Parole détectée")
                    else:
                        consecutive_silence_chunks += 1
                        logger.debug("🔇 Silence: %s chunks", consecutive_silence_chunks)
                        # Ne pas transcrire si silence prolongé
                        if consecutive_silence_chunks >= max_silence_chunks:
                            should_transcribe = False
                        # OPTIMISATION RAM: Buffer géré par deque (maxlen)
                        # Pas besoin de pop manuel

                # OPTIMISATION: Throttling - ne transcrire que si intervalle respecté ET parole détectée
                current_time = time.time()
                time_since_last_transcription = current_time - last_transcription_time
                should_transcribe = (
                    should_transcribe
                    and time_since_last_transcription >= transcription_interval
                    and len(audio_buffer) >= 2
                )

                # Transcription sur buffer complet (améliore précision)
                if should_transcribe:
                    audio_segment = np.concatenate(audio_buffer)

                    # OPTIMISATION RAM: Pool fichiers temporaires (réutiliser au lieu de créer/supprimer)
                    if not hasattr(self, "_temp_file_pool"):
                        self._temp_file_pool: list[Path] = []
                        self._max_temp_files = 3  # Pool de 3 fichiers max

                    # Réutiliser fichier depuis pool si disponible
                    temp_file = None
                    if self._temp_file_pool:
                        temp_file = self._temp_file_pool.pop(0)
                    else:
                        temp_file = (
                            Path(tempfile.gettempdir())
                            / f"bbia_whisper_stream_{os.getpid()}_"
                            f"{int(time.time() * 1000)}.wav"
                        )
                        # Limiter taille pool
                        if len(self._temp_file_pool) < self._max_temp_files:
                            pass  # Ajouter au pool après usage

                    try:
                        sf.write(temp_file, audio_segment, sample_rate)

                        # Transcription rapide (petit segment)
                        if self.model is None:
                            logger.error("❌ Modèle Whisper non chargé")
                            break

                        result = cast(
                            "dict[str, Any]",
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
                            last_transcription_time = current_time
                            logger.debug("📝 Chunk transcrit: '%s'", text)

                            # Callback si fourni
                            if callback:
                                try:
                                    callback(text, total_duration)
                                except Exception as callback_error:
                                    logger.debug("Erreur callback: %s", callback_error)

                    finally:
                        # OPTIMISATION RAM: Remettre fichier dans pool au lieu
                        # de supprimer
                        if temp_file and temp_file.exists():
                            # Remettre dans pool si espace disponible
                            if len(self._temp_file_pool) < self._max_temp_files:
                                self._temp_file_pool.append(temp_file)
                            else:
                                # Pool plein - supprimer fichier
                                try:
                                    temp_file.unlink()
                                except Exception as e:
                                    # Ignorer erreur suppression fichier temporaire
                                    logger.debug(
                                        f"Impossible de supprimer fichier temporaire: {e}"
                                    )

                total_duration += chunk_duration

                # OPTIMISATION: Arrêt si silence prolongé (économise CPU)
                if consecutive_silence_chunks >= max_silence_chunks * 2:
                    logger.info("🔇 Arrêt automatique (silence prolongé)")
                    break

            # Concaténer toutes les transcriptions
            final_text = " ".join(all_transcriptions).strip()
            if final_text:
                logger.info("✅ Streaming terminé: '%s'", final_text)
                return final_text

            logger.warning("⚠️ Aucune transcription générée")
            return None

        except ImportError:
            logger.exception("❌ sounddevice/soundfile requis pour streaming")
            return None
        except Exception as e:
            logger.exception("❌ Erreur streaming: %s", e)
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
            f"🗣️ Mappeur de commandes initialisé ({len(self.commands)} commandes)",
        )

    def map_command(self, text: str) -> dict[str, Any] | None:
        """Mappe un texte vers une action RobotAPI.

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
            logger.info("🎯 Commande mappée: '%s' → %s", text, action)
            return {"action": action, "confidence": 1.0}

        # Recherche partielle
        for command, action in self.commands.items():
            if command in text_lower:
                logger.info("🎯 Commande partielle mappée: '%s' → %s", text, action)
                return {"action": action, "confidence": 0.8}

        logger.warning("❓ Commande non reconnue: '%s'", text)
        return None


def create_whisper_stt(
    model_size: str = "tiny",
    language: str = "fr",
) -> WhisperSTT | None:
    """Factory function pour créer une instance WhisperSTT.

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

    logger.info("🧪 Test module Whisper STT")
    logger.info("=" * 40)

    # Test disponibilité
    logger.info("Whisper disponible: %s", WHISPER_AVAILABLE)

    if WHISPER_AVAILABLE:
        # Test création
        stt = create_whisper_stt("tiny", "fr")
        if stt:
            logger.info("✅ Module Whisper créé")

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
                logger.info("  '%s' → %s", cmd, result)
        else:
            logger.error("❌ Impossible de créer le module Whisper")
    else:
        logger.error("❌ Whisper non installé")
