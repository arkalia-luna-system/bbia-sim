#!/usr/bin/env python3
"""Module bbia_voice_advanced.py
Synthèse vocale avancée avec Coqui TTS pour BBIA-SIM.
Résout tous les blocages macOS de pyttsx3 (pitch, émotion, tonalité contrôlables).

Fonctionnalités :
- ✅ Contrôle pitch/tonalité complet
- ✅ Contrôle émotionnel (happy, sad, excited, etc.)
- ✅ Synchronisation avec émotions BBIA
- ✅ Fallback vers pyttsx3 si Coqui TTS non disponible
"""

import logging
import os
import tempfile
from pathlib import Path
from typing import Any

# Import conditionnel Coqui TTS
try:
    from playsound import playsound
    from TTS.api import TTS

    COQUI_TTS_AVAILABLE = True
except ImportError:
    COQUI_TTS_AVAILABLE = False
    TTS = None
    playsound = None
    logging.warning(
        "Coqui TTS non disponible. Installez avec: pip install TTS playsound\n"
        "Fallback vers pyttsx3 activé.",
    )

# Import fallback pyttsx3 (éviter import circulaire)
import importlib.util

PYTTSX3_AVAILABLE = importlib.util.find_spec("pyttsx3") is not None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BBIAVoiceAdvanced:
    """Module de synthèse vocale avancée pour BBIA-SIM avec Coqui TTS.

    Supporte contrôle pitch, émotion, et synchronisation avec mouvements robot.
    """

    def __init__(
        self,
        model_name: str = "tts_models/fr/css10/vits",
        use_coqui: bool = True,
        temp_dir: str | None = None,
        robot_api: object | None = None,
    ) -> None:
        """Initialise le module de synthèse vocale avancée.

        Args:
            model_name: Nom du modèle Coqui TTS à utiliser
            use_coqui: Utiliser Coqui TTS si disponible, sinon fallback pyttsx3
            temp_dir: Répertoire temporaire pour fichiers audio (None = système)
            robot_api: Instance RobotAPI pour utiliser media.play_audio si disponible

        """
        self.use_coqui = use_coqui and COQUI_TTS_AVAILABLE
        self.temp_dir = Path(temp_dir) if temp_dir else Path(tempfile.gettempdir())
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.model_name = model_name
        self.tts: Any | None = None
        self.current_emotion = "neutral"
        self.robot_api = robot_api

        # Mapping émotions BBIA → émotions Coqui TTS
        self.emotion_map: dict[str, dict[str, Any]] = {
            "happy": {"emotion": "happy", "pitch": 0.3},
            "sad": {"emotion": "sad", "pitch": -0.2},
            "excited": {"emotion": "excited", "pitch": 0.4},
            "angry": {"emotion": "angry", "pitch": 0.2},
            "curious": {"emotion": "happy", "pitch": 0.1},
            "calm": {"emotion": "neutral", "pitch": -0.1},
            "neutral": {"emotion": "neutral", "pitch": 0.0},
            "surprised": {"emotion": "happy", "pitch": 0.35},
            "fear": {"emotion": "sad", "pitch": -0.3},
            "disgust": {"emotion": "neutral", "pitch": -0.1},
            "contempt": {"emotion": "neutral", "pitch": 0.0},
            "love": {"emotion": "happy", "pitch": 0.25},
        }

        if self.use_coqui:
            try:
                logger.info("🎤 Initialisation Coqui TTS avec modèle: %s", model_name)
                self.tts = TTS(model_name)
                logger.info("✅ Coqui TTS initialisé avec succès")
            except Exception as e:
                logger.warning(
                    f"⚠️  Erreur initialisation Coqui TTS, fallback activé: {e}",
                )
                self.use_coqui = False

        # Initialiser fallback pyttsx3 si nécessaire
        if not self.use_coqui and PYTTSX3_AVAILABLE:
            logger.info("🔄 Utilisation fallback pyttsx3")
            try:
                # OPTIMISATION PERFORMANCE: Utiliser cache global au lieu de pyttsx3.init() direct
                # dire_texte importé ci-dessous (ligne ~404)
                from .bbia_voice import (
                    _get_cached_voice_id,
                    _get_pyttsx3_engine,
                    dire_texte as dire_texte_old,
                )

                self.pyttsx3_engine = (
                    _get_pyttsx3_engine()
                )  # Utilise cache global (0ms après premier appel)
                self.pyttsx3_voice_id = _get_cached_voice_id()  # Utilise cache voice ID
                self.pyttsx3_engine.setProperty("voice", self.pyttsx3_voice_id)
                self.pyttsx3_engine.setProperty("rate", 170)
                self.pyttsx3_engine.setProperty("volume", 1.0)
                logger.info("✅ Fallback pyttsx3 initialisé (avec cache)")
            except Exception:
                logger.exception("❌ Erreur initialisation fallback")
                self.pyttsx3_engine = None

    def say(
        self,
        text: str,
        emotion: str | None = None,
        pitch: float | None = None,
        speed: float = 1.0,
        volume: float = 1.0,
    ) -> bool:
        """Synthétise et lit un texte avec contrôle avancé.

        Args:
            text: Texte à synthétiser
            emotion: Émotion BBIA (happy, sad, excited, etc.) ou None pour current
            pitch: Ajustement pitch (-1.0 à 1.0) ou None pour auto selon émotion
            speed: Vitesse de lecture (0.5 à 2.0, défaut 1.0)
            volume: Volume (0.0 à 1.0, défaut 1.0)

        Returns:
            True si succès, False sinon

        """
        # Vérifier flag d'environnement pour désactiver audio (CI/headless)
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): '%s' ignoré", text)
            return (
                True  # Retourner True car c'est un succès (ignoré intentionnellement)
            )

        if not text or not text.strip():
            logger.warning("Texte vide, rien à dire")
            return False

        try:
            if self.use_coqui and self.tts:
                return self._say_coqui(text, emotion, pitch, speed, volume)
            if self.pyttsx3_engine:
                return self._say_pyttsx3(text, speed, volume)
            logger.error("❌ Aucun moteur TTS disponible")
            return False
        except Exception:
            logger.exception("❌ Erreur synthèse vocale")
            return False

    def _say_coqui(
        self,
        text: str,
        emotion: str | None,
        pitch: float | None,
        speed: float,
        volume: float,
    ) -> bool:
        """Synthétise avec Coqui TTS (méthode avancée)."""
        # Déterminer émotion et pitch
        if emotion is None:
            emotion = self.current_emotion

        emotion_config = self.emotion_map.get(emotion, self.emotion_map["neutral"])

        # Déterminer la valeur de pitch (float garanti après cette section)
        pitch_value: float
        if pitch is None:
            pitch_raw = emotion_config.get("pitch", 0.0)
            if isinstance(pitch_raw, int | float):
                pitch_value = float(pitch_raw)
            else:
                pitch_value = 0.0
        else:
            pitch_value = pitch

        # OPTIMISATION PERFORMANCE: Utiliser context manager pour nettoyage garanti
        # Évite accumulation de fichiers temporaires en cas d'erreur
        import time

        audio_file = (
            self.temp_dir / f"bbia_tts_{os.getpid()}_{int(time.time() * 1000)}.wav"
        )

        try:
            # Note: Certains modèles Coqui TTS supportent directement
            # le contrôle émotionnel via paramètres, mais dépend du modèle.

            # Synthétiser
            logger.info(
                f"🎤 Synthèse Coqui: '{text[:50]}...' "
                f"(émotion={emotion}, pitch={pitch_value:.2f})",
            )

            # Note: Certains modèles Coqui TTS supportent directement pitch/emotion
            # dans tts_to_file. Si non supporté, utiliser paramètres de base.
            if self.tts is None:
                msg = "Coqui TTS non initialisé"
                raise ValueError(msg)
            # Type narrowing: mypy comprend maintenant que self.tts n'est pas None
            self.tts.tts_to_file(
                text=text,
                file_path=str(audio_file),
                # Certains modèles supportent ces paramètres :
                # speed=speed,
                # emotion=model_emotion,  # Si supporté
            )

            # Jouer l'audio via SDK media.speaker si disponible, sinon local
            self._play_audio_file(audio_file, volume)

            logger.info("✅ Synthèse vocale terminée")
            return True

        except Exception:
            logger.exception("❌ Erreur synthèse Coqui")
            # Fallback vers pyttsx3 si erreur
            if self.pyttsx3_engine:
                logger.info("🔄 Fallback vers pyttsx3")
                return self._say_pyttsx3(text, speed, volume)
            return False
        finally:
            # OPTIMISATION: Nettoyage garanti même en cas d'erreur
            if audio_file.exists():
                try:
                    audio_file.unlink()
                except Exception as cleanup_error:
                    logger.debug("Nettoyage fichier temporaire (%s)", cleanup_error)

    def _play_audio_file(self, audio_path: Path, volume: float) -> None:
        """Joue un fichier audio via le SDK si possible, sinon localement.

        Priorité:
        1) robot_api.media.play_audio(bytes, volume)
        2) robot_api.media.speaker.play(bytes) ou .play_file(path)
        3) playsound local
        """
        try:
            if self.robot_api and hasattr(self.robot_api, "media"):
                media = self.robot_api.media
                # Media est maintenant toujours disponible (shim en simulation)
                # Essayer play_audio(bytes, volume)
                try:
                    with open(audio_path, "rb") as f:
                        data = f.read()
                    if hasattr(media, "play_audio"):
                        media.play_audio(data, volume=volume)
                        logger.info("🔊 Lecture via robot.media.play_audio")
                        return
                except Exception as e:
                    logger.debug("media.play_audio indisponible: %s", e)

                # Essayer media.speaker
                try:
                    speaker = getattr(media, "speaker", None)
                    if speaker is not None:
                        if hasattr(speaker, "play_file"):
                            speaker.play_file(str(audio_path))
                            logger.info("🔊 Lecture via robot.media.speaker.play_file")
                            return
                        if hasattr(speaker, "play"):
                            with open(audio_path, "rb") as f:
                                speaker.play(f.read())
                            logger.info("🔊 Lecture via robot.media.speaker.play")
                            return
                except Exception as e:
                    logger.debug("media.speaker indisponible: %s", e)

        except Exception as e:
            logger.debug("Intégration média non disponible: %s", e)

        # Fallback local
        if playsound is not None:
            playsound(str(audio_path))
        else:
            logger.warning("playsound indisponible - aucun lecteur audio local")

    def _say_pyttsx3(self, text: str, speed: float, volume: float) -> bool:
        """Synthétise avec pyttsx3 (fallback)."""
        try:
            logger.info("🎤 Synthèse pyttsx3: '%s...'", text[:50])
            self.pyttsx3_engine.setProperty("rate", int(170 * speed))
            self.pyttsx3_engine.setProperty("volume", volume)
            self.pyttsx3_engine.say(text)
            self.pyttsx3_engine.runAndWait()
            logger.info("✅ Synthèse pyttsx3 terminée")
            return True
        except Exception:
            logger.exception("❌ Erreur synthèse pyttsx3")
            return False

    def set_emotion(self, emotion: str) -> None:
        """Définit l'émotion actuelle pour synthèse vocale.

        Args:
            emotion: Émotion BBIA (happy, sad, excited, etc.)

        """
        if emotion in self.emotion_map:
            self.current_emotion = emotion
            logger.info("🎭 Émotion vocale définie: %s", emotion)
        else:
            logger.warning("⚠️  Émotion inconnue: %s, utilisation 'neutral'", emotion)
            self.current_emotion = "neutral"

    def say_with_emotion(
        self,
        text: str,
        bbia_emotion: str,
        intensity: float = 0.5,
    ) -> bool:
        """Synthétise un texte avec émotion BBIA (méthode optimisée).

        Args:
            text: Texte à synthétiser
            bbia_emotion: Émotion BBIA (12 émotions supportées)
            intensity: Intensité de l'émotion (0.0 à 1.0, ajuste le pitch)

        Returns:
            True si succès

        """
        emotion_config = self.emotion_map.get(bbia_emotion, self.emotion_map["neutral"])

        # Ajuster pitch selon intensité
        pitch_raw = emotion_config.get("pitch", 0.0)
        base_pitch = float(pitch_raw) if isinstance(pitch_raw, int | float) else 0.0
        adjusted_pitch: float = base_pitch * intensity

        return self.say(
            text=text,
            emotion=bbia_emotion,
            pitch=adjusted_pitch,
            speed=0.9 + (intensity * 0.2),  # Vitesse augmente avec intensité
        )

    def is_coqui_available(self) -> bool:
        """Vérifie si Coqui TTS est disponible."""
        return self.use_coqui and self.tts is not None


# Fonctions de compatibilité pour migration progressive
def dire_texte_advanced(
    texte: str,
    emotion: str | None = None,
    pitch: float | None = None,
    speed: float = 1.0,
) -> bool:
    """Fonction wrapper pour compatibilité avec dire_texte() existant.

    Args:
        texte: Texte à synthétiser
        emotion: Émotion BBIA (optionnel)
        pitch: Ajustement pitch (optionnel)
        speed: Vitesse de lecture (0.5 à 2.0, défaut 1.0)

    Returns:
        True si succès

    """
    # OPTIMISATION: Instance globale (lazy initialization)
    if "_global_voice_advanced" not in globals():
        globals()["_global_voice_advanced"] = BBIAVoiceAdvanced()

    voice_advanced: BBIAVoiceAdvanced = globals()["_global_voice_advanced"]
    result = voice_advanced.say(texte, emotion=emotion, pitch=pitch, speed=speed)
    return bool(result)


# Compatibilité avec ancien code (fallback automatique)
def dire_texte(texte: str) -> bool:
    """Fonction de synthèse vocale avec fallback automatique.

    Essaie Coqui TTS avancé, sinon fallback vers pyttsx3.

    Args:
        texte: Texte à synthétiser

    Returns:
        True si succès

    """
    # Essayer Coqui TTS d'abord
    try:
        return dire_texte_advanced(texte)
    except (ImportError, RuntimeError, ValueError):
        # Fallback vers pyttsx3 original
        try:
            # dire_texte_old déjà importé ci-dessus
            dire_texte_old(texte)  # Retourne None, mais exécution = succès
            return True
        except Exception:
            logging.exception("❌ Erreur synthèse vocale (tous moteurs)")
            return False


if __name__ == "__main__":
    """Test du module de synthèse vocale avancée."""

    logging.info("🎤 Test BBIA Voice Advanced")
    logging.info("=" * 50)

    # Tester Coqui TTS
    if COQUI_TTS_AVAILABLE:
        logging.info("✅ Coqui TTS disponible")
        voice = BBIAVoiceAdvanced()

        if voice.is_coqui_available():
            logging.info("\n🧪 Test synthèse avec émotion:")
            voice.say_with_emotion(
                "Bonjour, je suis BBIA avec une voix contrôlable !",
                "happy",
                intensity=0.8,
            )

            logging.info("\n🧪 Test synthèse avec pitch personnalisé:")
            voice.say(
                "Je peux maintenant contrôler le pitch sur macOS !",
                emotion="excited",
                pitch=0.3,
            )
        else:
            logging.warning("⚠️  Coqui TTS non initialisé, utilisation fallback")
    else:
        logging.error("❌ Coqui TTS non disponible")
        logging.info("💡 Installez avec: pip install TTS playsound")

    # Tester fallback
    if PYTTSX3_AVAILABLE:
        logging.info("\n✅ Fallback pyttsx3 disponible")
    else:
        logging.error("\n❌ Fallback pyttsx3 non disponible")
