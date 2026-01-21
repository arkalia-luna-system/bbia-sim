"""Module bbia_audio.py.

Gestion de l'audio pour BBIA : enregistrement, lecture, détection de son.
Utilise robot.media.microphone si disponible (SDK Reachy Mini - 4 microphones),
sinon utilise sounddevice pour compatibilité.
Compatible macOS, simple, portable, testé.
"""

import logging
import os
import sys
import wave
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

logger = logging.getLogger(__name__)


class _SoundDeviceShim:
    """Shim minimal pour permettre le patch dans les tests (sd.rec/play/wait).

    Les méthodes lèvent RuntimeError si utilisées sans patch effectif.
    """

    def rec(
        self,
        *args: Any,  # noqa: ARG002, ANN401
        **kwargs: Any,  # noqa: ARG002, ANN401
    ) -> Any:  # noqa: ANN401 - garde-fou pour module optionnel
        msg = "sounddevice indisponible: sd.rec non opérationnel"
        raise RuntimeError(msg)

    def wait(
        self,
        *args: Any,  # noqa: ARG002, ANN401
        **kwargs: Any,  # noqa: ARG002, ANN401
    ) -> None:
        msg = "sounddevice indisponible: sd.wait non opérationnel"
        raise RuntimeError(msg)

    def play(
        self,
        *args: Any,  # noqa: ARG002, ANN401
        **kwargs: Any,  # noqa: ARG002, ANN401
    ) -> None:
        msg = "sounddevice indisponible: sd.play non opérationnel"
        raise RuntimeError(msg)


sd: Any = _SoundDeviceShim()  # toujours patchable dans les tests

if TYPE_CHECKING:
    from .robot_api import RobotAPI


def _get_sd() -> Any | None:
    """Import paresseux de sounddevice pour éviter l'init PortAudio au import."""
    global sd
    if sd is not None:
        return sd
    try:  # pragma: no cover - dépend du runtime
        import sounddevice as _sd

        # Remplacer le shim par le vrai module sounddevice
        globals()["sd"] = _sd
        return _sd
    except (ImportError, RuntimeError, AttributeError):
        # Retourner le shim pour rester patchable dans les tests
        return sd


logging.basicConfig(level=logging.INFO)

# Constantes alignées SDK Reachy Mini officiel
DEFAULT_SAMPLE_RATE = 16000  # SDK Reachy Mini standard (16kHz)
DEFAULT_BUFFER_SIZE = 512  # SDK optimisé pour latence minimale
DEFAULT_CHANNELS = 1  # Mono par défaut

# Alias module-level pour permettre le patch dans les tests
soundfile: Any | None
try:  # pragma: no cover - import optionnel
    import soundfile as _soundfile

    soundfile = _soundfile
except (
    ImportError,
    RuntimeError,
    AttributeError,
):  # pragma: no cover - environnement sans soundfile
    soundfile = None


def _get_robot_media_microphone(
    robot_api: Optional["RobotAPI"] = None,
) -> object | None:
    """Récupère robot.media.microphone si disponible.

    Args:
        robot_api: Interface RobotAPI (optionnel)

    Returns:
        robot.media.microphone ou None (jamais None si robot_api.media est disponible)

    """
    if robot_api and hasattr(robot_api, "media"):
        try:
            media = robot_api.media
            # Media est maintenant toujours disponible (shim en simulation)
            if media:
                return getattr(media, "microphone", None)  # type: ignore[no-any-return]
        except (AttributeError, ImportError, RuntimeError):
            return None
    return None


# OPTIMISATION PERFORMANCE: Cache pour _is_safe_path
_temp_roots_cache: list[str] | None = None
_cwd_cache: str | None = None


def _is_safe_path(path: str) -> bool:
    """Validation simple de chemin pour éviter le path traversal.

    Autorise:
    - chemins relatifs sans ".."
    - chemins absolus sous le répertoire courant (projet)
    - chemins sous répertoires temporaires usuels (/tmp, /dev/shm, pytest tmp)
    """
    global _temp_roots_cache, _cwd_cache

    try:
        norm = os.path.normpath(path)
        if ".." in norm.split(os.sep):
            return False
        if os.path.isabs(norm):
            # OPTIMISATION: Cache cwd et temp_roots (calculés une seule fois)
            if _cwd_cache is None:
                _cwd_cache = os.path.abspath(os.getcwd())
            if _temp_roots_cache is None:
                import tempfile

                _temp_roots_cache = [
                    "/tmp",  # nosec B108
                    "/dev/shm",  # nosec B108
                    os.path.abspath(os.getenv("PYTEST_TMPDIR", "/tmp")),  # nosec B108
                    os.path.abspath(tempfile.gettempdir()),
                ]

            abs_path = os.path.abspath(norm)
            if any(
                abs_path.startswith(tr + os.sep) or abs_path == tr
                for tr in _temp_roots_cache
            ):
                return True
            return abs_path.startswith(_cwd_cache + os.sep) or abs_path == _cwd_cache
        return True
    except (OSError, ValueError, TypeError):
        return False


def _save_audio_to_wav(
    fichier: str,
    audio_data: bytes | np.ndarray | Any,
    frequence: int,
) -> None:
    """Sauvegarde les données audio dans un fichier WAV.

    Args:
        fichier: Chemin du fichier de sortie
        audio_data: Données audio (bytes, numpy array ou autre)
        frequence: Fréquence d'échantillonnage

    """
    with wave.open(fichier, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(frequence)
        if isinstance(audio_data, bytes):
            wf.writeframes(audio_data)
        elif isinstance(audio_data, np.ndarray):
            wf.writeframes((audio_data.astype(np.int16)).tobytes())
        else:
            wf.writeframes(bytes(audio_data))


def _enregistrer_via_sdk(
    fichier: str,
    duree: int,
    frequence: int,
    robot_api: Optional["RobotAPI"],
    microphone_sdk: object | None,
) -> bool | None:
    """Tente l'enregistrement via SDK Reachy Mini.

    Returns:
        True si succès, None si fallback nécessaire

    """
    if microphone_sdk is None:
        return None

    try:
        # Enregistrement via robot.media.record_audio()
        if (
            robot_api
            and hasattr(robot_api, "media")
            and robot_api.media
            and hasattr(robot_api.media, "record_audio")
        ):
            logging.info(
                "Enregistrement via SDK (4 microphones) (%ss) dans %s...",
                duree,
                fichier,
            )
            audio_data = robot_api.media.record_audio(
                duration=duree,
                sample_rate=frequence,
            )
            _save_audio_to_wav(fichier, audio_data, frequence)
            logging.info("Enregistrement SDK terminé.")
            return True

        # Alternative: microphone.record() si disponible
        if hasattr(microphone_sdk, "record"):
            audio_data = microphone_sdk.record(
                duration=duree,
                sample_rate=frequence,
            )
            _save_audio_to_wav(fichier, audio_data, frequence)
            logging.info("Enregistrement SDK terminé.")
            return True
    except (AttributeError, RuntimeError, OSError):
        logging.debug("Erreur enregistrement SDK (fallback sounddevice)")

    return None


def _enregistrer_via_sounddevice(
    fichier: str,
    duree: int,
    frequence: int,
) -> bool:
    """Enregistre via sounddevice (fallback).

    Returns:
        True si succès

    Raises:
        RuntimeError: Si enregistrement impossible

    """
    logging.info("Enregistrement audio (%ss) dans %s...", duree, fichier)
    _sd = _get_sd()
    if _sd is None:
        msg = "sounddevice indisponible (BBIA_DISABLE_AUDIO conseillé en CI)"
        raise RuntimeError(msg)

    # Issue #329: Gestion gracieuse canaux audio invalides
    audio = _try_record_audio(_sd, duree, frequence)

    _sd.wait()
    _save_audio_to_wav(fichier, audio, frequence)
    logging.info("Enregistrement terminé.")
    return True


def _try_record_audio(
    _sd: Any,
    duree: int,
    frequence: int,
) -> np.ndarray:
    """Tente d'enregistrer audio avec gestion des canaux.

    Returns:
        Données audio numpy array

    """
    try:
        return _sd.rec(
            int(duree * frequence),
            samplerate=frequence,
            channels=1,
            dtype="int16",
        )
    except Exception as channel_error:
        return _record_with_fallback_channels(_sd, duree, frequence, channel_error)


def _record_with_fallback_channels(
    _sd: Any,
    duree: int,
    frequence: int,
    channel_error: Exception,
) -> np.ndarray:
    """Tente enregistrement avec détection automatique des canaux."""
    is_ci = os.environ.get("CI", "false").lower() == "true"
    if is_ci:
        logging.debug(
            "Erreur canaux audio (Issue #329): %s. "
            "Tentative avec configuration par défaut...",
            channel_error,
        )
    else:
        logging.warning(
            "⚠️ Erreur canaux audio (Issue #329): %s. "
            "Tentative avec configuration par défaut...",
            channel_error,
        )

    try:
        sd_module = _get_sd()
        if sd_module is None:
            raise ImportError("sounddevice non disponible")

        default_device = sd_module.default.device
        device_info = sd_module.query_devices(default_device[0])
        channels = device_info.get("max_input_channels", 1)
        logging.info("📊 Canaux disponibles détectés: %d", channels)

        return _sd.rec(
            int(duree * frequence),
            samplerate=frequence,
            channels=min(channels, 1),
            dtype="int16",
        )
    except Exception as fallback_error:
        if is_ci:
            logging.debug(
                "Échec enregistrement audio même avec fallback: %s",
                fallback_error,
            )
        else:
            logging.error(
                "❌ Échec enregistrement audio même avec fallback: %s",
                fallback_error,
            )
        msg = f"Impossible d'enregistrer audio: {fallback_error}"
        raise RuntimeError(msg) from fallback_error


def enregistrer_audio(
    fichier: str,
    duree: int = 3,
    frequence: int = DEFAULT_SAMPLE_RATE,
    robot_api: Optional["RobotAPI"] = None,
    max_buffer_duration: int | None = None,
) -> bool:
    """Enregistre un fichier audio (WAV) depuis le micro.

    Vérifie flag BBIA_DISABLE_AUDIO et valide paramètres avant enregistrement.

    Utilise robot.media.microphone (4 microphones SDK) si disponible,
    sinon utilise sounddevice pour compatibilité.

    Issue #436: Limite taille buffer pour éviter OOM sur Raspberry Pi.
    Par défaut, limite à 3 minutes si max_buffer_duration non spécifié.

    Args:
        fichier: Chemin du fichier de sortie
        duree: Durée d'enregistrement en secondes
        frequence: Fréquence d'échantillonnage
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.microphone
        max_buffer_duration: Durée max buffer en secondes (défaut: 180s = 3 min)

    """
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): enregistrement ignoré")
        return False

    # Issue #436: Limiter durée buffer pour éviter OOM
    if max_buffer_duration is None:
        max_buffer_duration = int(
            os.environ.get("BBIA_MAX_AUDIO_BUFFER_DURATION", "180"),
        )

    if max_buffer_duration <= 0:
        max_buffer_duration = 180

    if duree > max_buffer_duration:
        logging.warning(
            "⚠️ Durée d'enregistrement (%ds) dépasse limite buffer (%ds). "
            "Limitation appliquée pour éviter OOM.",
            duree,
            max_buffer_duration,
        )
        duree = max_buffer_duration

    # Sécurité: valider chemin de sortie
    if not _is_safe_path(fichier):
        msg = "Chemin de sortie non autorisé (path traversal)"
        raise ValueError(msg)

    # Tenter enregistrement via SDK
    microphone_sdk = _get_robot_media_microphone(robot_api)
    sdk_result = _enregistrer_via_sdk(
        fichier, duree, frequence, robot_api, microphone_sdk
    )
    if sdk_result is True:
        return True

    # Fallback: sounddevice
    try:
        return _enregistrer_via_sounddevice(fichier, duree, frequence)
    except Exception as e:
        is_ci = os.environ.get("CI", "false").lower() == "true"
        if is_ci:
            logging.debug("Erreur d'enregistrement audio: %s", e)
        else:
            logging.error("Erreur d'enregistrement audio: %s", e)
        raise


def lire_audio(fichier: str, robot_api: Optional["RobotAPI"] = None) -> None:
    """Joue un fichier audio WAV.

    Utilise robot.media.speaker si disponible (haut-parleur 5W optimisé),
    sinon utilise sounddevice pour compatibilité.

    Args:
        fichier: Chemin du fichier audio
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.speaker

    """
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): '%s' ignoré", fichier)
        return

    # Sécurité: valider chemin d'entrée
    if not _is_safe_path(fichier):
        msg = "Chemin d'entrée non autorisé (path traversal)"
        raise ValueError(msg)

    # Validation format et sample rate SDK (si soundfile dispo)
    if soundfile is not None:
        try:
            info = soundfile.info(fichier)
            if info.samplerate != DEFAULT_SAMPLE_RATE:
                # Log en debug en CI (warning attendu)
                if os.environ.get("CI", "false").lower() == "true":
                    logging.debug(
                        "Sample rate %s Hz != SDK standard %s Hz. "
                        "Performance audio peut être dégradée.",
                        info.samplerate,
                        DEFAULT_SAMPLE_RATE,
                    )
                else:
                    logging.warning(
                        "⚠️  Sample rate %s Hz != SDK standard %s Hz. "
                        "Performance audio peut être dégradée.",
                        info.samplerate,
                        DEFAULT_SAMPLE_RATE,
                    )
        except (OSError, AttributeError, RuntimeError):
            # Ignorer toute erreur côté soundfile, fallback plus bas
            logger.debug("Impossible de lire métadonnées audio avec soundfile")

    # OPTIMISATION SDK: Utiliser robot.media.speaker si disponible
    # (toujours disponible via shim)
    if robot_api and hasattr(robot_api, "media"):
        media = robot_api.media
        # Media est maintenant toujours disponible (shim en simulation)
        if media:
            try:
                speaker = getattr(media, "speaker", None)
                play_audio = getattr(media, "play_audio", None)

                if play_audio is not None:
                    # OPTIMISATION SDK: Lecture via robot.media.play_audio()
                    # Bénéfice: Haut-parleur 5W optimisé hardware
                    # avec qualité supérieure
                    logging.info("Lecture via SDK (haut-parleur 5W) : %s...", fichier)
                    # Lire le fichier audio en bytes
                    with open(fichier, "rb") as f:
                        audio_bytes = f.read()
                    play_audio(audio_bytes)
                    logging.info("Lecture SDK terminée.")
                    return
                if speaker is not None and hasattr(speaker, "play"):
                    # Alternative: speaker.play() si disponible
                    with open(fichier, "rb") as f:
                        audio_bytes = f.read()
                    speaker.play(audio_bytes)
                    logging.info("Lecture SDK terminée.")
                    return
            except Exception:
                logging.debug("Erreur lecture SDK (fallback sounddevice)")
                # Fallback vers sounddevice

    # Fallback: sounddevice (compatibilité)
    try:
        with wave.open(fichier, "rb") as wf:
            frequence = wf.getframerate()
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype="int16")
            _sd = _get_sd()
            if _sd is None:
                msg = "sounddevice indisponible (lecture audio)"
                raise RuntimeError(msg)
            _sd.play(audio, frequence)
            _sd.wait()
        logging.info("Lecture de %s terminée.", fichier)
    except Exception as e:
        # Log en debug en CI ou dans les tests (erreurs attendues avec mocks)
        # Détecter si on est dans pytest
        is_pytest = "pytest" in sys.modules or os.environ.get("PYTEST_CURRENT_TEST")
        if os.environ.get("CI", "false").lower() == "true" or is_pytest:
            logging.debug("Erreur de lecture audio: %s", e)
        else:
            logging.exception("Erreur de lecture audio")
        raise


def detecter_son(fichier: str, seuil: int = 500) -> bool:
    """Détecte si un fichier audio contient du son.

    Args:
        fichier: Chemin du fichier audio
        seuil: Seuil d'amplitude pour détecter du son

    Returns:
        True si son détecté, False sinon

    """
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): détection ignorée")
        return False  # Retourner False car pas de son possible si audio désactivé
    try:
        with wave.open(fichier, "rb") as wf:
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype="int16")
            # Gérer le cas d'un tableau vide (fichier audio vide)
            if audio.size == 0:
                logging.info("Fichier audio vide, aucun son détecté")
                return False
            max_val: float = np.max(np.abs(audio))
            logging.info("Amplitude max détectée : %s", max_val)
            return max_val > seuil
    except Exception:
        logging.exception("Erreur de détection de son")
        return False


if __name__ == "__main__":
    fichier = "bbia_demo.wav"
    enregistrer_audio(fichier, duree=3, frequence=16000)
    if detecter_son(fichier, seuil=500):
        logger.info("Son détecté dans %s", fichier)
    else:
        logger.info("Aucun son détecté dans %s", fichier)
    lire_audio(fichier)
