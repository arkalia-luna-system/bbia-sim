"""Module bbia_audio.py
Gestion de l'audio pour BBIA : enregistrement, lecture, détection de son.
Utilise robot.media.microphone si disponible (SDK Reachy Mini officiel - 4 microphones),
sinon utilise sounddevice pour compatibilité.
Compatible macOS, simple, portable, testé.
"""

import logging
import os
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
        *args: Any,
        **kwargs: Any,
    ) -> Any:  # noqa: ANN401 - garde-fou pour module optionnel
        raise RuntimeError("sounddevice indisponible: sd.rec non opérationnel")

    def wait(
        self,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        raise RuntimeError("sounddevice indisponible: sd.wait non opérationnel")

    def play(
        self,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        raise RuntimeError("sounddevice indisponible: sd.play non opérationnel")


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


def enregistrer_audio(
    fichier: str,
    duree: int = 3,
    frequence: int = DEFAULT_SAMPLE_RATE,
    robot_api: Optional["RobotAPI"] = None,
) -> bool:
    """Enregistre un fichier audio (WAV) depuis le micro.

    Vérifie flag BBIA_DISABLE_AUDIO et valide paramètres avant enregistrement.

    Utilise robot.media.microphone (4 microphones SDK) si disponible,
    sinon utilise sounddevice pour compatibilité.

    Args:
        fichier: Chemin du fichier de sortie
        duree: Durée d'enregistrement en secondes
        frequence: Fréquence d'échantillonnage
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.microphone

    """
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): enregistrement ignoré")
        return False

    # Sécurité: valider chemin de sortie
    if not _is_safe_path(fichier):
        raise ValueError("Chemin de sortie non autorisé (path traversal)")

    # OPTIMISATION SDK: Utiliser robot.media.microphone si disponible
    # (toujours disponible via shim)
    microphone_sdk = _get_robot_media_microphone(robot_api)
    # microphone_sdk peut être None uniquement si robot_api.media n'existe pas du tout
    if microphone_sdk is not None:
        try:
            # OPTIMISATION SDK: Enregistrement via robot.media.record_audio()
            # Bénéfice: 4 microphones directionnels avec annulation de bruit automatique
            if (
                robot_api
                and hasattr(robot_api, "media")
                and robot_api.media
                and hasattr(robot_api.media, "record_audio")
            ):
                logging.info(
                    (
                        f"Enregistrement via SDK (4 microphones) ({duree}s) "
                        f"dans {fichier}..."
                    ),
                )
                audio_data = robot_api.media.record_audio(
                    duration=duree,
                    sample_rate=frequence,
                )
                # Sauvegarder dans fichier WAV
                with wave.open(fichier, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(frequence)
                    if isinstance(audio_data, bytes):
                        wf.writeframes(audio_data)
                    elif isinstance(audio_data, np.ndarray):
                        wf.writeframes((audio_data.astype(np.int16)).tobytes())
                    else:
                        # Convertir si nécessaire
                        wf.writeframes(bytes(audio_data))
                logging.info("Enregistrement SDK terminé.")
                return True
            if hasattr(microphone_sdk, "record"):
                # Alternative: microphone.record() si disponible
                audio_data = microphone_sdk.record(
                    duration=duree,
                    sample_rate=frequence,
                )
                with wave.open(fichier, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(frequence)
                    wf.writeframes(
                        (
                            bytes(audio_data)
                            if isinstance(audio_data, bytes | bytearray)
                            else audio_data.tobytes()
                        ),
                    )
                logging.info("Enregistrement SDK terminé.")
                return True
        except Exception as e:
            logging.debug(f"Erreur enregistrement SDK (fallback sounddevice): {e}")
            # Fallback vers sounddevice

    # Fallback: sounddevice (compatibilité)
    try:
        logging.info(f"Enregistrement audio ({duree}s) dans {fichier}...")
        _sd = _get_sd()
        if _sd is None:
            raise RuntimeError(
                "sounddevice indisponible (BBIA_DISABLE_AUDIO conseillé en CI)",
            )
        audio = _sd.rec(
            int(duree * frequence),
            samplerate=frequence,
            channels=1,
            dtype="int16",
        )
        _sd.wait()
        with wave.open(fichier, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(frequence)
            wf.writeframes(audio.tobytes())
        logging.info("Enregistrement terminé.")
        return True
    except Exception as e:
        logging.exception(f"Erreur d'enregistrement audio : {e}")
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
        logging.debug(f"Audio désactivé (BBIA_DISABLE_AUDIO=1): '{fichier}' ignoré")
        return

    # Sécurité: valider chemin d'entrée
    if not _is_safe_path(fichier):
        raise ValueError("Chemin d'entrée non autorisé (path traversal)")

    # Validation format et sample rate SDK (si soundfile dispo)
    if soundfile is not None:
        try:
            info = soundfile.info(fichier)
            if info.samplerate != DEFAULT_SAMPLE_RATE:
                logging.warning(
                    f"⚠️  Sample rate {info.samplerate} Hz != "
                    f"SDK standard {DEFAULT_SAMPLE_RATE} Hz. "
                    f"Performance audio peut être dégradée.",
                )
        except Exception as e:
            # Ignorer toute erreur côté soundfile, fallback plus bas
            logger.debug(
                f"Impossible de lire métadonnées audio avec soundfile, fallback: {e}"
            )

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
                    logging.info(f"Lecture via SDK (haut-parleur 5W) : {fichier}...")
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
            except Exception as e:
                logging.debug(f"Erreur lecture SDK (fallback sounddevice): {e}")
                # Fallback vers sounddevice

    # Fallback: sounddevice (compatibilité)
    try:
        with wave.open(fichier, "rb") as wf:
            frequence = wf.getframerate()
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype="int16")
            _sd = _get_sd()
            if _sd is None:
                raise RuntimeError("sounddevice indisponible (lecture audio)")
            _sd.play(audio, frequence)
            _sd.wait()
        logging.info(f"Lecture de {fichier} terminée.")
    except Exception as e:
        logging.exception(f"Erreur de lecture audio : {e}")
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
            max_val: float = np.max(np.abs(audio))
            logging.info(f"Amplitude max détectée : {max_val}")
            return max_val > seuil
    except Exception as e:
        logging.exception(f"Erreur de détection de son : {e}")
        return False


if __name__ == "__main__":
    fichier = "bbia_demo.wav"
    enregistrer_audio(fichier, duree=3, frequence=16000)
    if detecter_son(fichier, seuil=500):
        pass
    else:
        pass
    lire_audio(fichier)
