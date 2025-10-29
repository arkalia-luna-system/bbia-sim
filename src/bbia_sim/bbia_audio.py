"""Module bbia_audio.py
Gestion de l'audio pour BBIA : enregistrement, lecture, détection de son.
Utilise robot.media.microphone si disponible (SDK Reachy Mini officiel - 4 microphones),
sinon utilise sounddevice pour compatibilité.
Compatible macOS, simple, portable, testé.
"""

import atexit
import logging
import wave
from typing import TYPE_CHECKING, Optional

import numpy as np
import sounddevice as sd

if TYPE_CHECKING:
    from .robot_api import RobotAPI


# Gestion propre de PortAudio pour éviter les erreurs de terminaison
def _cleanup_sounddevice() -> None:
    """Nettoyage propre de sounddevice pour éviter les erreurs PortAudio."""
    try:
        sd._terminate()
    except Exception:  # nosec B110
        pass  # Ignorer les erreurs de terminaison


atexit.register(_cleanup_sounddevice)

logging.basicConfig(level=logging.INFO)


def _get_robot_media_microphone(
    robot_api: Optional["RobotAPI"] = None,  # noqa: ANN401
) -> Optional[object]:
    """Récupère robot.media.microphone si disponible.

    Args:
        robot_api: Interface RobotAPI (optionnel)

    Returns:
        robot.media.microphone ou None
    """
    if robot_api and hasattr(robot_api, "media") and robot_api.media:
        try:
            return getattr(robot_api.media, "microphone", None)
        except Exception:
            return None
    return None


def enregistrer_audio(
    fichier: str,
    duree: int = 3,
    frequence: int = 16000,
    robot_api: Optional["RobotAPI"] = None,  # noqa: ANN401
) -> bool:
    """Enregistre un fichier audio (WAV) depuis le micro.

    Utilise robot.media.microphone (4 microphones SDK) si disponible,
    sinon utilise sounddevice pour compatibilité.

    Args:
        fichier: Chemin du fichier de sortie
        duree: Durée d'enregistrement en secondes
        frequence: Fréquence d'échantillonnage
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.microphone
    """
    # OPTIMISATION SDK: Utiliser robot.media.microphone si disponible
    microphone_sdk = _get_robot_media_microphone(robot_api)
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
                    f"Enregistrement via SDK (4 microphones) ({duree}s) dans {fichier}..."
                )
                audio_data = robot_api.media.record_audio(
                    duration=duree, sample_rate=frequence
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
            elif hasattr(microphone_sdk, "record"):
                # Alternative: microphone.record() si disponible
                audio_data = microphone_sdk.record(
                    duration=duree, sample_rate=frequence
                )
                with wave.open(fichier, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(frequence)
                    wf.writeframes(
                        bytes(audio_data)
                        if isinstance(audio_data, (bytes, bytearray))
                        else audio_data.tobytes()
                    )
                logging.info("Enregistrement SDK terminé.")
                return True
        except Exception as e:
            logging.debug(f"Erreur enregistrement SDK (fallback sounddevice): {e}")
            # Fallback vers sounddevice

    # Fallback: sounddevice (compatibilité)
    try:
        logging.info(f"Enregistrement audio ({duree}s) dans {fichier}...")
        audio = sd.rec(
            int(duree * frequence), samplerate=frequence, channels=1, dtype="int16"
        )
        sd.wait()
        with wave.open(fichier, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(frequence)
            wf.writeframes(audio.tobytes())
        logging.info("Enregistrement terminé.")
        return True
    except Exception as e:
        logging.error(f"Erreur d'enregistrement audio : {e}")
        raise


def lire_audio(fichier: str, robot_api: Optional["RobotAPI"] = None) -> None:
    """Joue un fichier audio WAV.

    Utilise robot.media.speaker si disponible (haut-parleur 5W optimisé),
    sinon utilise sounddevice pour compatibilité.

    Args:
        fichier: Chemin du fichier audio
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.speaker
    """
    # OPTIMISATION SDK: Utiliser robot.media.speaker si disponible
    if robot_api and hasattr(robot_api, "media") and robot_api.media:
        try:
            speaker = getattr(robot_api.media, "speaker", None)
            play_audio = getattr(robot_api.media, "play_audio", None)

            if play_audio is not None:
                # OPTIMISATION SDK: Lecture via robot.media.play_audio()
                # Bénéfice: Haut-parleur 5W optimisé hardware avec qualité supérieure
                logging.info(f"Lecture via SDK (haut-parleur 5W) : {fichier}...")
                # Lire le fichier audio en bytes
                with open(fichier, "rb") as f:
                    audio_bytes = f.read()
                play_audio(audio_bytes)
                logging.info("Lecture SDK terminée.")
                return
            elif speaker is not None and hasattr(speaker, "play"):
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
            sd.play(audio, frequence)
            sd.wait()
        logging.info(f"Lecture de {fichier} terminée.")
    except Exception as e:
        logging.error(f"Erreur de lecture audio : {e}")
        # Ne rien retourner, la fonction déclare retourner None


def detecter_son(fichier: str, seuil: int = 500) -> bool:
    """Détecte la présence d'un son dans un fichier WAV (seuil simple).
    Retourne True si un son est détecté, False sinon.
    """
    try:
        with wave.open(fichier, "rb") as wf:
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype="int16")
            max_val: float = np.max(np.abs(audio))
            logging.info(f"Amplitude max détectée : {max_val}")
            return max_val > seuil
    except Exception as e:
        logging.error(f"Erreur de détection de son : {e}")
        return False


if __name__ == "__main__":
    fichier = "bbia_demo.wav"
    enregistrer_audio(fichier, duree=3, frequence=16000)
    if detecter_son(fichier, seuil=500):
        pass
    else:
        pass
    lire_audio(fichier)
