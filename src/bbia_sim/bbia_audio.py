"""Module bbia_audio.py
Gestion de l'audio pour BBIA : enregistrement, lecture, détection de son.
Compatible macOS, simple, portable, testé.
"""

import logging
import wave

import numpy as np
import sounddevice as sd

logging.basicConfig(level=logging.INFO)


def enregistrer_audio(fichier, duree=3, frequence=16000):
    """Enregistre un fichier audio (WAV) depuis le micro."""
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
    except Exception as e:
        logging.error(f"Erreur d'enregistrement audio : {e}")
        raise


def lire_audio(fichier):
    """Joue un fichier audio WAV."""
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
        raise


def detecter_son(fichier, seuil=500):
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
