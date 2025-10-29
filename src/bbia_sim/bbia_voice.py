"""Module bbia_voice.py.

Synthèse et reconnaissance vocale pour BBIA.
Compatible macOS, simple, portable, testé.

Voix : sélection automatique de la voix la plus proche de Reachy Mini Wireless (féminine, française si possible).
Aucune voix officielle n’est documentée publiquement : cette configuration vise à s’en rapprocher au maximum.
"""

import logging
import os
import sys
import threading
import time
import unicodedata
from typing import Any, Optional

import pyttsx3
import speech_recognition as sr

logging.basicConfig(level=logging.INFO)

# Liste des voix féminines douces/enfantines à privilégier sur macOS (ordre de préférence)
VOIX_FEMMES_MAC = [
    "Amelie",  # enfantine, douce
    "Audrey",  # douce
    "Virginie",  # douce
    "Julie",  # classique
]

# ⚡ OPTIMISATION PERFORMANCE: Cache global pour éviter réinitialisation répétée
_pyttsx3_engine_cache: Optional[Any] = None
_bbia_voice_id_cache: Optional[str] = None
_pyttsx3_lock = threading.Lock()


def _get_pyttsx3_engine() -> Any:
    """Retourne le moteur pyttsx3 en cache ou le crée si absent.

    ⚡ OPTIMISATION PERFORMANCE: Cache pour éviter réinitialisation (0.8s par appel).

    Returns:
        Moteur pyttsx3 réutilisable

    """
    global _pyttsx3_engine_cache
    if _pyttsx3_engine_cache is None:
        with _pyttsx3_lock:
            # Double check après lock (thread-safe)
            if _pyttsx3_engine_cache is None:
                _pyttsx3_engine_cache = pyttsx3.init()
                logging.debug("✅ Moteur pyttsx3 initialisé (cache créé)")
    return _pyttsx3_engine_cache


def _get_cached_voice_id() -> str:
    """Retourne l'ID de voix BBIA en cache.

    ⚡ OPTIMISATION PERFORMANCE: Cache pour éviter recherche répétée des voix.

    Returns:
        ID de voix BBIA (Amelie)

    """
    global _bbia_voice_id_cache
    if _bbia_voice_id_cache is None:
        engine = _get_pyttsx3_engine()
        _bbia_voice_id_cache = get_bbia_voice(engine)
        logging.debug(f"✅ Voice ID mis en cache: {_bbia_voice_id_cache}")
    return _bbia_voice_id_cache


def get_bbia_voice(engine: Any) -> str:
    """Force l’utilisation d’une seule voix féminine douce/enfantine sur macOS.
    Prend la première voix dont le nom contient 'Amelie' (toute variante, accent ou non),
    en priorité France (fr_FR), sinon Canada (fr_CA), sinon toute Amelie.
    Si aucune voix n’est trouvée, lève une erreur explicite avec un message d’aide.
    """
    voices = engine.getProperty("voices")

    def normalize(s: str) -> str:
        return (
            unicodedata.normalize("NFKD", s)
            .encode("ASCII", "ignore")
            .decode("ASCII")
            .lower()
        )

    # 1. Prio France
    for v in voices:
        if "amelie" in normalize(v.name) and ("fr_FR" in v.id or "fr-FR" in v.id):
            return str(v.id)  # type: ignore[no-any-return]
    # 2. Prio Canada
    for v in voices:
        if "amelie" in normalize(v.name) and ("fr_CA" in v.id or "fr-CA" in v.id):
            return str(v.id)  # type: ignore[no-any-return]
    # 3. Toute Amelie
    for v in voices:
        if "amelie" in normalize(v.name):
            return str(v.id)  # type: ignore[no-any-return]
    # 4. Sinon, message d’aide
    raise RuntimeError(
        "Aucune voix 'Amélie' n’est installée sur ce Mac. Va dans Préférences Système > Accessibilité > Parole > Voix du système et installe une voix française féminine (ex: Amélie).",
    )


def dire_texte(texte: str, robot_api: Optional[Any] = None) -> None:
    """Lit un texte à voix haute (TTS) avec la voix la plus fidèle à Reachy Mini Wireless.

    Utilise robot.media.speaker si disponible (haut-parleur 5W optimisé SDK),
    sinon utilise pyttsx3 pour compatibilité.

    Args:
        texte: Texte à prononcer
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.speaker

    """
    # OPTIMISATION SDK: Utiliser robot.media.speaker si disponible
    # CORRECTION EXPERTE: Logique alignée avec bbia_voice_advanced.py pour robustesse
    if robot_api and hasattr(robot_api, "media") and robot_api.media:
        try:
            media = robot_api.media

            # ⚡ OPTIMISATION PERFORMANCE: Générer audio une seule fois avec cache
            import tempfile

            engine = _get_pyttsx3_engine()
            voice_id = _get_cached_voice_id()
            engine.setProperty("voice", voice_id)
            engine.setProperty("rate", 170)
            engine.setProperty("volume", 1.0)

            # Sauvegarder dans fichier temporaire
            tmp_path = None
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                    tmp_path = tmp.name
                    engine.save_to_file(texte, tmp_path)
                    engine.runAndWait()

                # OPTIMISATION SDK: Priorité 1 - robot.media.play_audio(bytes, volume)
                # Méthode la plus directe du SDK officiel
                with open(tmp_path, "rb") as f:
                    audio_bytes = f.read()

                try:
                    if hasattr(media, "play_audio"):
                        # Essayer avec volume d'abord (optimal pour haut-parleur 5W)
                        try:
                            media.play_audio(audio_bytes, volume=1.0)  # type: ignore[attr-defined]
                            logging.info(
                                f"✅ Synthèse vocale SDK (haut-parleur 5W via play_audio) : {texte}",
                            )
                            return
                        except TypeError:
                            # Fallback si signature sans volume
                            media.play_audio(audio_bytes)  # type: ignore[attr-defined]
                            logging.info(
                                f"✅ Synthèse vocale SDK (haut-parleur 5W via play_audio) : {texte}",
                            )
                            return
                except Exception as e:
                    logging.debug(f"media.play_audio indisponible: {e}")

                # OPTIMISATION SDK: Priorité 2 - robot.media.speaker.play_file(path) ou .play(bytes)
                # Méthode alternative du SDK si play_audio non disponible
                try:
                    speaker = getattr(media, "speaker", None)
                    if speaker is not None:
                        # Essayer play_file si disponible (plus simple)
                        if hasattr(speaker, "play_file"):
                            speaker.play_file(tmp_path)  # type: ignore[attr-defined]
                            logging.info(
                                f"✅ Synthèse vocale SDK (haut-parleur 5W via speaker.play_file) : {texte}",
                            )
                            return
                        # Fallback: play(bytes)
                        if hasattr(speaker, "play"):
                            speaker.play(audio_bytes)  # type: ignore[attr-defined]
                            logging.info(
                                f"✅ Synthèse vocale SDK (haut-parleur 5W via speaker.play) : {texte}",
                            )
                            return
                        # Alternative: speaker.say() si TTS intégré dans SDK
                        if hasattr(speaker, "say"):
                            speaker.say(texte)  # type: ignore[attr-defined]
                            logging.info(
                                f"✅ Synthèse vocale SDK (haut-parleur 5W via speaker.say) : {texte}",
                            )
                            return
                except Exception as e:
                    logging.debug(f"media.speaker indisponible: {e}")

            finally:
                # Nettoyer fichier temporaire même en cas d'erreur
                if tmp_path and os.path.exists(tmp_path):
                    try:
                        os.unlink(tmp_path)
                    except Exception:  # noqa: B110
                        pass  # Ignorer erreurs de nettoyage

        except Exception as e:
            logging.debug(f"Erreur synthèse SDK (fallback pyttsx3): {e}")
            # Fallback pyttsx3

    # Fallback: pyttsx3 (compatibilité)
    try:
        logging.info(f"Synthèse vocale : {texte}")
        # ⚡ OPTIMISATION PERFORMANCE: Utiliser moteur en cache (évite 0.8s d'init)
        engine = _get_pyttsx3_engine()
        voice_id = _get_cached_voice_id()
        engine.setProperty("voice", voice_id)
        engine.setProperty("rate", 170)  # Vitesse normale
        engine.setProperty("volume", 1.0)
        # Pitch non supporté nativement par pyttsx3, dépend du moteur
        engine.say(texte)
        engine.runAndWait()
    except Exception as e:
        logging.error(f"Erreur de synthèse vocale : {e}")
        raise


def reconnaitre_parole(
    duree: int = 3, frequence: int = 16000, robot_api: Optional[Any] = None
) -> str | None:
    """Reconnaît la parole via le micro (STT, français par défaut).

    Utilise robot.media.microphone (4 microphones SDK) si disponible,
    sinon utilise speech_recognition pour compatibilité.

    Args:
        duree: Durée d'enregistrement en secondes
        frequence: Fréquence d'échantillonnage
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.microphone

    Returns:
        Texte reconnu ou None

    """
    # OPTIMISATION SDK: Utiliser robot.media.microphone si disponible
    if robot_api and hasattr(robot_api, "media") and robot_api.media:
        try:
            # OPTIMISATION SDK: Enregistrement via robot.media.record_audio()
            # Bénéfice: 4 microphones directionnels avec annulation de bruit automatique
            if hasattr(robot_api.media, "record_audio"):
                logging.info(
                    f"🎤 Enregistrement via SDK (4 microphones) ({duree}s) pour reconnaissance...",
                )
                audio_data = robot_api.media.record_audio(
                    duration=duree,
                    sample_rate=frequence,
                )

                # Convertir audio_data en format pour speech_recognition
                import io
                import wave

                # Créer fichier WAV temporaire en mémoire
                audio_io = io.BytesIO()
                with wave.open(audio_io, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(frequence)
                    if isinstance(audio_data, bytes):
                        wf.writeframes(audio_data)
                    else:
                        # Gérer numpy.ndarray ou autres types
                        try:
                            import numpy as np_module

                            if isinstance(audio_data, np_module.ndarray):
                                wf.writeframes(
                                    (audio_data.astype(np_module.int16)).tobytes(),
                                )
                            else:
                                wf.writeframes(bytes(audio_data))
                        except ImportError:
                            # Fallback si numpy non disponible
                            wf.writeframes(bytes(audio_data))

                audio_io.seek(0)

                # Reconnaître avec speech_recognition
                r = sr.Recognizer()
                with sr.AudioFile(audio_io) as source:
                    audio = r.record(source)
                    texte = r.recognize_google(audio, language="fr-FR")
                    logging.info(f"✅ Texte reconnu (SDK 4 microphones) : {texte}")
                    return str(texte)  # type: ignore[no-any-return]
        except Exception as e:
            logging.debug(
                f"Erreur reconnaissance SDK (fallback speech_recognition): {e}",
            )
            # Fallback vers speech_recognition

    # Fallback: speech_recognition (compatibilité)
    try:
        r = sr.Recognizer()
        with sr.Microphone(sample_rate=frequence) as source:
            try:
                logging.info("Écoute du micro pour la reconnaissance vocale...")
                audio = r.listen(source, phrase_time_limit=duree)
                texte = r.recognize_google(audio, language="fr-FR")
                logging.info(f"Texte reconnu : {texte}")
                return str(texte)  # type: ignore[no-any-return]
            except sr.UnknownValueError:
                logging.warning("Aucune parole reconnue.")
                return None
            except Exception as e:
                logging.error(f"Erreur de reconnaissance vocale : {e}")
                return None
    except Exception as e:
        logging.error(f"Erreur d'accès au microphone : {e}")
        logging.warning(
            "La reconnaissance vocale nécessite pyaudio. Installez-le avec : pip install pyaudio",
        )
        return None


def lister_voix_disponibles() -> None:
    """Affiche la liste des voix TTS disponibles avec leurs propriétés."""
    # ⚡ OPTIMISATION PERFORMANCE: Utiliser cache au lieu de pyttsx3.init()
    engine = _get_pyttsx3_engine()
    voices = engine.getProperty("voices")
    for _idx, v in enumerate(voices):
        try:
            _ = (
                v.languages[0].decode(errors="ignore")
                if hasattr(v.languages[0], "decode")
                else str(v.languages[0])
            )
        except Exception:
            _ = str(v.languages)  # type: ignore[no-any-return]


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "list-voices":
        lister_voix_disponibles()
        # Optionnel : démo d'écoute de chaque voix
        if len(sys.argv) > 2 and sys.argv[2] == "demo":
            # ⚡ OPTIMISATION PERFORMANCE: Utiliser cache au lieu de pyttsx3.init()
            engine = _get_pyttsx3_engine()
            voices = engine.getProperty("voices")
            for v in voices:
                engine.setProperty("voice", v.id)
                engine.setProperty("rate", 170)
                engine.setProperty("volume", 1.0)
                engine.say(f"Bonjour, je suis la voix {v.name}.")
                engine.runAndWait()
        sys.exit(0)

    # ⚡ OPTIMISATION PERFORMANCE: Utiliser cache au lieu de pyttsx3.init()
    engine = _get_pyttsx3_engine()
    voice_id = _get_cached_voice_id()
    engine.setProperty("voice", voice_id)
    engine.setProperty("rate", 170)
    engine.setProperty("volume", 1.0)
    demo_texte = "Bonjour, je suis BBIA. Je fonctionne sur votre Mac. Ceci est la voix la plus proche de Reachy Mini Wireless."
    engine.say(demo_texte)
    engine.runAndWait()

    sys.stdout.flush()
    time.sleep(0.5)
    # Note: robot_api non disponible dans __main__, donc fallback automatique
    texte = reconnaitre_parole(duree=3, frequence=16000, robot_api=None)
    if texte:
        dire_texte(f"Vous avez dit : {texte}")
    else:
        dire_texte("Je n'ai rien compris.")
