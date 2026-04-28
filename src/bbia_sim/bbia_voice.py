"""Module bbia_voice.py.

Synthèse et reconnaissance vocale pour BBIA.
Compatible macOS, simple, portable, testé.

Voix : sélection automatique de la voix la plus proche de
Reachy Mini Wireless (féminine, française si possible).
"""

import contextlib
import io
import logging
import os
import queue
import sys
import tempfile
import threading
import time
import unicodedata
import wave
from pathlib import Path
from typing import Any

import numpy as np
import pyttsx3
import speech_recognition as sr

# Import conditionnel soundfile (optionnel)
try:
    import soundfile as sf

    SOUNDFILE_AVAILABLE = True
except ImportError:
    SOUNDFILE_AVAILABLE = False
    sf = None  # type: ignore[assignment]

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
try:
    # Sélecteurs IA optionnels (TTS local type KittenTTS)
    from .ai_backends import get_tts_backend
except ImportError:  # pragma: no cover - non critique si absent
    get_tts_backend = None  # type: ignore[assignment]

# Import conditionnel voice_whisper (une seule fois)
try:
    from .voice_whisper import WHISPER_AVAILABLE, WhisperSTT
except ImportError:
    WHISPER_AVAILABLE = False
    WhisperSTT = None  # type: ignore[assignment,misc]

# Liste des voix féminines douces/enfantines à privilégier sur macOS
# (ordre de préférence)
VOIX_FEMMES_MAC = [
    "Amelie",  # enfantine, douce
    "Audrey",  # douce
    "Virginie",  # douce
    "Julie",  # classique
]

# ⚡ OPTIMISATION PERFORMANCE: Cache global pour éviter réinitialisation répétée
_pyttsx3_engine_cache: Any | None = None
_bbia_voice_id_cache: str | None = None
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
                # Vérifier si audio est désactivé (CI, tests)
                if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
                    logging.debug("⚠️ Audio désactivé, pyttsx3 non initialisé")
                    return None
                try:
                    _pyttsx3_engine_cache = pyttsx3.init()
                    logging.debug("✅ Moteur pyttsx3 initialisé (cache créé)")
                except (RuntimeError, OSError) as e:
                    # eSpeak non installé ou autre erreur système
                    # Log en debug en CI (warning attendu sans eSpeak)
                    if os.environ.get("CI", "false").lower() == "true":
                        logging.debug(f"pyttsx3 non disponible: {e}")
                    else:
                        logging.warning(f"⚠️ pyttsx3 non disponible: {e}")
                    _pyttsx3_engine_cache = None
                    return None
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
        if engine is None:
            # Audio désactivé ou eSpeak non disponible
            # Log en debug en CI (warning attendu sans eSpeak)
            if os.environ.get("CI", "false").lower() == "true":
                logging.debug("pyttsx3 non disponible, utilisation voix par défaut")
            else:
                logging.warning("⚠️ pyttsx3 non disponible, utilisation voix par défaut")
            _bbia_voice_id_cache = (
                "com.apple.speech.voice.Amelie.fr-FR"  # Fallback macOS
            )
        else:
            _bbia_voice_id_cache = get_bbia_voice(engine)
            logging.debug("✅ Voice ID mis en cache: %s", _bbia_voice_id_cache)
    return _bbia_voice_id_cache


# Constantes pour la sélection de voix
_MALE_VOICE_INDICATORS = frozenset(
    ["thomas", "jacques", "reed", "rocko", "eddy", "grandpa", "daniel"]
)
_FEMALE_FR_VOICES = frozenset(
    ["audrey", "virginie", "julie", "flo", "sandy", "shelley"]
)
_VOICE_PRIORITY_KEYS = [
    "aurelie_enhanced_fr",
    "amelie_enhanced_fr",
    "aurelie_fr_FR",
    "aurelie_fr_CA",
    "amelie_fr_FR",
    "amelie_fr_CA",
    "aurelie_any",
    "amelie_any",
    "femme_fr",
]


def _normalize_voice_name(s: str) -> str:
    """Normalise un nom de voix pour comparaison."""
    return (
        unicodedata.normalize("NFKD", s)
        .encode("ASCII", "ignore")
        .decode("ASCII")
        .lower()
    )


def _categorize_voice(v: Any) -> tuple[str | None, str]:
    """Catégorise une voix selon la priorité.

    Returns:
        Tuple (catégorie, voice_id) ou (None, "") si non catégorisée

    """
    v_id = str(v.id)
    v_id_lower = v_id.lower()
    v_name_lower = _normalize_voice_name(v.name)

    is_aurelie = "aurelie" in v_name_lower or "aurélie" in v_name_lower
    is_amelie = "amelie" in v_name_lower
    has_enhanced = "enhanced" in v_id_lower
    is_fr = "fr" in v_id_lower
    is_fr_FR = "fr_FR" in v_id or "fr-FR" in v_id
    is_fr_CA = "fr_CA" in v_id or "fr-CA" in v_id
    is_male = any(ind in v_name_lower for ind in _MALE_VOICE_INDICATORS)

    if is_aurelie and has_enhanced and is_fr:
        return "aurelie_enhanced_fr", v_id
    if is_amelie and has_enhanced and is_fr:
        return "amelie_enhanced_fr", v_id
    if is_aurelie and is_fr_FR:
        return "aurelie_fr_FR", v_id
    if is_aurelie and is_fr_CA:
        return "aurelie_fr_CA", v_id
    if is_amelie and is_fr_FR:
        return "amelie_fr_FR", v_id
    if is_amelie and is_fr_CA:
        return "amelie_fr_CA", v_id
    if is_aurelie:
        return "aurelie_any", v_id
    if is_amelie:
        return "amelie_any", v_id
    if not is_male and is_fr:
        if any(nom in v_name_lower for nom in _FEMALE_FR_VOICES):
            return "femme_fr", v_id

    return None, ""


def get_bbia_voice(engine: Any) -> str:
    """Force l'utilisation d'une voix féminine française de qualité sur macOS.

    Priorité de sélection :
    1. Aurelie Enhanced (fr-FR) - Meilleure qualité
    2. Amelie Enhanced (si disponible)
    3. Aurelie standard (fr-FR, fr-CA)
    4. Amelie (fr_FR, fr_CA, puis toute variante)
    5. Autres voix féminines françaises (Audrey, Virginie, Julie)

    Si aucune voix féminine française n'est trouvée, lève une erreur explicite.
    """
    if engine is None:
        return "com.apple.speech.voice.Amelie.fr-FR"

    voices = engine.getProperty("voices")
    candidates: dict[str, str | None] = dict.fromkeys(_VOICE_PRIORITY_KEYS, None)

    # Une seule passe sur toutes les voix
    for v in voices:
        category, voice_id = _categorize_voice(v)
        if category and candidates[category] is None:
            candidates[category] = voice_id

    # Retourner le premier candidat selon priorité
    for key in _VOICE_PRIORITY_KEYS:
        if candidates[key] is not None:
            return candidates[key]  # type: ignore[return-value]

    msg = (
        "Aucune voix française féminine n'est installée sur ce Mac. "
        "Va dans Préférences Système > Accessibilité > Parole > "
        "Voix du système et installe une voix française féminine "
        "(ex: Aurélie Enhanced, Amélie)."
    )
    raise RuntimeError(msg)


def _play_wav_via_sdk(
    wav_path: str,
    robot_api: Any | None,
) -> bool:
    """Tente de jouer un WAV via le SDK.

    Returns:
        True si succès, False sinon

    """
    if not robot_api or not hasattr(robot_api, "media") or not robot_api.media:
        return False

    try:
        media = robot_api.media
        with open(wav_path, "rb") as f:
            audio_bytes = f.read()

        if hasattr(media, "play_audio"):
            try:
                media.play_audio(audio_bytes, volume=1.0)
            except TypeError:
                media.play_audio(audio_bytes)
            return True

        speaker = getattr(media, "speaker", None)
        if speaker is not None:
            if hasattr(speaker, "play_file"):
                speaker.play_file(wav_path)
                return True
            if hasattr(speaker, "play"):
                speaker.play(audio_bytes)
                return True
    except (AttributeError, RuntimeError, OSError, TypeError) as e:
        logger.debug("Erreur lecture audio via SDK speaker: %s", e)

    return False


def _play_wav_via_sounddevice(wav_path: str) -> bool:
    """Tente de jouer un WAV via sounddevice.

    Returns:
        True si succès, False sinon

    """
    try:
        import sounddevice as _sd

        with wave.open(wav_path, "rb") as wf:
            sr = wf.getframerate()
            frames = wf.readframes(wf.getnframes())
            data = np.frombuffer(frames, dtype=np.int16)
            _sd.play(data, sr)
            _sd.wait()
        return True
    except (OSError, RuntimeError, ImportError) as e:
        logger.debug("Erreur lecture audio sounddevice: %s", e)
    except (TypeError, IndexError, KeyError) as e:
        logger.debug("Erreur lecture audio sounddevice (type/index/key): %s", e)
    except Exception as e:  # noqa: BLE001
        logger.debug("Erreur inattendue lecture audio sounddevice: %s", e)

    return False


def _synthesize_via_tts_backend(
    texte: str,
    robot_api: Any | None,
) -> bool:
    """Synthétise via backend TTS externe si configuré.

    Returns:
        True si succès, False sinon

    """
    tts_backend_name = os.environ.get("BBIA_TTS_BACKEND")
    if not tts_backend_name or get_tts_backend is None:
        return False

    try:
        backend = get_tts_backend()
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name

        if not backend.synthesize_to_wav(texte, wav_path):
            return False

        # Essayer SDK puis sounddevice
        if _play_wav_via_sdk(wav_path, robot_api):
            return True
        if _play_wav_via_sounddevice(wav_path):
            return True

    except (ImportError, RuntimeError, OSError) as e:
        logger.debug("Erreur synthèse vocale avancée, fallback pyttsx3: %s", e)
    except Exception as e:  # noqa: BLE001
        import traceback

        tb_str = "".join(traceback.format_exception(type(e), e, e.__traceback__))
        if "_get_pyttsx3_engine" in tb_str:
            logger.debug("Erreur _get_pyttsx3_engine, pas de fallback pyttsx3: %s", e)
            raise
        logger.debug("Erreur inattendue synthèse vocale avancée: %s", e)

    return False


def _generate_silence_wav() -> bytes:
    """Génère un court WAV de silence (100ms).

    Returns:
        Bytes du fichier WAV ou bytes vide si erreur

    """
    import struct as _struct

    try:
        sr = 16000
        num_samples = int(0.1 * sr)
        silence = b"".join(_struct.pack("<h", 0) for _ in range(num_samples))

        wav_buf = io.BytesIO()
        with wave.open(wav_buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sr)
            wf.writeframes(silence)
        return wav_buf.getvalue()
    except (ValueError, RuntimeError, TypeError):
        return b""


def _play_via_sdk_media(robot_api: Any | None) -> bool:
    """Tente de jouer via robot.media.* (SDK).

    Returns:
        True si succès, False sinon

    """
    if not robot_api or not hasattr(robot_api, "media") or not robot_api.media:
        return False

    media = robot_api.media
    try:
        sdk_audio_bytes = _generate_silence_wav()
        if not sdk_audio_bytes:
            return False

        # Priorité 1: media.play_audio
        if hasattr(media, "play_audio"):
            try:
                try:
                    media.play_audio(sdk_audio_bytes, volume=1.0)
                except TypeError:
                    media.play_audio(sdk_audio_bytes)
                return True
            except (TypeError, IndexError, KeyError, OSError) as e:
                logging.debug("media.play_audio a échoué: %s", e)
            except Exception as e:  # noqa: BLE001
                logging.debug("media.play_audio a échoué: %s", e)

        # Priorité 2: media.speaker
        speaker = getattr(media, "speaker", None)
        if speaker is not None:
            return _play_via_speaker(speaker, sdk_audio_bytes)

    except (TypeError, ValueError, AttributeError) as e:
        logging.debug("Erreur synthèse SDK: %s", e)
    except Exception as e:  # noqa: BLE001
        logging.debug("Erreur synthèse SDK (fallback pyttsx3): %s", e)

    return False


def _play_via_speaker(speaker: Any, audio_bytes: bytes) -> bool:
    """Tente de jouer via speaker."""
    try:
        if hasattr(speaker, "play"):
            speaker.play(audio_bytes)
            return True
    except (TypeError, IndexError, KeyError, OSError) as e:
        logging.debug("speaker.play a échoué: %s", e)
    except Exception as e:  # noqa: BLE001
        logging.debug("speaker.play a échoué: %s", e)

    # Essayer play_file
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name
            with open(tmp_path, "wb") as f:
                f.write(audio_bytes)
        if hasattr(speaker, "play_file"):
            speaker.play_file(tmp_path)
            return True
    finally:
        if tmp_path:
            try:
                Path(tmp_path).unlink(missing_ok=True)
            except (OSError, PermissionError) as e:
                logger.debug("Erreur nettoyage fichier temporaire: %s", e)

    return False


def _play_via_pyttsx3(texte: str) -> None:
    """Joue le texte via pyttsx3 (fallback)."""
    logging.info("Synthèse vocale : %s", texte)

    engine = _get_pyttsx3_engine()
    if engine is None:
        is_ci = os.environ.get("CI", "false").lower() == "true"
        if is_ci:
            logging.debug("pyttsx3 non disponible, synthèse vocale ignorée")
        else:
            logging.warning("⚠️ pyttsx3 non disponible, synthèse vocale ignorée")
        return

    voice_id = _get_cached_voice_id()
    if engine is None:
        logging.warning("⚠️ pyttsx3 non disponible après récupération voice_id")
        return

    engine.setProperty("voice", voice_id)
    engine.setProperty("rate", 170)
    engine.setProperty("volume", 1.0)
    engine.say(texte)
    engine.runAndWait()


def dire_texte(texte: str, robot_api: Any | None = None) -> None:
    """Lit un texte à voix haute (TTS) avec la voix la plus fidèle à Reachy Mini Wireless.

    Utilise robot.media.speaker si disponible (haut-parleur 5W optimisé SDK),
    sinon utilise pyttsx3 pour compatibilité.

    Args:
        texte: Texte à prononcer
        robot_api: Interface RobotAPI (optionnel) pour accès robot.media.speaker

    """
    # Vérifier flag d'environnement
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): '%s' ignoré", texte)
        return

    # Essayer backend TTS externe
    if _synthesize_via_tts_backend(texte, robot_api):
        return

    # Essayer SDK media
    if _play_via_sdk_media(robot_api):
        return

    # Fallback pyttsx3
    try:
        _play_via_pyttsx3(texte)
    except (RuntimeError, AttributeError, OSError) as e:
        logging.error("Erreur de synthèse vocale (runtime/attr/os): %s", e)
    except Exception as e:  # noqa: BLE001
        logging.error("Erreur de synthèse vocale: %s", e)
        raise


def _convert_audio_to_wav_bytes(
    audio_data: bytes | np.ndarray | Any,
    frequence: int,
) -> io.BytesIO:
    """Convertit les données audio en format WAV en mémoire."""
    audio_io = io.BytesIO()
    with wave.open(audio_io, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(frequence)
        if isinstance(audio_data, bytes):
            wf.writeframes(audio_data)
        elif isinstance(audio_data, np.ndarray):
            wf.writeframes((audio_data.astype(np.int16)).tobytes())
        else:
            wf.writeframes(bytes(audio_data))
    audio_io.seek(0)
    return audio_io


def _recognize_from_sdk_microphone(
    duree: int,
    frequence: int,
    robot_api: Any,
) -> str | None:
    """Reconnaissance vocale via SDK microphone."""
    if not hasattr(robot_api.media, "record_audio"):
        return None

    logging.info(
        "🎤 Enregistrement via SDK (4 microphones) (%ds) pour reconnaissance...",
        duree,
    )
    audio_data = robot_api.media.record_audio(duration=duree, sample_rate=frequence)
    audio_io = _convert_audio_to_wav_bytes(audio_data, frequence)

    r = sr.Recognizer()
    with sr.AudioFile(audio_io) as source:
        audio = r.record(source)
        texte = r.recognize_google(audio, language="fr-FR")
        logging.info("✅ Texte reconnu (SDK 4 microphones) : %s", texte)
        return str(texte)


def _recognize_from_local_microphone(
    duree: int,
    frequence: int,
) -> str | None:
    """Reconnaissance vocale via microphone local (speech_recognition)."""
    r = sr.Recognizer()
    with sr.Microphone(sample_rate=frequence) as source:
        logging.info("Écoute du micro pour la reconnaissance vocale...")
        audio = r.listen(source, phrase_time_limit=duree)
        texte = r.recognize_google(audio, language="fr-FR")
        logging.info("Texte reconnu : %s", texte)
        return str(texte)


def reconnaitre_parole(
    duree: int = 3,
    frequence: int = 16000,
    robot_api: Any | None = None,
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
    # Essayer SDK microphone
    if robot_api and hasattr(robot_api, "media") and robot_api.media:
        try:
            result = _recognize_from_sdk_microphone(duree, frequence, robot_api)
            if result is not None:
                return result
        except (RuntimeError, AttributeError, OSError) as e:
            logging.debug("Erreur reconnaissance SDK (runtime/attr/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logging.debug(
                "Erreur reconnaissance SDK (fallback speech_recognition): %s", e
            )

    # Fallback: speech_recognition local
    try:
        return _recognize_from_local_microphone(duree, frequence)
    except sr.UnknownValueError:
        logging.warning("Aucune parole reconnue.")
        return None
    except (RuntimeError, AttributeError, OSError) as e:
        logging.debug("Erreur de reconnaissance vocale (runtime/attr/os): %s", e)
        return None
    except Exception as e:  # noqa: BLE001
        logging.debug("Erreur d'accès au microphone: %s", e)
        logging.debug(
            "La reconnaissance vocale nécessite pyaudio. "
            "Installez-le avec : pip install pyaudio",
        )
        return None


def lister_voix_disponibles() -> list[Any]:
    """Retourne la liste des voix TTS disponibles avec leurs propriétés."""
    # ⚡ OPTIMISATION PERFORMANCE: Utiliser cache au lieu de pyttsx3.init()
    engine = _get_pyttsx3_engine()
    if engine is None:
        logging.warning("⚠️ pyttsx3 non disponible, aucune voix listée")
        return []
    voices = engine.getProperty("voices")
    result = []
    for _idx, v in enumerate(voices):
        try:
            # Gérer le cas où languages est vide ou None
            if hasattr(v, "languages") and v.languages and len(v.languages) > 0:
                try:
                    if hasattr(v.languages[0], "decode"):
                        _ = v.languages[0].decode(errors="ignore")
                    else:
                        _ = str(v.languages[0])
                except (
                    AttributeError,
                    TypeError,
                    ValueError,
                    UnicodeDecodeError,
                    Exception,
                ):
                    _ = str(v.languages[0]) if v.languages[0] else ""
            else:
                _ = ""
        except (AttributeError, TypeError, ValueError, IndexError, Exception):
            _ = str(v.languages) if hasattr(v, "languages") and v.languages else ""
        result.append(v)
    return result


# OPTIMISATION PERFORMANCE: Threading asynchrone pour transcription audio
# Utiliser Union pour permettre None comme signal d'arrêt

_transcribe_queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=5)
_transcribe_thread: threading.Thread | None = None
_transcribe_active = False
_transcribe_lock = threading.Lock()
_last_transcribe_result: str | None = None


def _transcribe_thread_worker() -> None:
    """Worker thread pour transcriptions asynchrones en arrière-plan."""
    global _last_transcribe_result
    logger.debug("🎤 Thread transcription asynchrone démarré")

    while _transcribe_active:
        task = None
        task_retrieved = False
        task_done_called = False
        try:
            # Attendre tâche de transcription
            task = _transcribe_queue.get(timeout=0.5)
            task_retrieved = True  # Marquer que la tâche a été récupérée

            if task is None:  # Signal d'arrêt
                _transcribe_queue.task_done()
                task_done_called = True
                break

            audio_data = task["audio_data"]
            sample_rate = task.get("sample_rate", 16000)
            model_size = task.get("model_size", "tiny")

            # Transcription synchrone dans le thread
            result = _transcribe_audio_sync(audio_data, sample_rate, model_size)
            _last_transcribe_result = result

            _transcribe_queue.task_done()
            task_done_called = True
        except queue.Empty:
            continue
        except (RuntimeError, AttributeError, OSError) as e:
            # Utiliser error au lieu de exception pour éviter traces complètes dans tests
            logger.error(
                "Erreur thread transcription asynchrone (runtime/attr/os): %s", e
            )
        except (
            Exception
        ) as e:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
            # Utiliser error au lieu de exception pour éviter traces complètes dans tests
            logger.error("Erreur thread transcription asynchrone: %s", e)
            # Appeler task_done() seulement si la tâche a été récupérée
            # et qu'on ne l'a pas déjà appelé
            if task_retrieved and task is not None and not task_done_called:
                _transcribe_queue.task_done()

    logger.debug("🎤 Thread transcription asynchrone arrêté")


def start_async_transcription() -> bool:
    """Démarre le thread de transcription asynchrone.

    Returns:
        True si démarré avec succès, False sinon

    """
    global _transcribe_thread, _transcribe_active

    with _transcribe_lock:
        if _transcribe_active:
            logger.debug("Transcription asynchrone déjà active")
            return True

        _transcribe_active = True
        _transcribe_thread = threading.Thread(
            target=_transcribe_thread_worker,
            daemon=True,
            name="BBIAVoice-TranscribeThread",
        )
        _transcribe_thread.start()
        logger.info("✅ Transcription asynchrone démarrée")
        return True


def stop_async_transcription() -> None:
    """Arrête le thread de transcription asynchrone."""
    global _transcribe_thread, _transcribe_active

    with _transcribe_lock:
        if not _transcribe_active:
            return

        _transcribe_active = False

        # Envoyer signal d'arrêt
        with contextlib.suppress(queue.Full):
            _transcribe_queue.put_nowait(None)

        if _transcribe_thread and _transcribe_thread.is_alive():
            _transcribe_thread.join(timeout=2.0)
            if _transcribe_thread.is_alive():
                logger.warning(
                    "Thread transcription asynchrone n'a pas pu être arrêté proprement",
                )

        logger.info("✅ Transcription asynchrone arrêtée")


def transcribe_audio_async(
    audio_data: Any,
    sample_rate: int = 16000,
    model_size: str = "tiny",
    timeout: float | None = None,
) -> str | None:
    """Transcrit audio de manière asynchrone (non-bloquant).

    Args:
        audio_data: Données audio (numpy array ou bytes)
        sample_rate: Fréquence d'échantillonnage (défaut: 16000)
        model_size: Taille du modèle Whisper (défaut: "tiny")
        timeout: Timeout en secondes (défaut: None = pas de timeout)

    Returns:
        Texte transcrit ou None si erreur/timeout

    """
    global _last_transcribe_result

    # Démarrer thread si nécessaire
    if not _transcribe_active:
        start_async_transcription()

    # Si transcription asynchrone active, ajouter à la queue
    if _transcribe_active:
        try:
            task = {
                "audio_data": audio_data,
                "sample_rate": sample_rate,
                "model_size": model_size,
            }
            _transcribe_queue.put_nowait(task)

            # Attendre résultat avec timeout
            if timeout:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if _last_transcribe_result is not None:
                        result = _last_transcribe_result
                        _last_transcribe_result = None
                        return result
                    time.sleep(0.1)
                return None

            # Pas de timeout, retourner dernier résultat disponible
            return _last_transcribe_result
        except queue.Full:
            logger.warning("Queue transcription pleine, fallback synchrone")
            # Fallback synchrone
            return _transcribe_audio_sync(audio_data, sample_rate, model_size)

    # Fallback: transcription synchrone si asynchrone non actif
    return _transcribe_audio_sync(audio_data, sample_rate, model_size)


def _transcribe_audio_sync(
    audio_data: Any,
    sample_rate: int = 16000,
    model_size: str = "tiny",
) -> str | None:
    """Version synchrone interne de transcribe_audio (utilisée par thread)."""
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): transcription ignorée")
        return None

    try:
        if not WHISPER_AVAILABLE:
            logging.debug("Whisper non disponible pour transcription")
            return None

        # OPTIMISATION PERFORMANCE: Utiliser WhisperSTT avec cache global
        # Le cache est géré automatiquement par WhisperSTT
        stt = WhisperSTT(model_size=model_size, language="fr")

        # Charger le modèle (utilise cache si disponible)
        if not stt.is_loaded and not stt.load_model():
            logging.warning("Impossible de charger le modèle Whisper")
            return None

        # Convertir audio_data en fichier temporaire si nécessaire
        # Créer fichier temporaire
        temp_file = tempfile.NamedTemporaryFile(
            suffix=".wav",
            delete=False,
        )
        temp_path = temp_file.name
        temp_file.close()

        try:
            # Sauvegarder audio_data dans fichier temporaire
            if not SOUNDFILE_AVAILABLE:
                raise ImportError("soundfile non disponible")

            if isinstance(audio_data, np.ndarray):
                sf.write(temp_path, audio_data, sample_rate)
            elif isinstance(audio_data, bytes):
                # Convertir bytes en numpy array si nécessaire
                audio_array = (
                    np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
                    / 32768.0
                )
                sf.write(temp_path, audio_array, sample_rate)
            else:
                logging.warning(f"Format audio non supporté: {type(audio_data)}")
                return None

            # Transcrit avec Whisper (utilise cache modèle)
            result = stt.transcribe_audio(temp_path)

            return str(result) if result is not None else None

        finally:
            # Nettoyer fichier temporaire
            try:
                os.unlink(temp_path)
            except (OSError, PermissionError) as cleanup_error:
                logging.debug(
                    "Nettoyage fichier Whisper (os/permission): %s", cleanup_error
                )
            except (
                Exception
            ) as cleanup_error:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
                logging.debug(
                    "Nettoyage fichier Whisper (erreur inattendue): %s", cleanup_error
                )

    except ImportError:
        logging.debug("Whisper non disponible (import échoué)")
        return None
    except Exception as e:
        logging.warning(f"Erreur transcription Whisper: {e}")
        return None


# OPTIMISATION PERFORMANCE: Fonction wrapper pour transcription audio avec Whisper
# Utilise le cache global pour éviter rechargement du modèle
# NOTE: Pour version asynchrone, utiliser transcribe_audio_async()
def transcribe_audio(
    audio_data: Any,
    sample_rate: int = 16000,
    model_size: str = "tiny",
) -> str | None:
    """Transcrit des données audio en texte avec Whisper (utilise cache).

    Args:
        audio_data: Données audio (numpy array ou bytes)
        sample_rate: Fréquence d'échantillonnage (défaut: 16000)
        model_size: Taille du modèle Whisper ("tiny", "base", etc.) - défaut: "tiny"

    Returns:
        Texte transcrit ou None si erreur

    """
    # Vérifier flag d'environnement pour désactiver audio (CI/headless)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logging.debug("Audio désactivé (BBIA_DISABLE_AUDIO=1): transcription ignorée")
        return None

    try:
        if not WHISPER_AVAILABLE:
            logging.debug("Whisper non disponible pour transcription")
            return None

        # OPTIMISATION PERFORMANCE: Utiliser WhisperSTT avec cache global
        # Le cache est géré automatiquement par WhisperSTT
        stt = WhisperSTT(model_size=model_size, language="fr")

        # Charger le modèle (utilise cache si disponible)
        if not stt.is_loaded and not stt.load_model():
            logging.warning("Impossible de charger le modèle Whisper")
            return None

        # Convertir audio_data en fichier temporaire si nécessaire
        # Créer fichier temporaire
        temp_file = tempfile.NamedTemporaryFile(
            suffix=".wav",
            delete=False,
        )
        temp_path = temp_file.name
        temp_file.close()

        try:
            # Sauvegarder audio_data dans fichier temporaire
            if not SOUNDFILE_AVAILABLE:
                raise ImportError("soundfile non disponible")

            if isinstance(audio_data, np.ndarray):
                sf.write(temp_path, audio_data, sample_rate)
            elif isinstance(audio_data, bytes):
                # Convertir bytes en numpy array si nécessaire
                audio_array = (
                    np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
                    / 32768.0
                )
                sf.write(temp_path, audio_array, sample_rate)
            else:
                logging.warning(f"Format audio non supporté: {type(audio_data)}")
                return None

            # Transcrit avec Whisper (utilise cache modèle)
            result = stt.transcribe_audio(temp_path)

            return str(result) if result is not None else None

        finally:
            # Nettoyer fichier temporaire
            try:
                os.unlink(temp_path)
            except (OSError, PermissionError) as cleanup_error:
                logging.debug(
                    "Nettoyage fichier Whisper (os/permission): %s", cleanup_error
                )
            except (
                Exception
            ) as cleanup_error:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
                logging.debug(
                    "Nettoyage fichier Whisper (erreur inattendue): %s", cleanup_error
                )

    except ImportError:
        logging.debug("Whisper non disponible (import échoué)")
        return None
    except Exception as e:
        logging.warning(f"Erreur transcription Whisper: {e}")
        return None


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "list-voices":
        lister_voix_disponibles()
        # Optionnel : démo d'écoute de chaque voix
        if len(sys.argv) > 2 and sys.argv[2] == "demo":
            # ⚡ OPTIMISATION PERFORMANCE: Utiliser cache au lieu de pyttsx3.init()
            engine = _get_pyttsx3_engine()
            if engine is None:
                logging.warning("⚠️ pyttsx3 non disponible, démo impossible")
                sys.exit(1)
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
    if engine is None:
        logging.warning("⚠️ pyttsx3 non disponible, démo impossible")
        sys.exit(1)
    voice_id = _get_cached_voice_id()
    engine.setProperty("voice", voice_id)
    engine.setProperty("rate", 170)
    engine.setProperty("volume", 1.0)
    demo_texte = (
        "Bonjour, je suis BBIA. Je fonctionne sur votre Mac. "
        "Ceci est la voix la plus proche de Reachy Mini Wireless."
    )
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
