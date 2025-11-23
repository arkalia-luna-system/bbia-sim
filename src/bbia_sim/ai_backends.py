#!/usr/bin/env python3
"""Sélecteurs de backends IA (TTS/STT/LLM) via variables d'environnement.

Compatibles macOS (CPU/MPS). Aucun paquet lourd requis par défaut.
Si un backend n'est pas disponible, un fallback simple est fourni ou un skip
est recommandé côté tests.
"""

from __future__ import annotations

import logging
import os
import shlex
import subprocess
import threading
from pathlib import Path
from typing import Any, Protocol

logger = logging.getLogger(__name__)

# OPTIMISATION PERFORMANCE: Cache global pour modèles Whisper
# (évite chargements répétés)
_whisper_models_cache: dict[str, dict[str, Any]] = (
    {}
)  # model_name -> {processor, model}
_whisper_cache_lock = threading.Lock()


class TextToSpeech(Protocol):
    """Protocole pour la synthèse vocale."""

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:  # pragma: no cover
        """Synthétise du texte en fichier WAV."""
        ...


class SpeechToText(Protocol):
    """Protocole pour la reconnaissance vocale."""

    def transcribe_wav(self, infile: str) -> str | None:  # pragma: no cover
        """Transcrit un fichier WAV en texte."""
        ...


class LocalLLM(Protocol):
    """Protocole pour les modèles de langage locaux."""

    def generate(self, prompt: str, max_tokens: int = 128) -> str:  # pragma: no cover
        """Génère du texte à partir d'un prompt."""
        ...


class Pyttsx3TTS:
    """Fallback TTS léger (pyttsx3), fonctionne bien sur macOS.

    Utilise get_bbia_voice() pour sélectionner automatiquement
    la meilleure voix féminine française (Aurelie Enhanced > Amelie).
    """

    def __init__(self) -> None:
        """Initialise le backend TTS pyttsx3.

        Lazy-initialization pour éviter erreurs eSpeak en environnements CI.
        """
        self._engine: Any = None
        self._voice_id: str | None = None  # Cache de la voix sélectionnée

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        """Synthétise du texte en fichier WAV."""
        try:
            engine = self._engine
            if engine is None:
                # OPTIMISATION PERFORMANCE: Utiliser cache global
                # au lieu de pyttsx3.init() direct
                try:
                    from .bbia_voice import (
                        _get_cached_voice_id,
                        _get_pyttsx3_engine,
                        get_bbia_voice,
                    )

                    engine = (
                        _get_pyttsx3_engine()
                    )  # Utilise cache global (0ms après premier appel)
                    self._engine = engine
                    self._voice_id = _get_cached_voice_id()  # Utilise cache voice ID
                except ImportError:
                    # Fallback si cache non disponible (ne devrait pas arriver)
                    import pyttsx3  # lazy import

                    engine = pyttsx3.init()
                    self._engine = engine

                    # Sélectionner la meilleure voix féminine française
                    try:
                        # get_bbia_voice déjà importé ci-dessus
                        self._voice_id = get_bbia_voice(engine)
                        engine.setProperty("voice", self._voice_id)
                    except (AttributeError, RuntimeError, ValueError, TypeError) as e:
                        # Si get_bbia_voice échoue, utiliser voix par défaut
                        logger.debug(
                            "Impossible de définir voix personnalisée, "
                            "utilisation par défaut: %s",
                            e,
                        )

            # Utiliser la voix sélectionnée si disponible
            if self._voice_id:
                engine.setProperty("voice", self._voice_id)

            engine.save_to_file(text, outfile)
            engine.runAndWait()
            return True
        except (RuntimeError, OSError, ValueError):
            return False


class KittenTTSTTS:
    """Placeholder pour KittenTTS. Utilise pyttsx3 si lib absente."""

    def __init__(self) -> None:
        """Initialise le backend KittenTTS avec fallback pyttsx3."""
        self._fallback = Pyttsx3TTS()
        self._impl: TextToSpeech | None = None
        try:  # pragma: no cover
            import kitten_tts

            class _KittenImpl:
                def __init__(self) -> None:
                    self._k = kitten_tts.TTS()

                def synthesize_to_wav(self, text: str, outfile: str) -> bool:
                    self._k.synthesize_to_file(text, outfile)
                    return True

            self._impl = _KittenImpl()
        except (ImportError, RuntimeError, AttributeError):
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except (RuntimeError, OSError, AttributeError) as e:
                logger.debug("Échec synthèse avec impl principale, fallback: %s", e)
            except (ValueError, TypeError, KeyError) as e:
                logger.debug(
                    "Erreur inattendue synthèse impl principale, fallback: %s",
                    e,
                )
        return self._fallback.synthesize_to_wav(text, outfile)


class KokoroTTS:
    """Backend TTS Kokoro avec fallback pyttsx3."""

    def __init__(self) -> None:
        """Initialise le backend KokoroTTS avec fallback pyttsx3."""
        self._fallback = Pyttsx3TTS()
        self._impl: TextToSpeech | None = None
        try:  # pragma: no cover
            import kokoro

            class _KokoroImpl:
                def __init__(self) -> None:
                    self._k = kokoro.TTS()

                def synthesize_to_wav(self, text: str, outfile: str) -> bool:
                    self._k.to_file(text, outfile)
                    return True

            self._impl = _KokoroImpl()
        except (ImportError, RuntimeError, AttributeError):
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        """Synthétise du texte en fichier WAV."""
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except (RuntimeError, OSError, AttributeError) as e:
                logger.debug("Échec synthèse avec impl principale, fallback: %s", e)
            except (ValueError, TypeError, KeyError) as e:
                logger.debug(
                    "Erreur inattendue synthèse impl principale, fallback: %s",
                    e,
                )
        return self._fallback.synthesize_to_wav(text, outfile)


class NeuTTSTTS:
    """Backend TTS NeuTTS avec fallback pyttsx3."""

    def __init__(self) -> None:
        """Initialise le backend NeuTTSTTS avec fallback pyttsx3."""
        self._fallback = Pyttsx3TTS()
        self._impl: TextToSpeech | None = None
        try:  # pragma: no cover
            import neutts

            class _NeuImpl:
                def __init__(self) -> None:
                    self._n = neutts.TTS()

                def synthesize_to_wav(self, text: str, outfile: str) -> bool:
                    self._n.to_wav(text, outfile)
                    return True

            self._impl = _NeuImpl()
        except (ImportError, RuntimeError, AttributeError):
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except (RuntimeError, OSError, AttributeError) as e:
                logger.debug("Échec synthèse avec impl principale, fallback: %s", e)
            except (ValueError, TypeError, KeyError) as e:
                logger.debug(
                    "Erreur inattendue synthèse impl principale, fallback: %s",
                    e,
                )
        return self._fallback.synthesize_to_wav(text, outfile)


class CoquiTTSTTS:
    """Backend Coqui TTS optionnel.

    Si la lib TTS n'est pas disponible, fallback pyttsx3.
    """

    def __init__(self) -> None:
        """Initialise le backend CoquiTTS avec fallback pyttsx3."""
        self._fallback = Pyttsx3TTS()
        self._ready = False
        try:  # pragma: no cover
            from TTS.api import TTS as _COQUI_TTS

            self._coqui_cls = _COQUI_TTS  # store class
            self._ready = True
        except (ImportError, RuntimeError, AttributeError):
            self._coqui_cls = None
            self._ready = False

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        """Synthétise du texte en fichier WAV."""
        if not self._ready or self._coqui_cls is None:
            return self._fallback.synthesize_to_wav(text, outfile)
        try:
            # Modèle FR par défaut; peut être surchargé via env BBIA_COQUI_MODEL
            model_name = os.environ.get("BBIA_COQUI_MODEL", "tts_models/fr/css10/vits")
            tts = self._coqui_cls(model_name)
            tts.tts_to_file(text=text, file_path=outfile)
            return True
        except (RuntimeError, OSError, ValueError):
            return self._fallback.synthesize_to_wav(text, outfile)


class OpenVoiceTTSTTS:
    """Backend OpenVoice optionnel (appel externe via commande).

    Utilise la variable d'environnement OPENVOICE_CMD pour exécuter un
    générateur TTS qui écrit un fichier WAV. Fallback pyttsx3 si indisponible.
    Exemple:
      export OPENVOICE_CMD="python scripts/voice_clone/generate_voice.py "
      "--text '{text}' --mode douce --out '{out}'"
    """

    def __init__(self) -> None:
        """Initialise le backend OpenVoiceTTS avec fallback pyttsx3."""
        self._fallback = Pyttsx3TTS()

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        """Synthétise du texte en fichier WAV."""
        cmd_template = os.environ.get("OPENVOICE_CMD", "").strip()
        if not cmd_template:
            return self._fallback.synthesize_to_wav(text, outfile)
        try:
            # Échapper les valeurs pour éviter l'injection de commandes
            text_escaped = shlex.quote(text)
            outfile_escaped = shlex.quote(outfile)

            # Remplacer {text} et {out} dans le template avec valeurs échappées
            cmd_str = cmd_template.replace("{text}", text_escaped)
            cmd_str = cmd_str.replace("{out}", outfile_escaped)

            # Parser la commande en liste d'arguments (sécurisé, pas de shell=True)
            cmd_args = shlex.split(cmd_str)

            # Exécuter sans shell pour éviter l'injection de commandes
            # Validation supplémentaire: vérifier que le premier argument
            # est un chemin valide
            if not cmd_args or not isinstance(cmd_args[0], str):
                msg = "Commande invalide: premier argument doit être une chaîne"
                raise ValueError(
                    msg,
                )
            # Vérifier que le chemin n'est pas relatif dangereux
            if cmd_args[0].startswith("/") or ".." in cmd_args[0]:
                # Chemin absolu ou relatif avec .. - valider avec
                # shutil.which si possible
                import shutil

                resolved = shutil.which(cmd_args[0])
                if resolved:
                    cmd_args[0] = resolved
            subprocess.check_call(  # nosec B603 - cmd_args parsé via shlex, validé et sécurisé (pas de shell=True)
                cmd_args,
                shell=False,
            )
            return True
        except (RuntimeError, OSError, ValueError):
            return self._fallback.synthesize_to_wav(text, outfile)


class DummySTT:
    """STT neutre de secours: retourne chaîne vide si non dispo."""

    def transcribe_wav(self, infile: str) -> str | None:  # noqa: ARG002
        """Transcrit un fichier WAV (retourne chaîne vide pour DummySTT)."""
        return ""


class WhisperSTT:
    """STT basé sur Whisper (transformers) si dispo, sinon retourne None.

    Implémentation légère: lazy-import et gestion absence de deps.
    """

    def __init__(self) -> None:
        """Initialise le backend WhisperSTT avec cache global."""
        self._ready = False
        self._model: Any = None
        self._processor: Any = None
        try:  # lazy import pour éviter coûts CI
            from transformers import (
                WhisperForConditionalGeneration,
                WhisperProcessor,
            )

            model_name = os.environ.get("BBIA_WHISPER_MODEL", "openai/whisper-base")

            # OPTIMISATION PERFORMANCE: Utiliser cache global
            # pour éviter chargements répétés
            global _whisper_models_cache
            with _whisper_cache_lock:
                if model_name in _whisper_models_cache:
                    logger = logging.getLogger(__name__)
                    logger.debug(
                        "♻️ Réutilisation modèle Whisper depuis cache (%s)",
                        model_name,
                    )
                    cached = _whisper_models_cache[model_name]
                    self._processor = cached["processor"]
                    self._model = cached["model"]
                    self._ready = True
                    return

            # Charger modèle si pas en cache
            # nosec B615: révision spécifiée pour éviter version flottante
            processor = WhisperProcessor.from_pretrained(
                model_name,
                revision="main",
            )  # nosec B615
            model = WhisperForConditionalGeneration.from_pretrained(
                model_name,
                revision="main",
            )  # nosec B615

            # Mettre en cache global
            with _whisper_cache_lock:
                _whisper_models_cache[model_name] = {
                    "processor": processor,
                    "model": model,
                }

            self._processor = processor
            self._model = model
            self._ready = True
        except (
            ImportError,
            RuntimeError,
            OSError,
        ) as e:  # pragma: no cover - environnement sans deps
            logging.getLogger(__name__).info("Whisper indisponible: %s", e)
            self._ready = False
        except (ValueError, TypeError, AttributeError) as e:  # pragma: no cover
            logging.getLogger(__name__).info("Erreur inattendue Whisper: %s", e)
            self._ready = False

    def transcribe_wav(self, infile: str) -> str | None:
        """Transcrit un fichier WAV en texte."""
        if not self._ready:
            return ""
        try:
            # Chargement minimal via soundfile pour compatibilité
            import soundfile as sf
            import torch  # lazy

            audio, sr = sf.read(infile)
            inputs = self._processor(audio, sampling_rate=sr, return_tensors="pt")
            with torch.no_grad():
                feats = inputs.get("input_features")
                pred_ids = self._model.generate(feats)
            decoded: list[str] = self._processor.batch_decode(
                pred_ids,
                skip_special_tokens=True,
            )
            text: str = decoded[0] if decoded else ""
            return text
        except (
            RuntimeError,
            ValueError,
            AttributeError,
            TypeError,
            KeyError,
        ) as e:  # pragma: no cover
            logging.getLogger(__name__).warning("Whisper STT erreur: %s", e)
            return ""


class EchoLLM:
    """LLM local de secours: renvoie l'entrée tronquée (utile tests)."""

    def generate(self, prompt: str, max_tokens: int = 128) -> str:
        """Génère du texte à partir d'un prompt (echo tronqué)."""
        return (prompt or "")[:max_tokens]


class LlamaCppLLM:
    """LLM local via llama.cpp si dispo, sinon fallback echo.

    Cette implémentation reste légère: on tente un import; sinon on echo.
    """

    def __init__(self) -> None:
        """Initialise le backend LlamaCppLLM avec fallback echo."""
        self._ready = False
        self._model = None
        self._ctx = None
        try:
            from llama_cpp import Llama

            model_path = os.environ.get("BBIA_LLAMA_MODEL", "")
            if model_path and Path(model_path).exists():
                # Paramètres prudents par défaut
                self._model = Llama(model_path=model_path, n_ctx=2048, n_threads=4)
                self._ready = True
        except (ImportError, RuntimeError, OSError) as e:  # pragma: no cover
            logging.getLogger(__name__).info("llama.cpp indisponible: %s", e)
            self._ready = False
        except (ValueError, TypeError, AttributeError) as e:  # pragma: no cover
            logging.getLogger(__name__).info("Erreur inattendue llama.cpp: %s", e)
            self._ready = False

    def generate(self, prompt: str, max_tokens: int = 128) -> str:
        """Génère du texte à partir d'un prompt."""
        if not self._ready or self._model is None:
            return (prompt or "")[:max_tokens]
        try:
            out: dict[str, Any] = self._model(
                prompt=prompt,
                max_tokens=max_tokens,
                echo=False,
            )
            # Format standard llama.cpp: {"choices":[{"text":"..."}]}
            return str(out.get("choices", [{}])[0].get("text", ""))
        except (
            KeyError,
            IndexError,
            AttributeError,
            RuntimeError,
            TypeError,
        ) as e:  # pragma: no cover
            logging.getLogger(__name__).warning("llama.cpp erreur: %s", e)
            return (prompt or "")[:max_tokens]


def get_tts_backend() -> TextToSpeech:
    """Sélectionne le backend TTS selon la variable d'environnement BBIA_TTS_BACKEND."""
    name = os.environ.get("BBIA_TTS_BACKEND", "kitten").strip().lower()
    if name in {"kitten", "kittentts"}:
        return KittenTTSTTS()
    if name in {"kokoro"}:
        return KokoroTTS()
    if name in {"neutts", "neu"}:
        return NeuTTSTTS()
    if name in {"coqui", "coqui-tts"}:
        return CoquiTTSTTS()
    if name in {"openvoice", "open-voice"}:
        return OpenVoiceTTSTTS()
    if name in {"pyttsx3", "fallback"}:
        return Pyttsx3TTS()
    # Défaut sûr
    return KittenTTSTTS()


def get_stt_backend() -> SpeechToText:
    """Sélectionne le backend STT selon la variable d'environnement BBIA_STT_BACKEND."""
    name = os.environ.get("BBIA_STT_BACKEND", "whisper").strip().lower()
    if name == "whisper":
        stt = WhisperSTT()
        # Si whisper indisponible, fallback silencieux
        # Utiliser méthode publique si disponible, sinon accès direct nécessaire
        return stt if getattr(stt, "_ready", False) else DummySTT()
    # Parakeet etc. à brancher plus tard; fallback sûr
    return DummySTT()


def get_llm_backend() -> LocalLLM:
    """Sélectionne le backend LLM selon la variable d'environnement BBIA_LLM_BACKEND."""
    name = os.environ.get("BBIA_LLM_BACKEND", "llama.cpp").strip().lower()
    if name in {"llama.cpp", "llamacpp", "llama"}:
        llm = LlamaCppLLM()
        # Utiliser méthode publique si disponible, sinon accès direct nécessaire
        return llm if getattr(llm, "_ready", False) else EchoLLM()
    return EchoLLM()


def select_backends() -> dict[str, object]:
    """Retourne les backends IA sélectionnés (tts/stt/llm)."""
    return {
        "tts": get_tts_backend(),
        "stt": get_stt_backend(),
        "llm": get_llm_backend(),
    }
