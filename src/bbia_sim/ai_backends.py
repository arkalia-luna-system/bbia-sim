"""Sélecteurs de backends IA (TTS/STT/LLM) via variables d'environnement.

Compatibles macOS (CPU/MPS). Aucun paquet lourd requis par défaut.
Si un backend n'est pas disponible, un fallback simple est fourni ou un skip
est recommandé côté tests.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Protocol


class TextToSpeech(Protocol):
    def synthesize_to_wav(self, text: str, outfile: str) -> bool:  # pragma: no cover
        ...


class SpeechToText(Protocol):
    def transcribe_wav(self, infile: str) -> str | None:  # pragma: no cover
        ...


class LocalLLM(Protocol):
    def generate(self, prompt: str, max_tokens: int = 128) -> str:  # pragma: no cover
        ...


class Pyttsx3TTS:
    """Fallback TTS léger (pyttsx3), fonctionne bien sur macOS."""

    def __init__(self) -> None:
        # Lazy-initialization pour éviter erreurs eSpeak en environnements CI
        self._engine = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        try:
            engine = self._engine
            if engine is None:
                import pyttsx3  # lazy import

                engine = pyttsx3.init()
                self._engine = engine
            engine.save_to_file(text, outfile)
            engine.runAndWait()
            return True
        except Exception:
            return False


class KittenTTSTTS:
    """Placeholder pour KittenTTS. Utilise pyttsx3 si lib absente."""

    def __init__(self) -> None:
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
        except Exception:
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except Exception:
                pass
        return self._fallback.synthesize_to_wav(text, outfile)


class KokoroTTS:
    def __init__(self) -> None:
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
        except Exception:
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except Exception:
                pass
        return self._fallback.synthesize_to_wav(text, outfile)


class NeuTTSTTS:
    def __init__(self) -> None:
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
        except Exception:
            self._impl = None

    def synthesize_to_wav(self, text: str, outfile: str) -> bool:
        if self._impl is not None:
            try:
                return self._impl.synthesize_to_wav(text, outfile)
            except Exception:
                pass
        return self._fallback.synthesize_to_wav(text, outfile)


class DummySTT:
    """STT neutre de secours: retourne chaîne vide si non dispo."""

    def transcribe_wav(self, infile: str) -> str | None:
        return ""


class WhisperSTT:
    """STT basé sur Whisper (transformers) si dispo, sinon retourne None.

    Implémentation légère: lazy-import et gestion absence de deps.
    """

    def __init__(self) -> None:
        self._ready = False
        self._model: Any = None
        self._processor: Any = None
        try:  # lazy import pour éviter coûts CI
            from transformers import (
                WhisperForConditionalGeneration,
                WhisperProcessor,
            )

            model_name = os.environ.get("BBIA_WHISPER_MODEL", "openai/whisper-base")
            # nosec B615: révision explicite pour éviter latest flottant
            self._processor = WhisperProcessor.from_pretrained(
                model_name, revision="main"
            )
            self._model = WhisperForConditionalGeneration.from_pretrained(
                model_name, revision="main"
            )
            self._ready = True
        except Exception as e:  # pragma: no cover - environnement sans deps
            logging.getLogger(__name__).info(f"Whisper indisponible: {e}")
            self._ready = False

    def transcribe_wav(self, infile: str) -> str | None:
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
                pred_ids, skip_special_tokens=True
            )
            text: str = decoded[0] if decoded else ""
            return text
        except Exception as e:  # pragma: no cover
            logging.getLogger(__name__).warning(f"Whisper STT erreur: {e}")
            return ""


class EchoLLM:
    """LLM local de secours: renvoie l'entrée tronquée (utile tests)."""

    def generate(self, prompt: str, max_tokens: int = 128) -> str:
        return (prompt or "")[:max_tokens]


class LlamaCppLLM:
    """LLM local via llama.cpp si dispo, sinon fallback echo.

    Cette implémentation reste légère: on tente un import; sinon on echo.
    """

    def __init__(self) -> None:
        self._ready = False
        self._model = None
        self._ctx = None
        try:
            from llama_cpp import Llama

            model_path = os.environ.get("BBIA_LLAMA_MODEL", "")
            if model_path and os.path.exists(model_path):
                # Paramètres prudents par défaut
                self._model = Llama(model_path=model_path, n_ctx=2048, n_threads=4)
                self._ready = True
        except Exception as e:  # pragma: no cover
            logging.getLogger(__name__).info(f"llama.cpp indisponible: {e}")
            self._ready = False

    def generate(self, prompt: str, max_tokens: int = 128) -> str:
        if not self._ready or self._model is None:
            return (prompt or "")[:max_tokens]
        try:
            out: dict[str, Any] = self._model(
                prompt=prompt, max_tokens=max_tokens, echo=False
            )
            # Format standard llama.cpp: {"choices":[{"text":"..."}]}
            text = str(out.get("choices", [{}])[0].get("text", ""))
            return text
        except Exception as e:  # pragma: no cover
            logging.getLogger(__name__).warning(f"llama.cpp erreur: {e}")
            return (prompt or "")[:max_tokens]


def get_tts_backend() -> TextToSpeech:
    name = os.environ.get("BBIA_TTS_BACKEND", "kitten").strip().lower()
    if name in {"kitten", "kittentts"}:
        return KittenTTSTTS()
    if name in {"kokoro"}:
        return KokoroTTS()
    if name in {"neutts", "neu"}:
        return NeuTTSTTS()
    if name in {"pyttsx3", "fallback"}:
        return Pyttsx3TTS()
    # Défaut sûr
    return KittenTTSTTS()


def get_stt_backend() -> SpeechToText:
    name = os.environ.get("BBIA_STT_BACKEND", "whisper").strip().lower()
    if name == "whisper":
        stt = WhisperSTT()
        # Si whisper indisponible, fallback silencieux
        return stt if stt._ready else DummySTT()
    # Parakeet etc. à brancher plus tard; fallback sûr
    return DummySTT()


def get_llm_backend() -> LocalLLM:
    name = os.environ.get("BBIA_LLM_BACKEND", "llama.cpp").strip().lower()
    if name in {"llama.cpp", "llamacpp", "llama"}:
        llm = LlamaCppLLM()
        return llm if llm._ready else EchoLLM()
    return EchoLLM()


def select_backends() -> dict[str, object]:
    """Retourne les backends IA sélectionnés (tts/stt/llm)."""
    return {
        "tts": get_tts_backend(),
        "stt": get_stt_backend(),
        "llm": get_llm_backend(),
    }
