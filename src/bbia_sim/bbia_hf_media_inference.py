"""Inférence média HF — vision / NLP / audio / VQA.

Extrait de ``BBIAHuggingFace`` : description d'image, sentiment, émotion,
transcription Whisper et VQA. Les méthodes instance du god-file délèguent ici.
"""

from __future__ import annotations

import logging
import warnings
from typing import Any

import numpy as np
import numpy.typing as npt
from PIL import Image

from .utils.types import SentimentResult

logger = logging.getLogger(__name__)

try:
    import torch

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from transformers import (
            BlipForConditionalGeneration,
            BlipProcessor,
            CLIPModel,
            CLIPProcessor,
            WhisperForConditionalGeneration,
            WhisperProcessor,
            pipeline,
        )
        from transformers.utils import logging as transformers_logging

        transformers_logging.set_verbosity_error()
    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    torch = None  # type: ignore[assignment]
    pipeline = None  # type: ignore[assignment]
    BlipForConditionalGeneration = None  # type: ignore[assignment,misc]
    BlipProcessor = None  # type: ignore[assignment,misc]
    CLIPModel = None  # type: ignore[assignment,misc]
    CLIPProcessor = None  # type: ignore[assignment,misc]
    WhisperForConditionalGeneration = None  # type: ignore[assignment,misc]
    WhisperProcessor = None  # type: ignore[assignment,misc]


def describe_image(
    state: Any,
    image: str | Image.Image | npt.NDArray[np.uint8],
    model_name: str = "blip",
) -> str:
    """Décrit une image avec BLIP ou CLIP.

    Args:
        image: Image à décrire (chemin, PIL Image, ou numpy array)
        model_name: Nom du modèle à utiliser

    Returns:
        Description textuelle de l'image

    """
    try:
        if isinstance(image, str):
            image = Image.open(image)
        elif isinstance(image, np.ndarray):
            image = Image.fromarray(image)
        if "blip" in model_name.lower():
            processor_key = f"{model_name}_processor"
            model_key = f"{model_name}_model"
            if processor_key not in state.processors or model_key not in state.models:
                state.load_model(model_name, "vision")
            processor = state.processors[processor_key]
            model = state.models[model_key]
            state._update_model_usage(model_key)
            inputs = processor(image, return_tensors="pt").to(state.device)
            out = model.generate(**inputs, max_length=50)
            description = processor.decode(out[0], skip_special_tokens=True)
            return str(description)
        if "clip" in model_name.lower():
            processor_key = f"{model_name}_processor"
            model_key = f"{model_name}_model"
            if processor_key not in state.processors or model_key not in state.models:
                state.load_model(model_name, "vision")
            processor = state.processors[processor_key]
            model = state.models[model_key]
            inputs = processor(
                text=["a photo of"], images=image, return_tensors="pt", padding=True
            ).to(state.device)
            outputs = model(**inputs)
            logits_per_image = outputs.logits_per_image
            probs = logits_per_image.softmax(dim=1)
            return f"CLIP analysis: {probs.cpu().numpy()}"
        if "smolvlm" in model_name.lower() or "moondream" in model_name.lower():
            processor_key = f"{model_name}_processor"
            model_key = f"{model_name}_model"
            if processor_key not in state.processors or model_key not in state.models:
                state.load_model(model_name, "multimodal")
            processor = state.processors[processor_key]
            model = state.models[model_key]
            prompt = "Décris cette image en détail."
            inputs = processor(images=image, text=prompt, return_tensors="pt").to(
                state.device
            )
            with torch.no_grad():
                outputs = model.generate(**inputs, max_new_tokens=100)
            description = processor.decode(outputs[0], skip_special_tokens=True)
            return str(description.strip())
        return "Erreur (describe_image): modèle non supporté — vérifiez le nom choisi"
    except (ValueError, RuntimeError, AttributeError, OSError):
        logger.exception("❌ Erreur description image:")
        return "Erreur (describe_image): échec de génération de description d'image"
    except (TypeError, IndexError) as e:
        logger.exception("❌ Erreur description image (type/index/os): %s", e)
        return "Erreur (describe_image): échec de génération de description d'image"
    except Exception as e:
        logger.exception("❌ Erreur inattendue description image: %s", e)
        return "Erreur (describe_image): échec de génération de description d'image"


def analyze_sentiment(
    state: Any,
    text: str,
    model_name: str = "cardiffnlp/twitter-roberta-base-sentiment-latest",
) -> SentimentResult:
    """Analyse le sentiment d'un texte.

    Args:
        text: Texte à analyser
        model_name: Nom du modèle à utiliser (modèle Hugging Face complet)

    Returns:
        Dictionnaire avec sentiment et score

    """
    try:
        model_key = f"{model_name}_pipeline"
        if model_key not in state.models:
            state.load_model(model_name, "nlp")
        pipeline = state.models[model_key]
        max_tokens = 512
        max_chars = 2000
        text_truncated = text
        try:
            tokenizer = pipeline.tokenizer
            if tokenizer is not None:
                tokens = tokenizer.encode(
                    text,
                    add_special_tokens=True,
                    max_length=max_tokens,
                    truncation=True,
                )
                text_truncated = tokenizer.decode(tokens, skip_special_tokens=True)
            else:
                text_truncated = text[:max_chars] if len(text) > max_chars else text
        except (AttributeError, TypeError):
            text_truncated = text[:max_chars] if len(text) > max_chars else text
        result: Any = pipeline(text_truncated)
        return {
            "text": text_truncated,
            "sentiment": str(result[0]["label"]),
            "score": float(result[0]["score"]),
            "model": model_name,
        }
    except (ValueError, RuntimeError, AttributeError, KeyError) as e:
        logger.exception("❌ Erreur analyse sentiment:")
        return {"error": str(e)}
    except (TypeError, IndexError, OSError) as e:
        logger.exception("❌ Erreur analyse sentiment (type/index/os): %s", e)
        return {"error": str(e)}
    except Exception as e:
        logger.exception("❌ Erreur inattendue analyse sentiment: %s", e)
        return {"error": str(e)}


def analyze_emotion(
    state: Any, text: str, model_name: str = "emotion"
) -> SentimentResult:
    """Analyse les émotions dans un texte.

    Args:
        text: Texte à analyser
        model_name: Nom du modèle à utiliser

    Returns:
        Dictionnaire avec émotion détectée et score

    """
    try:
        model_key = f"{model_name}_pipeline"
        if model_key not in state.models:
            state.load_model(model_name, "nlp")
        pipeline = state.models[model_key]
        result = pipeline(text)
        return {
            "text": text,
            "emotion": result[0]["label"],
            "score": result[0]["score"],
            "model": model_name,
        }
    except (ValueError, RuntimeError, AttributeError, KeyError) as e:
        logger.exception("❌ Erreur analyse émotion:")
        return {"error": str(e)}
    except (TypeError, IndexError, OSError) as e:
        logger.exception("❌ Erreur analyse émotion (type/index/os): %s", e)
        return {"error": str(e)}
    except Exception as e:
        logger.exception("❌ Erreur inattendue analyse émotion: %s", e)
        return {"error": str(e)}


def transcribe_audio(state: Any, audio_path: str, model_name: str = "whisper") -> str:
    """Transcrit un fichier audio avec Whisper.

    Args:
        audio_path: Chemin vers le fichier audio
        model_name: Nom du modèle Whisper à utiliser

    Returns:
        Texte transcrit

    """
    try:
        processor_key = f"{model_name}_processor"
        model_key = f"{model_name}_model"
        if processor_key not in state.processors or model_key not in state.models:
            state.load_model(model_name, "audio")
        processor = state.processors[processor_key]
        model = state.models[model_key]
        (audio, sample_rate) = processor.load_audio(audio_path)
        inputs = processor(audio, sampling_rate=sample_rate, return_tensors="pt").to(
            state.device
        )
        with torch.no_grad():
            predicted_ids = model.generate(inputs["input_features"])
        transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)[
            0
        ]
        return str(transcription)
    except (OSError, RuntimeError, ValueError, AttributeError):
        logger.exception("❌ Erreur transcription audio:")
        return "Erreur (transcribe_audio): problème pendant la transcription audio"
    except (TypeError, IndexError) as e:
        logger.exception("❌ Erreur transcription audio (type/index/os): %s", e)
        return "Erreur (transcribe_audio): problème pendant la transcription audio"
    except Exception as e:
        logger.exception("❌ Erreur inattendue transcription audio: %s", e)
        return "Erreur (transcribe_audio): problème pendant la transcription audio"


def answer_question(
    state: Any, image: str | Image.Image, question: str, model_name: str = "blip_vqa"
) -> str:
    """Répond à une question sur une image (VQA).

    Args:
        image: Image à analyser
        question: Question à poser
        model_name: Nom du modèle VQA à utiliser

    Returns:
        Réponse à la question

    """
    try:
        if isinstance(image, str):
            image = Image.open(image)
        elif isinstance(image, np.ndarray):
            image = Image.fromarray(image)
        processor_key = f"{model_name}_processor"
        model_key = f"{model_name}_model"
        if processor_key not in state.processors or model_key not in state.models:
            state.load_model(model_name, "multimodal")
        if processor_key not in state.processors:
            import os

            if os.environ.get("CI", "false").lower() != "true":
                logger.error(
                    f"❌ Processeur {processor_key} non disponible après chargement"
                )
            return "Erreur (answer_question): processeur non disponible"
        if model_key not in state.models:
            import os

            if os.environ.get("CI", "false").lower() != "true":
                logger.error(f"❌ Modèle {model_key} non disponible après chargement")
            return "Erreur (answer_question): modèle non disponible"
        processor = state.processors[processor_key]
        model = state.models[model_key]
        inputs = processor(image, question, return_tensors="pt").to(state.device)
        out = model.generate(**inputs, max_length=50)
        answer = processor.decode(out[0], skip_special_tokens=True)
        return str(answer)
    except (ValueError, RuntimeError, AttributeError, OSError):
        logger.exception("❌ Erreur VQA:")
        return "Erreur (answer_question): échec de l'analyse visuelle (VQA)"
    except (TypeError, IndexError) as e:
        logger.exception("❌ Erreur VQA (type/index/os): %s", e)
        return "Erreur (answer_question): échec de l'analyse visuelle (VQA)"
    except Exception as e:
        logger.exception("❌ Erreur inattendue VQA: %s", e)
        return "Erreur (answer_question): échec de l'analyse visuelle (VQA)"
