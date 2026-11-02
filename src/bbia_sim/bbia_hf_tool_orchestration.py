"""Orchestration détection / exécution d'outils HF.

Extrait de ``BBIAHuggingFace`` : NLP sentence-transformers, matching
mots-clés, construction de paramètres et exécution via ``BBIATools``.
Les méthodes instance du god-file délèguent ici (API publique inchangée).
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any, Protocol, cast

from . import bbia_hf_chat_tools as chat_tools

logger = logging.getLogger(__name__)

SentimentFn = Callable[[str], dict[str, Any]]
ExecuteToolFn = Callable[[str, dict[str, Any]], dict[str, Any] | None]


class NlpDetectionState(Protocol):
    """État mutable NLP (attributs sur ``BBIAHuggingFace``)."""

    _sentence_model: Any | None
    _use_nlp_detection: bool


def detect_tool_with_nlp(
    user_message: str,
    state: NlpDetectionState,
) -> tuple[str, float] | None:
    """Détecte un outil via similarité sémantique (sentence-transformers).

    Charge le modèle à la demande et persiste l'état sur ``state``.
    """
    try:
        if state._sentence_model is None:
            try:
                from sentence_transformers import (  # type: ignore[import-untyped]
                    SentenceTransformer,
                )

                logger.info("📥 Chargement modèle NLP (sentence-transformers)...")
                state._sentence_model = SentenceTransformer(
                    "sentence-transformers/all-MiniLM-L6-v2",
                )
                state._use_nlp_detection = True
                logger.info("✅ Modèle NLP chargé")
            except ImportError:
                logger.debug(
                    "ℹ️ sentence-transformers non disponible, fallback mots-clés",
                )
                state._use_nlp_detection = False
                return None

        if not state._use_nlp_detection or state._sentence_model is None:
            return None

        tool_descriptions = chat_tools.TOOL_NLP_DESCRIPTIONS
        try:
            from sklearn.metrics.pairwise import cosine_similarity
        except ImportError:
            logger.debug("ℹ️ scikit-learn non disponible, fallback mots-clés")
            return None

        message_embedding = state._sentence_model.encode([user_message])
        tool_embeddings = state._sentence_model.encode(
            list(tool_descriptions.values()),
        )
        similarities = cosine_similarity(message_embedding, tool_embeddings)[0]

        best_idx = int(similarities.argmax())
        best_score = float(similarities[best_idx])
        confidence_threshold = chat_tools.NLP_CONFIDENCE_THRESHOLD

        if best_score > confidence_threshold:
            tool_name = list(tool_descriptions.keys())[best_idx]
            return (tool_name, best_score)
        return None

    except (ImportError, RuntimeError, AttributeError, ValueError) as e:
        logger.debug("ℹ️ Erreur NLP détection (fallback mots-clés): %s", e)
        return None
    except (TypeError, IndexError, OSError) as e:
        logger.debug("ℹ️ Erreur NLP détection (type/index/os): %s", e)
        return None
    except Exception as e:  # noqa: BLE001 - Fallback final inattendu
        logger.debug(
            "ℹ️ Erreur inattendue NLP détection (fallback mots-clés): %s",
            e,
        )
        return None


def build_tool_params(
    tool_name: str,
    user_message: str,
    message_lower: str,
    *,
    analyze_sentiment: SentimentFn | None = None,
    extract_angle_fn: Callable[[str], float | None] | None = None,
    extract_intensity_fn: Callable[[str], float | None] | None = None,
    use_ner_extraction: bool = False,
) -> dict[str, Any]:
    """Construit les paramètres d'un outil détecté (mots-clés ou NLP)."""
    params: dict[str, Any] = {}
    angle_fn = extract_angle_fn or chat_tools.extract_angle
    intensity_fn = extract_intensity_fn or chat_tools.extract_intensity

    if tool_name == "move_head":
        for fr_dir, en_dir in chat_tools.MOVE_HEAD_MAPPINGS.items():
            if fr_dir in message_lower:
                params["direction"] = en_dir
                break
        if "direction" not in params:
            params["direction"] = "left"

        if use_ner_extraction:
            extracted_angle = angle_fn(user_message)
            if extracted_angle is not None:
                params["intensity"] = min(extracted_angle / 90.0, 1.0)
                logger.info(
                    "📐 Angle extrait: %s° → intensité: %.2f",
                    extracted_angle,
                    params["intensity"],
                )
            else:
                intensity = intensity_fn(user_message)
                params["intensity"] = intensity if intensity is not None else 0.5
        else:
            params["intensity"] = 0.5

    elif tool_name == "play_emotion":
        for fr_emo, en_emo in chat_tools.PLAY_EMOTION_MAPPINGS.items():
            if fr_emo in message_lower:
                params["emotion"] = en_emo
                break
        if "emotion" not in params:
            if analyze_sentiment is not None:
                try:
                    sentiment = analyze_sentiment(user_message)
                    params["emotion"] = chat_tools.SENTIMENT_TO_EMOTION.get(
                        sentiment.get("sentiment", "NEUTRAL"),
                        "neutral",
                    )
                except (KeyError, ValueError, TypeError):
                    params["emotion"] = "neutral"
            else:
                params["emotion"] = "neutral"
        params["intensity"] = 0.7

    elif tool_name == "dance":
        params.update(chat_tools.DEFAULT_DANCE_PARAMS)

    elif tool_name in (
        "stop_dance",
        "stop_emotion",
        "head_tracking",
        "do_nothing",
    ):
        if tool_name == "head_tracking":
            params["enabled"] = True
        elif tool_name == "do_nothing":
            params["duration"] = 2.0

    return params


def _run_tool_safe(
    execute_tool: ExecuteToolFn,
    tool_name: str,
    params: dict[str, Any],
) -> dict[str, Any] | None:
    """Exécute un outil avec journalisation d'erreurs centralisée."""
    try:
        return execute_tool(tool_name, params)
    except (AttributeError, RuntimeError, ValueError, KeyError) as e:
        logger.exception(
            "❌ Erreur exécution outil '%s' (critique): %s",
            tool_name,
            e,
        )
        return {
            "status": "error",
            "detail": f"Erreur lors de l'exécution: {e}",
        }
    except Exception as e:  # noqa: BLE001 - Exceptions non prévues
        logger.exception(
            "❌ Erreur inattendue exécution outil '%s' (critique): %s",
            tool_name,
            e,
        )
        return {
            "status": "error",
            "detail": f"Erreur lors de l'exécution: {e}",
        }


def execute_detected_tool(
    tool_name: str,
    user_message: str,
    message_lower: str,
    *,
    execute_tool: ExecuteToolFn,
    analyze_sentiment: SentimentFn | None = None,
    extract_angle_fn: Callable[[str], float | None] | None = None,
    extract_intensity_fn: Callable[[str], float | None] | None = None,
) -> str | None:
    """Exécute un outil déjà détecté (NLP ou mots-clés) et formate le résultat."""
    try:
        params = build_tool_params(
            tool_name,
            user_message,
            message_lower,
            analyze_sentiment=analyze_sentiment,
            extract_angle_fn=extract_angle_fn,
            extract_intensity_fn=extract_intensity_fn,
            use_ner_extraction=True,
        )
        result = execute_tool(tool_name, params)
        text = cast(str, chat_tools.format_tool_result_text(result, tool_name))
        if result is not None and result.get("status") == "success":
            logger.info(
                "✅ Outil '%s' exécuté: %s",
                tool_name,
                result.get("detail", "Action exécutée"),
            )
        elif result is not None:
            logger.warning(
                "⚠️ Erreur outil '%s': %s",
                tool_name,
                result.get("detail", "Erreur inconnue"),
            )
        return text

    except (AttributeError, RuntimeError, ValueError, KeyError) as e:
        logger.exception("❌ Erreur exécution outil '%s': %s", tool_name, e)
        return f"❌ Erreur lors de l'exécution: {e}"
    except (TypeError, IndexError, OSError) as e:
        logger.exception(
            "❌ Erreur exécution outil '%s' (type/index/os): %s",
            tool_name,
            e,
        )
        return f"❌ Erreur lors de l'exécution: {e}"
    except Exception as e:  # noqa: BLE001 - Fallback final
        logger.exception(
            "❌ Erreur inattendue exécution outil '%s': %s",
            tool_name,
            e,
        )
        return f"❌ Erreur lors de l'exécution: {e}"


def detect_and_execute_tools(
    user_message: str,
    *,
    state: NlpDetectionState,
    execute_tool: ExecuteToolFn | None,
    analyze_sentiment: SentimentFn | None = None,
    extract_angle_fn: Callable[[str], float | None] | None = None,
    extract_intensity_fn: Callable[[str], float | None] | None = None,
    tools_available: bool = True,
) -> str | None:
    """Pipeline complet : NLP → mots-clés → exécution → texte utilisateur."""
    if not tools_available or execute_tool is None:
        return None

    message_lower = user_message.lower()

    nlp_result = detect_tool_with_nlp(user_message, state)
    if nlp_result:
        tool_name, confidence = nlp_result
        logger.info(
            "🔍 NLP détecté outil '%s' (confiance: %.2f)",
            tool_name,
            confidence,
        )
        return execute_detected_tool(
            tool_name,
            user_message,
            message_lower,
            execute_tool=execute_tool,
            analyze_sentiment=analyze_sentiment,
            extract_angle_fn=extract_angle_fn,
            extract_intensity_fn=extract_intensity_fn,
        )

    tool_patterns = chat_tools.TOOL_KEYWORD_PATTERNS
    for tool_name, pattern in tool_patterns.items():
        for keyword in pattern["keywords"]:  # type: ignore[index]
            if keyword not in message_lower:
                continue
            try:
                params = build_tool_params(
                    tool_name,
                    user_message,
                    message_lower,
                    analyze_sentiment=analyze_sentiment,
                    extract_angle_fn=extract_angle_fn,
                    extract_intensity_fn=extract_intensity_fn,
                    use_ner_extraction=False,
                )
                result = _run_tool_safe(execute_tool, tool_name, params)
                text = cast(str, chat_tools.format_tool_result_text(result, tool_name))
                if result is not None and result.get("status") == "success":
                    logger.info(
                        "✅ Outil '%s' exécuté: %s",
                        tool_name,
                        result.get("detail", "Action exécutée"),
                    )
                elif result is not None:
                    logger.warning(
                        "⚠️ Erreur outil '%s': %s",
                        tool_name,
                        result.get("detail", "Erreur inconnue"),
                    )
                return text
            except Exception as e:  # noqa: BLE001 - Gestion non prévue
                logger.exception(
                    "❌ Erreur inattendue exécution outil '%s'",
                    tool_name,
                )
                return f"❌ Erreur lors de l'exécution: {e}"

    return None
