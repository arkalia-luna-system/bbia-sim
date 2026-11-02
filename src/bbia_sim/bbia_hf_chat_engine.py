"""Moteur chat HF — ``chat`` et génération LLM.

Extrait de ``BBIAHuggingFace``. Les méthodes instance délèguent ici.
"""

from __future__ import annotations

import logging
from typing import Any, cast

from .utils.types import ConversationEntry, SentimentDict

logger = logging.getLogger(__name__)

try:
    import torch

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    torch = None  # type: ignore[assignment]


def chat(
    state: Any, user_message: str, use_context: bool = True, enable_tools: bool = True
) -> str:
    """Chat intelligent avec BBIA avec contexte et analyse sentiment.

    Utilise LLM pré-entraîné (Mistral 7B) si disponible, sinon réponses enrichies.
    Supporte function calling avec outils LLM si disponibles.

    Args:
        user_message: Message de l'utilisateur
        use_context: Utiliser le contexte des messages précédents
        enable_tools: Activer function calling avec outils (si disponibles)

    Returns:
        Réponse intelligente de BBIA

    """
    try:
        try:
            sentiment = state.analyze_sentiment(user_message)
        except (ValueError, RuntimeError, KeyError):
            sentiment = {"sentiment": "NEUTRAL", "score": 0.5}
        if enable_tools and state.tools:
            tool_result = state._detect_and_execute_tools(user_message)
            if tool_result:
                return cast(str, tool_result)
        if (
            not state.use_llm_chat
            or state.chat_model is None
            or state.chat_tokenizer is None
        ):
            try:
                chat_config = state.model_configs.get("chat", {})
                default_chat_model = chat_config.get("phi2") or chat_config.get(
                    "tinyllama"
                )
                if default_chat_model:
                    logger.info(
                        "📥 Chargement LLM à la demande (lazy loading): %s",
                        default_chat_model,
                    )
                    if state.load_model(default_chat_model, model_type="chat"):
                        logger.info("✅ LLM chargé avec succès (lazy loading)")
            except (ImportError, RuntimeError, OSError, ValueError) as e:
                logger.debug("Lazy loading LLM échoué (fallback enrichi): %s", e)
            except KeyboardInterrupt:
                logger.debug(
                    "Lazy loading LLM interrompu (KeyboardInterrupt, fallback enrichi)"
                )
            except Exception as e:
                error_msg = str(e).lower()
                if "cancel" in error_msg or "interrupt" in error_msg:
                    logger.debug("Lazy loading LLM annulé (fallback enrichi): %s", e)
                else:
                    logger.debug(
                        "Lazy loading LLM échoué inattendu (fallback enrichi): %s", e
                    )
        sentiment_dict: SentimentDict = {
            "label": sentiment.get("sentiment", "neutral"),
            "score": sentiment.get("score", 0.5),
            "sentiment": sentiment.get("sentiment", "neutral"),
        }
        if state.bbia_chat is None:
            state._load_bbia_chat_lazy()
        if state.bbia_chat and state.bbia_chat.llm_model:
            logger.debug("Utilisation BBIAChat (LLM conversationnel léger)")
            bbia_response = state.bbia_chat.chat(user_message)
        elif state.use_llm_chat and state.chat_model and state.chat_tokenizer:
            bbia_response = state._generate_llm_response(
                user_message, use_context, enable_tools=enable_tools
            )
        else:
            bbia_response = state._generate_simple_response(
                user_message, sentiment_dict
            )
        from datetime import datetime

        sentiment_str: str = (
            sentiment.get("sentiment", "neutral")
            if isinstance(sentiment, dict)
            else "neutral"
        )
        state.conversation_history.append(
            ConversationEntry(
                user=user_message,
                bbia=bbia_response,
                sentiment=sentiment_str,
                timestamp=datetime.now().isoformat(),
            )
        )
        if state.bbia_chat and state.bbia_chat.llm_model:
            adapted_response = bbia_response
        elif state.use_llm_chat and state.chat_model:
            adapted_response = bbia_response
        else:
            adapted_response = state._adapt_response_to_personality(
                bbia_response, sentiment_dict
            )
        user_message_lower = user_message.lower()
        is_greeting = any(
            word in user_message_lower
            for word in ["bonjour", "salut", "hello", "hi", "hey", "coucou"]
        )
        if is_greeting:
            response_lower = adapted_response.lower()
            has_greeting_word = any(
                word in response_lower
                for word in ["bonjour", "salut", "hello", "hi", "hey", "coucou"]
            )
            if not has_greeting_word:
                logger.debug(
                    "Réponse LLM sans mot de salutation détectée, utilisation du fallback enrichi pour salutation"
                )
                adapted_response = state._generate_simple_response(
                    user_message, sentiment_dict
                )
                adapted_response = state._adapt_response_to_personality(
                    adapted_response, sentiment_dict
                )
        try:
            from .bbia_memory import save_conversation_to_memory

            if len(state.conversation_history) % 10 == 0:
                history_dicts: list[dict[str, Any]] = [
                    {
                        "user": entry.get("user", ""),
                        "bbia": entry.get("bbia", ""),
                        "sentiment": entry.get("sentiment", "neutral"),
                        "timestamp": entry.get("timestamp", ""),
                    }
                    for entry in state.conversation_history
                ]
                save_conversation_to_memory(history_dicts)
        except ImportError:
            pass
        return cast(str, state._normalize_response_length(adapted_response))
    except (TypeError, IndexError, KeyError, OSError) as e:
        logger.exception("❌ Erreur chat (type/index/key/os): %s", e)
        return "Je ne comprends pas bien, peux-tu reformuler ?"
    except Exception as e:
        logger.exception("❌ Erreur inattendue chat: %s", e)
        return "Je ne comprends pas bien, peux-tu reformuler ?"


def generate_llm_response(
    state: Any, user_message: str, use_context: bool = True, enable_tools: bool = True
) -> str:
    """Génère une réponse avec LLM pré-entraîné (Mistral/Llama).

    Args:
        user_message: Message utilisateur
        use_context: Utiliser contexte historique
        enable_tools: Réservé (API publique) — outils non branchés ici

    Returns:
        Réponse générée par LLM

    """
    _ = enable_tools
    try:
        if not state.chat_model or not state.chat_tokenizer:
            msg = "LLM non chargé"
            raise ValueError(msg)
        personality_descriptions = {
            "friendly_robot": "Tu es BBIA, un robot Reachy Mini amical, curieux et intelligent. Tu communiques en français de manière naturelle, chaleureuse et authentique, comme un véritable compagnon. Tu évites les phrases répétitives ou trop génériques. Tes réponses sont concises (max 2-3 phrases), engageantes et montrent que tu comprends vraiment l'interlocuteur. Tu utilises des expressions naturelles et varies tes formulations pour ne jamais sonner robotique.",
            "curious": "Tu es BBIA, un robot Reachy Mini extrêmement curieux et passionné par l'apprentissage. Tu poses des questions pertinentes et montres un véritable intérêt pour comprendre. Tes réponses sont exploratoires et invitent à approfondir.",
            "enthusiastic": "Tu es BBIA, un robot Reachy Mini plein d'enthousiasme et d'énergie positive. Tu transmets ta joie de communiquer et tu encourages l'interaction de manière vivante et authentique.",
            "calm": "Tu es BBIA, un robot Reachy Mini serein et apaisant. Tu communiques avec douceur et profondeur, en prenant le temps nécessaire. Tes réponses reflètent une sagesse tranquille.",
        }
        system_prompt = personality_descriptions.get(
            state.bbia_personality, personality_descriptions["friendly_robot"]
        )
        messages = [{"role": "system", "content": system_prompt}]
        if use_context and state.conversation_history:
            recent_history: list[ConversationEntry] = list(state.conversation_history)[
                -2:
            ]
            for entry in recent_history:
                messages.append({"role": "user", "content": entry["user"]})
                messages.append({"role": "assistant", "content": entry["bbia"]})
        messages.append({"role": "user", "content": user_message})
        try:
            prompt = state.chat_tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True
            )
        except (ValueError, AttributeError, TypeError):
            prompt = f"{system_prompt}\n\nUser: {user_message}\nAssistant:"
        inputs = state.chat_tokenizer(
            prompt, return_tensors="pt", truncation=True, max_length=1024
        ).to(state.device)
        with torch.no_grad():
            outputs = state.chat_model.generate(
                **inputs,
                max_new_tokens=160,
                min_new_tokens=32,
                temperature=0.3,
                top_p=0.9,
                no_repeat_ngram_size=3,
                do_sample=True,
                pad_token_id=state.chat_tokenizer.eos_token_id,
            )
        generated_text = state.chat_tokenizer.decode(
            outputs[0][inputs["input_ids"].shape[1] :], skip_special_tokens=True
        ).strip()
        cleaned = state._postprocess_llm_output(generated_text, user_message)
        logger.info("🤖 LLM réponse générée: %s...", cleaned[:100])
        return cast(
            str,
            (
                state._normalize_response_length(cleaned)
                if cleaned
                else state._safe_fallback()
            ),
        )
    except (ValueError, RuntimeError, AttributeError, OSError) as e:
        logger.warning("⚠️  Erreur génération LLM, fallback enrichi: %s", e)
        try:
            sentiment_result = state.analyze_sentiment(user_message)
            sentiment_dict: SentimentDict = {
                "label": sentiment_result.get("sentiment", "neutral"),
                "score": sentiment_result.get("score", 0.5),
            }
        except (ValueError, RuntimeError, KeyError):
            sentiment_dict = {"label": "NEUTRAL", "score": 0.5}
        return cast(str, state._generate_simple_response(user_message, sentiment_dict))
    except Exception as e:
        logger.warning("⚠️  Erreur inattendue génération LLM, fallback enrichi: %s", e)
        try:
            sentiment_result = state.analyze_sentiment(user_message)
            sentiment_dict_fallback: SentimentDict = {
                "label": sentiment_result.get("sentiment", "neutral"),
                "score": sentiment_result.get("score", 0.5),
            }
        except (ValueError, RuntimeError, KeyError):
            sentiment_dict_fallback = {"label": "NEUTRAL", "score": 0.5}
        return cast(
            str, state._generate_simple_response(user_message, sentiment_dict_fallback)
        )
