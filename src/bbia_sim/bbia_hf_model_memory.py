"""Mémoire / cycle de vie modèles HF.

Extrait de ``BBIAHuggingFace`` : enable/disable LLM, unload LRU, boucle
auto-unload partagée. Les méthodes instance délèguent ici.
"""

from __future__ import annotations

import logging
import operator
import time
from collections import deque
from typing import Any, cast

logger = logging.getLogger(__name__)

try:
    import torch

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    torch = None  # type: ignore[assignment]


def _hf_mod() -> Any:
    from . import bbia_huggingface as _hf

    return _hf


def enable_llm_chat(state: Any, model_name: str = "mistral") -> bool:
    """Active le LLM conversationnel (optionnel, lourd).

    Args:
        model_name: Modèle LLM à charger (alias: "mistral", "llama",
            "phi2", "tinyllama"
                   ou ID complet Hugging Face)

    Returns:
        True si chargé avec succès

    Note:
        - Mistral 7B / Llama 3 8B : ~14-16GB RAM (pas pour RPi 5)
        - Phi-2 : ~5GB RAM (recommandé pour RPi 5)
        - TinyLlama : ~2GB RAM (ultra-léger)
        - Premier chargement : 1-2 minutes
        - Support Apple Silicon (MPS) automatique

    """
    resolved_name = state._resolve_model_name(model_name, "chat")
    logger.info("📥 Activation LLM conversationnel: %s → %s", model_name, resolved_name)
    success = state.load_model(resolved_name, model_type="chat")
    if success:
        logger.info(
            "✅ LLM conversationnel activé - Conversations intelligentes disponibles"
        )
    else:
        logger.warning("⚠️  LLM non chargé - Utilisation réponses enrichies")
    return cast(bool, success)


def disable_llm_chat(state: Any) -> None:
    """Désactive le LLM conversationnel pour libérer mémoire."""
    try:
        if hasattr(state, "chat_model") and state.chat_model is not None:
            del state.chat_model
    except (AttributeError, RuntimeError) as e:
        logger.debug("Erreur suppression chat_model: %s", e)
    except (TypeError, KeyError, OSError) as e:
        logger.debug("Erreur suppression chat_model (type/key/os): %s", e)
    except Exception as e:
        logger.debug("Erreur inattendue suppression chat_model: %s", e)
    try:
        if hasattr(state, "chat_tokenizer") and state.chat_tokenizer is not None:
            del state.chat_tokenizer
    except (AttributeError, RuntimeError) as e:
        logger.debug("Erreur suppression chat_tokenizer: %s", e)
    except (TypeError, KeyError, OSError) as e:
        logger.debug("Erreur suppression chat_tokenizer (type/key/os): %s", e)
    except Exception as e:
        logger.debug("Erreur inattendue suppression chat_tokenizer: %s", e)
    state.chat_model = None
    state.chat_tokenizer = None
    state.use_llm_chat = False
    import gc

    gc.collect()
    if HF_AVAILABLE and torch.cuda.is_available():
        torch.cuda.empty_cache()
    logger.info("🗑️ LLM conversationnel désactivé - Mémoire libérée")


def unload_lru_model(state: Any) -> None:
    """OPTIMISATION RAM: Décharge le modèle LRU (Least Recently Used).

    Libère de la RAM en déchargant le modèle le moins récemment utilisé.
    """
    if not state._model_last_used:
        return
    oldest_key = min(state._model_last_used.items(), key=operator.itemgetter(1))[0]
    parts = oldest_key.rsplit("_", 1)
    if len(parts) == 2:
        model_name = parts[0]
        state.unload_model(model_name)
        if oldest_key in state._model_last_used:
            del state._model_last_used[oldest_key]
        logger.debug("♻️ Modèle LRU déchargé (optimisation RAM): %s", oldest_key)


def update_model_usage(state: Any, model_key: str) -> None:
    """OPTIMISATION RAM: Met à jour timestamp d'usage d'un modèle."""
    state._model_last_used[model_key] = time.time()


def unload_model(state: Any, model_name: str) -> bool:
    """Décharge un modèle de la mémoire.

    Args:
        model_name: Nom du modèle à décharger

    Returns:
        True si déchargé avec succès

    """
    try:
        keys_to_remove = [key for key in state.models if model_name in key]
        for key in keys_to_remove:
            del state.models[key]
        keys_to_remove = [key for key in state.processors if model_name in key]
        for key in keys_to_remove:
            del state.processors[key]
        import gc

        gc.collect()
        if HF_AVAILABLE:
            try:
                import torch

                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            except ImportError:
                pass
        logger.info("🗑️ Modèle %s déchargé - Mémoire libérée", model_name)
        return True
    except (AttributeError, RuntimeError, KeyError):
        logger.exception("❌ Erreur déchargement modèle %s:", model_name)
        return False
    except (TypeError, IndexError, OSError) as e:
        logger.exception(
            "❌ Erreur déchargement modèle %s (type/index/os): %s", model_name, e
        )
        return False
    except Exception as e:
        logger.exception(
            "❌ Erreur inattendue déchargement modèle %s: %s", model_name, e
        )
        return False


def shared_auto_unload_loop() -> None:
    BBIAHuggingFace = _hf_mod().BBIAHuggingFace
    """Boucle de déchargement automatique partagée pour toutes les instances."""
    while not BBIAHuggingFace._shared_unload_thread_stop.is_set():
        try:
            if BBIAHuggingFace._shared_unload_thread_stop.wait(10.0):
                break
            current_time = time.time()
            models_to_unload: deque[tuple[Any, str, float]] = deque(maxlen=50)
            with BBIAHuggingFace._shared_unload_thread_lock:
                active_instances = list(BBIAHuggingFace._shared_instances)
                for instance in active_instances:
                    try:
                        if not hasattr(instance, "_model_last_used"):
                            continue
                        model_last_used = getattr(instance, "_model_last_used", {})
                        inactivity_timeout = getattr(
                            instance, "_inactivity_timeout", 300.0
                        )
                        for model_key, last_used in list(model_last_used.items()):
                            inactivity = current_time - last_used
                            if inactivity > inactivity_timeout:
                                models_to_unload.append(
                                    (instance, model_key, inactivity)
                                )
                    except (AttributeError, RuntimeError):
                        continue
            for instance, model_key, inactivity in models_to_unload:
                try:
                    if not hasattr(instance, "unload_model"):
                        continue
                    parts = model_key.rsplit("_", 1)
                    if len(parts) == 2:
                        model_name = parts[0]
                        logger.debug(
                            "🗑️ Déchargement auto modèle inactif (%.0fs): %s",
                            inactivity,
                            model_key,
                        )
                        unload = getattr(instance, "unload_model", None)
                        if callable(unload):
                            unload(model_name)
                        model_last_used = getattr(instance, "_model_last_used", None)
                        if model_last_used is not None:
                            with BBIAHuggingFace._shared_unload_thread_lock:
                                if model_key in model_last_used:
                                    del model_last_used[model_key]
                except (AttributeError, RuntimeError, KeyError) as e:
                    logger.debug("Erreur déchargement auto %s: %s", model_key, e)
                except (TypeError, IndexError, OSError) as e:
                    logger.debug(
                        "Erreur déchargement auto %s (type/index/os): %s", model_key, e
                    )
                except Exception as e:
                    logger.debug(
                        "Erreur inattendue déchargement auto %s: %s", model_key, e
                    )
        except (RuntimeError, AttributeError) as e:
            logger.debug("Erreur boucle déchargement auto partagée: %s", e)
        except (TypeError, IndexError, KeyError, OSError) as e:
            logger.debug(
                "Erreur boucle déchargement auto partagée (type/index/key/os): %s", e
            )
        except Exception as e:
            logger.debug("Erreur inattendue boucle déchargement auto partagée: %s", e)
