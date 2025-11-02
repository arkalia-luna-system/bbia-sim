#!/usr/bin/env python3
"""Optimisation chargement modèles - Cache et lazy loading."""

import logging
from collections.abc import Callable
from typing import Any

logger = logging.getLogger(__name__)

# Cache global pour modèles chargés
_model_cache: dict[str, Any] = {}


def get_cached_model(
    model_key: str,
    loader_func: Callable[..., Any],
    *args: Any,
    **kwargs: Any,
) -> Any:
    """Charge modèle avec cache (évite rechargements).

    Args:
        model_key: Clé unique pour le modèle
        loader_func: Fonction qui charge le modèle
        *args: Arguments pour loader_func
        **kwargs: Keyword arguments pour loader_func

    Returns:
        Modèle chargé (depuis cache ou nouveau)

    """
    if model_key in _model_cache:
        logger.debug(f"📦 Modèle '{model_key}' chargé depuis cache")
        return _model_cache[model_key]

    logger.info(f"📥 Chargement modèle '{model_key}'...")
    model = loader_func(*args, **kwargs)
    _model_cache[model_key] = model
    logger.info(f"✅ Modèle '{model_key}' chargé et mis en cache")
    return model


def clear_model_cache() -> None:
    """Efface le cache de modèles (libère mémoire)."""
    global _model_cache
    _model_cache.clear()
    logger.info("🧹 Cache modèles effacé")


def get_cache_size() -> int:
    """Retourne nombre de modèles en cache."""
    return len(_model_cache)


def lazy_load_model(
    model_key: str,
    loader_func: Callable[..., Any],
) -> Callable[..., Any]:
    """Retourne fonction lazy loader (charge uniquement si appelée).

    Args:
        model_key: Clé unique pour le modèle
        loader_func: Fonction qui charge le modèle

    Returns:
        Fonction lazy qui charge modèle à la demande

    """

    def lazy_loader(*args: Any, **kwargs: Any) -> Any:
        return get_cached_model(model_key, loader_func, *args, **kwargs)

    return lazy_loader
