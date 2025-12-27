#!/usr/bin/env python3
"""Optimisation chargement modèles - Cache et lazy loading."""

import logging
from collections import OrderedDict
from collections.abc import Callable
from typing import Any

logger = logging.getLogger(__name__)

# OPTIMISATION RAM: Cache global LRU pour modèles chargés (max 10 modèles)
_model_cache: OrderedDict[str, Any] = OrderedDict()
_MAX_CACHE_SIZE = 10  # Maximum 10 modèles en cache


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
        # OPTIMISATION RAM: Déplacer en fin (LRU - Least Recently Used)
        _model_cache.move_to_end(model_key)
        logger.debug("📦 Modèle '%s' chargé depuis cache", model_key)
        return _model_cache[model_key]

    logger.info("📥 Chargement modèle '%s'...", model_key)
    model = loader_func(*args, **kwargs)

    # OPTIMISATION RAM: Vérifier limite cache et évincer LRU si nécessaire
    if len(_model_cache) >= _MAX_CACHE_SIZE:
        oldest_key = next(iter(_model_cache))
        del _model_cache[oldest_key]
        logger.debug("🗑️ Modèle évincé du cache: %s", oldest_key)

    _model_cache[model_key] = model
    _model_cache.move_to_end(model_key)  # Déplacer en fin (LRU)
    logger.info("✅ Modèle '%s' chargé et mis en cache", model_key)
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
